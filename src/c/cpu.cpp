/*
 * Class implementing the emulated OpenRISC 1000 CPU
 */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include <fstream>
#include <sstream>
#include <string>

#include <bzlib.h>

#include "util.hpp"
#include "dummy_device.hpp"
#include "uart_device.hpp"
#include "ata_device.hpp"
#include "fb_device.hpp"
#include "kb_device.hpp"
#include "eth_device.hpp"
#include "lpc32_device.hpp"

#include "cpu.hpp"

CPU::CPU(pp::Instance* pp_instance)
  : pp_instance(pp_instance)
{
  uint32_t i;

  ram = (char*)malloc(sizeof(char) * RAM_SIZE_BYTES);
  spr_generic = (char*)malloc(sizeof(char) * 1024 * 4);
  spr_dtlb = (char*)malloc(sizeof(char) * 1024 * 4);
  spr_itlb = (char*)malloc(sizeof(char) * 1024 * 4);

  if (ram == NULL) {
    fprintf(stderr, "ERROR: couldn't allocate RAM block!\n");
  }
  if (spr_generic == NULL) {
    fprintf(stderr, "ERROR: couldn't allocate SPR generic block!\n");
  }
  if (spr_dtlb == NULL) {
    fprintf(stderr, "ERROR: couldn't allocate SPR DTLB block!\n");
  }
  if (spr_itlb == NULL) {
    fprintf(stderr, "ERROR: couldn't allocate SPR ITLB block!\n");
  }

  ram_uint8 = (uint8_t*)ram;
  ram_uint16 = (uint16_t*)ram;
  ram_uint32 = (uint32_t*)ram;

  spr_generic_uint32 = (uint32_t*)spr_generic;
  spr_dtlb_uint32 = (uint32_t*)spr_dtlb;
  spr_itlb_uint32 = (uint32_t*)spr_itlb;
  
  for (i = 0; i < MAX_DEVICES; ++i) {
    device[i] = NULL;
  }

  // Devices are assigned addresses from 0x00000000 onwards.  Device 0 gets
  // address 0x00000000, device 1 gets 0x01000000, device 2 gets 0x02000000,
  // etc.  Devices that overlap with RAM will not be accessible by software
  // running on the emulated CPU
  uart_device = new UARTDevice(this);
  fb_device = new FrameBufferDevice(pp_instance, this, ram_uint32);
  kb_device = new KeyboardDevice(this);
  ata_device = new ATADevice(this);
  eth_device = new EthernetDevice(this, ram_uint32);

  device[0x90] = uart_device;               // 0x90000000
  device[0x91] = fb_device;                 // 0x91000000
  device[0x92] = eth_device;                // 0x92000000
  device[0x93] = new LPC32Device(this);     // 0x93000000
  device[0x94] = kb_device;                 // 0x94000000
  device[0x9e] = ata_device;                // 0x9e000000

  for (i = 0; i < MAX_DEVICES; ++i) {
    if (device[i] == NULL) {
      device[i] = new DummyDevice(this, 0x01000000 * i);
    }
  }

  cycle_count = 0;

  debug_message_count = 0;
  debug_instcount = 0;
  debug_pause_trace = false;
  debug_point_en = false;

  // Some start-up assertions
  assert(sizeof(uint32_t) == sizeof(float));
}

CPU::~CPU() {
  int32_t i;

  free(ram);
  ram = NULL;

  free(spr_generic);
  spr_generic = NULL;

  free(spr_dtlb);
  spr_dtlb = NULL;

  free(spr_itlb);
  spr_itlb = NULL;
  
  for (i = 0; i < MAX_DEVICES; ++i) {
    delete device[i];
    device[i] = NULL;
  }
}

inline void CPU::Reset(uint32_t& pc, uint32_t& next_pc) {
  std::lock_guard<std::mutex> lock(interrupt_mutex);

  int32_t i;
  
  delayed_ins = false;
  
  spr_generic_uint32[SPR_IMMUCFGR] = 0x1c; // 0 ITLB has one way and 128 sets
  spr_generic_uint32[SPR_DMMUCFGR] = 0x1c; // 0 DTLB has one way and 128 sets
  Exception(EXCEPT_RESET, 0x0, pc, next_pc); // Set nextpc value
  
  pc = next_pc;
  ++next_pc;

  cycle_count = 0;
  
  ipage_va = 0xffffffff;
  dpage_rd_int8_va = 0xffffffff;
  dpage_rd_uint8_va = 0xffffffff;
  dpage_rd_int16_va = 0xffffffff;
  dpage_rd_uint16_va = 0xffffffff;
  dpage_rd_uint32_va = 0xffffffff;
  dpage_wr_uint8_va = 0xffffffff;
  dpage_wr_uint16_va = 0xffffffff;
  dpage_wr_uint32_va = 0xffffffff;
  
  // REVISIT: is this necessary or can the regfile state be unpredictable after reset?
  for (i = 0; i < 32; ++i) {
    rf.r[i] = 0;
  }
  
  TTMR = 0x0;
  TTCR = 0x0;
  
  PICMR = 0x3;
  PICSR = 0x0;

  SR = (  (1 << SR_SM_POS)
        | (1 << SR_FO_POS));
  SR_F = false;
}

/*
 * Load the indicated boot image (from the NaCl virtual file system) into
 * emulated RAM.  Returns true if successful
 */
bool CPU::LoadRAMImage(const std::string& file_name) {
  std::ifstream fs;
  
  SendStrToTerminal("Loading RAM image %s...", file_name.c_str());
  
  // Attempt to open the boot image
  fs.open(file_name, std::ifstream::in | std::ifstream::binary);

  if (!fs.fail()) {
    char* buffer;
    uint32_t compressed_file_size;
    uint32_t file_size;
    int decompress_result;
    
    // Read the entire contents of the boot image into a temporary buffer
    fs.seekg(0, fs.end);
    compressed_file_size = fs.tellg();
    fs.seekg(0, fs.beg);
    
    buffer = (char*)malloc(sizeof(char) * compressed_file_size);
    if (buffer != NULL) {
      fs.read(buffer, compressed_file_size);
    }
    fs.close();

    if (buffer == NULL) {
      SendStrToTerminal("Critical error: failed to allocate buffer for compressed RAM image");
      return false;
    }

    // Decompress the image into the emulated RAM
    SendStrToTerminal("Decompressing image...");

    file_size = sizeof(char) * RAM_SIZE_BYTES;
    decompress_result = BZ2_bzBuffToBuffDecompress(ram,
                                                   &file_size,
                                                   buffer,
                                                   compressed_file_size,
                                                   0,
                                                   0);
    free(buffer);

    if (decompress_result == BZ_OK) {

      // Convert from big to little endian
      for (uint32_t i = 0; i < (file_size >> 2); ++i) {
        ram_uint32[i] = util::ByteSwap32(ram_uint32[i]);
      }
      
      SendStrToTerminal("RAM image loaded and decompressed successfully");

    } else {
      SendStrToTerminal("Critical error: RAM image decompression returned error code %d", decompress_result);
      return false;
    }
    
    return true;
  } else {
    SendStrToTerminal("Critical error: failed to load RAM image %s", file_name.c_str());
    return false;
  }
}

/*
 * Load the indicated hard disk drive image (from the NaCl virtual file system)
 * into the buffer of the emulated ATA device.  Returns true if successful
 */
bool CPU::LoadHDDImage(const std::string& file_name) {
  std::ifstream fs;
  
  SendStrToTerminal("Loading HDD image %s...", file_name.c_str());
  
  // Attempt to open the boot image
  fs.open(file_name, std::ifstream::in | std::ifstream::binary);

  if (!fs.fail()) {
    char* buffer;
    char* output_buffer;
    uint16_t* uint16_output_buffer;
    uint32_t compressed_file_size;
    uint32_t file_size;
    int decompress_result;
    
    // Read the entire contents of the boot image into a temporary buffer
    fs.seekg(0, fs.end);
    compressed_file_size = fs.tellg();
    fs.seekg(0, fs.beg);
    
    buffer = (char*)malloc(sizeof(char) * compressed_file_size);
    if (buffer != NULL) {
      fs.read(buffer, compressed_file_size);
    }
    fs.close();

    if (buffer == NULL) {
      SendStrToTerminal("Critical error: failed to allocate buffer for compressed HDD image");
      return false;
    }

    // Decompress the image into the emulated HDD
    SendStrToTerminal("Decompressing image...");

    file_size = sizeof(char) * MAX_HDD_IMAGE_SIZE_BYTES;
    output_buffer = (char*)malloc(file_size);
    if (output_buffer != NULL) {
      decompress_result = BZ2_bzBuffToBuffDecompress(output_buffer,
                                                     &file_size,
                                                     buffer,
                                                     compressed_file_size,
                                                     0,
                                                     0);
    }
    free(buffer);

    if (output_buffer == NULL ) {
      SendStrToTerminal("Critical error: failed to allocate buffer for decompressed HDD image");
      return false;
    }

    output_buffer = (char*)realloc(output_buffer, file_size);

    if (output_buffer == NULL ) {
      SendStrToTerminal("Critical error: failed to reallocate buffer for decompressed HDD image");
      return false;
    }

    if (decompress_result == BZ_OK) {
      // Convert from big to little endian (on half-words)
      uint16_output_buffer = (uint16_t*)output_buffer;
      for (uint32_t i = 0; i < (file_size >> 1); ++i) {
        uint16_output_buffer[i] = util::ByteSwap16(uint16_output_buffer[i]);
      }

      // Assign the disk image to the emulated ATA device
      ((ATADevice*)ata_device)->SetDiskImage(output_buffer,
                                             sizeof(char) * file_size);
      
      SendStrToTerminal("HDD image loaded and decompressed successfully");

    } else {
      SendStrToTerminal("Critical error: HDD image decompression returned error code %d", decompress_result);
      return false;
    }
    
    return true;
  } else {
    SendStrToTerminal("Critical error: failed to load HDD image %s", file_name.c_str());
    return false;
  }
}

/*
 * Store Linux kernel's TLB handler addresses in order to emulate TLB lookups
 * in hardware (for performance).
 *
 * This function should be called once after the kernel image has been loaded
 * into the emulator RAM but before the emulator is started
 */
void CPU::AnalyzeImage() {
  boot_dtlb_misshandler_address = ram_uint32[0x900 >> 2];
  boot_itlb_misshandler_address = ram_uint32[0xA00 >> 2];
  current_pgd = ((ram_uint32[0x2018 >> 2] & 0xFFF) << 16)
    | (ram_uint32[0x201C >> 2] & 0xFFFF);
}

inline void CPU::SetFlags(uint32_t x) {
  const uint32_t prev_SR_DME = SR & (1 << SR_DME_POS);
  const uint32_t prev_SR_IME = SR & (1 << SR_IME_POS);

  SR = x;
  SR_F = (x & (1 << SR_F_POS)) ? true : false;

  if (SR & (  (1 << SR_LEE_POS)
            | (0xf << SR_CID_POS)
            | (1 << SR_EPH_POS)
            | (1 << SR_DSX_POS))) {
    if (SR & (1 << SR_LEE_POS)) {
      DebugMessage("SetFlags() error: little endian not supported");
    }
    if (SR & (0xf << SR_CID_POS)) {
      DebugMessage("SetFlags() error: context ID not supported");
    }
    if (SR & (1 << SR_EPH_POS)) {
      DebugMessage("SetFlags() error: exception prefix not supported");
    }
    if (SR & (1 << SR_DSX_POS)) {
      DebugMessage("SetFlags() error: delay slot exception not supported");
    }
  }

  if (!(SR & (1 << SR_IME_POS)) && prev_SR_IME) {
    ipage_va = 0;
    ipage_adj_va = 0;
  }
  if (!(SR & (1 << SR_DME_POS)) && prev_SR_DME) {
    dpage_rd_int8_va = 0;
    dpage_rd_uint8_va = 0;
    dpage_rd_int16_va = 0;
    dpage_rd_uint16_va = 0;
    dpage_rd_uint32_va = 0;
    dpage_wr_uint8_va = 0;
    dpage_wr_uint16_va = 0;
    dpage_wr_uint32_va = 0;
    dpage_rd_int8_adj_va = 0;
    dpage_rd_uint8_adj_va = 0;
    dpage_rd_int16_adj_va = 0;
    dpage_rd_uint16_adj_va = 0;
    dpage_rd_uint32_adj_va = 0;
    dpage_wr_uint8_adj_va = 0;
    dpage_wr_uint16_adj_va = 0;
    dpage_wr_uint32_adj_va = 0;
  }
}

inline uint32_t CPU::GetFlags() {
  return (SR & (0xffffffff ^ (1 << SR_F_POS)))
    | (SR_F ? (1 << SR_F_POS) : 0);
}

void CPU::RaiseInterrupt(uint32_t line) {
  std::lock_guard<std::mutex> lock(interrupt_mutex);

  PICSR |= (1 << line);
}

void CPU::ClearInterrupt(uint32_t line) {
  std::lock_guard<std::mutex> lock(interrupt_mutex);

  PICSR &= ~(1 << line);
}

inline void CPU::SetSPR(uint32_t idx, uint32_t x) {
  uint32_t address = idx & 0x7FF;
  uint32_t group = (idx >> 11) & 0x1F;

  switch (group) {
  case 1:
    // Data MMU
    spr_dtlb_uint32[address] = x;
    return;
  case 2:
    // ins MMU
    spr_itlb_uint32[address] = x;
    return;  case 3:
    // data cache, not supported
  case 4:
    // ins cache, not supported
    return;
  case 9:
    // pic
    switch (address) {
    case 0:
      {
        std::lock_guard<std::mutex> lock(interrupt_mutex);

        PICMR = x | 0x3; // we use non maskable interrupt here
        // check immediate for interrupt
        
        if ((SR & (1 << SR_IEE_POS)) & PICMR & PICSR) {
          DebugMessage("Error in SetSPR: Direct triggering of interrupt exception not supported");
        }
      }
      break;
    case 2:
      {
        std::lock_guard<std::mutex> lock(interrupt_mutex);

        PICSR = x;
      }
      break;
    default:
      DebugMessage("Error in SetSPR: interrupt address not supported");
      break;
    }
    return;
  case 10:
    //tick timer
    switch (address) {
    case 0:
      TTMR = x;
      if (((TTMR >> 30) & 0x3) != 0x3) {
        DebugMessage("Error in SetSPR: Timer mode other than continuous not supported");
        return;
      }
      break;
    default:
      DebugMessage("Error in SetSPR: Tick timer address not supported");
      break;
    }
    return;
  default:
    break;
  }

  if (group != 0) {
    DebugMessage("Error in SetSPR: group %d not found", group);
    return;
  }

  switch (address) {
  case SPR_SR:
    SetFlags(x);
    break;
  case SPR_EEAR_BASE:
  case SPR_EPCR_BASE:
  case SPR_ESR_BASE:
    spr_generic_uint32[address] = x;
    break;
  default:
    DebugMessage("Error in SetSPR: address 0x%08x not found", address);
    break;
  }
}
  
inline uint32_t CPU::GetSPR(uint32_t idx) {
  uint32_t address = idx & 0x7FF;
  uint32_t group = (idx >> 11) & 0x1F;

  switch (group) {
  case 1:
    return spr_dtlb_uint32[address];
  case 2:
    return spr_itlb_uint32[address];

  case 9:
    // pic
    switch (address) {
    case 0:
      return PICMR;
    case 2:
      {
        std::lock_guard<std::mutex> lock(interrupt_mutex);
        return PICSR;
      }
    default:
      DebugMessage("Error in GetSPR: PIC address unknown");
      break;
    }
    break;

  case 10:
    // tick Timer
    switch (address) {
    case 0:
      return TTMR;
    case 1:
      return TTCR; // or clock
    default:
      DebugMessage("Error in GetSPR: Tick timer address unknown");
      break;
    }
    break;
  default:
    break;
  }

  if (group != 0) {
    DebugMessage("Error in GetSPR: group %d unknown", group);
    return 0;
  }

  switch (idx) {
  case SPR_SR:
    return GetFlags();

  case SPR_UPR:
    // UPR present
    // data mmu present
    // instruction mmu present
    // PIC present (architecture manual seems to be wrong here)
    // Tick timer present
    return 0x619;

  case SPR_IMMUCFGR:
  case SPR_DMMUCFGR:
  case SPR_EEAR_BASE:
  case SPR_EPCR_BASE:
  case SPR_ESR_BASE:
    return spr_generic_uint32[address];
  case SPR_ICCFGR:
    //return 0x48;
    return 0;
  case SPR_DCCFGR:
    //return 0x48;
    return 0;
  case SPR_VR:
    return 0x12000001;
  default:
    DebugMessage("Error in GetSPR: address unknown");
    return 0;
  }
}

inline void CPU::Exception(uint32_t except_type, uint32_t addr,
                           uint32_t& pc, uint32_t& next_pc) {
  uint32_t except_vector = except_type
    | ((SR & (1 << SR_EPH_POS)) ? 0xf0000000 : 0x0);

  //fprintf(stderr, "INFO (CPU): raising exception 0x%03x\n", except_type);

  SetSPR(SPR_EEAR_BASE, addr);
  SetSPR(SPR_ESR_BASE, GetFlags());

  SR &= (0xffffffff ^ (  (1 << SR_OVE_POS)
                       | (1 << SR_IEE_POS)
                       | (1 << SR_TEE_POS)
                       | (1 << SR_IME_POS)
                       | (1 << SR_DME_POS)));
  SR |= (1 << SR_SM_POS);

  ipage_va = 0;
  dpage_rd_int8_va = 0;
  dpage_rd_uint8_va = 0;
  dpage_rd_int16_va = 0;
  dpage_rd_uint16_va = 0;
  dpage_rd_uint32_va = 0;
  dpage_wr_uint8_va = 0;
  dpage_wr_uint16_va = 0;
  dpage_wr_uint32_va = 0;
  ipage_adj_va = 0;
  dpage_rd_int8_adj_va = 0;
  dpage_rd_uint8_adj_va = 0;
  dpage_rd_int16_adj_va = 0;
  dpage_rd_uint16_adj_va = 0;
  dpage_rd_uint32_adj_va = 0;
  dpage_wr_uint8_adj_va = 0;
  dpage_wr_uint16_adj_va = 0;
  dpage_wr_uint32_adj_va = 0;

  next_pc = (except_vector >> 2);

  switch (except_type) {
  case EXCEPT_RESET:
    break;
      
  case EXCEPT_ITLBMISS:
    debug_pause_trace = true;
  case EXCEPT_IPF:
    SetSPR(SPR_EPCR_BASE, addr - (delayed_ins ? 4 : 0));
    break;

  case EXCEPT_DTLBMISS:
    debug_pause_trace = true;
  case EXCEPT_DPF:
  case EXCEPT_BUSERR:
    SetSPR(SPR_EPCR_BASE, (pc << 2) - (delayed_ins ? 4 : 0));
    break;
      
  case EXCEPT_TICK:
  case EXCEPT_INT:
    debug_pause_trace = true;
    SetSPR(SPR_EPCR_BASE, (pc << 2) - (delayed_ins ? 4 : 0));
    break;

  case EXCEPT_SYSCALL:
    SetSPR(SPR_EPCR_BASE, (pc << 2) + 4 - (delayed_ins ? 4 : 0));
    break;

  default:
    DebugMessage("Error in Exception: exception type not supported");
    break;
  }
  delayed_ins = false;
}

// REVISIT: emulate hardware refill to boost emulator performance?
inline bool CPU::DTLBRefill(uint32_t addr, uint32_t nsets,
                            uint32_t& pc, uint32_t& next_pc) {
  Exception(EXCEPT_DTLBMISS, addr, pc, next_pc);
  return false;
}

// REVISIT: emulate hardware refill to boost emulator performance?
inline bool CPU::ITLBRefill(uint32_t addr, uint32_t nsets,
                            uint32_t& pc, uint32_t& next_pc) {
  Exception(EXCEPT_ITLBMISS, addr, pc, next_pc);
  return false;
}

inline uint32_t CPU::DTLBLookup(uint32_t addr, bool write,
                                uint32_t& pc, uint32_t& next_pc) {
  uint32_t setindex;
  uint32_t tlmbr;
  uint32_t tlbtr;

  if (!(SR & (1 << SR_DME_POS))) {
    return addr;
  }

  // pagesize is 8192 bytes
  // nways are 1
  // nsets are 64

  // Get the match register value in the TLB (containing the VA)
  setindex = (addr >> 13) & 0x7f;
  tlmbr = spr_dtlb_uint32[0x200 | setindex];

  if (!(tlmbr & 1) || (tlmbr >> 19) != (addr >> 19)) {
    if (DTLBRefill(addr, 64, pc, next_pc)) {
      tlmbr = spr_dtlb_uint32[0x200 + setindex];
    } else {
      return 0xffffffff;
    }
  }

  tlbtr = spr_dtlb_uint32[0x280 | setindex]; // translate register

  // Check for page fault
  if (SR & (1 << SR_SM_POS)) {
    if (((!write) && !(tlbtr & 0x100)) || // check if SRE
        ((write) && !(tlbtr & 0x200))     // check if SWE
        ) {
      Exception(EXCEPT_DPF, addr, pc, next_pc);
      return 0xffffffff;
    }
  } else {
    if (((!write) && !(tlbtr & 0x40)) || // check if URE
        ((write) && !(tlbtr & 0x80))     // check if UWE
        ) {
      Exception(EXCEPT_DPF, addr, pc, next_pc);
      return 0xffffffff;
    }
  }
  return ((tlbtr & 0xFFFFE000) | (addr & 0x1FFF));
}

inline uint32_t CPU::DTLBLookupNoExceptions(uint32_t addr, bool write) {
  uint32_t setindex;
  uint32_t tlmbr;
  uint32_t tlbtr;
    
  if (!(SR & (1 << SR_DME_POS))) {
    return addr;
  }

  // Get the match register value in the TLB (containing the VA)
  setindex = (addr >> 13) & 0x7f;
  tlmbr = spr_dtlb_uint32[0x200 | setindex];

  if ((tlmbr & 1) == 0 || (tlmbr >> 19) != (addr >> 19)) {
    // REVISIT: with optimised DTLB refills, need to call a passive version here that doesn't raise an exception or update the actual DTLB
    //if (DTLBRefill(addr, 64)) {
    //  tlmbr = spr_dtlb_uint32[0x200 + setindex];
    //} else {
    return 0xffffffff;
    //}
  }

  tlbtr = spr_dtlb_uint32[0x280 | setindex]; // translate register

  // Check for page fault
  if (SR & (1 << SR_SM_POS)) {
    if (((!write) && !(tlbtr & 0x100)) || // check if SRE
        ((write) && !(tlbtr & 0x200))     // check if SWE
        ) {
      return 0xffffffff;
    }
  } else {
    if (((!write) && !(tlbtr & 0x40)) || // check if URE
        ((write) && !(tlbtr & 0x80))     // check if UWE
        ) {
      return 0xffffffff;
    }
  }
  return ((tlbtr & 0xFFFFE000) | (addr & 0x1FFF));
}

inline uint8_t CPU::ReadDevMem8(uint32_t addr) {
  return device[(addr & 0xff000000) >> 24]->Read8((addr & 0x00ffffff));
}

inline uint16_t CPU::ReadDevMem16(uint32_t addr) {
  return device[(addr & 0xff000000) >> 24]->Read16((addr & 0x00ffffff));
}

inline uint32_t CPU::ReadDevMem32(uint32_t addr) {
  return device[(addr & 0xff000000) >> 24]->Read32((addr & 0x00ffffff));
}

inline void CPU::WriteDevMem8(uint32_t addr, uint8_t data) {
  device[(addr & 0xff000000) >> 24]->Write8((addr & 0x00ffffff), data);
}

inline void CPU::WriteDevMem16(uint32_t addr, uint16_t data) {
  device[(addr & 0xff000000) >> 24]->Write16((addr & 0x00ffffff), data);
}

inline void CPU::WriteDevMem32(uint32_t addr, uint32_t data) {
  device[(addr & 0xff000000) >> 24]->Write32((addr & 0x00ffffff), data);
}

uint32_t CPU::ReadMem32(uint32_t addr) {
  const uint32_t maskedAddr = addr & 0xfffffffc;

  if (maskedAddr > (sizeof(char) * RAM_SIZE_BYTES)) {
    return ReadDevMem32(maskedAddr);
  } else {
    return ram_uint32[maskedAddr >> 2];
  }
}

void CPU::WriteMem32(uint32_t addr, uint32_t data) {
  const uint32_t maskedAddr = addr & 0xfffffffc;

  if (maskedAddr > (sizeof(char) * RAM_SIZE_BYTES)) {
    WriteDevMem32(maskedAddr, data);
  } else {
    ram_uint32[maskedAddr >> 2] = data;
  }
}

/*
 * Execute instructions on the virtual machine in the main emulator execution
 * loop
 */
void CPU::Run() {
  register uint32_t ins;
  uint32_t rindex;
  uint32_t unsigned_imm;
  uint32_t imm;
  uint32_t addr;
  uint32_t paddr;
  uint32_t rA;
  uint32_t rB;
  uint32_t rD;
  register uint32_t pc;
  register uint32_t next_pc;
  register uint8_t steps;
  register int8_t i;

  steps = 0xff;

  Reset(pc, next_pc);

  /*
   * Main execution loop
   */
  do {

    /* Handle timer updates and interrupt handling */
    if (!steps) {
      cycle_count += 256; // "clock speed"

      // Advance the timer (if enabled)
      if (TTMR & 0xc0000000) {
        uint32_t TTCR_next;
      
        TTCR_next = TTCR + 256; // "clock speed"
      
        // If timer interrupt is enabled and the timer has passed the match
        // value then pend a timer interrupt.  Note that this differs from
        // the architectural behaviour of the timer which is supposed to fire
        // only when the counter equals the match value
        if (((TTCR & 0x0FFFFFFF) < (TTMR & 0x0FFFFFFF))
            && ((TTCR_next & 0x0FFFFFFF) >= (TTMR & 0x0FFFFFFF))
            && (TTMR & (1 << 29))) {
          TTMR |= (1 << 28);
        }
      
        TTCR = TTCR_next;
      }

      // If a timer interrupt is pending and enabled then raise it
      if (TTMR & 0x10000000) {
        if (SR & (1 << SR_TEE_POS)) {
          Exception(EXCEPT_TICK, spr_generic_uint32[SPR_EEAR_BASE], pc, next_pc);
          pc = next_pc++;
        }
      } else {
        // If a non-timer interrupt is pending and interrupts are enabled then
        // raise it
        std::lock_guard<std::mutex> lock(interrupt_mutex);

        if ((SR & (1 << SR_IEE_POS)) && (PICMR & PICSR)) {
          Exception(EXCEPT_INT, spr_generic_uint32[SPR_EEAR_BASE], pc, next_pc);
          pc = next_pc++;
        }
      }
    }

    /* Detect page change (for the next instruction fetch) */
    if ((ipage_va ^ pc) >> 11) {
      ipage_va = pc;
        
      if (!(SR & (1 << SR_IME_POS))) {
        // Page tables not enabled so VA == PA
        ipage_adj_va = 0;
      } else {
        uint32_t setindex;
        uint32_t tlmbr;
        uint32_t tlbtr;
          
        // Get the match register value in the TLB (containing the VA)
        setindex = (pc >> 11) & 0x7f;
        tlmbr = spr_itlb_uint32[0x200 | setindex];

        // Check for a TLB hit (TLB entry is valid and page VA matches that of
        // the PC).  On a miss, request a refill of the TLB entry
        if (!(tlmbr & 1) ||
            (tlmbr >> 19) != (pc >> 17)) {
          if (ITLBRefill(pc << 2, 64, pc, next_pc)) {
            tlmbr = spr_itlb_uint32[0x200 | setindex];
          } else {
            pc = next_pc++;
            continue;
          }
        }

        // Use the translation value to calculate a value to XOR with the
        // VA to get the PA
        tlbtr = spr_itlb_uint32[0x280 | setindex];
        ipage_adj_va = ((tlbtr ^ tlmbr) >> 13) << 11;
      }
    }
    
    // <DEBUG>
    //if ((pc ^ ipage_adj_va) > (sizeof(char) * RAM_SIZE_BYTES)) {
    //  fprintf(stderr, "ERROR (CPU): instcount=%lld, physical PC=0x%08x is out of memory range\n", debug_instcount, (pc ^ ipage_adj_va) << 2);
    //}
    // </DEBUG>

    /* Fetch the next instruction to execute */
    ins = ram_uint32[(pc ^ ipage_adj_va) & ((sizeof(char) * RAM_SIZE_BYTES) - 1)];

    // <DEBUG>
    //++debug_instcount;
    //if (debug_instcount > (DEBUG_INST_LIMIT - 200)) {
    //if (debug_instcount > 3133243) {
      //fprintf(stderr, "%lld: pc=0x%08x, paddr_pc=0x%08x, inst=0x%08x\n", debug_instcount, (pc << 2), ((pc ^ ipage_adj_va) << 2), ins);
      //DisassembleInstr(ins);
    //}
    //if (debug_point_en) {
    //  DisassembleInstr(ins);
    //}
    // </DEBUG>

    /* Decode and execute the fetched instruction */
    switch ((ins >> 26) & 0x3f) {
    case 0x0:
      // j
      addr = pc + ((ins & 0x02000000)
                   ? ((ins & 0x03FFFFFF) | 0xfc000000)
                   : (ins & 0x03FFFFFF));
      pc = next_pc;
      next_pc = addr;
      delayed_ins = true;
      --steps;
      continue;
      
    case 0x1:
      // jal
      addr = pc + ((ins & 0x02000000)
                   ? ((ins & 0x03FFFFFF) | 0xfc000000)
                   : (ins & 0x03FFFFFF));
      if (!delayed_ins) {
        rf.r[9] = (next_pc << 2) + 4;
      }
      pc = next_pc;
      next_pc = addr;
      delayed_ins = true;
      --steps;
      continue;
      
    case 0x3:
      // bnf
      if (SR_F) {
        break;
      }
      addr = pc + ((ins & 0x02000000)
                   ? ((ins & 0x03FFFFFF) | 0xfc000000)
                   : (ins & 0x03FFFFFF));
      pc = next_pc;
      next_pc = addr;
      delayed_ins = true;
      --steps;
      continue;
      
    case 0x4:
      // bf
      if (!SR_F) {
        break;
      }
      addr = pc + ((ins & 0x02000000)
                   ? ((ins & 0x03FFFFFF) | 0xfc000000)
                   : (ins & 0x03FFFFFF));
      pc = next_pc;
      next_pc = addr;
      delayed_ins = true;
      --steps;
      continue;
      
    case 0x5:
      // nop
      imm = ins & 0xffff;
      if (imm == 0x4) {
        //fprintf(stderr, "CHAR: %c\n", rf.r[3] & 0xff, stderr);
        putc(rf.r[3] & 0xff, stderr);
        //debug_point_en = true;
      }
      break;

    case 0x6:
      // movhi or macrc
      rindex = (ins >> 21) & 0x1F;
      // if 16th bit is set
      if (ins & 0x10000) {
        DebugMessage("Error: macrc not supported\n");
      } else {
        rf.r[rindex] = ((ins & 0xFFFF) << 16); // movhi
      }
      break;
        
    case 0x8:
      // sys
      if (!delayed_ins) {
        Exception(EXCEPT_SYSCALL, spr_generic_uint32[SPR_EEAR_BASE], pc, next_pc);
      }
      break;
        
    case 0x9:
      // rfe
      if (!delayed_ins) {
        next_pc = (GetSPR(SPR_EPCR_BASE) >> 2);
        SetFlags(GetSPR(SPR_ESR_BASE));
        
        ipage_va = 0xffffffff;
        dpage_rd_int8_va = 0xffffffff;
        dpage_rd_uint8_va = 0xffffffff;
        dpage_rd_int16_va = 0xffffffff;
        dpage_rd_uint16_va = 0xffffffff;
        dpage_rd_uint32_va = 0xffffffff;
        dpage_wr_uint8_va = 0xffffffff;
        dpage_wr_uint16_va = 0xffffffff;
        dpage_wr_uint32_va = 0xffffffff;
        
        debug_pause_trace = false;
      }
      break;
        
    case 0x11:
      // jr
      addr = (rf.r[(ins >> 11) & 0x1F] >> 2);
      pc = next_pc;
      next_pc = addr;
      delayed_ins = true;
      --steps;
      continue;
      
    case 0x12:
      // jalr
      addr = (rf.r[(ins >> 11) & 0x1F] >> 2);
      if (!delayed_ins) {
        rf.r[9] = (next_pc << 2) + 4;
      }
      pc = next_pc;
      next_pc = addr;
      delayed_ins = true;
      --steps;
      continue;
        
    case 0x21:
    case 0x22:
      // lwz/lws
      addr = rf.r[(ins >> 16) & 0x1F] + ((ins & 0x00008000)
                                         ? ((ins & 0x0000ffff) | 0xffff0000)
                                         : (ins & 0x0000ffff));
      if (addr & 0x3) {
        DebugMessage("Error in lwz/lws: no unaligned access allowed");
      }

      if ((dpage_rd_uint32_va ^ addr) >> 13) {
        paddr = DTLBLookup(addr, false, pc, next_pc);
        if (paddr == 0xffffffff) {
          break;
        }
        dpage_rd_uint32_va = addr;
        dpage_rd_uint32_adj_va = (paddr ^ addr) & 0xffffe000;
      }
      paddr = dpage_rd_uint32_adj_va ^ addr;
      
      if (paddr > (sizeof(char) * RAM_SIZE_BYTES)) {
        rf.r[(ins >> 21) & 0x1F] = ReadDevMem32(paddr);
      } else {
        rf.r[(ins >> 21) & 0x1F] = ram_uint32[paddr >> 2];
      }
      break;

    case 0x23:
      // lbz
      addr = rf.r[(ins >> 16) & 0x1F] + ((ins & 0x00008000)
                                         ? ((ins & 0x0000ffff) | 0xffff0000)
                                         : (ins & 0x0000ffff));

      if ((dpage_rd_uint8_va ^ addr) >> 13) {
        paddr = DTLBLookup(addr, false, pc, next_pc);
        if (paddr == 0xffffffff) {
          break;
        }
        dpage_rd_uint8_va = addr;
        dpage_rd_uint8_adj_va = (paddr ^ addr) & 0xffffe000;
      }
      paddr = dpage_rd_uint8_adj_va ^ addr;

      if (paddr > (sizeof(char) * RAM_SIZE_BYTES)) {
        rf.r[(ins >> 21) & 0x1F] = ReadDevMem8(paddr);
      } else {
        rf.r[(ins >> 21) & 0x1F] = (uint32_t)ram_uint8[paddr ^ 0x3];
      }
      break;
        
    case 0x24:
      // lbs
      addr = rf.r[(ins >> 16) & 0x1F] + ((ins & 0x00008000)
                                         ? ((ins & 0x0000ffff) | 0xffff0000)
                                         : (ins & 0x0000ffff));

      if ((dpage_rd_int8_va ^ addr) >> 13) {
        paddr = DTLBLookup(addr, false, pc, next_pc);
        if (paddr == 0xffffffff) {
          break;
        }
        dpage_rd_int8_va = addr;
        dpage_rd_int8_adj_va = (paddr ^ addr) & 0xffffe000;
      }
      paddr = dpage_rd_int8_adj_va ^ addr;

      if (paddr > (sizeof(char) * RAM_SIZE_BYTES)) {
        rf.r[(ins >> 21) & 0x1F] = (int32_t)((int8_t)ReadDevMem8(paddr));
      } else {
        rf.r[(ins >> 21) & 0x1F] = (int32_t)((int8_t)ram_uint8[paddr ^ 0x3]);
      }
      break;
        
    case 0x25:
      // lhz 
      addr = rf.r[(ins >> 16) & 0x1F] + ((ins & 0x00008000)
                                         ? ((ins & 0x0000ffff) | 0xffff0000)
                                         : (ins & 0x0000ffff));

      if ((dpage_rd_uint16_va ^ addr) >> 13) {
        paddr = DTLBLookup(addr, false, pc, next_pc);
        if (paddr == 0xffffffff) {
          break;
        }
        dpage_rd_uint16_va = addr;
        dpage_rd_uint16_adj_va = (paddr ^ addr) & 0xffffe000;
      }
      paddr = dpage_rd_uint16_adj_va ^ addr;

      if (paddr > (sizeof(char) * RAM_SIZE_BYTES)) {
        rf.r[(ins >> 21) & 0x1F] = ReadDevMem16(paddr);
      } else {
        rf.r[(ins >> 21) & 0x1F] =
          ((uint32_t)ram_uint8[((paddr ^ 2) + 1)] << 8)
          | (uint32_t)ram_uint8[(paddr ^ 2)];
      }
      break;
        
    case 0x26:
      // lhs
      addr = rf.r[(ins >> 16) & 0x1F] + ((ins & 0x00008000)
                                         ? ((ins & 0x0000ffff) | 0xffff0000)
                                         : (ins & 0x0000ffff));

      if ((dpage_rd_int16_va ^ addr) >> 13) {
        paddr = DTLBLookup(addr, false, pc, next_pc);
        if (paddr == 0xffffffff) {
          break;
        }
        dpage_rd_int16_va = addr;
        dpage_rd_int16_adj_va = (paddr ^ addr) & 0xffffe000;
      }
      paddr = dpage_rd_int16_adj_va ^ addr;

      if (paddr > (sizeof(char) * RAM_SIZE_BYTES)) {
        rf.r[(ins >> 21) & 0x1F] = (int32_t)((int16_t)ReadDevMem16(paddr));
      } else {
        rf.r[(ins >> 21) & 0x1F] =
          (int32_t)((int16_t)(((int16_t)ram_uint8[((paddr ^ 2) + 1)] << 8)
                              | (int16_t)ram_uint8[(paddr ^ 2)]));
      }
      break;
      
    case 0x27:
      // addi signed
      imm = ((ins & 0x8000) ? ((ins & 0xffff) | 0xffff0000)
                            : (ins & 0xffff));
      rf.r[(ins >> 21) & 0x1F] = rf.r[(ins >> 16) & 0x1F] + imm;
      break;
        
    case 0x29:
      // andi
      rf.r[(ins >> 21) & 0x1F] = rf.r[(ins >> 16) & 0x1F] & (ins & 0xffff);
      break;
        
    case 0x2A:
      // ori
      rf.r[(ins >> 21) & 0x1F] = rf.r[(ins >> 16) & 0x1F] | (ins & 0xffff);
      break;
            
    case 0x2B:
      // xori
      imm = ((ins & 0x8000) ? ((ins & 0xffff) | 0xffff0000)
                            : (ins & 0xffff));
      rf.r[(ins >> 21) & 0x1F] = rf.r[(ins >> 16) & 0x1F] ^ imm;
      break;
        
    case 0x2D:
      // mfspr
      rf.r[(ins >> 21) & 0x1F] = GetSPR(rf.r[(ins >> 16) & 0x1F] | (ins & 0xffff));
      break;
        
    case 0x2E:
      switch ((ins >> 6) & 0x3) {
      case 0:
        // slli
        rf.r[(ins >> 21) & 0x1F] = (rf.r[(ins >> 16) & 0x1F] << (ins & 0x1F));
        break;
      case 1:
        // srli
        rA = rf.r[(ins >> 16) & 0x1F];
        rf.r[(ins >> 21) & 0x1F] = (rA >> (ins & 0x1F));
        break;
      case 2:
        // srai
        rA = rf.r[(ins >> 16) & 0x1f];
        rf.r[(ins >> 21) & 0x1f] = (rA >> (ins & 0x1f))
          | (((rA & 0x80000000) && (ins & 0x1f))
             ? (0xffffffff << (32 - (ins & 0x1f))) : 0);
        break;
      default:
        DebugMessage("Error: l.rori opcode not implemented");
        break;
      }
      break;
        
    case 0x2F:
      // sf...i
      imm = (ins & 0x8000) ? ((0xffff0000) | (ins & 0x0000ffff))
                           : (ins & 0x0000ffff);
      switch ((ins >> 21) & 0x1F) {
      case 0x0:
        // sfeqi
        SR_F = (rf.r[(ins >> 16) & 0x1F] == imm);
        break;
      case 0x1:
        // sfnei
        SR_F = (rf.r[(ins >> 16) & 0x1F] != imm);
        break;
      case 0x2:
        // sfgtui
        SR_F = (rf.r[(ins >> 16) & 0x1F] > imm);
        break;
      case 0x3:
        // sfgeui
        SR_F = (rf.r[(ins >> 16) & 0x1F] >= imm);
        break;
      case 0x4:
        // sfltui
        SR_F = (rf.r[(ins >> 16) & 0x1F] < imm);
        break;
      case 0x5:
        // sfleui
        SR_F = (rf.r[(ins >> 16) & 0x1F] <= imm);
        break;
      case 0xa:
        // sfgtsi
        SR_F = ((int32_t)rf.r[(ins >> 16) & 0x1F] > (int32_t)imm);
        break;
      case 0xb:
        // sfgesi
        SR_F = ((int32_t)rf.r[(ins >> 16) & 0x1F] >= (int32_t)imm);
        break;
      case 0xc:
        // sfltsi
        SR_F = ((int32_t)rf.r[(ins >> 16) & 0x1F] < (int32_t)imm);
        break;
      case 0xd:
        // sflesi
        SR_F = ((int32_t)rf.r[(ins >> 16) & 0x1F] <= (int32_t)imm);
        break;
      default:
        DebugMessage("Error: sf...i not supported yet");
        break;
      }
      break;
            
    case 0x30:
      // mtspr
      imm = (ins & 0x7FF) | ((ins >> 10) & 0xf800);
      SetSPR(rf.r[(ins >> 16) & 0x1F] | imm, rf.r[(ins >> 11) & 0x1F]);
      break;
        
    case 0x32:
      // floating point
      fprintf(stderr, "DEBUG: floating point op detected\n");
      rA = (ins >> 16) & 0x1F;
      rB = (ins >> 11) & 0x1F;
      rD = (ins >> 21) & 0x1F;
      switch (ins & 0xFF) {
      case 0x0:
        // lf.add.s
        rf.f[rD] = rf.f[rA] + rf.f[rB];
        break;
      case 0x1:
        // lf.sub.s
        rf.f[rD] = rf.f[rA] - rf.f[rB];
        break;
      case 0x2:
        // lf.mul.s
        rf.f[rD] = rf.f[rA] * rf.f[rB];
        break;
      case 0x3:
        // lf.div.s
        rf.f[rD] = rf.f[rA] / rf.f[rB];
        break;
      case 0x4:
        // lf.itof.s
        rf.f[rD] = rf.r[rA];
        break;
      case 0x5:
        // lf.ftoi.s
        rf.r[rD] = rf.f[rA];
        break;
      case 0x7:
        // lf.madd.s
        rf.f[rD] += rf.f[rA] * rf.f[rB];
        break;
      case 0x8:
        // lf.sfeq.s
        SR_F = (rf.f[rA] == rf.f[rB]);
        break;
      case 0x9:
        // lf.sfne.s
        SR_F = (rf.f[rA] != rf.f[rB]);
        break;
      case 0xa:
        // lf.sfgt.s
        SR_F = (rf.f[rA] > rf.f[rB]);
        break;
      case 0xb:
        // lf.sfge.s
        SR_F = (rf.f[rA] >= rf.f[rB]);
        break;
      case 0xc:
        // lf.sflt.s
        SR_F = (rf.f[rA] < rf.f[rB]);
        break;
      case 0xd:
        // lf.sfle.s
        SR_F = (rf.f[rA] <= rf.f[rB]);
        break;
      default:
        DebugMessage("Error: lf. function 0x%02x not supported yet", (ins & 0xff));
        break;
      }
      break;

    case 0x35:
      // sw
      unsigned_imm = (((ins >> 10) & 0xF800) | (ins & 0x000007ff));
      imm = (unsigned_imm & 0x8000) ? (0xffff0000 | unsigned_imm) : unsigned_imm;
      addr = rf.r[(ins >> 16) & 0x1F] + imm;

      if (addr & 0x3) {
        DebugMessage("Error in sw: no unaligned memory access");
      }

      if ((dpage_wr_uint32_va ^ addr) >> 13) {
        paddr = DTLBLookup(addr, true, pc, next_pc);
        if (paddr == 0xffffffff) {
          break;
        }
        dpage_wr_uint32_va = addr;
        dpage_wr_uint32_adj_va = (paddr ^ addr) & 0xffffe000;
      }
      paddr = dpage_wr_uint32_adj_va ^ addr;

      if (paddr > (sizeof(char) * RAM_SIZE_BYTES)) {
        WriteDevMem32(paddr, rf.r[(ins >> 11) & 0x1F]);
      } else {
        ram_uint32[paddr >> 2] = rf.r[(ins >> 11) & 0x1F];
      }
      break;


    case 0x36:
      // sb
      unsigned_imm = (((ins >> 10) & 0xF800) | (ins & 0x000007ff));
      imm = (unsigned_imm & 0x8000) ? (0xffff0000 | unsigned_imm) : unsigned_imm;
      addr = rf.r[(ins >> 16) & 0x1F] + imm;

      if ((dpage_wr_uint8_va ^ addr) >> 13) {
        paddr = DTLBLookup(addr, true, pc, next_pc);
        if (paddr == 0xffffffff) {
          break;
        }
        dpage_wr_uint8_va = addr;
        dpage_wr_uint8_adj_va = (paddr ^ addr) & 0xffffe000;
      }
      paddr = dpage_wr_uint8_adj_va ^ addr;
      
      if (paddr > (sizeof(char) * RAM_SIZE_BYTES)) {
        WriteDevMem8(paddr, rf.r[(ins >> 11) & 0x1F]);
      } else {
        ram_uint8[paddr ^ 3] = rf.r[(ins >> 11) & 0x1F];
      }
      break;

    case 0x37:
      // sh
      unsigned_imm = (((ins >> 10) & 0xF800) | (ins & 0x000007ff));
      imm = (unsigned_imm & 0x8000) ? (0xffff0000 | unsigned_imm) : unsigned_imm;
      addr = rf.r[(ins >> 16) & 0x1F] + imm;

      if ((dpage_wr_uint16_va ^ addr) >> 13) {
        paddr = DTLBLookup(addr, true, pc, next_pc);
        if (paddr == 0xffffffff) {
          break;
        }
        dpage_wr_uint16_va = addr;
        dpage_wr_uint16_adj_va = (paddr ^ addr) & 0xffffe000;
      }
      paddr = dpage_wr_uint16_adj_va ^ addr;

      if (paddr > (sizeof(char) * RAM_SIZE_BYTES)) {
        WriteDevMem16(paddr, rf.r[(ins >> 11) & 0x1F]);
      } else {
        ram_uint8[(paddr ^ 2) + 1] = (rf.r[(ins >> 11) & 0x1F] >> 8);
        ram_uint8[(paddr ^ 2)] = rf.r[(ins >> 11) & 0x1F];
      }
      break;

    case 0x38:
      // three operands commands
      rA = rf.r[(ins >> 16) & 0x1f];
      rB = rf.r[(ins >> 11) & 0x1f];
      rindex = (ins >> 21) & 0x1f;
      switch (ins & 0x30f) {
      case 0x0:
        // add 
        rf.r[rindex] = rA + rB;
        break;
      case 0x2:
        // sub
        rf.r[rindex] = rA - rB;             
        break;
      case 0x3:
        // and
        rf.r[rindex] = rA & rB;
        break;
      case 0x4:
        // or
        rf.r[rindex] = rA | rB;
        break;
      case 0x5:
        // xor
        rf.r[rindex] = rA ^ rB;
        break;
      case 0x8:
        switch ((ins & 0xc0) >> 6) {
        case 0:
          // sll
          rf.r[rindex] = (rA << (rB & 0x1f));
          break;
        case 1:
          // srl not signed
          rf.r[rindex] = (rA >> (rB & 0x1f));
          break;
        case 2:
          // sra signed
          rf.r[rindex] = (rA >> (rB & 0x1f))
            | (((rA & 0x80000000) && (rB & 0x1f))
               ? (0xffffffff << (32 - (rB & 0x1f))) : 0);
          break;
          
        default:
          DebugMessage("Error: op38 opcode 0x%03x not supported yet", (ins & 0x3cf));
          break;
        }
        break;
      case 0xf:
        // ff1
        rf.r[rindex] = 0;
        for (i = 0; i < 32; ++i) {
          if (rA & (1 << i)) {
            rf.r[rindex] = i + 1;
            break;
          }
        }
        break;
      case 0x10f:
        // fl1
        rf.r[rindex] = 0;
        for (i = 31; i >= 0; --i) {
          if (rA & (1 << i)) {
            rf.r[rindex] = i + 1;
            break;
          }
        }
        break;
      case 0x306:
        // mul signed (specification seems to be wrong)
        {
          //int64_t mul64Res = (int64_t)((int32_t)rA) * (int64_t)((int32_t)rB);
          //int64_t umul64Res = (uint64_t)rA * (uint64_t)rB;
          //rf.r[rindex] = (int32_t)mul64Res;

          //SR_OV = (mul64Res > 0x7fffffff) | (mul64Res < -2147483648);
          //SR_CY = umul64Res > 0xffffffff;
          rf.r[rindex] = (int32_t)rA * (int32_t)rB;
          //fprintf(stderr, "%lld: l.mul %d * %d = %d -- 0x%08x * 0x%08x = 0x%08x\n", debug_instcount, (int32_t)rA, (int32_t)rB, (int32_t)rf.r[rindex], (int32_t)rA, (int32_t)rB, (int32_t)rf.r[rindex]);
        }
        break;
      case 0x30a:
        // divu (specification seems to be wrong)
        if (rB != 0) {
          rf.r[rindex] = rA / rB;
          //fprintf(stderr, "%lld: l.divu %u / %u = %u -- 0x%08x / 0x%08x = 0x%08x\n", debug_instcount, rA, rB, rf.r[rindex], rA, rB, rf.r[rindex]);
        }
        break;
      case 0x309:
        // div (specification seems to be wrong)
        if (rB != 0) {
          rf.r[rindex] = (int32_t)rA / (int32_t)rB;
          //fprintf(stderr, "%lld: l.div %d / %d = %d -- 0x%08x / 0x%08x = 0x%08x\n", debug_instcount, (int32_t)rA, (int32_t)rB, (int32_t)rf.r[rindex], (int32_t)rA, (int32_t)rB, (int32_t)rf.r[rindex]);
        }
        break;
      default:
        DebugMessage("Error: op38 opcode 0x%03x not supported yet", (ins & 0x3cf));
        break;
      }
      break;

    case 0x39:
      // sf....
      switch ((ins >> 21) & 0x1F) {
      case 0x0:
        // sfeq
        SR_F = (rf.r[(ins >> 16) & 0x1F] == rf.r[(ins >> 11) & 0x1F]);
        break;
      case 0x1:
        // sfne
        SR_F = (rf.r[(ins >> 16) & 0x1F] != rf.r[(ins >> 11) & 0x1F]);
        break;
      case 0x2:
        // sfgtu
        SR_F = (rf.r[(ins >> 16) & 0x1F] > rf.r[(ins >> 11) & 0x1F]);
        break;
      case 0x3:
        // sfgeu
        SR_F = (rf.r[(ins >> 16) & 0x1F] >= rf.r[(ins >> 11) & 0x1F]);
        break;
      case 0x4:
        // sfltu
        SR_F = (rf.r[(ins >> 16) & 0x1F] < rf.r[(ins >> 11) & 0x1F]);
        break;
      case 0x5:
        // sfleu
        SR_F = (rf.r[(ins >> 16) & 0x1F] <= rf.r[(ins >> 11) & 0x1F]);
        break;
      case 0xa:
        // sfgts
        SR_F = ((int32_t)rf.r[(ins >> 16) & 0x1F] > (int32_t)rf.r[(ins >> 11) & 0x1F]);
        break;
      case 0xb:
        // sfges
        SR_F = ((int32_t)rf.r[(ins >> 16) & 0x1F] >= (int32_t)rf.r[(ins >> 11) & 0x1F]);
        break;
      case 0xc:
        // sflts
        SR_F = ((int32_t)rf.r[(ins >> 16) & 0x1F] < (int32_t)rf.r[(ins >> 11) & 0x1F]);
        break;
      case 0xd:
        // sfles
        SR_F = ((int32_t)rf.r[(ins >> 16) & 0x1F] <= (int32_t)rf.r[(ins >> 11) & 0x1F]);
        break;
      default:
        DebugMessage("Error: sf.... function supported yet");
        break;
      }
      break;

    default:
      DebugMessage("ERROR (CPU): undecodable instruction 0x%08x, header 0x%02x", ins, ((ins >> 26) & 0x3f));
    }
      
    pc = next_pc++;
    delayed_ins = false;
  } while (true);
}

// Disassembles the passed instruction code in the current emulator context.
// Should be called _before_ the instruction is executed
void CPU::DisassembleInstr(uint32_t ins, uint32_t pc, uint32_t next_pc) {
  char* decode_str;
  uint32_t rindex;
  uint32_t unsigned_imm;
  uint32_t imm;
  uint32_t addr;
  uint32_t paddr;
  uint32_t rA;
  uint32_t rB;
  uint32_t result;
  uint32_t idx;
  int8_t i;

  decode_str = (char*)malloc(sizeof(char) * 100);
    
  switch ((ins >> 26) & 0x3f) {
  case 0x0:
    addr = pc + ((ins & 0x02000000)
                 ? ((ins & 0x03FFFFFF) | 0xfc000000)
                 : (ins & 0x03FFFFFF));
    snprintf(decode_str, 100, "l.j 0x%08x", (addr << 2));
    break;
        
  case 0x1:
    addr = pc + ((ins & 0x02000000)
                 ? ((ins & 0x03FFFFFF) | 0xfc000000)
                 : (ins & 0x03FFFFFF));
    snprintf(decode_str, 100, "l.jal 0x%08x (r9=0x%08x)", (addr << 2), (next_pc << 2) + 4);
    break;
      
  case 0x3:
    addr = pc + ((ins & 0x02000000)
                 ? ((ins & 0x03FFFFFF) | 0xfc000000)
                 : (ins & 0x03FFFFFF));
    if (SR_F) {
      snprintf(decode_str, 100, "(ccfail) l.bnf 0x%08x", (addr << 2));
    } else {
      snprintf(decode_str, 100, "l.bnf 0x%08x", (addr << 2));
    }
    break;
      
  case 0x4:
    addr = pc + ((ins & 0x02000000)
                 ? ((ins & 0x03FFFFFF) | 0xfc000000)
                 : (ins & 0x03FFFFFF));
    if (!SR_F) {
      snprintf(decode_str, 100, "(ccfail) l.bf 0x%08x", (addr << 2));
    } else {
      snprintf(decode_str, 100, "l.bf 0x%08x", (addr << 2));
    }
    break;
      
  case 0x5:
    snprintf(decode_str, 100, "l.nop");
    break;
      
  case 0x6:
    rindex = (ins >> 21) & 0x1F;
    if (ins & 0x10000) {
      snprintf(decode_str, 100, "l.macrc?");
    } else {
      snprintf(decode_str, 100, "l.movhi r%d,0x%04x (r%d=0x%08x)", rindex, ins & 0xFFFF, rindex, ((ins & 0xFFFF) << 16));
    }
    break;
      
  case 0x8:
    snprintf(decode_str, 100, "l.sys 0x%04x", ins & 0xffff);
    break;
      
  case 0x9:
    snprintf(decode_str, 100, "l.rfe");
    break;
        
  case 0x11:
    addr = (rf.r[(ins >> 11) & 0x1F] >> 2);
    snprintf(decode_str, 100, "l.jr r%d (0x%08x)", (ins >> 11) & 0x1F, (addr << 2));
    break;

  case 0x12:
    addr = (rf.r[(ins >> 11) & 0x1F] >> 2);
    snprintf(decode_str, 100, "l.jalr r%d (0x%08x) (r9=0x%08x)", (ins >> 11) & 0x1F, (addr << 2), (next_pc << 2) + 4);
    break;
        
  case 0x21:
    addr = rf.r[(ins >> 16) & 0x1F] + ((ins & 0x00008000)
                                       ? ((ins & 0x0000ffff) | 0xffff0000)
                                       : (ins & 0x0000ffff));
    paddr = DTLBLookupNoExceptions(addr, false);
    if (paddr > (sizeof(char) * RAM_SIZE_BYTES)) {
      snprintf(decode_str, 100, "l.lwz r%d,I(r%d) (VA=0x%08x, PA=0x%08x, INVALID)", (ins >> 21) & 0x1F, (ins >> 16) & 0x1F, addr, paddr);
    } else {
      snprintf(decode_str, 100, "l.lwz r%d,I(r%d) (r%d=0x%08x, VA=0x%08x, PA=0x%08x)", (ins >> 21) & 0x1F, (ins >> 16) & 0x1F, (ins >> 21) & 0x1F, ram_uint32[paddr >> 2], addr, paddr);
    }
    break;

  case 0x22:
    addr = rf.r[(ins >> 16) & 0x1F] + ((ins & 0x00008000)
                                       ? ((ins & 0x0000ffff) | 0xffff0000)
                                       : (ins & 0x0000ffff));
    paddr = DTLBLookupNoExceptions(addr, false);
    if (paddr > (sizeof(char) * RAM_SIZE_BYTES)) {
      snprintf(decode_str, 100, "l.lws r%d,I(r%d) (VA=0x%08x, PA=0x%08x, INVALID)", (ins >> 21) & 0x1F, (ins >> 16) & 0x1F, addr, paddr);
    } else {
      snprintf(decode_str, 100, "l.lws r%d,I(r%d) (r%d=0x%08x, VA=0x%08x, PA=0x%08x)", (ins >> 21) & 0x1F, (ins >> 16) & 0x1F, (ins >> 21) & 0x1F, ram_uint32[paddr >> 2], addr, paddr);
    }
    break;

  case 0x23:
    addr = rf.r[(ins >> 16) & 0x1F] + ((ins & 0x00008000)
                                       ? ((ins & 0x0000ffff) | 0xffff0000)
                                       : (ins & 0x0000ffff));
    paddr = DTLBLookupNoExceptions(addr, false);
    if (paddr > (sizeof(char) * RAM_SIZE_BYTES)) {
      snprintf(decode_str, 100, "l.lbz r%d,I(r%d) (VA=0x%08x, PA=0x%08x, INVALID)", (ins >> 21) & 0x1F, (ins >> 16) & 0x1F, addr, paddr);
          
    } else {
      snprintf(decode_str, 100, "l.lbz r%d,I(r%d) (r%d=0x%08x, VA=0x%08x, PA=0x%08x)", (ins >> 21) & 0x1F, (ins >> 16) & 0x1F, (ins >> 21) & 0x1F, ram_uint8[paddr ^ 3], addr, paddr);
    }
    break;
        
  case 0x24:
    addr = rf.r[(ins >> 16) & 0x1F] + ((ins & 0x00008000)
                                       ? ((ins & 0x0000ffff) | 0xffff0000)
                                       : (ins & 0x0000ffff));
    paddr = DTLBLookupNoExceptions(addr, false);
    if (paddr > (sizeof(char) * RAM_SIZE_BYTES)) {
      snprintf(decode_str, 100, "l.lbs r%d,I(r%d) (VA=0x%08x, PA=0x%08x, INVALID)", (ins >> 21) & 0x1F, (ins >> 16) & 0x1F, addr, paddr);
          
    } else {
      snprintf(decode_str, 100, "l.lbs r%d,I(r%d) (r%d=0x%08x, VA=0x%08x, PA=0x%08x)", (ins >> 21) & 0x1F, (ins >> 16) & 0x1F, (ins >> 21) & 0x1F, (int32_t)((int8_t)ram_uint8[paddr ^ 3]), addr, paddr);
    }
    break;
        
  case 0x25:
    addr = rf.r[(ins >> 16) & 0x1F] + ((ins & 0x00008000)
                                       ? ((ins & 0x0000ffff) | 0xffff0000)
                                       : (ins & 0x0000ffff));
    paddr = DTLBLookupNoExceptions(addr, false);
    if (paddr > (sizeof(char) * RAM_SIZE_BYTES)) {
      snprintf(decode_str, 100, "l.lhz r%d,I(r%d) (VA=0x%08x, PA=0x%08x, INVALID)", (ins >> 21) & 0x1F, (ins >> 16) & 0x1F, addr, paddr);
          
    } else {
      snprintf(decode_str, 100, "l.lhz r%d,I(r%d) (r%d=0x%08x, VA=0x%08x, PA=0x%08x)", (ins >> 21) & 0x1F, (ins >> 16) & 0x1F, (ins >> 21) & 0x1F, (ram_uint8[((paddr ^ 2) + 1)] << 8) | ram_uint8[(paddr ^ 2)], addr, paddr);
    }
    break;
        
  case 0x26:
    addr = rf.r[(ins >> 16) & 0x1F] + ((ins & 0x00008000)
                                       ? ((ins & 0x0000ffff) | 0xffff0000)
                                       : (ins & 0x0000ffff));
    paddr = DTLBLookupNoExceptions(addr, false);
    if (paddr > (sizeof(char) * RAM_SIZE_BYTES)) {
      snprintf(decode_str, 100, "l.lhs r%d,I(r%d) (VA=0x%08x, PA=0x%08x, INVALID)", (ins >> 21) & 0x1F, (ins >> 16) & 0x1F, addr, paddr);
          
    } else {
      snprintf(decode_str, 100, "l.lhs r%d,I(r%d) (r%d=0x%08x, VA=0x%08x, PA=0x%08x)", (ins >> 21) & 0x1F, (ins >> 16) & 0x1F, (ins >> 21) & 0x1F, (int32_t)((int16_t)((ram_uint8[((paddr ^ 2) + 1)] << 8) | ram_uint8[(paddr ^ 2)])), addr, paddr);
    }
    break;
      
  case 0x27:
    rindex = (ins >> 21) & 0x1F;
    imm = ((ins & 0x8000) ? ((ins & 0xffff) | 0xffff0000) : (ins & 0xffff));
    snprintf(decode_str, 100, "l.addi r%d,r%d,0x%04x (dest r%d=0x%08x)", (ins >> 21) & 0x1F, (ins >> 16) & 0x1F, (int32_t)imm, (ins >> 21) & 0x1F, rf.r[(ins >> 16) & 0x1F] + imm);
    break;
        
  case 0x29:
    snprintf(decode_str, 100, "l.andi r%d,r%d,0x%04x (dest r%d=0x%08x)", (ins >> 21) & 0x1F, (ins >> 16) & 0x1F, (ins & 0xffff), (ins >> 21) & 0x1F, rf.r[(ins >> 16) & 0x1F] & (ins & 0xffff));
    break;
        
  case 0x2A:
    snprintf(decode_str, 100, "l.ori r%d,r%d,0x%04x (dest r%d=0x%08x)", (ins >> 21) & 0x1F, (ins >> 16) & 0x1F, (ins & 0xffff), (ins >> 21) & 0x1F, rf.r[(ins >> 16) & 0x1F] | (ins & 0xffff));
    break;
            
  case 0x2B:
    imm = ((ins & 0x8000) ? ((ins & 0xffff) | 0xffff0000) : (ins & 0xffff));
    snprintf(decode_str, 100, "l.xori r%d,r%d,0x%04x (dest r%d=0x%08x)", (ins >> 21) & 0x1F, (ins >> 16) & 0x1F, imm, (ins >> 21) & 0x1F, rf.r[(ins >> 16) & 0x1F] ^ imm);
    break;
        
  case 0x2D:
    idx = rf.r[(ins >> 16) & 0x1F] | (ins & 0xffff);
    snprintf(decode_str, 100, "l.mfspr r%d,r%d,0x%04x (SPR group %d, index 0x%03x, SPR value = 0x%08x)", (ins >> 21) & 0x1F, (ins >> 16) & 0x1F, (ins & 0xffff), (idx >> 11) & 0x1F, idx & 0x7FF, GetSPR(rf.r[(ins >> 16) & 0x1F] | (ins & 0xffff)));
    break;
        
  case 0x2E:
    switch ((ins >> 6) & 0x3) {
    case 0:
      snprintf(decode_str, 100, "l.slli r%d,r%d,0x%02x (dest r%d=0x%08x)", (ins >> 21) & 0x1F, (ins >> 16) & 0x1F, (ins & 0x1f), (ins >> 21) & 0x1F, rf.r[(ins >> 16) & 0x1F] << (ins & 0x1F));
      break;
    case 1:
      rA = rf.r[(ins >> 16) & 0x1F];
      snprintf(decode_str, 100, "l.srli r%d,r%d,%d (dest r%d=0x%08x)", (ins >> 21) & 0x1F, (ins >> 16) & 0x1F, (ins & 0x1f), (ins >> 21) & 0x1F, (rA >> (ins & 0x1F)));
      break;
    case 2:
      rA = rf.r[(ins >> 16) & 0x1f];
      snprintf(decode_str, 100, "l.srai r%d,r%d,%d (dest r%d=0x%08x)", (ins >> 21) & 0x1F, (ins >> 16) & 0x1F, (ins & 0x1f), (ins >> 21) & 0x1F, (rA >> (ins & 0x1f)) | ((rA & 0x80000000) ? (0xffffffff << (32 - (ins & 0x1f))) : 0));
      break;
    default:
      snprintf(decode_str, 100, "UNKNOWN");
      return;
    }
    break;
        
  case 0x2F:
    // sf...i
    imm = ins & 0xffff;
    switch ((ins >> 21) & 0x1F) {
    case 0x0:
      snprintf(decode_str, 100, "l.sfeqi r%d,0x%04x (SR[F]=%d)", (ins >> 16) & 0x1F, imm, (rf.r[(ins >> 16) & 0x1F] == imm));
      break;
    case 0x1:
      snprintf(decode_str, 100, "l.sfnei r%d,0x%04x (SR[F]=%d)", (ins >> 16) & 0x1F, imm, (rf.r[(ins >> 16) & 0x1F] != imm));
      break;
    case 0x2:
      snprintf(decode_str, 100, "l.sfgtui r%d,0x%04x (SR[F]=%d)", (ins >> 16) & 0x1F, imm, (rf.r[(ins >> 16) & 0x1F] > imm));
      break;
    case 0x3:
      snprintf(decode_str, 100, "l.sfgeui r%d,0x%04x (SR[F]=%d)", (ins >> 16) & 0x1F, imm, (rf.r[(ins >> 16) & 0x1F] >= imm));
      break;
    case 0x4:
      snprintf(decode_str, 100, "l.sfltui r%d,0x%04x (SR[F]=%d)", (ins >> 16) & 0x1F, imm, (rf.r[(ins >> 16) & 0x1F] < imm));
      break;
    case 0x5:
      snprintf(decode_str, 100, "l.sfleui r%d,0x%04x (SR[F]=%d)", (ins >> 16) & 0x1F, imm, (rf.r[(ins >> 16) & 0x1F] <= imm));
      break;
    case 0xa:
      snprintf(decode_str, 100, "l.sfgtsi r%d,0x%04x (SR[F]=%d)", (ins >> 16) & 0x1F, imm, ((int32_t)rf.r[(ins >> 16) & 0x1F] > (int32_t)imm));
      break;
    case 0xb:
      snprintf(decode_str, 100, "l.sfgesi r%d,0x%04x (SR[F]=%d)", (ins >> 16) & 0x1F, imm, ((int32_t)rf.r[(ins >> 16) & 0x1F] >= (int32_t)imm));
      break;
    case 0xc:
      snprintf(decode_str, 100, "l.sfltsi r%d,0x%04x (SR[F]=%d)", (ins >> 16) & 0x1F, imm, ((int32_t)rf.r[(ins >> 16) & 0x1F] < (int32_t)imm));
      break;
    case 0xd:
      snprintf(decode_str, 100, "l.sflesi r%d,0x%04x (SR[F]=%d)", (ins >> 16) & 0x1F, imm, ((int32_t)rf.r[(ins >> 16) & 0x1F] <= (int32_t)imm));
      break;
    default:
      snprintf(decode_str, 100, "UNKNOWN");
      return;
    }
    break;
            
  case 0x30:
    imm = (ins & 0x7FF) | ((ins >> 10) & 0xf800);
    idx = rf.r[(ins >> 16) & 0x1F] | (ins & 0xffff);
    snprintf(decode_str, 100, "l.mtspr r%d,r%d,0x%04x (SPR group %d, index 0x%03x, r%d = 0x%08x)", (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, imm, (idx >> 11) & 0x1F, idx & 0x7FF, (ins >> 11) & 0x1F, rf.r[(ins >> 11) & 0x1F]);
    break;
        
    /*case 0x32:
    // floating point
    rA = (ins >> 16) & 0x1F;
    rB = (ins >> 11) & 0x1F;
    rD = (ins >> 21) & 0x1F;
    switch (ins & 0xFF) {
    case 0x0:
    // lf.add.s
    rf.f[rD] = rf.f[rA] + rf.f[rB];
    break;
    case 0x1:
    // lf.sub.s
    rf.f[rD] = rf.f[rA] - rf.f[rB];
    break;
    case 0x2:
    // lf.mul.s
    rf.f[rD] = rf.f[rA] * rf.f[rB];
    break;
    case 0x3:
    // lf.div.s
    rf.f[rD] = rf.f[rA] / rf.f[rB];
    break;
    case 0x4:
    // lf.itof.s
    rf.f[rD] = rf.r[rA];
    break;
    case 0x5:
    // lf.ftoi.s
    rf.r[rD] = rf.f[rA];
    break;
    case 0x7:
    // lf.madd.s
    rf.f[rD] += rf.f[rA] * rf.f[rB];
    break;
    case 0x8:
    // lf.sfeq.s
    SR_F = (rf.f[rA] == rf.f[rB]);
    break;
    case 0x9:
    // lf.sfne.s
    SR_F = (rf.f[rA] != rf.f[rB]);
    break;
    case 0xa:
    // lf.sfgt.s
    SR_F = (rf.f[rA] > rf.f[rB]);
    break;
    case 0xb:
    // lf.sfge.s
    SR_F = (rf.f[rA] >= rf.f[rB]);
    break;
    case 0xc:
    // lf.sflt.s
    SR_F = (rf.f[rA] < rf.f[rB]);
    break;
    case 0xd:
    // lf.sfle.s
    SR_F = (rf.f[rA] <= rf.f[rB]);
    break;
    default:
    snprintf(decode_str, 100, "UNKNOWN");
    return;
    }
    break;*/

  case 0x35:
    unsigned_imm = (((ins >> 10) & 0xF800) | (ins & 0x000007ff));
    imm = (unsigned_imm & 0x8000) ? (0xffff0000 | unsigned_imm) : unsigned_imm;
    addr = rf.r[(ins >> 16) & 0x1F] + imm;

    paddr = DTLBLookupNoExceptions(addr, false);
    if (paddr > (sizeof(char) * RAM_SIZE_BYTES)) {
      snprintf(decode_str, 100, "l.sw I(r%d),r%d (r%d=0x%08x, VA=0x%08x, PA=0x%08x, INVALID)", (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, (ins >> 16) & 0x1F, rf.r[(ins >> 16) & 0x1F], addr, paddr);
    } else {
      snprintf(decode_str, 100, "l.sw I(r%d),r%d (r%d=0x%08x, r%d=0x%08x, VA=0x%08x, PA=0x%08x)", (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, (ins >> 16) & 0x1F, rf.r[(ins >> 16) & 0x1F], (ins >> 11) & 0x1F, rf.r[(ins >> 11) & 0x1F], addr, paddr);
    }
    break;


  case 0x36:
    unsigned_imm = (((ins >> 10) & 0xF800) | (ins & 0x000007ff));
    imm = (unsigned_imm & 0x8000) ? (0xffff0000 | unsigned_imm) : unsigned_imm;
    addr = rf.r[(ins >> 16) & 0x1F] + imm;

    paddr = DTLBLookupNoExceptions(addr, false);
    if (paddr > (sizeof(char) * RAM_SIZE_BYTES)) {
      snprintf(decode_str, 100, "l.sb I(r%d),r%d (VA=0x%08x, PA=0x%08x, INVALID)", (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, addr, paddr);
    } else {
      snprintf(decode_str, 100, "l.sb I(r%d),r%d (r%d=0x%08x, VA=0x%08x, PA=0x%08x)", (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, (ins >> 11) & 0x1F, rf.r[(ins >> 11) & 0x1F] & 0xff, addr, paddr);
    }
    break;

  case 0x37:
    unsigned_imm = (((ins >> 10) & 0xF800) | (ins & 0x000007ff));
    imm = (unsigned_imm & 0x8000) ? (0xffff0000 | unsigned_imm) : unsigned_imm;
    addr = rf.r[(ins >> 16) & 0x1F] + imm;

    paddr = DTLBLookupNoExceptions(addr, false);
    if (paddr > (sizeof(char) * RAM_SIZE_BYTES)) {
      snprintf(decode_str, 100, "l.sh I(r%d),r%d (VA=0x%08x, PA=0x%08x, INVALID)", (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, addr, paddr);
    } else {
      snprintf(decode_str, 100, "l.sh I(r%d),r%d (r%d=0x%08x, VA=0x%08x, PA=0x%08x)", (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, (ins >> 11) & 0x1F, rf.r[(ins >> 11) & 0x1F] * 0xffff, addr, paddr);
    }
    break;

  case 0x38:
    // three operands commands
    rA = rf.r[(ins >> 16) & 0x1F];
    rB = rf.r[(ins >> 11) & 0x1F];
    rindex = (ins >> 21) & 0x1F;
    switch (ins & 0x3CF) {
    case 0x0:
      snprintf(decode_str, 100, "l.add r%d,r%d,r%d (dest r%d=0x%08x)", rindex, (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, rindex, (int32_t)rA + (int32_t)rB);
      break;
    case 0x2:
      snprintf(decode_str, 100, "l.sub r%d,r%d,r%d (dest r%d=0x%08x)", rindex, (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, rindex, (int32_t)rA - (int32_t)rB);
      break;
    case 0x3:
      snprintf(decode_str, 100, "l.and r%d,r%d,r%d (dest r%d=0x%08x)", rindex, (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, rindex, rA & rB);
      break;
    case 0x4:
      snprintf(decode_str, 100, "l.or r%d,r%d,r%d (dest r%d=0x%08x)", rindex, (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, rindex, rA | rB);
      break;
    case 0x5:
      snprintf(decode_str, 100, "l.xor r%d,r%d,r%d (dest r%d=0x%08x)", rindex, (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, rindex, rA ^ rB);
      break;
    case 0x8:
      switch ((ins & 0xc0) >> 6) {
      case 0:
        // sll
      snprintf(decode_str, 100, "l.sll r%d,r%d,r%d (dest r%d=0x%08x)", rindex, (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, rindex, rA << (rB & 0x1F));
        break;
      case 1:
        // srl not signed
        snprintf(decode_str, 100, "l.srl r%d,r%d,r%d (dest r%d=0x%08x)", rindex, (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, rindex, rA >> (rB & 0x1F));
        break;
      case 2:
        // sra signed
        snprintf(decode_str, 100, "l.sra r%d,r%d,r%d (dest r%d=0x%08x)", rindex, (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, rindex, rA >> (rB & 0x1F) | ((rA & 0x80000000) ? (0xffffffff << (32 - (rB & 0x1F))) : 0));
        break;
        
      default:
        DebugMessage("Error: op38 opcode 0x%03x not supported yet", (ins & 0x3cf));
        break;
      }
      break;
    case 0xf:
      result = 0;
      for (i = 0; i < 32; i++) {
        if (rA & (1 << i)) {
          result = i + 1;
          break;
        }
      }
      snprintf(decode_str, 100, "l.ff1 r%d,r%d,r%d (dest r%d=0x%08x)", rindex, (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, rindex, result);
      break;
    case 0x10f:
      result = 0;
      for (i = 31; i >= 0; i--) {
        if (rA & (1 << i)) {
          result = i + 1;
          break;
        }
      }
      snprintf(decode_str, 100, "l.fl1 r%d,r%d,r%d (dest r%d=0x%08x)", rindex, (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, rindex, result);
      break;
    case 0x306:
      {
        //int64_t mul64Res = (int64_t)((int32_t)rA) * (int64_t)((int32_t)rB);
        //int64_t umul64Res = (uint64_t)rA * (uint64_t)rB;

        //snprintf(decode_str, 100, "l.mul r%d,r%d,r%d (dest r%d=0x%08x, SR[OV]=%d, SR[CY]=%d)", rindex, (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, rindex, (int32_t)mul64Res, (mul64Res > 0x7fffffff) | (mul64Res < -2147483648), umul64Res > 0xffffffff);
        //fprintf(stderr, "%d x %d = %lld -> %d\n", (int32_t)rA, (int32_t)rB, mul64Res, (int32_t)mul64Res);
        snprintf(decode_str, 100, "l.mul r%d,r%d,r%d (dest r%d=0x%08x", rindex, (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, rindex, (int32_t)rA * (int32_t)rB);
        //fprintf(stderr, "%d x %d = %d\n", (int32_t)rA, (int32_t)rB, (int32_t)rA * (int32_t)rB);
      }
      break;
    case 0x30a:
      snprintf(decode_str, 100, "l.divu r%d,r%d,r%d (dest r%d=0x%08x, SR[OV]=%d, SR[CY]=%d)", rindex, (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, rindex, (rB == 0) ? rf.r[rindex] : (rA / rB), false, rB == 0);
      break;
    case 0x309:
      snprintf(decode_str, 100, "l.div r%d,r%d,r%d (dest r%d=0x%08x, SR[OV]=%d, SR[CY]=%d)", rindex, (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, rindex, (rB == 0) ? rf.r[rindex] : ((int32_t)rA / (int32_t)rB), false, rB == 0);
      break;
    default:
      snprintf(decode_str, 100, "UNKNOWN");
      return;
    }
    break;

  case 0x39:
    // sf....
    switch ((ins >> 21) & 0x1F) {
    case 0x0:
      snprintf(decode_str, 100, "l.sfeq r%d,r%d (SR[F]=%d)", (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, (rf.r[(ins >> 16) & 0x1F] == rf.r[(ins >> 11) & 0x1F]));
      break;
    case 0x1:
      snprintf(decode_str, 100, "l.sfne r%d,r%d (SR[F]=%d)", (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, (rf.r[(ins >> 16) & 0x1F] != rf.r[(ins >> 11) & 0x1F]));
      break;
    case 0x2:
      snprintf(decode_str, 100, "l.sfgtu r%d,r%d (SR[F]=%d)", (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, (rf.r[(ins >> 16) & 0x1F] > rf.r[(ins >> 11) & 0x1F]));
      break;
    case 0x3:
      snprintf(decode_str, 100, "l.sfgeu r%d,r%d (SR[F]=%d)", (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, (rf.r[(ins >> 16) & 0x1F] >= rf.r[(ins >> 11) & 0x1F]));
      break;
    case 0x4:
      snprintf(decode_str, 100, "l.sfltu r%d,r%d (SR[F]=%d)", (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, (rf.r[(ins >> 16) & 0x1F] < rf.r[(ins >> 11) & 0x1F]));
      break;
    case 0x5:
      snprintf(decode_str, 100, "l.sfleu r%d,r%d (SR[F]=%d)", (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, (rf.r[(ins >> 16) & 0x1F] <= rf.r[(ins >> 11) & 0x1F]));
      break;
    case 0xa:
      snprintf(decode_str, 100, "l.sfgts r%d,r%d (SR[F]=%d)", (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, ((int32_t)rf.r[(ins >> 16) & 0x1F] > (int32_t)rf.r[(ins >> 11) & 0x1F]));
      break;
    case 0xb:
      snprintf(decode_str, 100, "l.sfges r%d,r%d (SR[F]=%d)", (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, ((int32_t)rf.r[(ins >> 16) & 0x1F] >= (int32_t)rf.r[(ins >> 11) & 0x1F]) );
      break;
    case 0xc:
      snprintf(decode_str, 100, "l.sflts r%d,r%d (SR[F]=%d)", (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, ((int32_t)rf.r[(ins >> 16) & 0x1F] < (int32_t)rf.r[(ins >> 11) & 0x1F]));
      break;
    case 0xd:
      snprintf(decode_str, 100, "l.sfles r%d,r%d (SR[F]=%d)", (ins >> 16) & 0x1F, (ins >> 11) & 0x1F, ((int32_t)rf.r[(ins >> 16) & 0x1F] <= (int32_t)rf.r[(ins >> 11) & 0x1F]));
      break;
    default:
      snprintf(decode_str, 100, "UNKNOWN");
      return;
    }
    break;

  default:
    snprintf(decode_str, 100, "UNKNOWN");
    break;
  }

  fprintf(stderr, "CPU: decoding instcount=%lld, pc=0x%08x, inst=0x%08x, header=0x%02x  %s\n", debug_instcount, (pc << 2), ins, ((ins >> 26) & 0x3f), decode_str);
  free(decode_str);
}

void CPU::OnExtMessage(uint32_t msg) {
  const uint8_t target = (msg >> 24);
  msg &= 0x00ffffff;

  switch (target) {
  case 0x01:
    ((UARTDevice*)uart_device)->ReceiveChar(msg);
    break;

  case 0x02:
    // OnKeyPress() -- currently not handled
    break;

  case 0x03:
    ((KeyboardDevice*)kb_device)->OnKeyDown(msg);
    break;

  case 0x04:
    ((KeyboardDevice*)kb_device)->OnKeyUp(msg);
    break;

  case 0x05:
    // Send the current cycle count (in MIPS form) to the messenger and then
    // reset the count
    PostMessage(pp::Var(0x01000000 | ((int32_t)(cycle_count / 1000000) & 0x00ffffff)));
    //PostMessage(pp::Var(0x01000000 | ((int32_t)(cycle_count) & 0x00ffffff)));
    cycle_count = 0;
    break;

  default:
    break;
  }
}

void CPU::OnViewChange(const pp::View& view) {
  if (pp_graphics.is_null()) {
    pp_graphics = pp::Graphics2D(pp_instance, view.GetRect().size(), true);
    
    if (!pp_instance->BindGraphics(pp_graphics)) {
      DebugMessage("ERROR (CPU): Unable to bind 2D graphics context to PP::Instance");
      pp_graphics = pp::Graphics2D();
    }
    ((FrameBufferDevice*)fb_device)->SetGraphics2DContext(pp_graphics);
  }
  
  ((FrameBufferDevice*)fb_device)->OnViewScaleChange(1.0 / view.GetDeviceScale());
}

void CPU::SendStrToTerminal(const char* format, ...) {
    char* string;
    va_list args;
    
    va_start(args, format);
    string = util::VprintfToNewString(format, args);
    va_end(args);
    
    for (int32_t i = 0; i < strlen(string); ++i) {
      PostMessage(pp::Var((int32_t)string[i]));
    }
    PostMessage(pp::Var((int32_t)'\r'));
    PostMessage(pp::Var((int32_t)'\n'));

    free(string);
}

void CPU::PostMessage(pp::Var var) {
  pp_instance->PostMessage(var);
}

void CPU::DebugMessage(const char* format, ...) {
  if (debug_message_count < DEBUG_MESSAGE_LIMIT) {
    char* string;
    va_list args;
    pp::Var var;
    
    va_start(args, format);
    string = util::VprintfToNewString(format, args);
    va_end(args);
    
    var = pp::Var(string);
    free(string);
    
    PostMessage(var);

    ++debug_message_count;

    if (debug_message_count >= DEBUG_MESSAGE_LIMIT) {
      PostMessage(pp::Var("INFO (CPU): Maximum number of debug messages reached"));
    }
  }
}
