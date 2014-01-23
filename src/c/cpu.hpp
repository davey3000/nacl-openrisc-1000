/*
 * Header file for the class implementing the emulated OpenRISC 1000 CPU
 */

#ifndef OR_EMULATOR_CPU_HPP_
#define OR_EMULATOR_CPU_HPP_

#include <mutex>

#include "ppapi/cpp/graphics_2d.h"
#include "ppapi/cpp/instance.h"
#include "ppapi/cpp/var.h"

#include "device.hpp"

//#define DEBUG_INST_LIMIT 1000000000LL

// The emulated OpenRisc CPU
class CPU {

private:
  // Emulated register file with each register accessible as either a 32-bit
  // integer or a single-precision float
  typedef union {
    uint32_t r[32];
    float f[32];
  } regfile_t;
  
public:
  // Configuration values
  static const uint32_t RAM_SIZE_BYTES = 128*1024*1024;
  static const uint32_t MAX_HDD_IMAGE_SIZE_BYTES = 200*1024*1024;
  static const uint32_t MAX_DEVICES = 16;

  static const uint32_t DEBUG_MESSAGE_LIMIT = 100;
  
private:
  // Special purpose register indices
  static const uint32_t SPR_UPR = 1; // unit present register
  static const uint32_t SPR_SR = 17; // supervision register
  static const uint32_t SPR_EEAR_BASE = 48; // exception ea register
  static const uint32_t SPR_EPCR_BASE = 32; // exception pc register
  static const uint32_t SPR_ESR_BASE = 64; // exception sr register
  static const uint32_t SPR_IMMUCFGR = 4; // Instruction MMU Configuration register
  static const uint32_t SPR_DMMUCFGR = 3; // Data MMU Configuration register
  static const uint32_t SPR_ICCFGR = 6; // Instruction Cache configuration register
  static const uint32_t SPR_DCCFGR = 5; // Data Cache Configuration register
  static const uint32_t SPR_VR = 0; // Version register
  
  // Exception types and addresses
  static const uint32_t EXCEPT_ITLBMISS = 0xA00; // instruction translation lookaside buffer miss
  static const uint32_t EXCEPT_IPF = 0x400; // instruction page fault
  static const uint32_t EXCEPT_RESET = 0x100; // reset the processor
  static const uint32_t EXCEPT_DTLBMISS = 0x900; // data translation lookaside buffer miss
  static const uint32_t EXCEPT_DPF = 0x300; // instruction page fault
  static const uint32_t EXCEPT_BUSERR = 0x200; // wrong memory access
  static const uint32_t EXCEPT_TICK = 0x500; // tick counter interrupt
  static const uint32_t EXCEPT_INT = 0x800; // interrupt of external devices
  static const uint32_t EXCEPT_SYSCALL = 0xc00; // syscall, jump into supervisor mode
  
  // PP instance that created the CPU along with other resources related to the
  // PP instance
  pp::Instance* pp_instance;
  pp::Graphics2D pp_graphics;

  // Emulated memory resources
  regfile_t rf;
  char* ram;
  char* spr_generic;
  char* spr_dtlb;
  char* spr_itlb;

  uint8_t* ram_uint8;
  uint16_t* ram_uint16;
  uint32_t* ram_uint32;

  uint32_t* spr_generic_uint32;
  uint32_t* spr_dtlb_uint32;
  uint32_t* spr_itlb_uint32;

  // Emulated devices
  Device* device[MAX_DEVICES];
  Device* uart_device;
  Device* fb_device;
  Device* ata_device;
  Device* kb_device;

  // Program counter trackers (instruction indices, so refer to the locations
  // of aligned multiples of 4 bytes)
  uint32_t pc;
  uint32_t next_pc;

  // Branch/interrupt variables
  bool delayed_ins;
  //bool interrupt_pending;

  // Special registers stored individually
  uint32_t TTMR;
  uint32_t TTCR;
  uint_least32_t PICMR;
  uint_least32_t PICSR;

  // Flags
  bool SR_SM;
  bool SR_TEE;
  bool SR_IEE;
  bool SR_DCE;
  bool SR_ICE;
  bool SR_DME;
  bool SR_IME;
  bool SR_LEE;
  bool SR_CE;
  bool SR_F;
  bool SR_CY;
  bool SR_OV;
  bool SR_OVE;
  bool SR_DSX;
  bool SR_EPH;
  bool SR_FO;
  bool SR_SUMRA;
  uint32_t SR_CID;

  // Mutex used to control access to interrupt registers (PICMR, PICSR,
  // interrupt_pending and SR_IEE)
  std::mutex interrupt_mutex;

  // TLB optimization variables (OS-specific)
  // REVISIT: optimization not currently implemented
  uint32_t boot_dtlb_misshandler_address;
  uint32_t boot_itlb_misshandler_address;
  uint32_t current_pgd;

  // Last-accessed I/DTLB pages (to speed up VA->PA translations).  Convert
  // between VA and PA by XORing with *_adj_va
  uint32_t ipage_va;
  uint32_t dpage_rd_int8_va;
  uint32_t dpage_rd_uint8_va;
  uint32_t dpage_rd_int16_va;
  uint32_t dpage_rd_uint16_va;
  uint32_t dpage_rd_uint32_va;
  uint32_t dpage_wr_uint8_va;
  uint32_t dpage_wr_uint16_va;
  uint32_t dpage_wr_uint32_va;
  
  uint32_t ipage_adj_va;
  uint32_t dpage_rd_int8_adj_va;
  uint32_t dpage_rd_uint8_adj_va;
  uint32_t dpage_rd_int16_adj_va;
  uint32_t dpage_rd_uint16_adj_va;
  uint32_t dpage_rd_uint32_adj_va;
  uint32_t dpage_wr_uint8_adj_va;
  uint32_t dpage_wr_uint16_adj_va;
  uint32_t dpage_wr_uint32_adj_va;

  // Debug variables
  // REVISIT: should be private!
public:
  uint32_t debug_message_count;
  uint64_t debug_instcount;
  bool debug_pause_trace;
  bool debug_point_en;
  
public:
  CPU(pp::Instance* pp_instance);
  virtual ~CPU();

  virtual void Reset();
  bool LoadRAMImage(const std::string& file_name);
  bool LoadHDDImage(const std::string& file_name);
  virtual void AnalyzeImage();

  // Used internally and by devices to raise/clear the specified interrupt line.
  // Thread-safe
  void RaiseInterrupt(uint32_t line);
  void ClearInterrupt(uint32_t line);

  // Memory access operations (by physical address).  The passed address is
  // masked by the access size.  No endian swizzling is performed
  uint32_t ReadMem32(uint32_t addr);
  void WriteMem32(uint32_t addr, uint32_t data);

  // Execute <steps> opcodes on the virtual CPU 
  virtual void Step(int32_t steps, uint32_t clock_speed);

  /*
   * Signal external messages to the CPU (probably for passing on to devices).
   * Thread-safe
   *
   * REVISIT: need a list of valid message encodings.  Currently only passes key-press character codes
   */
  void OnExtMessage(uint32_t msg);

  /*
   * Signal a view change (size/scale) to the CPU to be passed on to the
   * frame buffer device.  Will always be called when the emulator module first
   * starts, and will create the 2D graphics context at this point
   */
  void OnViewChange(const pp::View& view);


  void SendStrToTerminal(const char* format, ...);
  void PostMessage(pp::Var var);
  void DebugMessage(const char* format, ...);

protected:
  virtual void SetFlags(uint32_t x);
  virtual uint32_t GetFlags();
  //void CheckForInterrupt();
  virtual void SetSPR(uint32_t idx, uint32_t x);
  virtual uint32_t GetSPR(uint32_t idx);
  void Exception(uint32_t except_type, uint32_t addr);
  bool DTLBRefill(uint32_t addr, uint32_t nsets);
  bool ITLBRefill(uint32_t addr, uint32_t nsets);
  uint32_t DTLBLookup(uint32_t addr, bool write);
  uint32_t DTLBLookupNoExceptions(uint32_t addr, bool write);
  
  // Device access operations (by physical address)
  uint8_t ReadDevMem8(uint32_t addr);
  uint16_t ReadDevMem16(uint32_t addr);
  uint32_t ReadDevMem32(uint32_t addr);
  void WriteDevMem8(uint32_t addr, uint8_t data);
  void WriteDevMem16(uint32_t addr, uint16_t data);
  void WriteDevMem32(uint32_t addr, uint32_t data);

  // Disassemble the indicated instruction with the current CPU state and
  // send the result as a debug message
  virtual void DisassembleInstr(uint32_t ins);
  
};

#endif
