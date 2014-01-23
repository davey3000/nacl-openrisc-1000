/*
 * Class implementing an emulated keyboard controller device
 */

// REVISIT: consider adding a FIFO to prevent loss of keypress events

#include "kb_device.hpp"

const uint8_t KeyboardDevice::KEY_CODES[256] = {
    // 0
    0,      //
    0,      //
    0,      //
    0,      //
    0,      //
    0,      //
    0,      //
    0,      //
    14,     // backspace
    15,     // tab
    
    // 10
    0,      //
    0,      //
    0,      //
    28,     // enter
    0,      //
    0,      //
    42,     // shift
    29,     // ctrl
    56,     // alt
    197,    // pause/break
    
    // 20
    58,     // caps lock
    0,      // 
    0,      //
    0,      //
    0,      //
    0,      //
    0,      //
    1,      // escape
    0,      //
    0,      //
    
    // 30
    0,      //
    0,      //
    57,     // space
    201,    // page up
    209,    // page down
    207,    // end
    199,    // home
    203,    // left arrow
    200,    // up arrow
    205,    // right arrow
    
    // 40
    208,    // down arror
    0,      //
    0,      //
    0,      //
    0,      //
    210,    // insert
    211,    // delete
    0,      //
    11,     // 0
    2,      // 1
    
    // 50
    3,      // 2
    4,      // 3
    5,      // 4
    6,      // 5
    7,      // 6
    8,      // 7
    9,      // 8
    10,     // 9
    0,      // 
    0,      // 
    
    // 60
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    30,     // a
    48,     // b
    46,     // c
    32,     // d
    18,     // e
    
    // 70
    33,     // f
    34,     // g
    35,     // h
    23,     // i
    36,     // j
    37,     // k
    38,     // l
    50,     // m
    49,     // n
    24,     // o
    
    // 80
    25,     // p
    16,     // q
    19,     // r
    31,     // s
    20,     // t
    22,     // u
    47,     // v
    17,     // w
    45,     // x
    21,     // y
    
    // 90
    44,     // z
    219,    // left window key
    220,    // right window key
    183,    // select key
    0,      // 
    0,      // 
    82,     // numpad 0
    79,     // numpad 1
    80,     // numpad 2
    81,     // numpad 3
    
    // 100
    75,     // numpad 4
    76,     // numpad 5
    77,     // numpad 6
    71,     // numpad 7
    72,     // numpad 8
    73,     // numpad 9
    55,     // multiply
    77,     // add
    0,      // 
    12,     // subtract
    
    // 110
    83,     // decimal point
    181,    // divide
    59,     // F1
    60,     // F2
    61,     // F3
    62,     // F4
    63,     // F5
    64,     // F6
    65,     // F7
    66,     // F8
    
    // 120
    67,     // F9
    68,     // F10
    87,     // F11
    88,     // F12
    0,      //
    0,      //
    0,      //
    0,      //
    0,      //
    0,      //
    
    // 130
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      //
    
    // 140
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    69,     // num lock
    70,     // scroll lock
    0,      // 
    0,      // 
    0,      // 
    0,      //
    
    // 150
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      //
    
    // 160
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      //
    
    // 170
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      //
    
    // 180
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    39,     // semi-colon
    13,     // equal sign
    51,     // comma
    12,     // dash
    
    // 190
    52,     // period
    53,     // forward slash
    40,     // grave accent
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      //
    
    // 200
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      //
    
    // 210
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    26,     // open bracket
    
    // 220
    43,     // back slash
    27,     // close bracket
    40,     // single quote
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      //
    
    // 230
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      //
    
    // 240
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      //
    
    // 250
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0,      // 
    0       //
  };

KeyboardDevice::KeyboardDevice(CPU* cpu) :
  cpu(cpu)
{
  Reset();
}

KeyboardDevice::~KeyboardDevice() {
}

void KeyboardDevice::Reset() {
}

void KeyboardDevice::OnKeyDown(uint8_t rawKeyCode) {
  const uint8_t keyCode = KEY_CODES[rawKeyCode];

  if (keyCode) {
    cpu->RaiseInterrupt(5);
  }
}

void KeyboardDevice::OnKeyUp(uint8_t rawKeyCode) {
  const uint8_t keyCode = KEY_CODES[rawKeyCode] | 0x80;

  if (keyCode) {
    cpu->RaiseInterrupt(5);
  }
}

uint8_t KeyboardDevice::Read8(uint32_t offset) {
  cpu->ClearInterrupt(5);
  return 0;
}

uint16_t KeyboardDevice::Read16(uint32_t offset) {
  return 0;
}

uint32_t KeyboardDevice::Read32(uint32_t offset) {
  return 0;
}

void KeyboardDevice::Write8(uint32_t offset, uint8_t data) {
}

void KeyboardDevice::Write16(uint32_t offset, uint16_t data) {
}

void KeyboardDevice::Write32(uint32_t offset, uint32_t data) {
}
