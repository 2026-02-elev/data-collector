// CellGuard Nano firmware - hardware abstraction
// Author: Yongho Ahn <nanhosoft@gmail.com>
//
// The control logic only talks to this interface so it can be compiled and
// tested on a PC (tests/host). The AVR implementation is cg_hal_avr.cpp.
#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef ARDUINO
#include <avr/pgmspace.h>
#else
// Host build: flash strings are ordinary strings.
#define PROGMEM
#define PSTR(s) (s)
#define pgm_read_byte(p) (*(const uint8_t*)(p))
#endif

namespace cg {

// Pointer to a string stored in flash (PSTR) on AVR, ordinary string on host.
typedef const char* PStr;

class Hal {
 public:
  virtual ~Hal() {}
  virtual uint32_t millis() = 0;
  virtual void pinWrite(uint8_t pin, uint8_t level) = 0;
  virtual void pwmWrite(uint8_t pin, uint8_t duty) = 0;
  // One ADC conversion against the EXTERNAL reference. channel 0..7 = A0..A7,
  // 14 = internal 1.1 V bandgap.
  virtual uint16_t adcConvert(uint8_t channel) = 0;
  virtual int rxRead() = 0;  // -1 when no byte is available
  virtual void txWrite(const char* data, size_t len) = 0;
  virtual void watchdogKick() = 0;
  virtual uint16_t freeRam() = 0;
};

}  // namespace cg
