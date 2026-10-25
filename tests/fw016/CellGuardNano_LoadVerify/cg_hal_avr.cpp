// CellGuard Nano firmware - ATmega328P hardware layer
// Author: Yongho Ahn <nanhosoft@gmail.com>
//
// Compiled only for the Arduino target. Host tests provide their own Hal.
#ifdef ARDUINO
#include "cg_hal_avr.h"

#include <Arduino.h>
#include <avr/io.h>
#include <avr/wdt.h>

#include "pinmap.h"

// MCUSR captured before anything else runs (survives into setup()).
uint8_t cg_mcusr_at_boot __attribute__((section(".noinit")));

// Runs from .init3, i.e. before C++ constructors and before setup(). Puts every
// actuator pin into its de-energised level as early as software can. This does
// NOT cover the time before reset is released or while the bootloader runs:
// during that window only the external gate/OE resistors hold the outputs off.
extern "C" void cg_early_init(void) __attribute__((naked, used, section(".init3")));
extern "C" void cg_early_init(void) {
  cg_mcusr_at_boot = MCUSR;
  MCUSR = 0;
  wdt_disable();  // a watchdog reset leaves the WDT running; stop it before init
  // Port D: D3 D5 D6 heaters LOW; D7 /OE HIGH (595 outputs Hi-Z).
  PORTD = (uint8_t)((PORTD & ~(_BV(PD3) | _BV(PD5) | _BV(PD6))) | _BV(PD7));
  DDRD |= _BV(PD3) | _BV(PD5) | _BV(PD6) | _BV(PD7);
  // Port B: D8 relay LOW, D9 charge P-MOS HIGH (off), D10 discharge LOW, D11 heater LOW.
  PORTB = (uint8_t)((PORTB & ~(_BV(PB0) | _BV(PB2) | _BV(PB3))) | _BV(PB1));
  DDRB |= _BV(PB0) | _BV(PB1) | _BV(PB2) | _BV(PB3);
}

namespace cg {

void AvrHal::begin() {
  // External REF3040 (4.096 V) only. analogReference() stores the setting;
  // this HAL never calls analogRead() and programs ADMUX with REFS1:0 = 00.
  analogReference(EXTERNAL);
  ADMUX = 0;
  DIDR0 = 0x3F;  // A0..A5 digital input buffers off (A6/A7 are analog-only)
  pinMode(pin::SR_DATA, OUTPUT);
  pinMode(pin::SR_CLOCK, OUTPUT);
  pinMode(pin::SR_LATCH, OUTPUT);
  digitalWrite(pin::SR_DATA, LOW);
  digitalWrite(pin::SR_CLOCK, LOW);
  digitalWrite(pin::SR_LATCH, LOW);
  pinMode(pin::UNUSED_D12, INPUT);
  Serial.begin(115200);
}

uint32_t AvrHal::millis() { return ::millis(); }

void AvrHal::pinWrite(uint8_t p, uint8_t level) {
  digitalWrite(p, level ? HIGH : LOW);
  pinMode(p, OUTPUT);
}

void AvrHal::pwmWrite(uint8_t p, uint8_t duty) {
  analogWrite(p, duty);  // 0 -> digital LOW, 255 -> digital HIGH
  pinMode(p, OUTPUT);
}

uint16_t AvrHal::adcConvert(uint8_t channel) {
  // REFS1:0 = 00 -> AREF pin (REF3040). ADLAR = 0. MUX3:0 = channel.
  ADMUX = (uint8_t)(channel & 0x0F);
  ADCSRA |= _BV(ADSC);
  while (ADCSRA & _BV(ADSC)) {
  }
  return ADC;
}

int AvrHal::rxRead() { return Serial.read(); }

void AvrHal::txWrite(const char* data, size_t len) { Serial.write((const uint8_t*)data, len); }

void AvrHal::watchdogKick() { wdt_reset(); }

extern "C" char* __brkval;
extern "C" char __heap_start;

uint16_t AvrHal::freeRam() {
  char top;
  char* heapEnd = __brkval ? __brkval : &__heap_start;
  return (uint16_t)(&top - heapEnd);
}

uint8_t AvrHal::resetFlags() {
  // MCUSR: PORF=bit0, EXTRF=bit1, BORF=bit2, WDRF=bit3 (same layout as CGS.resetFlags)
  return (uint8_t)(cg_mcusr_at_boot & 0x0F);
}

void AvrHal::watchdogStart() { wdt_enable(WDTO_2S); }

}  // namespace cg
#endif  // ARDUINO
