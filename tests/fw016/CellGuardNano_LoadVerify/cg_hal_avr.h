// CellGuard Nano firmware - ATmega328P hardware layer
// Author: Yongho Ahn <nanhosoft@gmail.com>
#pragma once
#ifdef ARDUINO
#include "cg_hal.h"

namespace cg {

class AvrHal : public Hal {
 public:
  void begin();
  uint32_t millis() override;
  void pinWrite(uint8_t pin, uint8_t level) override;
  void pwmWrite(uint8_t pin, uint8_t duty) override;
  uint16_t adcConvert(uint8_t channel) override;
  int rxRead() override;
  void txWrite(const char* data, size_t len) override;
  void watchdogKick() override;
  uint16_t freeRam() override;
  uint8_t resetFlags();
  void watchdogStart();
};

}  // namespace cg
#endif  // ARDUINO
