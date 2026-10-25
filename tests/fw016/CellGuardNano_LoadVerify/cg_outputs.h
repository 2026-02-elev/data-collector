// CellGuard Nano firmware - actuator output driver
// Author: Yongho Ahn <nanhosoft@gmail.com>
//
// Holds the COMMANDED output state and writes it to the pins. Nothing here
// observes the physical result (relay contact, MOSFET conduction, current).
#pragma once
#include <stdint.h>

#include "cg_hal.h"
#include "pinmap.h"

namespace cg {

static const uint8_t BENCH_NONE = 0xFF;

struct OutputState {
  uint8_t relay;      // RLY2 coil (D8)
  uint8_t charge;     // Q3 charge P-MOS on (D9 LOW)
  uint8_t discharge;  // Q4 discharge N-MOS on (D10 HIGH)
  uint8_t esrMask;    // logical, bit0 = cell1
  uint8_t leakMask;   // logical, bit0 = cell1
  uint8_t heat[CELL_COUNT];
  uint8_t benchChannel;  // BENCH_NONE or 0..14 (bench builds only)

  bool anyOn() const;
};

void clearOutputs(OutputState& o);

class OutputDriver {
 public:
  explicit OutputDriver(Hal& hal) : hal_(hal), srEnabled_(false) {}

  // Drives every pin to its de-energised level, clears the 595 register and
  // only then enables the 595 outputs (so they actively drive zeros).
  void forceSafe();

  // Writes the full commanded state. Power MOSFETs are written first when
  // turning off and last when turning on.
  void apply(const OutputState& o);

  // Physical 595 byte for a commanded state (includes bench channels 0..7).
  static uint8_t srByte(const OutputState& o);

 private:
  void shiftOut595(uint8_t value);
  Hal& hal_;
  bool srEnabled_;
};

}  // namespace cg
