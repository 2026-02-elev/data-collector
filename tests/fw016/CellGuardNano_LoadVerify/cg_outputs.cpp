// CellGuard Nano firmware - actuator output driver
// Author: Yongho Ahn <nanhosoft@gmail.com>
#include "cg_outputs.h"

namespace cg {

namespace {
// Bench channels 11..14 map to the heater pins in pin-number order.
const uint8_t BENCH_HEATER_PINS[4] = {pin::HEAT_PORT_H1_1, pin::HEAT_PORT_H_1, pin::HEAT_PORT_H7_1,
                                      pin::HEAT_PORT_H8_1};
}  // namespace

bool OutputState::anyOn() const {
  if (relay || charge || discharge || esrMask || leakMask) return true;
  for (uint8_t i = 0; i < CELL_COUNT; i++)
    if (heat[i]) return true;
  return benchChannel != BENCH_NONE;
}

void clearOutputs(OutputState& o) {
  o.relay = 0;
  o.charge = 0;
  o.discharge = 0;
  o.esrMask = 0;
  o.leakMask = 0;
  for (uint8_t i = 0; i < CELL_COUNT; i++) o.heat[i] = 0;
  o.benchChannel = BENCH_NONE;
}

uint8_t OutputDriver::srByte(const OutputState& o) {
  uint8_t v = srByteFromLogical(o.esrMask, o.leakMask);
  if (o.benchChannel < 8) v |= (uint8_t)(1u << o.benchChannel);
  return v;
}

void OutputDriver::shiftOut595(uint8_t value) {
  // MSB first: after 8 SHCP pulses bit7 is in Q7 and bit0 in Q0.
  hal_.pinWrite(pin::SR_LATCH, 0);
  for (int8_t b = 7; b >= 0; b--) {
    hal_.pinWrite(pin::SR_DATA, (uint8_t)((value >> b) & 1u));
    hal_.pinWrite(pin::SR_CLOCK, 1);
    hal_.pinWrite(pin::SR_CLOCK, 0);
  }
  hal_.pinWrite(pin::SR_LATCH, 1);  // STCP rising edge copies to the outputs
  hal_.pinWrite(pin::SR_LATCH, 0);
}

void OutputDriver::forceSafe() {
  hal_.pinWrite(pin::CHARGE_PMOS, pin::SAFE_LEVEL_CHARGE_PMOS);
  hal_.pinWrite(pin::DISCHARGE_NMOS, pin::SAFE_LEVEL_DISCHARGE_NMOS);
  hal_.pinWrite(pin::PACK_RELAY, pin::SAFE_LEVEL_PACK_RELAY);
  for (uint8_t i = 0; i < CELL_COUNT; i++) hal_.pwmWrite(CELLS[i].heaterPin, pin::SAFE_LEVEL_HEATER);
  if (!srEnabled_) hal_.pinWrite(pin::SR_OE_N, pin::SAFE_LEVEL_SR_OE_N);
  shiftOut595(0);
  // The 595 has no reset (/MR tied high) and powers up with random contents.
  // /OE stays high (R20 pull-up) until the register holds zeros; then the
  // outputs are enabled so the ULN2803 inputs are actively driven low.
  hal_.pinWrite(pin::SR_OE_N, 0);
  srEnabled_ = true;
}

void OutputDriver::apply(const OutputState& o) {
  bool benchCharge = o.benchChannel == 9;
  bool benchDischarge = o.benchChannel == 10;
  bool chargeOn = o.charge || benchCharge;
  bool dischargeOn = o.discharge || benchDischarge;
  if (chargeOn && dischargeOn) {  // never both: fail safe
    chargeOn = false;
    dischargeOn = false;
  }
  // Off-going power outputs first.
  if (!chargeOn) hal_.pinWrite(pin::CHARGE_PMOS, 1);
  if (!dischargeOn) hal_.pinWrite(pin::DISCHARGE_NMOS, 0);

  hal_.pinWrite(pin::PACK_RELAY, (o.relay || o.benchChannel == 8) ? 1 : 0);
  shiftOut595(srByte(o));
  if (srEnabled_) hal_.pinWrite(pin::SR_OE_N, 0);

  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    uint8_t duty = o.heat[i];
    if (o.benchChannel >= 11 && o.benchChannel <= 14 && BENCH_HEATER_PINS[o.benchChannel - 11] == CELLS[i].heaterPin)
      duty = 255;
    hal_.pwmWrite(CELLS[i].heaterPin, duty);
  }

  // On-going power outputs last.
  if (chargeOn) hal_.pinWrite(pin::CHARGE_PMOS, 0);
  if (dischargeOn) hal_.pinWrite(pin::DISCHARGE_NMOS, 1);
}

}  // namespace cg
