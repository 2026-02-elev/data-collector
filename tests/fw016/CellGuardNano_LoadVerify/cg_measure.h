// CellGuard Nano firmware - ADC conversion and measurement quality
// Author: Yongho Ahn <nanhosoft@gmail.com>
//
// A0..A3 measure CUMULATIVE taps through 47k/43k dividers against the 4.096 V
// REF3040. Individual cell voltages are differences of adjacent taps.
// A4..A7 see the raw NTC divider (+5V - 10k - TEMP_OUT - NTC - GND) with NO
// extra divider. With the new AREF, 25 degC is normally in range, but an open
// NTC and sufficiently cold temperatures can still saturate identically. The
// cold-saturation point depends on the actual +5V_Cell (near 0 degC at 5.26V).
// Saturation is reported as SATURATED_OR_OPEN, never as a temperature.
#pragma once
#include <stdint.h>

#include "cg_config.h"
#include "cg_hal.h"
#include "pinmap.h"

namespace cg {

struct Calibration {
  bool measured;  // false = NOMINAL constants (not measured)
  PStr id;
  uint16_t arefMv;
  uint16_t v5Mv;
  int32_t gainPpm[4];
  int16_t offsetMv[4];
  uint16_t ntcB;
  uint16_t ntcR25;
  uint16_t ntcPullup;
};

// Calibration selected by calibration.h (NOMINAL unless CG_CAL_USE_MEASURED).
void activeCalibration(Calibration& out);
void nominalCalibration(Calibration& out);

enum TempQuality : uint8_t {
  TQ_VALID = 0,
  TQ_SATURATED_OR_OPEN = 1,
  TQ_SHORT_OR_LOW = 2,
  TQ_ADC_INVALID = 3,
};

enum VoltageQualityBit : uint8_t {
  VQ_ADC_SATURATED = 1u << 0,
  VQ_TAP_INVERSION = 1u << 1,
  VQ_AREF_INVALID = 1u << 2,
  VQ_STALE = 1u << 3,
  VQ_PACK_NEAR_ZERO = 1u << 4,  // warning only: an empty pack and a disconnected one look alike
};
static const uint8_t VQ_ERROR_MASK = VQ_ADC_SATURATED | VQ_TAP_INVERSION | VQ_AREF_INVALID | VQ_STALE;

struct Measurement {
  uint32_t tMs;
  bool present;                     // a sample has been taken
  uint16_t raw[adc::CHANNELS];      // averaged raw A0..A7
  uint16_t bandgapRaw;              // 1.1 V bandgap against AREF
  int16_t tapMv[CELL_COUNT];        // reconstructed cumulative taps
  int16_t cellMv[CELL_COUNT];       // tap differences, cfg::INVALID_I16 when invalid
  int16_t tempCc[CELL_COUNT];       // cfg::INVALID_I16 unless tempQ == TQ_VALID
  uint8_t tempQ[CELL_COUNT];
  uint8_t vq;                       // VoltageQualityBit mask

  int16_t packMv() const { return tapMv[CELL_COUNT - 1]; }
  bool voltageOk() const { return present && (vq & VQ_ERROR_MASK) == 0; }
  uint8_t tempQualityMask() const;
};

void clearMeasurement(Measurement& m);

// Raw -> tap voltage for tap channel ch (0..3) in mV.
int16_t tapMvFromRaw(uint16_t raw, const Calibration& cal, uint8_t ch);

// Raw -> NTC temperature. Returns the quality; tempCc is valid only for TQ_VALID.
uint8_t ntcFromRaw(uint16_t raw, const Calibration& cal, bool arefOk, int16_t& tempCc);

// Fills tapMv/cellMv/tempCc/tempQ/vq from raw[] and bandgapRaw.
void evaluateMeasurement(Measurement& m, const Calibration& cal);

}  // namespace cg
