// CellGuard Nano firmware - ADC conversion and measurement quality
// Author: Yongho Ahn <nanhosoft@gmail.com>
#include "cg_measure.h"

#include <math.h>

#include "calibration.h"

namespace cg {

void nominalCalibration(Calibration& c) {
  c.measured = false;
  c.id = PSTR(CG_CAL_NOMINAL_ID);
  c.arefMv = CG_CAL_NOMINAL_AREF_MV;
  c.v5Mv = CG_CAL_NOMINAL_V5_MV;
  for (uint8_t i = 0; i < 4; i++) {
    c.gainPpm[i] = CG_CAL_NOMINAL_GAIN_PPM;
    c.offsetMv[i] = CG_CAL_NOMINAL_OFFSET_MV;
  }
  c.ntcB = CG_CAL_NOMINAL_NTC_B;
  c.ntcR25 = CG_CAL_NOMINAL_NTC_R25;
  c.ntcPullup = CG_CAL_NOMINAL_NTC_PULLUP;
}

void activeCalibration(Calibration& c) {
#if CG_CAL_USE_MEASURED
  c.measured = true;
  c.id = PSTR(CG_CAL_MEASURED_ID);
  c.arefMv = CG_CAL_MEASURED_AREF_MV;
  c.v5Mv = CG_CAL_MEASURED_V5_MV;
  c.gainPpm[0] = CG_CAL_MEASURED_GAIN0_PPM;
  c.gainPpm[1] = CG_CAL_MEASURED_GAIN1_PPM;
  c.gainPpm[2] = CG_CAL_MEASURED_GAIN2_PPM;
  c.gainPpm[3] = CG_CAL_MEASURED_GAIN3_PPM;
  c.offsetMv[0] = CG_CAL_MEASURED_OFFSET0_MV;
  c.offsetMv[1] = CG_CAL_MEASURED_OFFSET1_MV;
  c.offsetMv[2] = CG_CAL_MEASURED_OFFSET2_MV;
  c.offsetMv[3] = CG_CAL_MEASURED_OFFSET3_MV;
  c.ntcB = CG_CAL_MEASURED_NTC_B;
  c.ntcR25 = CG_CAL_MEASURED_NTC_R25;
  c.ntcPullup = CG_CAL_MEASURED_NTC_PULLUP;
#else
  nominalCalibration(c);
#endif
}

uint8_t Measurement::tempQualityMask() const {
  uint8_t mask = 0;
  for (uint8_t i = 0; i < CELL_COUNT; i++) mask |= (uint8_t)((tempQ[i] & 3u) << (2 * i));
  return mask;
}

void clearMeasurement(Measurement& m) {
  m.tMs = 0;
  m.present = false;
  for (uint8_t i = 0; i < adc::CHANNELS; i++) m.raw[i] = 0;
  m.bandgapRaw = 0;
  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    m.tapMv[i] = 0;
    m.cellMv[i] = cfg::INVALID_I16;
    m.tempCc[i] = cfg::INVALID_I16;
    m.tempQ[i] = TQ_ADC_INVALID;
  }
  m.vq = VQ_STALE;
}

int16_t tapMvFromRaw(uint16_t raw, const Calibration& cal, uint8_t ch) {
  // V_tap = raw * Vref / 1023 * (R_top + R_bottom) / R_bottom, then gain/offset.
  const float divider = (float)(cfg::TAP_DIV_TOP_OHM + cfg::TAP_DIV_BOTTOM_OHM) / (float)cfg::TAP_DIV_BOTTOM_OHM;
  float mv = (float)raw * (float)cal.arefMv / 1023.0f * divider;
  mv = mv * ((float)cal.gainPpm[ch] / 1000000.0f) + (float)cal.offsetMv[ch];
  if (mv > 32000.0f) mv = 32000.0f;
  if (mv < -32000.0f) mv = -32000.0f;
  return (int16_t)lroundf(mv);
}

uint8_t ntcFromRaw(uint16_t raw, const Calibration& cal, bool arefOk, int16_t& tempCc) {
  tempCc = cfg::INVALID_I16;
  if (!arefOk) return TQ_ADC_INVALID;
  if (raw >= cfg::ADC_SAT_RAW) return TQ_SATURATED_OR_OPEN;
  if (raw <= cfg::NTC_SHORT_RAW) return TQ_SHORT_OR_LOW;
  float v = (float)raw * (float)cal.arefMv / 1023.0f;  // TEMP_OUT in mV
  float v5 = (float)cal.v5Mv;
  if (v >= v5) return TQ_ADC_INVALID;
  float rNtc = (float)cal.ntcPullup * v / (v5 - v);
  float invT = 1.0f / 298.15f + logf(rNtc / (float)cal.ntcR25) / (float)cal.ntcB;
  float c = 1.0f / invT - 273.15f;
  long cc = lroundf(c * 100.0f);
  if (cc < cfg::TEMP_VALID_MIN_CC || cc > cfg::TEMP_VALID_MAX_CC) return TQ_ADC_INVALID;
  tempCc = (int16_t)cc;
  return TQ_VALID;
}

void evaluateMeasurement(Measurement& m, const Calibration& cal) {
  uint8_t vq = 0;
  bool arefOk = m.bandgapRaw >= cfg::BANDGAP_MIN_RAW && m.bandgapRaw <= cfg::BANDGAP_MAX_RAW;
  if (!arefOk) vq |= VQ_AREF_INVALID;

  bool sat[CELL_COUNT];
  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    uint16_t raw = m.raw[CELLS[i].tapAdc];
    sat[i] = raw >= cfg::ADC_SAT_RAW;
    if (sat[i]) vq |= VQ_ADC_SATURATED;
    m.tapMv[i] = tapMvFromRaw(raw, cal, i);
  }
  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    int16_t below = (i == 0) ? 0 : m.tapMv[i - 1];
    // Negative differences are kept as-is (not clamped) and flagged.
    if (m.tapMv[i] < below - cfg::TAP_INVERSION_TOL_MV) vq |= VQ_TAP_INVERSION;
    bool invalid = !arefOk || sat[i] || (i > 0 && sat[i - 1]);
    m.cellMv[i] = invalid ? cfg::INVALID_I16 : (int16_t)(m.tapMv[i] - below);
  }
  if (m.packMv() < cfg::PACK_NEAR_ZERO_MV) vq |= VQ_PACK_NEAR_ZERO;

  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    m.tempQ[i] = ntcFromRaw(m.raw[CELLS[i].ntcAdc], cal, arefOk, m.tempCc[i]);
  }
  m.vq = vq;
}

}  // namespace cg
