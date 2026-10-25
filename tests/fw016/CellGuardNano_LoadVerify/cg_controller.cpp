// CellGuard Nano firmware - safety state machine, command handling, telemetry
// Author: Yongho Ahn <nanhosoft@gmail.com>
#include "cg_controller.h"

#include "hardware_verification.h"

#ifndef CG_HOST_SIM_BUILD
#define CG_HOST_SIM_BUILD 0
#endif

namespace cg {

// ---- Gates -------------------------------------------------------------------

uint16_t Gates::cgvFlags() const {
  uint16_t f = 0;
  if (mapVerified) f |= 1u << 0;
  if (thermalProtectionVerified) f |= 1u << 1;
  if (ina219Declared) f |= 1u << 2;
  if (packRelayVerified) f |= 1u << 3;
  if (esrRelayVerified) f |= 1u << 4;
  if (powerMosVerified) f |= 1u << 5;
  if (r17Verified) f |= 1u << 6;
  if (ntcRevised) f |= 1u << 7;
  if (benchBuild) f |= 1u << 8;
  if (hostSimBuild) f |= 1u << 9;
  if (powerUnlocked()) f |= 1u << 10;
  if (esrUnlocked()) f |= 1u << 11;
  if (leakUnlocked()) f |= 1u << 12;
  if (heatUnlocked()) f |= 1u << 13;
  return f;
}

void compiledGates(Gates& g) {
  g.mapVerified = CG_HW_MAP_VERIFIED != 0;
  g.thermalProtectionVerified = CG_HW_THERMAL_PROTECTION_VERIFIED != 0;
  g.ina219Declared = CG_HW_INA219_DECLARED != 0;
  g.packRelayVerified = CG_HW_PACK_RELAY_VERIFIED != 0;
  g.esrRelayVerified = CG_HW_ESR_RELAY_VERIFIED != 0;
  g.powerMosVerified = CG_HW_POWER_MOS_VERIFIED != 0;
  g.r17Verified = CG_HW_R17_VERIFIED != 0;
  g.ntcRevised = CG_HW_NTC_REVISED != 0;
  g.benchBuild = CG_BENCH_BUILD != 0;
  g.hostSimBuild = CG_HOST_SIM_BUILD != 0;
}

// ---- Names -------------------------------------------------------------------

PStr stateName(uint8_t s) {
  switch (s) {
    case ST_BOOT_SAFE: return PSTR("BOOT_SAFE");
    case ST_SELF_TEST: return PSTR("SELF_TEST");
    case ST_LOCKED: return PSTR("LOCKED");
    case ST_READY: return PSTR("READY");
    case ST_ARMED: return PSTR("ARMED");
    case ST_ACTIVE: return PSTR("ACTIVE");
    case ST_TRIPPED: return PSTR("TRIPPED");
  }
  return PSTR("UNKNOWN");
}

PStr tripName(uint8_t code) {
  switch (code) {
    case TRIP_NONE: return PSTR("NONE");
    case TRIP_PACK_OV: return PSTR("PACK_OV");
    case TRIP_CELL_OV: return PSTR("CELL_OV");
    case TRIP_OVER_TEMP: return PSTR("OVER_TEMP");
    case TRIP_HEARTBEAT_TIMEOUT: return PSTR("HEARTBEAT_TIMEOUT");
    case TRIP_ADC_SATURATED: return PSTR("ADC_SATURATED");
    case TRIP_TAP_INVERSION: return PSTR("TAP_INVERSION");
    case TRIP_AREF_INVALID: return PSTR("AREF_INVALID");
    case TRIP_HOST_TRIP: return PSTR("HOST_TRIP");
    case TRIP_WATCHDOG_RESET: return PSTR("WATCHDOG_RESET");
    case TRIP_BROWNOUT_RESET: return PSTR("BROWNOUT_RESET");
    case TRIP_STALE_SAMPLE: return PSTR("STALE_SAMPLE");
    case TRIP_INTERNAL: return PSTR("INTERNAL");
    case TRIP_CHARGE_NO_RESPONSE: return PSTR("CHARGE_NO_RESPONSE");
    case TRIP_TAP_STEP: return PSTR("TAP_STEP");
    case TRIP_TEMP_SENSOR_FAULT: return PSTR("TEMP_SENSOR_FAULT");
    case TRIP_CELL_IMBALANCE: return PSTR("CELL_IMBALANCE");
    case TRIP_DISCHARGE_NO_RESPONSE: return PSTR("DISCHARGE_NO_RESPONSE");
  }
  return PSTR("UNKNOWN");
}

namespace {
uint8_t popcount4(uint8_t v) {
  uint8_t n = 0;
  for (uint8_t i = 0; i < CELL_COUNT; i++) n += (v >> i) & 1u;
  return n;
}
}  // namespace

// ---- Construction / boot -------------------------------------------------------

Controller::Controller(Hal& hal, const Gates& gates, const Calibration& cal)
    : hal_(hal), out_(hal), gates_(gates), cal_(cal) {
  clearMeasurement(m_);
  clearOutputs(o_);
  state_ = ST_BOOT_SAFE;
  trip_ = TRIP_NONE;
  resetFlags_ = 0;
  armed_ = false;
  synced_ = false;
  inCommand_ = false;
  lastSeq_ = -1;
  ackValid_ = false;
  ackOk_ = false;
  ackSeq_ = -1;
  ackText_ = PSTR("NONE");
  hbValid_ = false;
  lastHeartbeatMs_ = 0;
  powerPhase_ = PP_IDLE;
  powerDir_ = DIR_NONE;
  lastPowerDir_ = DIR_NONE;
  powerOffValid_ = false;
  powerPhaseMs_ = lastPowerOffMs_ = chargeStartMs_ = dischargeStartMs_ = benchStartMs_ = 0;
  chargeStartPackMv_ = 0;
#if CG_FIRST_CHARGE_TEST_BUILD
  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    chargeStartCellMv_[i] = 0;
    dischargeStartCellMv_[i] = 0;
  }
  rampTargetMv_ = 0;
  drainTargetMv_ = 0;
  rampNextCheckpointMv_ = cfg::RAMP_CHECKPOINT_STEP_MV;
  bootMosOnMs_ = 0;
  bootDrainOnMs_ = 0;
#endif
  chargeResponseOk_ = false;
#if CG_FIRST_CHARGE_TEST_BUILD
  chargePulseSampleMs_ = 0;
  chargeMosOnMs_ = 0;
  chargeQuietSinceMs_ = 0;
  chargeLastLoadedCheckMs_ = 0;
  loadTapVerifyPending_ = false;
  chargeTotalMosOnMs_ = 0;
  chargePulseCount_ = 0;
#endif
  for (uint8_t i = 0; i < CELL_COUNT; i++) prevTapMv_[i] = 0;
  prevTapValid_ = false;
  for (uint8_t i = 0; i < CELL_COUNT; i++) esrStartMs_[i] = leakStartMs_[i] = heatStartMs_[i] = 0;
  now_ = 0;
  lastSampleMs_ = 0;
  samplesTaken_ = 0;
  lastTelemetryMs_ = 0;
  telSeq_ = 0;
  eventSeq_ = 0;
  rxOk_ = rxCrcErr_ = rxFormatErr_ = rxOverflow_ = loopOverruns_ = seqGaps_ = 0;
  eventCount_ = 0;
}

void Controller::begin(uint8_t resetFlags) {
  out_.forceSafe();
  clearOutputs(o_);
  resetFlags_ = resetFlags;
  now_ = hal_.millis();
  lastTelemetryMs_ = now_;

  PStr bootReason = PSTR("UNKNOWN");
  if (resetFlags & RESET_WATCHDOG) bootReason = PSTR("WATCHDOG");
  else if (resetFlags & RESET_POWER_ON) bootReason = PSTR("POWER_ON");
  else if (resetFlags & RESET_BROWNOUT) bootReason = PSTR("BROWNOUT");
  else if (resetFlags & RESET_EXTERNAL) bootReason = PSTR("EXTERNAL");

  sendCgv();
  queueEvent(PSTR("BOOT"), 0, resetFlags, 0, bootReason);
  setState(ST_SELF_TEST);
  flushEvents();
}

void Controller::finishSelfTest() {
  if (resetFlags_ & RESET_WATCHDOG) {
    tripNow(TRIP_WATCHDOG_RESET);  // a watchdog reset must never silently re-arm
  } else if ((resetFlags_ & RESET_BROWNOUT) && !(resetFlags_ & RESET_POWER_ON)) {
    tripNow(TRIP_BROWNOUT_RESET);
  } else {
    setState(desiredState());
  }
}

// ---- Main loop -------------------------------------------------------------------

void Controller::poll() {
  now_ = hal_.millis();
  hal_.watchdogKick();
#if CG_FIRST_CHARGE_TEST_BUILD
  // Do this before potentially lengthy UART parsing: a missed deadline must
  // never leave the charging MOSFET energised while RX is processed.
  serviceFirstChargeQuiet(now_);
#endif

  for (uint8_t i = 0; i < 64; i++) {  // bounded so RX floods cannot starve sampling
    int c = hal_.rxRead();
    if (c < 0) break;
    handleRxByte((char)c);
  }

  now_ = hal_.millis();
#if CG_FIRST_CHARGE_TEST_BUILD
  serviceFirstChargeQuiet(now_);
#endif
  uint32_t sinceSample = now_ - lastSampleMs_;
  bool quietSampleAllowed = true;
#if CG_FIRST_CHARGE_TEST_BUILD
  if ((powerDir_ == DIR_CHARGE || powerDir_ == DIR_DISCHARGE) && powerPhase_ == PP_ON) {
    // The loaded tap values in the 0.1.2 CGX were ~319 mV for cell 3,
    // while the measured cap was ~7.7 mV after switching OFF. During the
    // targeted commissioning sequence, sample only after the off-settle time.
    quietSampleAllowed = !o_.charge && !o_.discharge &&
        now_ - chargeQuietSinceMs_ >= cfg::FIRST_CHARGE_QUIET_MS;
  }
#endif
  if ((!m_.present || sinceSample >= cfg::SAMPLE_PERIOD_MS) && quietSampleAllowed) {
    if (m_.present && sinceSample >= 2u * cfg::SAMPLE_PERIOD_MS) loopOverruns_++;
    if (m_.present && sinceSample > cfg::STALE_SAMPLE_MS && armed_) tripNow(TRIP_STALE_SAMPLE);
    acquire(now_);
    if (state_ == ST_SELF_TEST && samplesTaken_ >= cfg::SELF_TEST_SAMPLES) finishSelfTest();
    evaluateSafety();
  } else if (m_.present && sinceSample > cfg::STALE_SAMPLE_MS && armed_) {
    // Also protects a stalled quiet-window cycle: never keep a power-path
    // armed indefinitely without a valid, rested voltage measurement.
    tripNow(TRIP_STALE_SAMPLE);
  }

  runTimers(now_);
  updateState();

  if (m_.present && now_ - lastTelemetryMs_ >= cfg::TELEMETRY_PERIOD_MS) {
    lastTelemetryMs_ = now_;
    sendCgt();
    sendCgs();
    telSeq_++;
  }
  flushEvents();
}

#if CG_FIRST_CHARGE_TEST_BUILD
void Controller::checkFirstChargeLoaded(uint32_t now) {
  // Independent emergency guard for power-ON terminal readings. Normal target
  // and all voltage thresholds are checked with BOTH MOSFETs OFF. Loaded tap
  // inversion is a special *suspicion*, not evidence of stored cell reversal:
  // immediately remove drive, then validate a genuinely rested ADC sample.
  // Only four tap channels and AREF are acquired to minimise MOS-on time.
  uint16_t raw[CELL_COUNT];
  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    raw[i] = readAveraged(CELLS[i].tapAdc, cfg::ADC_DISCARD);
  }
  uint16_t bandgap = readAveraged(adc::BANDGAP, cfg::BANDGAP_DISCARD);
  uint8_t fault = TRIP_NONE;
  bool invertedOnLoad = false;
  if (bandgap < cfg::BANDGAP_MIN_RAW || bandgap > cfg::BANDGAP_MAX_RAW) {
    fault = TRIP_AREF_INVALID;
  } else {
    int16_t below = 0;
    for (uint8_t i = 0; i < CELL_COUNT; i++) {
      if (raw[i] >= cfg::ADC_SAT_RAW) { fault = TRIP_ADC_SATURATED; break; }
      int16_t tap = tapMvFromRaw(raw[i], cal_, i);
      if (tap < below - cfg::TAP_INVERSION_TOL_MV) {
        invertedOnLoad = true;
      }
      if (tap - below >= cfg::FIRST_CHARGE_LIVE_CELL_HARD_MV) {
        fault = TRIP_CELL_OV;
        break;
      }
      if (i == CELL_COUNT - 1 && tap >= cfg::FIRST_CHARGE_LIVE_PACK_HARD_MV) {
        fault = TRIP_PACK_OV;
        break;
      }
      below = tap;
    }
  }
  chargeLastLoadedCheckMs_ = now;
  if (fault != TRIP_NONE) {
    // OFF first. Even diagnostic ADC conversion / CGX formatting must not
    // extend a pulse once the emergency trip has been detected.
    o_.charge = 0;
    o_.discharge = 0;
    chargeQuietSinceMs_ = hal_.millis();
    applyOutputs();
    // Keep CGX diagnostic compatibility, with the last *loaded* raw A0..A3.
    for (uint8_t i = 0; i < CELL_COUNT; i++) m_.raw[CELLS[i].tapAdc] = raw[i];
    m_.bandgapRaw = bandgap;
    evaluateMeasurement(m_, cal_);
    queueEvent(PSTR("LIVE_GUARD"), 0, 1, 0, tripName(fault));
    tripNow(fault);
    return;
  }
  if (invertedOnLoad) {
    // Absolutely no additional MOS-on time for UART formatting, diagnostics,
    // or confirmation. The relay is left closed while waiting for the quiet
    // ADC sample, but the power MOSFETs are forcibly OFF immediately.
    const uint8_t direction = powerDir_;
    const uint32_t offAt = hal_.millis();
    o_.charge = 0;
    o_.discharge = 0;
    applyOutputs();
    chargeQuietSinceMs_ = offAt;
    const uint32_t pulseOnMs = offAt - chargeMosOnMs_;
    chargeTotalMosOnMs_ += (uint16_t)pulseOnMs;
    if (direction == DIR_CHARGE) bootMosOnMs_ += pulseOnMs;
    else bootDrainOnMs_ += pulseOnMs;
    loadTapVerifyPending_ = true;
    queueEvent(PSTR("LOAD_TAP_SUSPECT"), 0, 1, 0, PSTR("INVERSION_ONLOAD"));
    sendLoadTapSuspect(now, direction, raw, bandgap);
  }
}

void Controller::sendLoadTapSuspect(uint32_t atMs, uint8_t direction,
                                    const uint16_t raw[CELL_COUNT], uint16_t bandgap) {
  // CGY is an ON-load diagnostic, not the trusted CGT reading and not a trip.
  // CGY,time_ms,direction,raw_A0..A3,tap_mV_A0..A3,bandgap_raw
  fb_.start(PSTR("CGY"));
  fb_.addU(atMs);
  fb_.addU(direction);
  for (uint8_t i = 0; i < CELL_COUNT; i++) fb_.addU(raw[i]);
  for (uint8_t i = 0; i < CELL_COUNT; i++) fb_.addI(tapMvFromRaw(raw[i], cal_, i));
  fb_.addU(bandgap);
  fb_.send(hal_);
}

void Controller::serviceFirstChargeQuiet(uint32_t now) {
  if (state_ == ST_TRIPPED || powerPhase_ != PP_ON) return;
  const bool charging = powerDir_ == DIR_CHARGE && o_.charge;
  const bool draining = powerDir_ == DIR_DISCHARGE && o_.discharge;
  if (!charging && !draining) return;
  uint32_t onMs = now - chargeMosOnMs_;
  if (onMs > cfg::FIRST_CHARGE_OVERRUN_TRIP_MS) {
    // A delayed main loop is a fault, NOT permission for a longer pulse.
    o_.charge = 0;
    o_.discharge = 0;
    chargeQuietSinceMs_ = now;
    applyOutputs();
    tripNow(TRIP_INTERNAL);
    return;
  }
  // Total MOS-on energy budgets cannot be reset by SYNC, OFF or changing a
  // target. Resetting the MCU still resets them; unattended use is forbidden.
  if ((charging && bootMosOnMs_ + onMs >= cfg::RAMP_BOOT_ON_BUDGET_MS) ||
      (draining && bootDrainOnMs_ + onMs >= cfg::DRAIN_BOOT_ON_BUDGET_MS)) {
    if (charging) bootMosOnMs_ += onMs;
    else bootDrainOnMs_ += onMs;
    chargeTotalMosOnMs_ += onMs;
    o_.charge = 0;
    o_.discharge = 0;
    chargeQuietSinceMs_ = now;
    applyOutputs();
    stopPower(PSTR("BUDGET"));
    return;
  }
  if (onMs >= cfg::FIRST_CHARGE_MAX_ON_MS) {
    o_.charge = 0;
    o_.discharge = 0;
    chargeQuietSinceMs_ = now;
    chargeTotalMosOnMs_ += (uint16_t)onMs;
    if (charging) bootMosOnMs_ += onMs;
    else bootDrainOnMs_ += onMs;
    applyOutputs();
    return;
  }
  if (now - chargeLastLoadedCheckMs_ >= cfg::FIRST_CHARGE_LIVE_CHECK_MS) {
    checkFirstChargeLoaded(now);
    // Time spent reading the ADC counts toward the 120 ms MOS-on bound.
    if (o_.charge) serviceFirstChargeQuiet(hal_.millis());
  }
}
#endif

// ---- Measurement -------------------------------------------------------------------

uint16_t Controller::readAveraged(uint8_t channel, uint8_t discard) {
  for (uint8_t i = 0; i < discard; i++) (void)hal_.adcConvert(channel);
  uint16_t sum = 0;
  for (uint8_t i = 0; i < cfg::ADC_AVG; i++) sum += hal_.adcConvert(channel);
  return (uint16_t)((sum + cfg::ADC_AVG / 2) / cfg::ADC_AVG);
}

void Controller::acquire(uint32_t now) {
  for (uint8_t ch = 0; ch < adc::CHANNELS; ch++) m_.raw[ch] = readAveraged(ch, cfg::ADC_DISCARD);
  m_.bandgapRaw = readAveraged(adc::BANDGAP, cfg::BANDGAP_DISCARD);
  m_.tMs = now;
  m_.present = true;
  evaluateMeasurement(m_, cal_);
  lastSampleMs_ = now;
  if (samplesTaken_ < 255) samplesTaken_++;
}

void Controller::evaluateSafety() {
  if (state_ == ST_BOOT_SAFE || state_ == ST_SELF_TEST) return;
  const bool arefOk = (m_.vq & VQ_AREF_INVALID) == 0;
  // Previous taps for the step check; refreshed every sample.
  const bool hadPrev = prevTapValid_;
  int16_t prevTap[CELL_COUNT];
  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    prevTap[i] = prevTapMv_[i];
    prevTapMv_[i] = m_.tapMv[i];
  }
  prevTapValid_ = arefOk;

  if (state_ != ST_TRIPPED) {
    if (!arefOk) {
      if (armed_) tripNow(TRIP_AREF_INVALID);  // otherwise LOCKED via updateState()
    } else if (m_.vq & VQ_ADC_SATURATED) {
      tripNow(TRIP_ADC_SATURATED);  // a saturated tap is above the 5.05 V trip anyway
    } else if (m_.packMv() >= cfg::PACK_TRIP_MV) {
      tripNow(TRIP_PACK_OV);
    } else {
      for (uint8_t i = 0; i < CELL_COUNT; i++) {
        if (m_.cellMv[i] != cfg::INVALID_I16 && m_.cellMv[i] >= cfg::CELL_TRIP_MV) {
          tripNow(TRIP_CELL_OV);
          break;
        }
      }
    }
    if (state_ != ST_TRIPPED && armed_ && (m_.vq & VQ_TAP_INVERSION)) {
#if CG_FIRST_CHARGE_TEST_BUILD
      if (loadTapVerifyPending_)
        queueEvent(PSTR("LOAD_TAP_VERIFY"), 0, 1, 0, PSTR("REST_INVERTED"));
#endif
      tripNow(TRIP_TAP_INVERSION);
    }
    if (state_ != ST_TRIPPED) {
      for (uint8_t i = 0; i < CELL_COUNT; i++) {
        if (m_.tempQ[i] == TQ_VALID && m_.tempCc[i] >= cfg::OVER_TEMP_TRIP_CC) {
          tripNow(TRIP_OVER_TEMP);
          break;
        }
      }
    }
    // A distinguishable NTC fault (short / out of range) while anything is
    // energised. Saturation (cold OR open) cannot be distinguished and is
    // handled by blocking the heaters only - see docs/known_limitations.md.
    if (state_ != ST_TRIPPED && actuatorsOn() && tempSensorFault()) tripNow(TRIP_TEMP_SENSOR_FAULT);
    // Measurement path plausibility while charging: a lost 8-pin harness or
    // tap divider reads ~0 V and the 4.80 V stop would never be reached.
    if (state_ != ST_TRIPPED && powerDir_ == DIR_CHARGE && hadPrev && arefOk) {
      for (uint8_t i = 0; i < CELL_COUNT; i++) {
        if (prevTap[i] - m_.tapMv[i] > cfg::TAP_STEP_DROP_MV) {
          tripNow(TRIP_TAP_STEP);
          break;
        }
      }
    }
    if (state_ != ST_TRIPPED && powerDir_ == DIR_CHARGE && powerPhase_ == PP_ON && !chargeResponseOk_) {
      if (m_.packMv() - chargeStartPackMv_ >= cfg::CHARGE_RESPONSE_MIN_RISE_MV) {
        chargeResponseOk_ = true;
      } else if (now_ - chargeStartMs_ >= cfg::CHARGE_RESPONSE_WINDOW_MS) {
        tripNow(TRIP_CHARGE_NO_RESPONSE);
      }
    }
#if CG_FIRST_CHARGE_TEST_BUILD
    // Every RAMP checks that each cell actually rises; an initially charged
    // cell must not be mistaken for a healthy path that never receives energy.
    if (state_ != ST_TRIPPED && powerDir_ == DIR_CHARGE && powerPhase_ == PP_ON) {
      for (uint8_t i = 0; i < CELL_COUNT; i++) {
        if (m_.tempQ[i] != TQ_VALID) {
          tripNow(TRIP_TEMP_SENSOR_FAULT);
          break;
        }
        if (now_ - chargeStartMs_ >= cfg::STAGE_CELL_RISE_CHECK_MS &&
            m_.cellMv[i] - chargeStartCellMv_[i] < cfg::STAGE_CELL_MIN_RISE_MV) {
          tripNow(TRIP_CHARGE_NO_RESPONSE);
          break;
        }
      }
    }
    if (state_ != ST_TRIPPED && powerDir_ == DIR_DISCHARGE && powerPhase_ == PP_ON &&
        now_ - dischargeStartMs_ >= cfg::DRAIN_CELL_FALL_CHECK_MS) {
      for (uint8_t i = 0; i < CELL_COUNT; i++) {
        if (dischargeStartCellMv_[i] - m_.cellMv[i] < cfg::DRAIN_CELL_MIN_FALL_MV) {
          tripNow(TRIP_DISCHARGE_NO_RESPONSE);
          break;
        }
      }
    }
    if (state_ != ST_TRIPPED &&
        (powerDir_ == DIR_CHARGE || powerDir_ == DIR_DISCHARGE) && powerPhase_ == PP_ON) {
      int16_t low = m_.cellMv[0];
      int16_t high = m_.cellMv[0];
      for (uint8_t i = 1; i < CELL_COUNT; i++) {
        if (m_.cellMv[i] < low) low = m_.cellMv[i];
        if (m_.cellMv[i] > high) high = m_.cellMv[i];
      }
      if (high - low > cfg::RAMP_MAX_SPREAD_MV) tripNow(TRIP_CELL_IMBALANCE);
      if (state_ != ST_TRIPPED && powerDir_ == DIR_CHARGE &&
          low >= rampNextCheckpointMv_) {
        // Send checkpoints only after ALL four rested cell measurements reach
        // the milestone; the Pi also checks the post-relay settled values.
        queueEvent(PSTR("CHECKPOINT"), 0, rampNextCheckpointMv_, low, PSTR("MIN_CELL_MV"));
        rampNextCheckpointMv_ += cfg::RAMP_CHECKPOINT_STEP_MV;
      }
    }
#endif
  }
  if (state_ == ST_TRIPPED) return;

#if CG_FIRST_CHARGE_TEST_BUILD
  if (loadTapVerifyPending_) {
    // Only reached on a fresh, MOS-OFF, >=80ms-rested acquisition. A real
    // inversion/AREF/ADC/high-voltage fault has already latched above. Even
    // if this sample is normal, DO NOT auto-restart: abort this stage,
    // disarm, and require explicit review/ARM for any later operation.
    loadTapVerifyPending_ = false;
    if (!m_.voltageOk()) {
      tripNow(TRIP_INTERNAL);
      return;
    }
    queueEvent(PSTR("LOAD_TAP_VERIFY"), 0, 1, 0, PSTR("REST_OK"));
    stopPower(PSTR("LOAD_TAP_UNSTABLE"));
    return;
  }
#endif

  // Normal end-of-operation limits (no automatic restart).
  if (powerDir_ == DIR_CHARGE) {
    if (m_.packMv() >= cfg::PACK_CHARGE_STOP_MV) {
      stopPower(PSTR("PACK_LIMIT"));
    } else {
      for (uint8_t i = 0; i < CELL_COUNT; i++) {
        if (m_.cellMv[i] >= cfg::CELL_CHARGE_STOP_MV) {
          stopPower(PSTR("CELL_LIMIT"));
          break;
        }
      }
    }
    if (powerDir_ == DIR_CHARGE && rampTargetMv_ > 0) {
      for (uint8_t i = 0; i < CELL_COUNT; i++) {
        if (m_.cellMv[i] >= rampTargetMv_) {
          stopPower(PSTR("TARGET"));
          break;
        }
      }
    }
  } else if (powerDir_ == DIR_DISCHARGE) {
    if (m_.packMv() <= cfg::PACK_DISCHARGE_STOP_MV) {
      stopPower(PSTR("PACK_FLOOR"));
    } else {
      for (uint8_t i = 0; i < CELL_COUNT; i++) {
        if (m_.cellMv[i] <= cfg::CELL_DISCHARGE_STOP_MV) {
          stopPower(PSTR("CELL_FLOOR"));
          break;
        }
        if (drainTargetMv_ > 0 && m_.cellMv[i] <= drainTargetMv_) {
          stopPower(PSTR("DRAIN_TARGET"));
          break;
        }
      }
    }
  }
  // Heaters run only on a VALID reading below the soft limit.
  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    if (!o_.heat[i]) continue;
    if (m_.tempQ[i] != TQ_VALID) stopHeat(i, PSTR("TEMP_INVALID"));
    else if (m_.tempCc[i] >= cfg::HEAT_SOFT_OFF_CC) stopHeat(i, PSTR("TEMP_SOFT_LIMIT"));
  }
}

bool Controller::tempSensorFault() const {
  // Without a sample or with a bad AREF every channel is invalid anyway; that is
  // reported (and handled) as MEAS_INVALID / AREF_INVALID, not as an NTC fault.
  if (!m_.present || (m_.vq & (VQ_AREF_INVALID | VQ_STALE))) return false;
  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    if (m_.tempQ[i] == TQ_SHORT_OR_LOW || m_.tempQ[i] == TQ_ADC_INVALID) return true;
  }
  return false;
}

bool Controller::resetConditionsOk() const {
  if (!m_.voltageOk()) return false;
  if (m_.packMv() >= cfg::PACK_TRIP_MV) return false;
  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    if (m_.cellMv[i] == cfg::INVALID_I16 || m_.cellMv[i] >= cfg::CELL_TRIP_MV) return false;
    if (m_.tempQ[i] == TQ_VALID && m_.tempCc[i] >= cfg::REARM_MAX_TEMP_CC) return false;
  }
  return true;
}

void Controller::runTimers(uint32_t now) {
  if (armed_ && (!hbValid_ || now - lastHeartbeatMs_ >= cfg::HEARTBEAT_TIMEOUT_MS)) {
    tripNow(TRIP_HEARTBEAT_TIMEOUT);
    return;
  }

  // Power path sequencer: relay closes before the MOSFET, MOSFET opens before the relay.
  if (powerPhase_ == PP_CLOSING && now - powerPhaseMs_ >= cfg::RELAY_SETTLE_MS) {
    powerPhase_ = PP_ON;
    if (powerDir_ == DIR_CHARGE) {
#if CG_FIRST_CHARGE_TEST_BUILD
      // The relay is closed but MOS stays OFF until a NEW, rested ADC sample
      // passes the normal safety checks. Never energise on a stale sample.
      o_.charge = 0;
      chargePulseSampleMs_ = lastSampleMs_;
      chargeQuietSinceMs_ = now;
      chargeTotalMosOnMs_ = 0;
      chargePulseCount_ = 0;
#else
      o_.charge = 1;
#endif
      chargeStartMs_ = now;
      chargeStartPackMv_ = m_.packMv();
#if CG_FIRST_CHARGE_TEST_BUILD
      for (uint8_t i = 0; i < CELL_COUNT; i++) chargeStartCellMv_[i] = m_.cellMv[i];
#endif
      chargeResponseOk_ = chargeStartPackMv_ >= cfg::CHARGE_RESPONSE_BELOW_MV;
      queueEvent(PSTR("CHARGE_START"), 0, 1, 1, PSTR("HOST"));
    } else if (powerDir_ == DIR_DISCHARGE) {
#if CG_FIRST_CHARGE_TEST_BUILD
      // Discharging also has an ESR voltage sag: do not infer stored charge
      // from an on-load ADC sample. Use the same 120/80ms quiet-pulse policy.
      o_.discharge = 0;
      chargePulseSampleMs_ = lastSampleMs_;
      chargeQuietSinceMs_ = now;
      chargeTotalMosOnMs_ = 0;
      chargePulseCount_ = 0;
      for (uint8_t i = 0; i < CELL_COUNT; i++) dischargeStartCellMv_[i] = m_.cellMv[i];
#else
      o_.discharge = 1;
#endif
      dischargeStartMs_ = now;
      queueEvent(PSTR("DRAIN_START"), 0, 1, 1, PSTR("HOST"));
    }
    applyOutputs();
  } else if (powerPhase_ == PP_OPENING && now - powerPhaseMs_ >= cfg::RELAY_SETTLE_MS) {
    powerPhase_ = PP_IDLE;
    o_.relay = 0;
    queueEvent(PSTR("RELAY_OFF"), 0, 0, 0, PSTR("SEQUENCER"));
    applyOutputs();
  }

  if (powerDir_ == DIR_CHARGE && powerPhase_ == PP_ON &&
      now - chargeStartMs_ >= cfg::CHARGE_MAX_MS) stopPower(PSTR("TIMEOUT"));
  if (powerDir_ == DIR_DISCHARGE && powerPhase_ == PP_ON &&
      now - dischargeStartMs_ >= cfg::DISCHARGE_MAX_MS) stopPower(PSTR("TIMEOUT"));
#if CG_FIRST_CHARGE_TEST_BUILD
  // Charge and drain both require independent bounded MOS pulses with a new
  // rested ADC sample between pulses; no continuous-on discharge mode.
  const bool activeCharge = powerDir_ == DIR_CHARGE &&
      now - chargeStartMs_ < cfg::CHARGE_MAX_MS - cfg::FIRST_CHARGE_MAX_ON_MS &&
      bootMosOnMs_ < cfg::RAMP_BOOT_ON_BUDGET_MS;
  const bool activeDrain = powerDir_ == DIR_DISCHARGE &&
      now - dischargeStartMs_ < cfg::DISCHARGE_MAX_MS - cfg::FIRST_CHARGE_MAX_ON_MS &&
      bootDrainOnMs_ < cfg::DRAIN_BOOT_ON_BUDGET_MS;
  if (state_ != ST_TRIPPED && armed_ && (activeCharge || activeDrain) && powerPhase_ == PP_ON &&
      !loadTapVerifyPending_ && !o_.charge && !o_.discharge &&
      lastSampleMs_ != chargePulseSampleMs_ &&
      now - lastSampleMs_ <= 25u && now - chargeQuietSinceMs_ >= cfg::FIRST_CHARGE_QUIET_MS &&
      (activeCharge || activeDrain)) {
    chargePulseSampleMs_ = lastSampleMs_;
    chargeMosOnMs_ = now;
    chargeLastLoadedCheckMs_ = now;
    chargePulseCount_++;
    if (activeCharge) o_.charge = 1;
    else o_.discharge = 1;
    applyOutputs();
  }
#endif
  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    if ((o_.esrMask & (1u << i)) && now - esrStartMs_[i] >= cfg::ESR_MAX_MS) stopEsr(i, PSTR("TIMEOUT"));
    if ((o_.leakMask & (1u << i)) && now - leakStartMs_[i] >= cfg::LEAK_MAX_MS) stopLeak(i, PSTR("TIMEOUT"));
    if (o_.heat[i] && now - heatStartMs_[i] >= cfg::HEAT_MAX_MS) stopHeat(i, PSTR("TIMEOUT"));
  }
  if (o_.benchChannel != BENCH_NONE && now - benchStartMs_ >= cfg::BENCH_MAX_MS) stopBench(PSTR("TIMEOUT"));

  // Internal invariant: never both power directions.
  if (o_.charge && o_.discharge) tripNow(TRIP_INTERNAL);
}

// ---- State -------------------------------------------------------------------------

void Controller::setState(State s) {
  if (s == state_) return;
  queueEvent(PSTR("STATE"), 0, state_, s, stateName(s));
  state_ = s;
}

State Controller::desiredState() const {
  if (armed_) return (o_.anyOn() || powerDir_ != DIR_NONE) ? ST_ACTIVE : ST_ARMED;
#if CG_FIRST_CHARGE_TEST_BUILD
  // Normal verified-hardware gate values remain locked. This isolated build
  // permits multiple EXPLICITLY commanded, budgeted low-voltage sessions.
  if (!m_.voltageOk() ||
      bootMosOnMs_ >= cfg::RAMP_BOOT_ON_BUDGET_MS ||
      bootDrainOnMs_ >= cfg::DRAIN_BOOT_ON_BUDGET_MS ||
      m_.packMv() > cfg::STAGE_START_PACK_MAX_MV ||
      m_.packMv() < cfg::STAGE_START_PACK_MIN_MV) return ST_LOCKED;
  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    if (m_.tempQ[i] != TQ_VALID ||
        m_.cellMv[i] < cfg::STAGE_START_CELL_MIN_MV ||
        m_.cellMv[i] > cfg::STAGE_START_CELL_MAX_MV) return ST_LOCKED;
  }
  return ST_READY;
#endif
  bool locked = !m_.voltageOk() || (!gates_.anyUnlocked() && !gates_.benchBuild);
  return locked ? ST_LOCKED : ST_READY;
}

void Controller::updateState() {
  if (state_ == ST_BOOT_SAFE || state_ == ST_SELF_TEST || state_ == ST_TRIPPED) return;
  setState(desiredState());
}

void Controller::tripNow(uint8_t code) {
  if (state_ == ST_TRIPPED) {
    allOff(PSTR("TRIP"));  // keep the first cause, still force everything off
    return;
  }
  queueEvent(PSTR("TRIP"), 0, 0, 0, tripName(code));
  allOff(PSTR("TRIP"));
  if (armed_) {
    armed_ = false;
    queueEvent(PSTR("DISARM"), 0, 0, 0, PSTR("TRIP"));
  }
  trip_ = code;
  setState(ST_TRIPPED);

  // Capture the measurement that CAUSED the trip, rather than the next 1 Hz
  // telemetry measurement after the charge path has already switched off.
  // The MOSFET/relay have already been commanded safe by allOff() above:
  // UART output must NEVER delay that shutdown.
  // CGX: time_ms,trip_code,raw_A0..A3,tap_mV_A0..A3,bandgap_raw,vq
  fb_.start(PSTR("CGX"));
  fb_.addU(now_);
  fb_.addU(code);
  for (uint8_t i = 0; i < CELL_COUNT; i++) fb_.addU(m_.raw[CELLS[i].tapAdc]);
  for (uint8_t i = 0; i < CELL_COUNT; i++) fb_.addI(m_.tapMv[i]);
  fb_.addU(m_.bandgapRaw);
  fb_.addU(m_.vq);
  fb_.send(hal_);
}

// ---- Outputs -----------------------------------------------------------------------

void Controller::allOff(PStr reason) {
  if (powerDir_ != DIR_NONE) stopPower(reason);
  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    if (o_.esrMask & (1u << i)) stopEsr(i, reason);
    if (o_.leakMask & (1u << i)) stopLeak(i, reason);
    if (o_.heat[i]) stopHeat(i, reason);
  }
  if (o_.benchChannel != BENCH_NONE) stopBench(reason);
  if (o_.relay) queueEvent(PSTR("RELAY_OFF"), 0, 0, 0, reason);
  clearOutputs(o_);
  powerPhase_ = PP_IDLE;
  powerDir_ = DIR_NONE;
  out_.forceSafe();
}

void Controller::stopPower(PStr reason) {
  if (powerDir_ == DIR_NONE) return;
#if CG_FIRST_CHARGE_TEST_BUILD
  loadTapVerifyPending_ = false;
#endif
  if (powerDir_ == DIR_CHARGE) {
#if CG_FIRST_CHARGE_TEST_BUILD
    if (o_.charge) {
      uint32_t onMs = now_ - chargeMosOnMs_;
      chargeTotalMosOnMs_ += onMs;
      bootMosOnMs_ += onMs;
    }
#endif
    o_.charge = 0;
    queueEvent(PSTR("CHARGE_STOP"), 0, 1, 0, reason);
#if CG_FIRST_CHARGE_TEST_BUILD
    queueEvent(PSTR("PULSE_TOTAL"), 0, chargePulseCount_,
               (int16_t)chargeTotalMosOnMs_, PSTR("ON_MS"));
#endif
  }
  if (powerDir_ == DIR_DISCHARGE) {
#if CG_FIRST_CHARGE_TEST_BUILD
    if (o_.discharge) {
      uint32_t onMs = now_ - chargeMosOnMs_;
      chargeTotalMosOnMs_ += onMs;
      bootDrainOnMs_ += onMs;
    }
#endif
    o_.discharge = 0;
    queueEvent(PSTR("DRAIN_STOP"), 0, 1, 0, reason);
#if CG_FIRST_CHARGE_TEST_BUILD
    queueEvent(PSTR("PULSE_TOTAL"), 0, chargePulseCount_,
               (int16_t)chargeTotalMosOnMs_, PSTR("DRAIN_ON_MS"));
#endif
  }
  lastPowerDir_ = powerDir_;
  lastPowerOffMs_ = now_;
  powerOffValid_ = true;
  powerDir_ = DIR_NONE;
#if CG_FIRST_CHARGE_TEST_BUILD
  // Every completed stage disarms. A NEW ARM and explicit RAMP/DRAIN command
  // are mandatory; no automatic retry or automatic direction reversal.
  armed_ = false;
  hbValid_ = false;
#endif
  // Relay opens only after the MOSFET is off and the settle time elapsed.
  if (o_.relay) {
    powerPhase_ = PP_OPENING;
    powerPhaseMs_ = now_;
  } else {
    powerPhase_ = PP_IDLE;
  }
  applyOutputs();
}

void Controller::stopEsr(uint8_t idx, PStr reason) {
  o_.esrMask &= (uint8_t)~(1u << idx);
  queueEvent(PSTR("ESR_STOP"), (uint8_t)(idx + 1), 1, 0, reason);
  applyOutputs();
}

void Controller::stopLeak(uint8_t idx, PStr reason) {
  o_.leakMask &= (uint8_t)~(1u << idx);
  queueEvent(PSTR("LEAK_STOP"), (uint8_t)(idx + 1), 1, 0, reason);
  applyOutputs();
}

void Controller::stopHeat(uint8_t idx, PStr reason) {
  int16_t was = o_.heat[idx];
  o_.heat[idx] = 0;
  queueEvent(PSTR("HEAT_STOP"), (uint8_t)(idx + 1), was, 0, reason);
  applyOutputs();
}

void Controller::stopBench(PStr reason) {
  int16_t ch = o_.benchChannel;
  o_.benchChannel = BENCH_NONE;
  queueEvent(PSTR("BENCH_OFF"), 0, ch, 0, reason);
  applyOutputs();
}

// ---- RX / command handling ------------------------------------------------------------

void Controller::handleRxByte(char c) {
  switch (rx_.feed(c)) {
    case RX_NONE: break;
    case RX_FRAME_OK: handleFrame(rx_.payload()); break;
    case RX_CRC_ERROR: rxCrcErr_++; break;  // no reply: the seq cannot be trusted
    case RX_FORMAT_ERROR: rxFormatErr_++; break;
    case RX_OVERFLOW: rxOverflow_++; break;
  }
}

void Controller::handleFrame(char* payload) {
  Command c;
  ParseResult pr = parseCommand(payload, c, gates_.benchBuild);
  if (pr != PARSE_OK) {
    rxFormatErr_++;
    PStr why = pr == PARSE_UNKNOWN_CMD ? PSTR("UNKNOWN_CMD") : pr == PARSE_BAD_ARGS ? PSTR("BAD_ARGS") : PSTR("BAD_FORMAT");
    sendAck(c.seq, false, why);
    return;
  }
  rxOk_++;
  const bool safeStop = c.type == CMD_OFF || c.type == CMD_TRIP;
  inCommand_ = true;

  if (c.type == CMD_SYNC || !synced_) {
    Result r;
    if (c.type == CMD_SYNC) {
      r = execute(c);
    } else if (c.type == CMD_PING || c.type == CMD_STATUS || c.type == CMD_RAW || c.type == CMD_CAL || safeStop) {
      r = execute(c);
    } else {
      r.ok = false;
      r.text = PSTR("NOSYNC");
    }
    sendAck(c.seq, r.ok, r.text);
    if (c.type == CMD_SYNC) {
      ackValid_ = true;
      ackOk_ = r.ok;
      ackSeq_ = c.seq;
      ackText_ = r.text;
    }
    if (r.ok) sendExtras(c);
    inCommand_ = false;
    flushEvents();
    return;
  }

  uint16_t d = (uint16_t)((uint16_t)c.seq - (uint16_t)lastSeq_);
  if (d == 0) {  // duplicate: re-send the original reply, do not execute again
    if (safeStop) (void)execute(c);  // idempotent safety commands still act
    if (ackValid_ && ackSeq_ == c.seq) sendAck(ackSeq_, ackOk_, ackText_);
    inCommand_ = false;
    flushEvents();
    return;
  }
  if (d >= 32768u) {  // older than the last executed command
    if (safeStop) {
      (void)execute(c);
      sendAck(c.seq, true, PSTR("OLDSEQ"));
    } else {
      sendAck(c.seq, false, PSTR("OLDSEQ"));
    }
    inCommand_ = false;
    flushEvents();
    return;
  }
  if (d > 1) seqGaps_++;
  lastSeq_ = c.seq;

  Result r = execute(c);
  if (r.ok) {
    hbValid_ = true;  // only accepted, newly executed commands refresh the heartbeat
    lastHeartbeatMs_ = now_;
  }
  ackValid_ = true;
  ackOk_ = r.ok;
  ackSeq_ = c.seq;
  ackText_ = r.text;
  sendAck(c.seq, r.ok, r.text);
  if (r.ok) sendExtras(c);
  inCommand_ = false;
  flushEvents();
}

void Controller::sendExtras(const Command& c) {
  switch (c.type) {
    case CMD_SYNC: sendCgv(); break;
    case CMD_STATUS: sendCgs(); break;
    case CMD_RAW: sendCgd(c.seq); break;
    case CMD_CAL: sendCgk(c.seq); break;
    default: break;
  }
}

Controller::Result Controller::execute(const Command& c) {
  Result ok = {true, PSTR("NONE")};
  switch (c.type) {
    case CMD_SYNC:
      allOff(PSTR("SYNC"));
      if (armed_) {
        armed_ = false;
        queueEvent(PSTR("DISARM"), 0, 0, 0, PSTR("SYNC"));
      }
      synced_ = true;
      lastSeq_ = c.seq;
      ackValid_ = false;
      queueEvent(PSTR("SYNC"), 0, (int16_t)(c.seq & 0x7FFF), 0, PSTR("HOST"));
      updateState();
      return ok;
    case CMD_PING:
    case CMD_STATUS:
    case CMD_RAW:
    case CMD_CAL:
      return ok;
    case CMD_ARM: return cmdArm();
    case CMD_OFF: return cmdOff(PSTR("OFF"));
    case CMD_TRIP:
      tripNow(TRIP_HOST_TRIP);
      return ok;
    case CMD_RESET: return cmdReset();
    case CMD_CHARGE:
      if (c.arg[0]) return {false, PSTR("USE_RAMP")};
      return cmdPower(DIR_CHARGE, false);
    case CMD_DISCHARGE:
      if (c.arg[0]) return {false, PSTR("USE_DRAIN")};
      return cmdPower(DIR_DISCHARGE, false);
    case CMD_RAMP: return cmdRamp((uint16_t)c.arg[0]);
    case CMD_DRAIN: return cmdDrain((uint16_t)c.arg[0]);
    case CMD_ESR: return cmdEsr((uint8_t)c.arg[0], c.arg[1] != 0);
    case CMD_LEAK: return cmdLeak((uint8_t)c.arg[0], c.arg[1] != 0);
    case CMD_HEAT: return cmdHeat((uint8_t)c.arg[0], (uint8_t)c.arg[1]);
    case CMD_BENCH: return cmdBench((uint8_t)c.arg[0], c.arg[1] != 0);
    case CMD_NONE: break;
  }
  Result bad = {false, PSTR("UNKNOWN_CMD")};
  return bad;
}

Controller::Result Controller::checkActuationAllowed(bool gateOpen) const {
  Result r = {false, PSTR("NONE")};
  if (state_ == ST_TRIPPED) r.text = PSTR("TRIPPED");
  else if (!armed_) r.text = PSTR("NOT_ARMED");
  else if (!gateOpen) r.text = PSTR("HARDWARE_LOCKED");
  else if (!m_.voltageOk()) r.text = PSTR("MEAS_INVALID");
  else if (tempSensorFault()) r.text = PSTR("TEMP_INVALID");
  else r.ok = true;
  return r;
}

Controller::Result Controller::cmdArm() {
  Result r = {false, PSTR("NONE")};
  if (state_ == ST_TRIPPED) { r.text = PSTR("TRIPPED"); return r; }
  if (armed_) { r.ok = true; r.text = PSTR("ALREADY"); return r; }
  if (!m_.voltageOk()) { r.text = PSTR("MEAS_INVALID"); return r; }
  if (powerDir_ != DIR_NONE || powerPhase_ != PP_IDLE || o_.relay) {
    r.text = PSTR("WAIT_RELAY"); return r;
  }
  if (state_ == ST_LOCKED) { r.text = PSTR("HARDWARE_LOCKED"); return r; }
  if (state_ != ST_READY) { r.text = PSTR("BAD_STATE"); return r; }
  armed_ = true;
  hbValid_ = true;
  lastHeartbeatMs_ = now_;
  queueEvent(PSTR("ARM"), 0, 1, 1, PSTR("HOST"));
  updateState();
  r.ok = true;
  return r;
}

Controller::Result Controller::cmdOff(PStr reason) {
  allOff(reason);
  if (armed_) {
    armed_ = false;
    queueEvent(PSTR("DISARM"), 0, 0, 0, reason);
  }
  updateState();
  Result r = {true, PSTR("NONE")};
  return r;
}

Controller::Result Controller::cmdReset() {
  Result r = {true, PSTR("NONE")};
  if (state_ != ST_TRIPPED) { r.text = PSTR("NOT_TRIPPED"); return r; }
  if (!resetConditionsOk()) { r.ok = false; r.text = PSTR("RESET_BLOCKED"); return r; }
  trip_ = TRIP_NONE;
  queueEvent(PSTR("RESET"), 0, 1, 1, PSTR("HOST"));
  setState(desiredState());  // READY or LOCKED; RESET never re-arms
  return r;
}

Controller::Result Controller::cmdRamp(uint16_t cellTargetMv) {
  Result r = {false, PSTR("NONE")};
  if (cellTargetMv < cfg::RAMP_TARGET_MIN_MV || cellTargetMv > cfg::RAMP_TARGET_MAX_MV) {
    r.text = PSTR("BAD_ARGS"); return r;
  }
  if (powerDir_ != DIR_NONE || powerPhase_ != PP_IDLE || o_.relay) {
    r.text = PSTR("CONFLICT"); return r;
  }
  if (!armed_) { r.text = PSTR("NOT_ARMED"); return r; }
  if (m_.packMv() >= (int16_t)(4 * cellTargetMv)) {
    r.text = PSTR("LIMIT"); return r;
  }
  int16_t low = m_.cellMv[0];
  int16_t high = low;
  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    if (m_.cellMv[i] >= (int16_t)cellTargetMv) { r.text = PSTR("LIMIT"); return r; }
    if (m_.cellMv[i] < low) low = m_.cellMv[i];
    if (m_.cellMv[i] > high) high = m_.cellMv[i];
  }
  if (high - low > cfg::RAMP_MAX_SPREAD_MV) {
    r.text = PSTR("IMBALANCE"); return r;
  }
  rampTargetMv_ = (int16_t)cellTargetMv;
  rampNextCheckpointMv_ =
      (int16_t)(((low / cfg::RAMP_CHECKPOINT_STEP_MV) + 1) * cfg::RAMP_CHECKPOINT_STEP_MV);
  r = cmdPower(DIR_CHARGE, true);
  if (r.ok) queueEvent(PSTR("RAMP_TARGET"), 0, (int16_t)cellTargetMv, 0, PSTR("MV_PER_CELL"));
  else rampTargetMv_ = 0;
  return r;
}

Controller::Result Controller::cmdDrain(uint16_t cellTargetMv) {
  Result r = {false, PSTR("NONE")};
  if (cellTargetMv < cfg::DRAIN_TARGET_MIN_MV || cellTargetMv > cfg::DRAIN_TARGET_MAX_MV) {
    r.text = PSTR("BAD_ARGS"); return r;
  }
  if (powerDir_ != DIR_NONE || powerPhase_ != PP_IDLE || o_.relay) {
    r.text = PSTR("CONFLICT"); return r;
  }
  if (!armed_) { r.text = PSTR("NOT_ARMED"); return r; }
  int16_t low = m_.cellMv[0];
  int16_t high = low;
  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    if (m_.cellMv[i] <= (int16_t)cellTargetMv) { r.text = PSTR("LIMIT"); return r; }
    if (m_.cellMv[i] < low) low = m_.cellMv[i];
    if (m_.cellMv[i] > high) high = m_.cellMv[i];
  }
  if (high - low > cfg::RAMP_MAX_SPREAD_MV) {
    r.text = PSTR("IMBALANCE"); return r;
  }
  drainTargetMv_ = (int16_t)cellTargetMv;
  r = cmdPower(DIR_DISCHARGE, true);
  if (r.ok) queueEvent(PSTR("DRAIN_TARGET_SET"), 0, (int16_t)cellTargetMv, 0, PSTR("MV_PER_CELL"));
  else drainTargetMv_ = 0;
  return r;
}

Controller::Result Controller::cmdPower(uint8_t dir, bool on) {
  Result r = {true, PSTR("NONE")};
  if (!on) {
    if (powerDir_ == dir) stopPower(PSTR("HOST"));
    else r.text = PSTR("ALREADY");
    return r;
  }
#if CG_FIRST_CHARGE_TEST_BUILD
  if ((dir != DIR_CHARGE && dir != DIR_DISCHARGE) ||
      (dir == DIR_CHARGE && (rampTargetMv_ == 0 ||
                            bootMosOnMs_ >= cfg::RAMP_BOOT_ON_BUDGET_MS)) ||
      (dir == DIR_DISCHARGE && (drainTargetMv_ == 0 ||
                               bootDrainOnMs_ >= cfg::DRAIN_BOOT_ON_BUDGET_MS))) {
    r.ok = false;
    r.text = PSTR("HARDWARE_LOCKED");
    return r;
  }
  // Only explicitly targeted, pulse-controlled RAMP and DRAIN are permitted.
  // All unrelated high-power or heater commands remain hardware-locked.
  r = checkActuationAllowed(true);
  if (r.ok) {
    if (m_.packMv() > cfg::STAGE_START_PACK_MAX_MV ||
        m_.packMv() < cfg::STAGE_START_PACK_MIN_MV) {
      r.ok = false;
      r.text = PSTR("LIMIT");
    }
    for (uint8_t i = 0; i < CELL_COUNT; i++) {
      if (m_.tempQ[i] != TQ_VALID ||
          m_.cellMv[i] < cfg::STAGE_START_CELL_MIN_MV ||
          m_.cellMv[i] > cfg::STAGE_START_CELL_MAX_MV) {
        r.ok = false;
        r.text = PSTR("MEAS_INVALID");
        break;
      }
    }
  }
#else
  r = checkActuationAllowed(gates_.powerUnlocked());
#endif
  if (!r.ok) return r;
  r.ok = false;
  if (powerDir_ == dir) { r.ok = true; r.text = PSTR("ALREADY"); return r; }
  if (powerDir_ != DIR_NONE || o_.benchChannel != BENCH_NONE) { r.text = PSTR("CONFLICT"); return r; }
  if (powerOffValid_ && lastPowerDir_ != dir && now_ - lastPowerOffMs_ < cfg::DEADTIME_MS) {
    r.text = PSTR("DEADTIME");
    return r;
  }
  if (dir == DIR_CHARGE) {
    bool atLimit = m_.packMv() >= cfg::PACK_CHARGE_STOP_MV;
    for (uint8_t i = 0; i < CELL_COUNT; i++) atLimit = atLimit || m_.cellMv[i] >= cfg::CELL_CHARGE_STOP_MV;
    if (atLimit) { r.text = PSTR("LIMIT"); return r; }
  } else {
    if (m_.packMv() <= cfg::PACK_DISCHARGE_STOP_MV) {
      r.text = PSTR("LIMIT"); return r;
    }
    for (uint8_t i = 0; i < CELL_COUNT; i++) {
      if (m_.cellMv[i] <= cfg::CELL_DISCHARGE_STOP_MV) { r.text = PSTR("LIMIT"); return r; }
    }
  }
  powerDir_ = dir;
  r.ok = true;
  if (o_.relay) {
    // Relay still closed from a stop moments ago (same direction): MOSFET on now.
    powerPhase_ = PP_CLOSING;
    powerPhaseMs_ = now_ - cfg::RELAY_SETTLE_MS;
    runTimers(now_);
  } else {
    o_.relay = 1;
    powerPhase_ = PP_CLOSING;
    powerPhaseMs_ = now_;
    queueEvent(PSTR("RELAY_ON"), 0, 1, 1, PSTR("HOST"));
    applyOutputs();
    r.text = PSTR("PENDING");
  }
  updateState();
  return r;
}

Controller::Result Controller::cmdEsr(uint8_t cell, bool on) {
  uint8_t idx = (uint8_t)(cell - 1);
  uint8_t bit = (uint8_t)(1u << idx);
  Result r = {true, PSTR("NONE")};
  if (!on) {
    if (o_.esrMask & bit) stopEsr(idx, PSTR("HOST"));
    else r.text = PSTR("ALREADY");
    return r;
  }
  r = checkActuationAllowed(gates_.esrUnlocked());
  if (!r.ok) return r;
  if (o_.esrMask & bit) { r.text = PSTR("ALREADY"); return r; }
  o_.esrMask |= bit;
  esrStartMs_[idx] = now_;
  queueEvent(PSTR("ESR_START"), cell, 1, 1, PSTR("HOST"));
  applyOutputs();
  updateState();
  return r;
}

Controller::Result Controller::cmdLeak(uint8_t cell, bool on) {
  uint8_t idx = (uint8_t)(cell - 1);
  uint8_t bit = (uint8_t)(1u << idx);
  Result r = {true, PSTR("NONE")};
  if (!on) {
    if (o_.leakMask & bit) stopLeak(idx, PSTR("HOST"));
    else r.text = PSTR("ALREADY");
    return r;
  }
  r = checkActuationAllowed(gates_.leakUnlocked());
  if (!r.ok) return r;
  if (o_.leakMask & bit) { r.text = PSTR("ALREADY"); return r; }
  if (popcount4(o_.leakMask) >= cfg::MAX_LEAK_CELLS) { r.ok = false; r.text = PSTR("CONCURRENCY"); return r; }
  o_.leakMask |= bit;
  leakStartMs_[idx] = now_;
  queueEvent(PSTR("LEAK_START"), cell, 1, 1, PSTR("HOST"));
  applyOutputs();
  updateState();
  return r;
}

Controller::Result Controller::cmdHeat(uint8_t cell, uint8_t pwm) {
  uint8_t idx = (uint8_t)(cell - 1);
  Result r = {true, PSTR("NONE")};
  if (pwm == 0) {
    if (o_.heat[idx]) stopHeat(idx, PSTR("HOST"));
    else r.text = PSTR("ALREADY");
    return r;
  }
  r = checkActuationAllowed(gates_.heatUnlocked());
  if (!r.ok) return r;
  r.ok = false;
  if (pwm > cfg::MAX_HEAT_PWM) { r.text = PSTR("CONCURRENCY"); return r; }
  if (m_.tempQ[idx] != TQ_VALID) { r.text = PSTR("TEMP_INVALID"); return r; }
  if (m_.tempCc[idx] >= cfg::HEAT_SOFT_OFF_CC) { r.text = PSTR("LIMIT"); return r; }
  uint8_t others = 0;
  for (uint8_t i = 0; i < CELL_COUNT; i++)
    if (i != idx && o_.heat[i]) others++;
  if (others >= cfg::MAX_HEAT_CELLS) { r.text = PSTR("CONCURRENCY"); return r; }
  r.ok = true;
  if (o_.heat[idx] == pwm) { r.text = PSTR("ALREADY"); return r; }
  if (o_.heat[idx] == 0) {
    heatStartMs_[idx] = now_;  // changing the duty later never extends the time limit
    queueEvent(PSTR("HEAT_START"), cell, pwm, pwm, PSTR("HOST"));
  } else {
    queueEvent(PSTR("HEAT_SET"), cell, pwm, pwm, PSTR("HOST"));
  }
  o_.heat[idx] = pwm;
  applyOutputs();
  updateState();
  return r;
}

Controller::Result Controller::cmdBench(uint8_t channel, bool on) {
  Result r = {true, PSTR("NONE")};
  if (!on) {
    if (o_.benchChannel == channel) stopBench(PSTR("HOST"));
    else r.text = PSTR("ALREADY");
    return r;
  }
  r.ok = false;
  if (state_ == ST_TRIPPED) { r.text = PSTR("TRIPPED"); return r; }
  if (!armed_) { r.text = PSTR("NOT_ARMED"); return r; }
  if (o_.benchChannel == channel) { r.ok = true; r.text = PSTR("ALREADY"); return r; }
  if (o_.benchChannel != BENCH_NONE) { r.text = PSTR("CONCURRENCY"); return r; }
  OutputState probe = o_;
  probe.benchChannel = BENCH_NONE;
  if (probe.anyOn() || powerDir_ != DIR_NONE) { r.text = PSTR("CONFLICT"); return r; }
  o_.benchChannel = channel;
  benchStartMs_ = now_;
  queueEvent(PSTR("BENCH_ON"), 0, channel, 1, PSTR("HOST"));
  applyOutputs();
  updateState();
  r.ok = true;
  return r;
}

// ---- TX ------------------------------------------------------------------------------

void Controller::sendAck(int32_t seq, bool ok, PStr text) {
  fb_.start(PSTR("CGA"));
  fb_.addI(seq);
  fb_.addP(ok ? PSTR("OK") : PSTR("ERR"));
  fb_.addP(text);
  fb_.send(hal_);
}

void Controller::sendCgt() {
  fb_.start(PSTR("CGT"));
  fb_.addU(telSeq_);
  fb_.addU(m_.tMs);
  for (uint8_t i = 0; i < CELL_COUNT; i++) fb_.addI(m_.cellMv[i]);
  for (uint8_t i = 0; i < CELL_COUNT; i++) fb_.addI(m_.tempQ[i] == TQ_VALID ? m_.tempCc[i] : cfg::INVALID_I16);
  fb_.addU((o_.charge || o_.benchChannel == 9) ? 1 : 0);
  fb_.addU((o_.discharge || o_.benchChannel == 10) ? 1 : 0);
  fb_.addU(armed_ ? 1 : 0);
  fb_.addU(o_.esrMask);
  fb_.addU(o_.leakMask);
  for (uint8_t i = 0; i < CELL_COUNT; i++) fb_.addU(o_.heat[i]);
  fb_.addU(trip_);
  fb_.addU(m_.present ? m_.vq : (uint8_t)VQ_STALE);
  fb_.addU(m_.tempQualityMask());
  fb_.send(hal_);
}

void Controller::sendCgs() {
  uint8_t lockMask = 0;
  if (!gates_.powerUnlocked()) lockMask |= 1u << 0;
  if (!gates_.esrUnlocked()) lockMask |= 1u << 1;
  if (!gates_.leakUnlocked()) lockMask |= 1u << 2;
  if (!gates_.heatUnlocked()) lockMask |= 1u << 3;
  if (!m_.voltageOk()) lockMask |= 1u << 4;
  if (!synced_) lockMask |= 1u << 5;
  if (tempSensorFault()) lockMask |= 1u << 6;
  uint32_t hbAge = 999999UL;
  if (hbValid_ && now_ - lastHeartbeatMs_ < hbAge) hbAge = now_ - lastHeartbeatMs_;

  fb_.start(PSTR("CGS"));
  fb_.addU(telSeq_);
  fb_.addU(now_);
  fb_.addP(stateName(state_));
  fb_.addU(trip_);
  fb_.addU((o_.relay || o_.benchChannel == 8) ? 1 : 0);
  fb_.addU(lockMask);
  fb_.addU(hbAge);
  fb_.addI(synced_ ? lastSeq_ : -1);
  fb_.addU(rxOk_);
  fb_.addU(rxCrcErr_);
  fb_.addU(rxFormatErr_);
  fb_.addU(rxOverflow_);
  fb_.addU(resetFlags_);
  fb_.addU(loopOverruns_);
  fb_.addU(hal_.freeRam());
  fb_.addU(m_.bandgapRaw);
  fb_.send(hal_);
}

void Controller::sendCgd(int32_t seq) {
  fb_.start(PSTR("CGD"));
  fb_.addI(seq);
  for (uint8_t ch = 0; ch < adc::CHANNELS; ch++) fb_.addU(m_.raw[ch]);
  fb_.send(hal_);
}

void Controller::sendCgk(int32_t seq) {
  fb_.start(PSTR("CGK"));
  fb_.addI(seq);
  fb_.addP(cal_.measured ? PSTR("MEASURED") : PSTR("NOMINAL"));
  fb_.addP(cal_.id);
  fb_.addU(cal_.arefMv);
  fb_.addU(cal_.v5Mv);
  for (uint8_t i = 0; i < 4; i++) fb_.addI(cal_.gainPpm[i]);
  for (uint8_t i = 0; i < 4; i++) fb_.addI(cal_.offsetMv[i]);
  fb_.addU(cal_.ntcB);
  fb_.addU(cal_.ntcR25);
  fb_.send(hal_);
}

void Controller::sendCgv() {
  fb_.start(PSTR("CGV"));
  fb_.addP(PSTR(CG_FW_VERSION));
  fb_.addP(PSTR(CG_PROTOCOL_VERSION));
  fb_.addP(PSTR(CG_MAP_VERSION));
  fb_.addU(gates_.cgvFlags());
  fb_.send(hal_);
}

void Controller::queueEvent(PStr code, uint8_t cell, int16_t requested, int16_t commanded, PStr reason) {
  // Events are held while a command is processed so the CGA goes out first.
  // A full queue is flushed immediately rather than dropping events.
  if (eventCount_ >= EVENT_QUEUE) flushEvents();
  Event& e = events_[eventCount_++];
  e.tMs = now_;
  e.code = code;
  e.reason = reason;
  e.cell = cell;
  e.requested = requested;
  e.commanded = commanded;
}

void Controller::flushEvents() {
  for (uint8_t i = 0; i < eventCount_; i++) {
    const Event& e = events_[i];
    fb_.start(PSTR("CGE"));
    fb_.addU(eventSeq_++);
    fb_.addU(e.tMs);
    fb_.addP(e.code);
    fb_.addU(e.cell);
    fb_.addI(e.requested);
    fb_.addI(e.commanded);
    fb_.addP(e.reason);
    fb_.send(hal_);
  }
  eventCount_ = 0;
}

}  // namespace cg
