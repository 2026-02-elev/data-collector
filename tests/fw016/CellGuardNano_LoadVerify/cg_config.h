// CellGuard Nano firmware - versions, thresholds and timing
// Author: Yongho Ahn <nanhosoft@gmail.com>
//
// Thresholds are the INITIAL PROPOSED values of the firmware spec section 10.
// They are not approval for unattended operation.
#pragma once
#include <stdint.h>

// Commissioning-only build. Runtime RAMP targets replace repeated firmware
// flashing, but do NOT lift the hardware-verified limits of this board.
#define CG_FIRST_CHARGE_TEST_BUILD 1
#define CG_FW_VERSION "0.1.6-LOAD-VERIFY"
#define CG_PROTOCOL_VERSION "1.0"
#define CG_MAP_VERSION "PCB2-20260918-R2"

#ifndef CG_BENCH_BUILD
#define CG_BENCH_BUILD 0  // 1 = enables the BENCH single-output test command
#endif

namespace cg {
namespace cfg {

// ---- Timing ----
static const uint16_t SAMPLE_PERIOD_MS = 200;      // 5 Hz measurement + safety loop
static const uint16_t STALE_SAMPLE_MS = 600;       // > 3 periods without a sample -> STALE
// 5 F supercaps can have sizeable instantaneous ESR voltage at ~0.2 A.
// Keep the MOSFET OFF for 80 ms before each 200 ms ADC measurement.
static const uint16_t FIRST_CHARGE_QUIET_MS = 80;
static const uint16_t FIRST_CHARGE_MAX_ON_MS = SAMPLE_PERIOD_MS - FIRST_CHARGE_QUIET_MS;  // 120 ms
static const uint16_t FIRST_CHARGE_OVERRUN_TRIP_MS = 160;  // missed MOS-off deadline => latched trip
static const uint16_t FIRST_CHARGE_LIVE_CHECK_MS = 45;
// A load-only tap inversion is never ignored or retried automatically: cut
// both MOSFETs at once, then use the NEXT rested sample to distinguish a
// persistent inversion (TRIP) from untrustworthy live sensing (abort stage).
// Independent *loaded* emergency limits. Rested thresholds are checked
// separately every quiet interval. Calibrated hardware safety is NOT implied.
static const int16_t FIRST_CHARGE_LIVE_CELL_HARD_MV = 1800;
static const int16_t FIRST_CHARGE_LIVE_PACK_HARD_MV = 5200;
static const uint16_t TELEMETRY_PERIOD_MS = 1000;  // 1 Hz CGT + CGS
static const uint16_t HEARTBEAT_TIMEOUT_MS = 5000;
static const uint16_t RELAY_SETTLE_MS = 50;        // RLY2 close before MOS on / MOS off before RLY2 open
static const uint16_t DEADTIME_MS = 200;           // charge <-> discharge break-before-make
static const uint8_t SELF_TEST_SAMPLES = 2;

// ---- Maximum on-times (every actuator expires) ----
static const uint32_t CHARGE_MAX_MS = 25000UL;     // individual RAMP command; requires Pi PING every <5 s
static const uint32_t RAMP_BOOT_ON_BUDGET_MS = 60000UL; // accumulated actual MOS-ON time, all sessions
static const uint32_t DISCHARGE_MAX_MS = 30000UL;   // bounded DRAIN command, Pi PING required
static const uint32_t DRAIN_BOOT_ON_BUDGET_MS = 60000UL;
static const uint32_t ESR_MAX_MS = 600000UL;        // 10 min
static const uint32_t LEAK_MAX_MS = 1800000UL;      // 30 min (leak slope needs long windows)
static const uint32_t HEAT_MAX_MS = 300000UL;       // 5 min
static const uint32_t BENCH_MAX_MS = 5000UL;        // bench single output pulse

// ---- Concurrency limits ----
static const uint8_t MAX_LEAK_CELLS = 2;
static const uint8_t MAX_HEAT_CELLS = 1;
static const uint8_t MAX_HEAT_PWM = 128;  // ~50 % duty, 0.27 W per heater (ideal)

// ---- Voltage limits (mV) ----
// Runtime targets are RESTED per-cell mV, never a direct PWM command.
// 1000 mV/cell max deliberately leaves margin below the four-cell pack's
// 5.26 V input rail and below the external-REF ADC's 8.57 V tap range.
// This is NOT a way to charge the 2.7 V rated cells to full voltage.
static const uint16_t RAMP_TARGET_MIN_MV = 150;
static const uint16_t RAMP_TARGET_MAX_MV = 1000;
static const uint16_t DRAIN_TARGET_MIN_MV = 100;
static const uint16_t DRAIN_TARGET_MAX_MV = 950;
static const uint16_t RAMP_CHECKPOINT_STEP_MV = 100;
static const int16_t RAMP_MAX_SPREAD_MV = 80;
static const int16_t PACK_CHARGE_STOP_MV = 4200;  // static, independent of host target
static const int16_t CELL_CHARGE_STOP_MV = 1050;
static const int16_t PACK_TRIP_MV = 4600;
static const int16_t CELL_TRIP_MV = 1200;
static const int16_t STAGE_START_PACK_MAX_MV = 4200;
static const int16_t STAGE_START_CELL_MAX_MV = 1050;
static const int16_t STAGE_START_CELL_MIN_MV = -20;
static const int16_t STAGE_START_PACK_MIN_MV = -20;
static const int16_t CELL_DISCHARGE_STOP_MV = 25; // independent lower floor
static const int16_t PACK_DISCHARGE_STOP_MV = 80;
static const int16_t TAP_INVERSION_TOL_MV = 50;
static const int16_t PACK_NEAR_ZERO_MV = 50;
// Measurement-path plausibility while charging (detects a disconnected 8-pin
// harness / tap divider, which reads ~0 V and would never reach the stop limit).
static const int16_t CHARGE_RESPONSE_BELOW_MV = 1000;   // check only when the pack starts below this
static const int16_t CHARGE_RESPONSE_MIN_RISE_MV = 16;
static const uint16_t CHARGE_RESPONSE_WINDOW_MS = 2000;
static const int16_t STAGE_CELL_MIN_RISE_MV = 8;       // per-cell increase from START, not absolute voltage
static const uint16_t STAGE_CELL_RISE_CHECK_MS = 3000;
static const int16_t DRAIN_CELL_MIN_FALL_MV = 8;
static const uint16_t DRAIN_CELL_FALL_CHECK_MS = 4000;
static const int16_t TAP_STEP_DROP_MV = 80;

// ---- Temperature limits (centi-degC, only applied to VALID readings) ----
static const int16_t HEAT_SOFT_OFF_CC = 5200;
static const int16_t OVER_TEMP_TRIP_CC = 6000;
static const int16_t REARM_MAX_TEMP_CC = 4500;
static const int16_t TEMP_VALID_MIN_CC = -4000;
static const int16_t TEMP_VALID_MAX_CC = 15000;

// ---- ADC ----
static const uint8_t ADC_AVG = 4;           // conversions averaged per channel
static const uint8_t ADC_DISCARD = 1;       // conversions discarded after a mux change
static const uint8_t BANDGAP_DISCARD = 3;   // bandgap needs extra settling
static const uint16_t ADC_SAT_RAW = 1020;   // >= : saturated
static const uint16_t NTC_SHORT_RAW = 5;    // <= : NTC short / abnormal low
// Internal bandgap nominal ~1.1 V measured against external REF3040 (4.096 V):
// 1023 * 1.1 / 4.096 ~= 275 counts. Bounds retain the original broad tolerance
// for ATmega328P bandgap variability; they are a plausibility test, NOT a
// precision verification of the external reference.
static const uint16_t BANDGAP_MIN_RAW = 230;
static const uint16_t BANDGAP_MAX_RAW = 330;
// Tap divider: R_top 47k (tap -> ADC), R_bottom 43k (ADC -> GND).
static const uint32_t TAP_DIV_TOP_OHM = 47000UL;
static const uint32_t TAP_DIV_BOTTOM_OHM = 43000UL;

static const int16_t INVALID_I16 = -32768;  // sentinel for invalid mV / cC fields

}  // namespace cfg
}  // namespace cg
