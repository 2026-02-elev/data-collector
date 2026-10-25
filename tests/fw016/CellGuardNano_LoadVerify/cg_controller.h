// CellGuard Nano firmware - safety state machine, command handling, telemetry
// Author: Yongho Ahn <nanhosoft@gmail.com>
//
// State machine (docs/safety_states.md):
//   BOOT_SAFE -> SELF_TEST -> LOCKED / READY -> ARMED -> ACTIVE
//   any -> TRIPPED (latched; RESET -> READY/LOCKED when conditions are normal)
// Local protection never depends on the Pi or the cloud.
#pragma once
#include <stdint.h>

#include "cg_config.h"
#include "cg_hal.h"
#include "cg_measure.h"
#include "cg_outputs.h"
#include "cg_protocol.h"

namespace cg {

enum State : uint8_t {
  ST_BOOT_SAFE = 0,
  ST_SELF_TEST = 1,
  ST_LOCKED = 2,
  ST_READY = 3,
  ST_ARMED = 4,
  ST_ACTIVE = 5,
  ST_TRIPPED = 6,
};

enum TripCode : uint8_t {
  TRIP_NONE = 0,
  TRIP_PACK_OV = 1,
  TRIP_CELL_OV = 2,
  TRIP_OVER_TEMP = 3,
  TRIP_HEARTBEAT_TIMEOUT = 4,
  TRIP_ADC_SATURATED = 5,
  TRIP_TAP_INVERSION = 6,
  TRIP_AREF_INVALID = 7,
  TRIP_HOST_TRIP = 8,
  TRIP_WATCHDOG_RESET = 9,
  TRIP_BROWNOUT_RESET = 10,
  TRIP_STALE_SAMPLE = 11,
  TRIP_INTERNAL = 12,
  TRIP_CHARGE_NO_RESPONSE = 13,  // pack did not rise after charge start (tap path open?)
  TRIP_TAP_STEP = 14,            // a tap fell abruptly while charging (harness lost?)
  TRIP_TEMP_SENSOR_FAULT = 15,   // NTC short / invalid while an actuator is on
  TRIP_CELL_IMBALANCE = 16,      // cells separated by >80mV during charge
  TRIP_DISCHARGE_NO_RESPONSE = 17,
};

// MCUSR bits as reported in CGS.resetFlags.
enum ResetFlag : uint8_t {
  RESET_POWER_ON = 1u << 0,
  RESET_EXTERNAL = 1u << 1,
  RESET_BROWNOUT = 1u << 2,
  RESET_WATCHDOG = 1u << 3,
};

// Hardware verification gates (hardware_verification.h). Host tests build
// their own instances; the firmware uses compiledGates().
struct Gates {
  bool mapVerified;
  bool thermalProtectionVerified;
  bool ina219Declared;
  bool packRelayVerified;
  bool esrRelayVerified;
  bool powerMosVerified;
  bool r17Verified;
  bool ntcRevised;
  bool benchBuild;
  bool hostSimBuild;

  // Charging is only allowed when the voltage measurement path of every cell
  // (8-pin harness -> 47k/43k tap) is verified, too: an unconnected tap reads 0 V.
  bool powerUnlocked() const { return mapVerified && packRelayVerified && powerMosVerified && r17Verified; }
  bool esrUnlocked() const { return mapVerified && esrRelayVerified; }
  bool leakUnlocked() const { return mapVerified; }
  bool heatUnlocked() const { return mapVerified && thermalProtectionVerified; }
  bool anyUnlocked() const { return powerUnlocked() || esrUnlocked() || leakUnlocked() || heatUnlocked(); }
  uint16_t cgvFlags() const;
};

void compiledGates(Gates& g);

PStr stateName(uint8_t s);
PStr tripName(uint8_t code);

class Controller {
 public:
  Controller(Hal& hal, const Gates& gates, const Calibration& cal);

  // Call once after the pins were put in their safe levels.
  void begin(uint8_t resetFlags);
  // Call continuously from loop(). Never blocks for long.
  void poll();

  // Read-only accessors (tests, diagnostics).
  State state() const { return state_; }
  uint8_t trip() const { return trip_; }
  bool armed() const { return armed_; }
  bool synced() const { return synced_; }
  const OutputState& outputs() const { return o_; }
  const Measurement& measurement() const { return m_; }

 private:
  enum PowerPhase : uint8_t { PP_IDLE, PP_CLOSING, PP_ON, PP_OPENING };
  enum PowerDir : uint8_t { DIR_NONE = 0, DIR_CHARGE = 1, DIR_DISCHARGE = 2 };

  struct Result {
    bool ok;
    PStr text;
  };
  struct Event {
    uint32_t tMs;
    PStr code;
    PStr reason;
    uint8_t cell;
    int16_t requested;
    int16_t commanded;
  };
  static const uint8_t EVENT_QUEUE = 12;

  // RX / commands
  void handleRxByte(char c);
  void handleFrame(char* payload);
  Result execute(const Command& c);
  void sendExtras(const Command& c);
  Result cmdArm();
  Result cmdOff(PStr reason);
  Result cmdReset();
  Result cmdPower(uint8_t dir, bool on);
  Result cmdRamp(uint16_t cellTargetMv);
  Result cmdDrain(uint16_t cellTargetMv);
  Result cmdEsr(uint8_t cell, bool on);
  Result cmdLeak(uint8_t cell, bool on);
  Result cmdHeat(uint8_t cell, uint8_t pwm);
  Result cmdBench(uint8_t channel, bool on);
  Result checkActuationAllowed(bool gateOpen) const;

  // Measurement and protection
  void acquire(uint32_t now);
  uint16_t readAveraged(uint8_t channel, uint8_t discard);
  void evaluateSafety();
  void runTimers(uint32_t now);
#if CG_FIRST_CHARGE_TEST_BUILD
  void serviceFirstChargeQuiet(uint32_t now);
  void checkFirstChargeLoaded(uint32_t now);
  void sendLoadTapSuspect(uint32_t atMs, uint8_t direction,
                          const uint16_t raw[CELL_COUNT], uint16_t bandgap);
#endif
  void finishSelfTest();
  bool resetConditionsOk() const;
  bool tempSensorFault() const;  // a distinguishable NTC fault (short / invalid) on any cell
  bool actuatorsOn() const { return powerDir_ != DIR_NONE || o_.esrMask || o_.leakMask || o_.heat[0] || o_.heat[1] || o_.heat[2] || o_.heat[3]; }

  // Outputs
  void allOff(PStr reason);
  void stopPower(PStr reason);
  void stopEsr(uint8_t idx, PStr reason);
  void stopLeak(uint8_t idx, PStr reason);
  void stopHeat(uint8_t idx, PStr reason);
  void stopBench(PStr reason);
  void tripNow(uint8_t code);
  void applyOutputs() { out_.apply(o_); }
  State desiredState() const;  // state implied by armed_/outputs/measurement/gates
  void updateState();
  void setState(State s);

  // TX
  void sendAck(int32_t seq, bool ok, PStr text);
  void sendCgt();
  void sendCgs();
  void sendCgd(int32_t seq);
  void sendCgk(int32_t seq);
  void sendCgv();
  void queueEvent(PStr code, uint8_t cell, int16_t requested, int16_t commanded, PStr reason);
  void flushEvents();

  Hal& hal_;
  OutputDriver out_;
  Gates gates_;
  Calibration cal_;
  RxAssembler rx_;
  FrameBuilder fb_;
  Measurement m_;
  OutputState o_;

  State state_;
  uint8_t trip_;
  uint8_t resetFlags_;
  bool armed_;
  bool synced_;
  bool inCommand_;
  int32_t lastSeq_;

  bool ackValid_;
  bool ackOk_;
  int32_t ackSeq_;
  PStr ackText_;

  bool hbValid_;
  uint32_t lastHeartbeatMs_;

  uint8_t powerPhase_;
  uint8_t powerDir_;
  uint8_t lastPowerDir_;
  bool powerOffValid_;
  uint32_t powerPhaseMs_;
  uint32_t lastPowerOffMs_;
  uint32_t chargeStartMs_;
  int16_t chargeStartPackMv_;
#if CG_FIRST_CHARGE_TEST_BUILD
  int16_t chargeStartCellMv_[CELL_COUNT];
  int16_t rampTargetMv_;          // host-requested RESTED cell voltage
  int16_t drainTargetMv_;         // host-requested RESTED lower voltage
  int16_t rampNextCheckpointMv_;  // minimum-cell voltage milestone
  uint32_t bootMosOnMs_;         // summed for every RAMP since MCU reset
  uint32_t bootDrainOnMs_;
  int16_t dischargeStartCellMv_[CELL_COUNT];
#endif
  bool chargeResponseOk_;
#if CG_FIRST_CHARGE_TEST_BUILD
  uint32_t chargePulseSampleMs_;     // last *rested* ADC sample used to start one MOS pulse
  uint32_t chargeMosOnMs_;           // independent hardware-on timer
  uint32_t chargeQuietSinceMs_;      // off-time prior to trusted ADC measurement
  uint32_t chargeLastLoadedCheckMs_;
  bool loadTapVerifyPending_;      // MOS forcibly OFF until quiet verdict
  uint16_t chargeTotalMosOnMs_;
  uint16_t chargePulseCount_;
#endif
  int16_t prevTapMv_[CELL_COUNT];
  bool prevTapValid_;
  uint32_t dischargeStartMs_;
  uint32_t esrStartMs_[CELL_COUNT];
  uint32_t leakStartMs_[CELL_COUNT];
  uint32_t heatStartMs_[CELL_COUNT];
  uint32_t benchStartMs_;

  uint32_t now_;
  uint32_t lastSampleMs_;
  uint8_t samplesTaken_;
  uint32_t lastTelemetryMs_;
  uint16_t telSeq_;
  uint16_t eventSeq_;

  uint16_t rxOk_;
  uint16_t rxCrcErr_;
  uint16_t rxFormatErr_;
  uint16_t rxOverflow_;
  uint16_t loopOverruns_;
  uint16_t seqGaps_;

  Event events_[EVENT_QUEUE];
  uint8_t eventCount_;
};

}  // namespace cg
