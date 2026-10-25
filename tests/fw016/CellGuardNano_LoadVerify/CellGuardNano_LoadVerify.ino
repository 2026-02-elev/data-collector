// CellGuard 0.1.6: target-controlled charge/discharge + loaded tap verification.
// A loaded tap inversion immediately cuts both power MOSFETs; a subsequent
// rested sample determines latched TRIP vs. aborted/unarmed test stage.
// Not for unattended or full-rated-voltage use; see README_LOAD_VERIFY.md.
#include "cg_config.h"
#include "cg_controller.h"
#include "cg_hal_avr.h"
#include "cg_measure.h"

static cg::AvrHal hal;
static cg::Gates gates;
static cg::Calibration calibration;
static cg::Controller* controller = nullptr;

void setup() {
  hal.begin();
  cg::compiledGates(gates);
  cg::activeCalibration(calibration);
  static cg::Controller instance(hal, gates, calibration);
  controller = &instance;
  controller->begin(hal.resetFlags());
  hal.watchdogStart();
}

void loop() { controller->poll(); }
