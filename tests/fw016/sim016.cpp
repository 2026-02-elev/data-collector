// CellGuard 0.1.6-LOAD-VERIFY host simulator - the UNMODIFIED firmware controller behind a line protocol
// Author: Yongho Ahn <nanhosoft@gmail.com>
//
// SIMULATED DEVICE for the data-collector tests only. The controller code is
// compiled from ./CellGuardNano_LoadVerify (copied byte-for-byte from the
// firmware actually used on the rig, see SHA256SUMS) with CG_HOST_SIM_BUILD=1,
// so CGV.flags bit 9 is set and nothing produced here can be labelled measured.
// The plant (5 F cells, 22 ohm charge path from a 5.26 V rail, 10 ohm drain
// resistor, loaded-tap offsets) follows validation/load_verify_smoke.cpp that
// ships with that firmware. It is NOT an electrical model of the real board.
//
// Same stdin protocol as arudino-firmware/tests/host/nano_host_sim:
//   ADV <ms>                    advance simulated time (5 ms steps, plant + poll)
//   TX <payload> / TXRAW <text> bytes to the Nano
//   CELLS <c1> <c2> <c3> <c4>   stored (rested) cell voltages in mV
//   INJECT LOADED_INVERSION 0|1 while draining, tap 3 reads 612 mV (2026-10-25 capture)
//   INJECT REST_INVERSION 0|1   ... and stays inverted after the MOSFET is off (real fault)
//   INJECT NO_CHARGE <cell>     that cell receives no charge (open charge path)
//   LINK DOWN|UP                drop every byte in both directions (cable pulled)
//   QUIT
// stdout: every frame the Nano sends, then "#DONE" after each input line.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

#include "cg_controller.h"
#include "cg_crc16.h"
#include "pinmap.h"

class PlantHal : public cg::Hal {
 public:
  uint32_t now = 0;
  bool relay = false, charge = false, drain = false, linkUp = true;
  bool loadedInversion = false, restInversion = false, injectedLive = false;
  uint16_t bandgap = 275;  // 1.1 V against REF3040
  uint16_t ntcRaw = 624;   // a valid room temperature with REF3040
  double storedMv[4] = {126, 134, 134, 142};  // post-0.1.4 bench readings used by the firmware's own tests
  bool cellCharges[4] = {true, true, true, true};
  double chargeLoadedExtra[4] = {42, 100, 319, 100};
  std::string tx, rx;

  uint32_t millis() override { return now; }
  void pinWrite(uint8_t pin, uint8_t high) override {
    if (pin == cg::pin::CHARGE_PMOS) charge = (high == 0);
    if (pin == cg::pin::DISCHARGE_NMOS) drain = high != 0;
    if (pin == cg::pin::PACK_RELAY) relay = high != 0;
  }
  void pwmWrite(uint8_t, uint8_t) override {}
  uint16_t adcConvert(uint8_t ch) override {
    if (ch == cg::adc::BANDGAP) return bandgap;
    if (ch < 4) {
      double pack = storedMv[0] + storedMv[1] + storedMv[2] + storedMv[3];
      double cRatio = (5260.0 - pack) / 5260.0;
      if (cRatio < 0) cRatio = 0;
      double mv = 0;
      for (uint8_t i = 0; i <= ch; ++i)
        mv += storedMv[i] + (charge && relay ? chargeLoadedExtra[i] * cRatio : 0) - (drain && relay ? 8 : 0);
      if (drain && loadedInversion && ch == 2) {
        mv = 612.0;
        injectedLive = true;
      }
      if (!(charge || drain) && restInversion && injectedLive && ch == 2) mv = 620.0;
      double raw = mv * 43000.0 / 90000.0 * 1023.0 / 4096.0;
      if (raw < 0) raw = 0;
      if (raw > 1023) raw = 1023;
      return (uint16_t)std::lround(raw);
    }
    if (ch < 8) return ntcRaw;
    return 0;
  }
  int rxRead() override {
    if (rx.empty()) return -1;
    int c = (unsigned char)rx[0];
    rx.erase(0, 1);
    return c;
  }
  void txWrite(const char* data, size_t n) override {
    if (linkUp) tx.append(data, n);
  }
  void watchdogKick() override {}
  uint16_t freeRam() override { return 700; }

  void plant(uint16_t delta) {
    double pack = storedMv[0] + storedMv[1] + storedMv[2] + storedMv[3];
    if (charge && relay) {
      double ma = (5260.0 - pack) / 22.0;
      if (ma < 0) ma = 0;
      for (uint8_t i = 0; i < 4; ++i)
        if (cellCharges[i]) storedMv[i] += ma * delta / 5000.0;
    }
    if (drain && relay) {
      double ma = pack / 10.0;
      for (uint8_t i = 0; i < 4; ++i) storedMv[i] -= ma * delta / 5000.0;
    }
  }
};

static std::string frameFor(const std::string& payload) {
  char crc[8];
  std::snprintf(crc, sizeof(crc), "%04X", cg::crc16(payload.data(), payload.size()));
  return "$" + payload + "*" + crc + "\n";
}

int main(int argc, char** argv) {
  uint8_t reset = cg::RESET_POWER_ON;
  for (int i = 1; i + 1 < argc; i++) {  // --gates is accepted and ignored (compiled gates are used)
    if (!std::strcmp(argv[i], "--reset")) {
      std::string r = argv[++i];
      reset = r == "wdt" ? cg::RESET_WATCHDOG : r == "ext" ? cg::RESET_EXTERNAL : cg::RESET_POWER_ON;
    }
  }
  cg::Calibration cal;
  cg::activeCalibration(cal);
  cg::Gates gates;
  cg::compiledGates(gates);
  PlantHal hal;
  cg::Controller ctl(hal, gates, cal);
  ctl.begin(reset);

  auto out = [&]() {
    size_t pos;
    while ((pos = hal.tx.find('\n')) != std::string::npos) {
      std::string line = hal.tx.substr(0, pos);
      hal.tx.erase(0, pos + 1);
      if (!line.empty() && line.back() == '\r') line.pop_back();
      std::cout << line << "\n";
    }
  };
  out();
  std::cout << "#DONE" << std::endl;

  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream in(line);
    std::string op;
    in >> op;
    if (op == "QUIT") break;
    if (op == "ADV") {
      long ms = 0;
      in >> ms;
      uint32_t end = hal.now + (uint32_t)ms;
      for (;;) {
        ctl.poll();
        out();
        if (hal.now >= end) break;
        hal.plant(5);
        hal.now += 5;
      }
    } else if (op == "TX" || op == "TXRAW") {
      std::string rest = line.size() > op.size() + 1 ? line.substr(op.size() + 1) : "";
      std::string bytes = op == "TX" ? frameFor(rest) : rest + "\n";
      if (hal.linkUp) hal.rx += bytes;
      do {  // the firmware reads at most 64 bytes per poll
        ctl.poll();
        out();
      } while (!hal.rx.empty());
    } else if (op == "CELLS") {
      for (int i = 0; i < 4; i++) in >> hal.storedMv[i];
    } else if (op == "INJECT") {
      std::string what;
      int v = 0;
      in >> what >> v;
      if (what == "LOADED_INVERSION") hal.loadedInversion = v != 0;
      else if (what == "REST_INVERSION") hal.restInversion = v != 0;
      else if (what == "NO_CHARGE" && v >= 1 && v <= 4) hal.cellCharges[v - 1] = false;
      else std::cout << "#ERROR unknown injection: " << what << "\n";
    } else if (op == "LINK") {
      std::string s;
      in >> s;
      hal.linkUp = s != "DOWN";
    } else if (!op.empty()) {
      std::cout << "#ERROR unknown input: " << op << "\n";
    }
    std::cout << "#DONE" << std::endl;
  }
  return 0;
}
