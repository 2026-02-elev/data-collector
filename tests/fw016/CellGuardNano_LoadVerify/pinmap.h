// CellGuard Nano firmware - pin map (single source of truth)
// Author: Yongho Ahn <nanhosoft@gmail.com>
//
// Evidence tags (spec section 0):
//   N = NETLIST (PCB2 2026-09-18 rev2, reference/netlists/), D = DATASHEET,
//   I = INTENT, H = HARDWARE PENDING (must be verified on the real rig).
//
// The "//@pin" and "//@net" lines are machine-checked against the original
// netlists by tests/python/test_pinmap_netlist.py. Do not edit one without the
// other. "//@pin NAME Dx|Ay U4_1.pad" asserts the Arduino pin and the PCB2 pad of
// the Nano footprint; "//@net A B" asserts that pins A and B share one net.
//
// Arduino logical pin numbers (Dx/Ay) and PCB footprint pads (U4_1.nn) are NOT
// the same numbers. The pad numbering assumes the standard 30-pin Nano footprint
// (pad 1 = D1/TX ... pad 30 = VIN) - H: confirm pin 1 of the real module.
#pragma once
#include <stdint.h>

namespace cg {
namespace pin {

// ---- UART to Raspberry Pi (N) --------------------------------------------
//@pin UART_RX D0 U4_1.2
//@net U4_1.2 U7_1.8
static const uint8_t UART_RX = 0;  // <- Pi header 8 (GPIO14/TXD, 3.3V)
//@pin UART_TX D1 U4_1.1
//@net U4_1.1 R15_1.2
//@net R15_1.1 U7_1.10
// PCB2 netlist calls for R15=1k / R16=2k. A 1.5k replacement for R16 was
// discussed during assembly; confirm the installed value before Pi connection.
static const uint8_t UART_TX = 1;  // -> R15/R16 divider -> Pi header 10 (GPIO15/RXD)

// ---- 74HC595 serial interface (N + D) -----------------------------------
//@pin SR_LATCH D2 U4_1.5
//@net U4_1.5 U8_1.12
static const uint8_t SR_LATCH = 2;  // STCP (storage register clock)
//@pin SR_DATA D4 U4_1.7
//@net U4_1.7 U8_1.14
static const uint8_t SR_DATA = 4;  // DS
//@pin SR_OE_N D7 U4_1.10
//@net U4_1.10 U8_1.13
//@net U4_1.10 R20_1.1
static const uint8_t SR_OE_N = 7;  // /OE, R20 10k pull-up to +5V_Cell: HIGH = outputs Hi-Z
//@pin SR_CLOCK D13 U4_1.16
//@net U4_1.16 U8_1.11
static const uint8_t SR_CLOCK = 13;  // SHCP (also the Nano on-board LED pin)

// ---- Heater PWM gates, one per 8-pin cell port (N) ------------------------
//@pin HEAT_PORT_H1_1 D3 U4_1.6
//@net U4_1.6 H1_1.5
static const uint8_t HEAT_PORT_H1_1 = 3;
//@pin HEAT_PORT_H_1 D5 U4_1.8
//@net U4_1.8 H_1.5
static const uint8_t HEAT_PORT_H_1 = 5;
//@pin HEAT_PORT_H7_1 D6 U4_1.9
//@net U4_1.9 H7_1.5
static const uint8_t HEAT_PORT_H7_1 = 6;
//@pin HEAT_PORT_H8_1 D11 U4_1.14
//@net U4_1.14 H8_1.5
static const uint8_t HEAT_PORT_H8_1 = 11;

// ---- Power path (N; MOSFET pin model = I, polarity = H) -------------------
//@pin PACK_RELAY D8 U4_1.11
//@net U4_1.11 R13_1.1
//@net R13_1.2 Q2_1.1
//@net Q2_1.3 RLY2_1.1
static const uint8_t PACK_RELAY = 8;  // HIGH -> Q2 (AO3400A) on -> RLY2 coil energised (contact NO/NC = H)
//@pin CHARGE_PMOS D9 U4_1.12
//@net U4_1.12 R19_1.2
//@net R19_1.1 Q3_1.1
//@net Q3_1.1 R18_1.2
static const uint8_t CHARGE_PMOS = 9;  // LOW -> Q3 (AO3401A P-MOS) on. R18 10k pulls gate to +5V (off)
//@pin DISCHARGE_NMOS D10 U4_1.13
//@net U4_1.13 R22_1.2
//@net R22_1.1 Q4_1.1
//@net Q4_1.1 R23_1.1
static const uint8_t DISCHARGE_NMOS = 10;  // HIGH -> Q4 (AO3400A N-MOS) on. R23 10k pulls gate to GND (off)

// ---- Not connected on PCB2 (N) -------------------------------------------
static const uint8_t UNUSED_D12 = 12;  // U4_1.15: no net. Left as input; do not add functions.

// ---- Safe (de-energised) levels ------------------------------------------
static const uint8_t SAFE_LEVEL_PACK_RELAY = 0;      // coil off
static const uint8_t SAFE_LEVEL_CHARGE_PMOS = 1;     // gate high = P-MOS off
static const uint8_t SAFE_LEVEL_DISCHARGE_NMOS = 0;  // gate low = N-MOS off
static const uint8_t SAFE_LEVEL_SR_OE_N = 1;         // 595 outputs Hi-Z until register cleared
static const uint8_t SAFE_LEVEL_HEATER = 0;          // AO3400A gate low = heater off
}  // namespace pin

// ---- ADC channels (N) -----------------------------------------------------
// AREF is driven by the fitted REF3040 (4.096 V). Use only the
// EXTERNAL reference (REFS1:0 = 00). Selecting AVCC/INTERNAL would short the
// internal reference against the REF3040 output on the AREF pin.
//@pin AREF AREF U4_1.18
//@net U4_1.18 U10_1.2
namespace adc {
//@pin TAP_A0 A0 U4_1.19
//@net U4_1.19 R10_1.1
//@net R10_1.2 H_1.7
//@net U4_1.19 R24_1.1
static const uint8_t TAP_A0 = 0;  // cumulative tap 1 (cell1+)
//@pin TAP_A1 A1 U4_1.20
//@net U4_1.20 R9_1.1
//@net R9_1.2 H8_1.7
//@net U4_1.20 R27_1.2
static const uint8_t TAP_A1 = 1;  // cumulative tap 2 (cell1+cell2)
//@pin TAP_A2 A2 U4_1.21
//@net U4_1.21 R12_1.1
//@net R12_1.2 H7_1.7
//@net U4_1.21 R26_1.2
static const uint8_t TAP_A2 = 2;  // cumulative tap 3
//@pin TAP_A3 A3 U4_1.22
//@net U4_1.22 R11_1.1
//@net R11_1.2 H1_1.7
//@net U4_1.22 R25_1.2
static const uint8_t TAP_A3 = 3;  // cumulative tap 4 = pack+
//@pin NTC_A4 A4 U4_1.23
//@net U4_1.23 H_1.6
static const uint8_t NTC_A4 = 4;
//@pin NTC_A5 A5 U4_1.24
//@net U4_1.24 H8_1.6
static const uint8_t NTC_A5 = 5;
//@pin NTC_A6 A6 U4_1.25
//@net U4_1.25 H7_1.6
static const uint8_t NTC_A6 = 6;  // analog-only pin
//@pin NTC_A7 A7 U4_1.26
//@net U4_1.26 H1_1.6
static const uint8_t NTC_A7 = 7;  // analog-only pin
static const uint8_t BANDGAP = 14;  // internal 1.1 V reference (MUX 1110), AREF sanity check
static const uint8_t CHANNELS = 8;
}  // namespace adc

// ---- 74HC595 -> ULN2803 -> 8-pin port (N + D) ------------------------------
// Register bit n drives HC595 output Qn (Q0 = pin 15, Q1..Q7 = pins 1..7).
// Data is shifted MSB first, so after 8 clocks bit7 sits in Q7 and bit0 in Q0.
// bit = 1 -> ULN2803 input high -> its output sinks (coil / opto LED on).
//@net U8_1.15 U9_1.1
//@net U9_1.18 H1_1.3
//@net U8_1.1 U9_1.2
//@net U9_1.17 H7_1.3
//@net U8_1.2 U9_1.3
//@net U9_1.16 H8_1.3
//@net U8_1.3 U9_1.4
//@net U9_1.15 H_1.3
//@net U8_1.4 U9_1.5
//@net U9_1.14 H1_1.4
//@net U8_1.5 U9_1.6
//@net U9_1.13 H7_1.4
//@net U8_1.6 U9_1.7
//@net U9_1.12 H8_1.4
//@net U8_1.7 U9_1.8
//@net U9_1.11 H_1.4
namespace srbit {
static const uint8_t ESR_H1_1 = 0;
static const uint8_t ESR_H7_1 = 1;
static const uint8_t ESR_H8_1 = 2;
static const uint8_t ESR_H_1 = 3;
static const uint8_t LEAK_H1_1 = 4;
static const uint8_t LEAK_H7_1 = 5;
static const uint8_t LEAK_H8_1 = 6;
static const uint8_t LEAK_H_1 = 7;
}  // namespace srbit

// ---- Logical cell map (H/I: harness pairing is NOT netlist-verified) -------
// The netlist only guarantees two separate orderings:
//   series 2-pin path (GND -> pack+): H5_1 -> H6_1 -> H3_1 -> H2_1
//   8-pin control ports (low -> high tap): H_1 -> H8_1 -> H7_1 -> H1_1
// Pairing them per cell is an assumption until the cables are checked
// (hardware_verification.h: CG_HW_MAP_VERIFIED). ESR/LEAK 595 bits run in
// reverse order relative to the cell id.
struct CellChannels {
  uint8_t tapAdc;     // cumulative tap channel (A0..A3)
  uint8_t ntcAdc;     // NTC channel (A4..A7)
  uint8_t heaterPin;  // PWM pin
  uint8_t esrBit;     // 595 register bit for the RLY1 coil sink
  uint8_t leakBit;    // 595 register bit for the PC817 LED sink
};

//@cell 1 H5_1 H_1 A0 A4 D5 3 7
//@cell 2 H6_1 H8_1 A1 A5 D11 2 6
//@cell 3 H3_1 H7_1 A2 A6 D6 1 5
//@cell 4 H2_1 H1_1 A3 A7 D3 0 4
static const uint8_t CELL_COUNT = 4;
static const CellChannels CELLS[CELL_COUNT] = {
    {adc::TAP_A0, adc::NTC_A4, pin::HEAT_PORT_H_1, srbit::ESR_H_1, srbit::LEAK_H_1},      // cell 1
    {adc::TAP_A1, adc::NTC_A5, pin::HEAT_PORT_H8_1, srbit::ESR_H8_1, srbit::LEAK_H8_1},   // cell 2
    {adc::TAP_A2, adc::NTC_A6, pin::HEAT_PORT_H7_1, srbit::ESR_H7_1, srbit::LEAK_H7_1},   // cell 3
    {adc::TAP_A3, adc::NTC_A7, pin::HEAT_PORT_H1_1, srbit::ESR_H1_1, srbit::LEAK_H1_1},   // cell 4
};

// Converts logical masks (bit0 = cell1) into the physical 595 register byte.
inline uint8_t srByteFromLogical(uint8_t esrMask, uint8_t leakMask) {
  uint8_t out = 0;
  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    if (esrMask & (1u << i)) out |= (uint8_t)(1u << CELLS[i].esrBit);
    if (leakMask & (1u << i)) out |= (uint8_t)(1u << CELLS[i].leakBit);
  }
  return out;
}

}  // namespace cg
