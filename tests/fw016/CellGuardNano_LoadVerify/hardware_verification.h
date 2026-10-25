// CellGuard Nano firmware - hardware verification gates
// Author: Yongho Ahn <nanhosoft@gmail.com>
//
// Every flag below defaults to 0 (NOT verified). While a flag is 0 the
// firmware refuses the matching actuator commands with ERR,HARDWARE_LOCKED.
//
// RULES
//  * Only a person who performed and recorded the bench check may change a
//    flag, and must fill in the record line next to it (who / date / document).
//  * Never flip a flag to make a test pass. Host tests construct their own
//    gate sets in code and do not depend on these values.
//  * No UART command can change these gates at runtime (by design).
//  * Flags declare what was verified; they do NOT make unattended operation
//    safe. Independent hardware protection is still required (docs/known_limitations.md).
#pragma once

// Cell harness: each PCB2 2-pin port (H5_1/H6_1/H3_1/H2_1) is paired with the
// expected 8-pin port (H_1/H8_1/H7_1/H1_1) and PCB1 polarity/pin 1 confirmed.
// Record: <name> / <YYYY-MM-DD> / <checklist reference>
#ifndef CG_HW_MAP_VERIFIED
#define CG_HW_MAP_VERIFIED 0
#endif

// Independent over-temperature cut-off for the 47R heaters exists and was
// tested (the unmodified NTC cannot distinguish an open from a saturated cold
// reading, even when REF3040 makes room-temperature readings valid).
// Record:
#ifndef CG_HW_THERMAL_PROTECTION_VERIFIED
#define CG_HW_THERMAL_PROTECTION_VERIFIED 0
#endif

// NTC front end was modified so TEMP_OUT stays below the 4.096 V AREF over the
// entire required working range, including sensor-open cases (requires a
// documented hardware revision + new conversion/diagnostics).
// The firmware conversion in this release assumes the UNMODIFIED circuit and
// refuses to build with this flag set.
#ifndef CG_HW_NTC_REVISED
#define CG_HW_NTC_REVISED 0
#endif

// External INA219 module installed: I2C on H9_1 AND shunt in series via H4_1.
// Informational only - the Nano never reads current.
// Record:
#ifndef CG_HW_INA219_DECLARED
#define CG_HW_INA219_DECLARED 0
#endif

// RLY2 COM/NO/NC measured; pack is connected ONLY while the coil is energised.
// Record:
#ifndef CG_HW_PACK_RELAY_VERIFIED
#define CG_HW_PACK_RELAY_VERIFIED 0
#endif
// 1 = pack connected when coil energised (required for unlocking).
#ifndef CG_HW_PACK_RELAY_CONNECTS_WHEN_ENERGISED
#define CG_HW_PACK_RELAY_CONNECTS_WHEN_ENERGISED 1
#endif

// RLY1 (each PCB1) contacts measured: coil energised = 0.47R inserted in series.
// Record:
#ifndef CG_HW_ESR_RELAY_VERIFIED
#define CG_HW_ESR_RELAY_VERIFIED 0
#endif
#ifndef CG_HW_ESR_INSERTED_WHEN_ENERGISED
#define CG_HW_ESR_INSERTED_WHEN_ENERGISED 1
#endif

// Q2/Q3/Q4 body markings, footprint pin 1 and gate polarity measured
// (D8 HIGH = relay coil on, D9 LOW = charge on, D10 HIGH = discharge on).
// Record:
#ifndef CG_HW_POWER_MOS_VERIFIED
#define CG_HW_POWER_MOS_VERIFIED 0
#endif

// R17 actually fitted value measured (intent: 22R 5W; netlist says 10R).
// Record:
#ifndef CG_HW_R17_VERIFIED
#define CG_HW_R17_VERIFIED 0
#endif

// ---- Build-time consistency checks ---------------------------------------
#if CG_HW_PACK_RELAY_VERIFIED && !CG_HW_PACK_RELAY_CONNECTS_WHEN_ENERGISED
#error "RLY2 connects the pack when de-energised: not fail-safe at reset. Rewire before unlocking."
#endif
#if CG_HW_ESR_RELAY_VERIFIED && !CG_HW_ESR_INSERTED_WHEN_ENERGISED
#error "RLY1 inserts ESR when de-energised: fault would be active at reset. Rewire before unlocking."
#endif
#if CG_HW_NTC_REVISED
#error "NTC hardware revision needs its own documented conversion; not implemented in this release."
#endif
