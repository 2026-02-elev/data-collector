// CellGuard Nano firmware - calibration constants
// Author: Yongho Ahn <nanhosoft@gmail.com>
//
// Two sets are kept apart on purpose (FR-12):
//   NOMINAL  - datasheet / purchase-spec values. NOT measured.
//   MEASURED - values a person measured on this specific board (bench
//              checklist, docs/calibration_and_bench_checklist.md section C).
// The active set and its id are reported in the CGK frame (CAL command) so
// that every logged sample can be traced to the constants used.
#pragma once

// 0 = use NOMINAL (default). 1 = use MEASURED (fill in every value + id first).
#ifndef CG_CAL_USE_MEASURED
#define CG_CAL_USE_MEASURED 0
#endif

// ---- NOMINAL (hardware specification, not a board-specific measurement) ----
#define CG_CAL_NOMINAL_ID "NOMINAL_UNCALIBRATED"
#define CG_CAL_NOMINAL_AREF_MV 4096      // fitted REF3040 nominal output (4.096 V)
#define CG_CAL_NOMINAL_V5_MV 5000        // +5V_Cell nominal (NTC pull-up supply)
#define CG_CAL_NOMINAL_GAIN_PPM 1000000L // tap channel gain (47k/43k tolerance)
#define CG_CAL_NOMINAL_OFFSET_MV 0
#define CG_CAL_NOMINAL_NTC_B 3950        // MF52A103F3950 purchase spec
#define CG_CAL_NOMINAL_NTC_R25 10000
#define CG_CAL_NOMINAL_NTC_PULLUP 10000  // PCB1 R7

// ---- MEASURED (fill in from the bench record) ----
#define CG_CAL_MEASURED_ID "SET_ME_BOARD_DATE_OPERATOR"
// Placeholders only: these are NOT measurements. CG_CAL_USE_MEASURED stays 0.
#define CG_CAL_MEASURED_AREF_MV 4096
#define CG_CAL_MEASURED_V5_MV 5000
#define CG_CAL_MEASURED_GAIN0_PPM 1000000L
#define CG_CAL_MEASURED_GAIN1_PPM 1000000L
#define CG_CAL_MEASURED_GAIN2_PPM 1000000L
#define CG_CAL_MEASURED_GAIN3_PPM 1000000L
#define CG_CAL_MEASURED_OFFSET0_MV 0
#define CG_CAL_MEASURED_OFFSET1_MV 0
#define CG_CAL_MEASURED_OFFSET2_MV 0
#define CG_CAL_MEASURED_OFFSET3_MV 0
#define CG_CAL_MEASURED_NTC_B 3950
#define CG_CAL_MEASURED_NTC_R25 10000
#define CG_CAL_MEASURED_NTC_PULLUP 10000
