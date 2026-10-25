// CellGuard Nano firmware - UART framing and command parsing (protocol v1.0)
// Author: Yongho Ahn <nanhosoft@gmail.com>
#include "cg_protocol.h"

#include "cg_config.h"

#include <string.h>

#include "cg_crc16.h"

namespace cg {

namespace {
const char HEX_DIGITS[] = "0123456789ABCDEF";

int hexValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;  // lowercase is rejected: the protocol mandates uppercase
}
}  // namespace

bool pstrEquals(const char* ram, PStr flash) {
  for (;;) {
    char f = (char)pgm_read_byte(flash);
    if (*ram != f) return false;
    if (f == 0) return true;
    ram++;
    flash++;
  }
}

// ---- FrameBuilder ------------------------------------------------------------

void FrameBuilder::putChar(char c) {
  if (len_ >= MAX_PAYLOAD_BYTES) {
    overflow_ = true;
    return;
  }
  buf_[len_++] = c;
  buf_[len_] = 0;
}

void FrameBuilder::putP(PStr s) {
  for (;;) {
    char c = (char)pgm_read_byte(s++);
    if (!c) break;
    putChar(c);
  }
}

void FrameBuilder::start(PStr type) {
  len_ = 0;
  overflow_ = false;
  buf_[0] = 0;
  putP(type);
}

void FrameBuilder::addP(PStr s) {
  putChar(',');
  putP(s);
}

void FrameBuilder::putU(uint32_t v) {
  char tmp[10];
  uint8_t n = 0;
  do {
    tmp[n++] = (char)('0' + (v % 10));
    v /= 10;
  } while (v);
  while (n) putChar(tmp[--n]);
}

void FrameBuilder::addU(uint32_t v) {
  putChar(',');
  putU(v);
}

void FrameBuilder::addI(int32_t v) {
  putChar(',');
  if (v < 0) {
    putChar('-');
    putU((uint32_t)(-(v + 1)) + 1u);  // safe for INT32_MIN
  } else {
    putU((uint32_t)v);
  }
}

void FrameBuilder::crcTail(char* tail) const {
  uint16_t crc = crc16(buf_, len_);
  tail[0] = '*';
  tail[1] = HEX_DIGITS[(crc >> 12) & 0xF];
  tail[2] = HEX_DIGITS[(crc >> 8) & 0xF];
  tail[3] = HEX_DIGITS[(crc >> 4) & 0xF];
  tail[4] = HEX_DIGITS[crc & 0xF];
  tail[5] = '\n';
}

uint8_t FrameBuilder::render(char* out) const {
  out[0] = '$';
  memcpy(out + 1, buf_, len_);
  crcTail(out + 1 + len_);
  uint8_t n = (uint8_t)(len_ + 7);
  out[n] = 0;
  return n;
}

bool FrameBuilder::send(Hal& hal) {
  if (overflow_) return false;
  char tail[6];
  crcTail(tail);
  hal.txWrite("$", 1);
  hal.txWrite(buf_, len_);
  hal.txWrite(tail, sizeof(tail));
  return true;
}

// ---- RxAssembler -------------------------------------------------------------

void RxAssembler::reset() {
  len_ = 0;
  total_ = 0;
  mode_ = IDLE;
  badChar_ = false;
  buf_[0] = 0;
}

RxResult RxAssembler::feed(char c) {
  switch (mode_) {
    case IDLE:
      if (c == '$') {
        mode_ = IN_FRAME;
        len_ = 0;
        total_ = 1;
        badChar_ = false;
      }
      return RX_NONE;  // bytes outside a frame are line noise

    case DISCARD:
      if (c == '\n') {
        mode_ = IDLE;
        return RX_OVERFLOW;
      }
      return RX_NONE;

    case IN_FRAME:
      if (c == '\n') {
        total_++;
        mode_ = IDLE;
        if (total_ > MAX_FRAME_BYTES) return RX_OVERFLOW;
        return finish();
      }
      if (c == '$') {
        // A new frame started before the previous one ended.
        len_ = 0;
        total_ = 1;
        badChar_ = false;
        return RX_FORMAT_ERROR;
      }
      total_++;
      if (total_ >= MAX_FRAME_BYTES) {  // no room left for '\n'
        mode_ = DISCARD;
        return RX_NONE;
      }
      if ((c < 0x20 || c > 0x7E) && c != '\r') badChar_ = true;
      buf_[len_++] = c;
      buf_[len_] = 0;
      return RX_NONE;
  }
  return RX_NONE;
}

RxResult RxAssembler::finish() {
  if (len_ > 0 && buf_[len_ - 1] == '\r') buf_[--len_] = 0;
  if (badChar_) return RX_FORMAT_ERROR;
  // Expect "...*HHHH" with exactly one '*'.
  if (len_ < 6 || buf_[len_ - 5] != '*') return RX_FORMAT_ERROR;
  uint8_t payloadLen = (uint8_t)(len_ - 5);
  for (uint8_t i = 0; i < payloadLen; i++) {
    if (buf_[i] == '*' || buf_[i] == '\r') return RX_FORMAT_ERROR;
  }
  uint16_t rxCrc = 0;
  for (uint8_t i = 0; i < 4; i++) {
    int h = hexValue(buf_[payloadLen + 1 + i]);
    if (h < 0) return RX_FORMAT_ERROR;
    rxCrc = (uint16_t)((rxCrc << 4) | (uint16_t)h);
  }
  if (crc16(buf_, payloadLen) != rxCrc) return RX_CRC_ERROR;
  buf_[payloadLen] = 0;
  return RX_FRAME_OK;
}

// ---- Command parsing ---------------------------------------------------------

bool parseUnsigned(const char* s, uint32_t maxValue, uint32_t& out) {
  if (!s || !*s) return false;
  uint32_t v = 0;
  uint8_t digits = 0;
  for (; *s; s++) {
    if (*s < '0' || *s > '9') return false;
    if (++digits > 10) return false;
    uint32_t d = (uint32_t)(*s - '0');
    if (v > (maxValue - d) / 10) return false;  // would exceed maxValue
    v = v * 10 + d;
  }
  out = v;
  return true;
}

namespace {
struct CmdSpec {
  CmdType type;
  uint8_t argc;
  uint16_t max0;
  uint16_t min0;
  uint16_t max1;
};

bool lookupCommand(const char* name, CmdSpec& spec, bool benchEnabled) {
  // arg ranges: (min0..max0), (0..max1)
  if (pstrEquals(name, PSTR("SYNC"))) { spec = {CMD_SYNC, 0, 0, 0, 0}; return true; }
  if (pstrEquals(name, PSTR("PING"))) { spec = {CMD_PING, 0, 0, 0, 0}; return true; }
  if (pstrEquals(name, PSTR("STATUS"))) { spec = {CMD_STATUS, 0, 0, 0, 0}; return true; }
  if (pstrEquals(name, PSTR("RAW"))) { spec = {CMD_RAW, 0, 0, 0, 0}; return true; }
  if (pstrEquals(name, PSTR("CAL"))) { spec = {CMD_CAL, 0, 0, 0, 0}; return true; }
  if (pstrEquals(name, PSTR("ARM"))) { spec = {CMD_ARM, 0, 0, 0, 0}; return true; }
  if (pstrEquals(name, PSTR("OFF"))) { spec = {CMD_OFF, 0, 0, 0, 0}; return true; }
  if (pstrEquals(name, PSTR("TRIP"))) { spec = {CMD_TRIP, 0, 0, 0, 0}; return true; }
  if (pstrEquals(name, PSTR("RESET"))) { spec = {CMD_RESET, 0, 0, 0, 0}; return true; }
  if (pstrEquals(name, PSTR("CHARGE"))) { spec = {CMD_CHARGE, 1, 1, 0, 0}; return true; }
  if (pstrEquals(name, PSTR("RAMP"))) {
    spec = {CMD_RAMP, 1, cfg::RAMP_TARGET_MAX_MV, cfg::RAMP_TARGET_MIN_MV, 0};
    return true;
  }
  if (pstrEquals(name, PSTR("DRAIN"))) {
    spec = {CMD_DRAIN, 1, cfg::DRAIN_TARGET_MAX_MV, cfg::DRAIN_TARGET_MIN_MV, 0};
    return true;
  }
  if (pstrEquals(name, PSTR("DISCHARGE"))) { spec = {CMD_DISCHARGE, 1, 1, 0, 0}; return true; }
  if (pstrEquals(name, PSTR("ESR"))) { spec = {CMD_ESR, 2, 4, 1, 1}; return true; }
  if (pstrEquals(name, PSTR("LEAK"))) { spec = {CMD_LEAK, 2, 4, 1, 1}; return true; }
  if (pstrEquals(name, PSTR("HEAT"))) { spec = {CMD_HEAT, 2, 4, 1, 255}; return true; }
  if (benchEnabled && pstrEquals(name, PSTR("BENCH"))) { spec = {CMD_BENCH, 2, 14, 0, 1}; return true; }
  return false;
}
}  // namespace

ParseResult parseCommand(char* payload, Command& out, bool benchEnabled) {
  out.seq = -1;
  out.type = CMD_NONE;
  out.argc = 0;
  out.arg[0] = out.arg[1] = 0;

  const uint8_t MAX_TOKENS = 5;  // CGC,seq,CMD,a,b
  char* tok[MAX_TOKENS + 1];
  uint8_t n = 0;
  char* p = payload;
  tok[n++] = p;
  for (; *p; p++) {
    if (*p == ',') {
      *p = 0;
      if (n > MAX_TOKENS) return PARSE_BAD_FORMAT;
      tok[n++] = p + 1;
    }
  }
  if (n > MAX_TOKENS) return PARSE_BAD_FORMAT;
  if (n < 3 || !pstrEquals(tok[0], PSTR("CGC"))) return PARSE_BAD_FORMAT;

  uint32_t seq;
  if (!parseUnsigned(tok[1], 65535UL, seq)) return PARSE_BAD_FORMAT;
  out.seq = (int32_t)seq;

  CmdSpec spec;
  if (!lookupCommand(tok[2], spec, benchEnabled)) return PARSE_UNKNOWN_CMD;
  if ((uint8_t)(n - 3) != spec.argc) return PARSE_BAD_FORMAT;

  for (uint8_t i = 0; i < spec.argc; i++) {
    uint32_t v;
    if (!parseUnsigned(tok[3 + i], 65535UL, v)) return PARSE_BAD_FORMAT;
    uint16_t lo = (i == 0) ? spec.min0 : 0;
    uint16_t hi = (i == 0) ? spec.max0 : spec.max1;
    if (v < lo || v > hi) return PARSE_BAD_ARGS;
    out.arg[i] = (int16_t)v;
  }
  out.argc = spec.argc;
  out.type = spec.type;
  return PARSE_OK;
}

}  // namespace cg
