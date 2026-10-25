// CellGuard Nano firmware - UART framing and command parsing (protocol v1.0)
// Author: Yongho Ahn <nanhosoft@gmail.com>
// Frame: $<PAYLOAD>*<CRC16 uppercase hex 4>\n  (see docs/uart_protocol.md)
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "cg_hal.h"

namespace cg {

static const uint8_t MAX_FRAME_BYTES = 192;  // '$' .. '\n' inclusive (with optional '\r')
static const uint8_t MAX_PAYLOAD_BYTES = MAX_FRAME_BYTES - 8;  // '$' '*' CRC4 '\r' '\n'

bool pstrEquals(const char* ram, PStr flash);

// ---- Outgoing frame builder ----------------------------------------------
class FrameBuilder {
 public:
  FrameBuilder() : len_(0), overflow_(false) { buf_[0] = 0; }
  void start(PStr type);
  void addP(PStr s);       // ",<flash string>"
  void addU(uint32_t v);   // ",<unsigned>"
  void addI(int32_t v);    // ",<signed>"
  // Writes the complete frame. Returns false (and writes nothing) on overflow.
  bool send(Hal& hal);
  const char* payload() const { return buf_; }
  uint8_t payloadLen() const { return len_; }
  bool overflow() const { return overflow_; }
  // Renders the full frame into out (size >= MAX_FRAME_BYTES + 1); returns length.
  uint8_t render(char* out) const;

 private:
  void putChar(char c);
  void putP(PStr s);
  void putU(uint32_t v);
  void crcTail(char* tail) const;  // "*HHHH\n"
  char buf_[MAX_PAYLOAD_BYTES + 1];
  uint8_t len_;
  bool overflow_;
};

// ---- Incoming frame assembler --------------------------------------------
enum RxResult : uint8_t {
  RX_NONE = 0,
  RX_FRAME_OK,
  RX_CRC_ERROR,
  RX_FORMAT_ERROR,
  RX_OVERFLOW,
};

class RxAssembler {
 public:
  RxAssembler() { reset(); }
  void reset();
  RxResult feed(char c);
  // Valid after RX_FRAME_OK: NUL-terminated payload (mutable, for tokenising).
  char* payload() { return buf_; }

 private:
  enum Mode : uint8_t { IDLE, IN_FRAME, DISCARD };
  RxResult finish();
  char buf_[MAX_FRAME_BYTES + 1];
  uint8_t len_;    // bytes stored after '$'
  uint8_t total_;  // bytes counted from '$'
  Mode mode_;
  bool badChar_;
};

// ---- Command parsing -------------------------------------------------------
enum CmdType : uint8_t {
  CMD_NONE = 0,
  CMD_SYNC,
  CMD_PING,
  CMD_STATUS,
  CMD_RAW,
  CMD_CAL,
  CMD_ARM,
  CMD_OFF,
  CMD_TRIP,
  CMD_RESET,
  CMD_CHARGE,
  CMD_RAMP,
  CMD_DRAIN,
  CMD_DISCHARGE,
  CMD_ESR,
  CMD_LEAK,
  CMD_HEAT,
  CMD_BENCH,
};

enum ParseResult : uint8_t {
  PARSE_OK = 0,
  PARSE_BAD_FORMAT,
  PARSE_BAD_ARGS,
  PARSE_UNKNOWN_CMD,
};

struct Command {
  int32_t seq;  // -1 when the sequence field could not be parsed
  CmdType type;
  uint8_t argc;
  int16_t arg[2];
};

// Tokenises payload in place. benchEnabled selects whether BENCH is known.
ParseResult parseCommand(char* payload, Command& out, bool benchEnabled);

// Strict unsigned decimal parse: digits only, no sign/space, value <= maxValue.
bool parseUnsigned(const char* s, uint32_t maxValue, uint32_t& out);

}  // namespace cg
