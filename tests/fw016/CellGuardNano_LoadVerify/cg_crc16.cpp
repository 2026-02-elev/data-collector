// CellGuard Nano firmware - CRC-16/CCITT-FALSE
// Author: Yongho Ahn <nanhosoft@gmail.com>
#include "cg_crc16.h"

namespace cg {

uint16_t crc16Update(uint16_t crc, uint8_t byte) {
  crc ^= (uint16_t)byte << 8;
  for (uint8_t i = 0; i < 8; i++) {
    crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

uint16_t crc16(const char* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) crc = crc16Update(crc, (uint8_t)data[i]);
  return crc;
}

}  // namespace cg
