// CellGuard Nano firmware - CRC-16/CCITT-FALSE
// Author: Yongho Ahn <nanhosoft@gmail.com>
// poly 0x1021, init 0xFFFF, no reflection, xorout 0. "123456789" -> 0x29B1.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace cg {

uint16_t crc16Update(uint16_t crc, uint8_t byte);
uint16_t crc16(const char* data, size_t len);

}  // namespace cg
