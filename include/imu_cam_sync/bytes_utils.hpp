#pragma once
#ifndef ICAM_SYNC_BYTES_UTILS_HPP_
#define ICAM_SYNC_BYTES_UTILS_HPP_
#include <cstdint>

namespace ic_sync {
namespace utils {

/**
 *  @brief Get a 16-bit integer stored in big-endian format.
 *
 *  Get a 16-bit integer, stored in big-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the big-endian 16-bit integer to get.
 *
 *  @return 16-bit integer in host endianness.
 */
static inline uint16_t sys_get_be16(const uint8_t src[2]) {
  return ((uint16_t)src[0] << 8) | src[1];
}

/**
 *  @brief Get a 32-bit integer stored in big-endian format.
 *
 *  Get a 32-bit integer, stored in big-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the big-endian 32-bit integer to get.
 *
 *  @return 32-bit integer in host endianness.
 */
static inline uint32_t sys_get_be32(const uint8_t src[4]) {
  return ((uint32_t)sys_get_be16(&src[0]) << 16) | sys_get_be16(&src[2]);
}

/**
 *  @brief Get a 64-bit integer stored in big-endian format.
 *
 *  Get a 64-bit integer, stored in big-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the big-endian 64-bit integer to get.
 *
 *  @return 64-bit integer in host endianness.
 */
static inline uint64_t sys_get_be64(const uint8_t src[8]) {
  return ((uint64_t)sys_get_be32(&src[0]) << 32) | sys_get_be32(&src[4]);
}

static inline int32_t decode_int32(const uint8_t bytes[4]) {
  return bytes[0] << 24 | bytes[1] << 16 | bytes[2] << 8 | bytes[3] << 0;
}

} // namespace utils

} // namespace ic_sync

#endif