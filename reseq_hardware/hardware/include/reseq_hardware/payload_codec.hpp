#pragma once

#include <stddef.h>   // for size_t
#include <stdint.h>   // for uint8_t, int32_t, ...
#include <algorithm>  // for clamp
#include <cmath>      // for isnan, lround, isfinite
#include <cstring>    // for memcpy
#include <limits>     // for numeric_limits
#include <string>     // for string

namespace reseq_hardware
{

/**
 * @brief Binary encoding of a single CAN payload field.
 *
 * All multi-byte values are little-endian, matching the STM32 firmware
 * (Cortex-M is little-endian and copies payloads with memcpy).
 */
enum class FieldDataType
{
  FLOAT32,
  INT8,
  UINT8,
  INT16,
  UINT16,
  INT32,
  UINT32,
  UNKNOWN
};

inline FieldDataType parse_data_type(const std::string & s)
{
  if (s == "float32") {return FieldDataType::FLOAT32;}
  if (s == "int8") {return FieldDataType::INT8;}
  if (s == "uint8") {return FieldDataType::UINT8;}
  if (s == "int16") {return FieldDataType::INT16;}
  if (s == "uint16") {return FieldDataType::UINT16;}
  if (s == "int32") {return FieldDataType::INT32;}
  if (s == "uint32") {return FieldDataType::UINT32;}
  return FieldDataType::UNKNOWN;
}

/// @return Size in bytes of the data type, 0 for UNKNOWN.
inline size_t data_type_size(FieldDataType t)
{
  switch (t) {
    case FieldDataType::INT8:
    case FieldDataType::UINT8:
      return 1;
    case FieldDataType::INT16:
    case FieldDataType::UINT16:
      return 2;
    case FieldDataType::FLOAT32:
    case FieldDataType::INT32:
    case FieldDataType::UINT32:
      return 4;
    default:
      return 0;
  }
}

namespace detail
{
template<typename T>
inline void store(uint8_t * data, size_t offset, T v)
{
  std::memcpy(data + offset, &v, sizeof(T));
}

template<typename T>
inline T load(const uint8_t * data, size_t offset)
{
  T v;
  std::memcpy(&v, data + offset, sizeof(T));
  return v;
}

/// Rounds and saturates a double into an integer type (no UB on overflow).
template<typename T>
inline T saturate(double value)
{
  const double lo = static_cast<double>(std::numeric_limits<T>::min());
  const double hi = static_cast<double>(std::numeric_limits<T>::max());
  return static_cast<T>(std::llround(std::clamp(value, lo, hi)));
}

inline uint32_t load_unsigned(const uint8_t * data, size_t offset, FieldDataType t)
{
  switch (data_type_size(t)) {
    case 1: return load<uint8_t>(data, offset);
    case 2: return load<uint16_t>(data, offset);
    case 4: return load<uint32_t>(data, offset);
    default: return 0;
  }
}

inline void store_unsigned(uint8_t * data, size_t offset, FieldDataType t, uint32_t v)
{
  switch (data_type_size(t)) {
    case 1: store<uint8_t>(data, offset, static_cast<uint8_t>(v)); break;
    case 2: store<uint16_t>(data, offset, static_cast<uint16_t>(v)); break;
    case 4: store<uint32_t>(data, offset, v); break;
    default: break;
  }
}
}  // namespace detail

/**
 * @brief Encodes a raw (already scaled) value into the payload.
 *
 * Integer types are rounded to nearest and saturated to the type range.
 * NaN/inf must be filtered by the caller.
 *
 * If @p bitmask is non-zero the field is a flag: a non-zero @p value sets all
 * the bits of @p bitmask in the integer word at @p offset, zero clears them.
 * Other bits of the word are preserved, so several flag fields can share the
 * same word (e.g. the TORQUE_ENABLE_DISABLE bitfield).
 *
 * @return false if the data type is unsupported (float32 cannot be a bitmask).
 */
inline bool encode_value(
  uint8_t * data, size_t offset, FieldDataType type, double value, uint32_t bitmask = 0)
{
  if (bitmask != 0) {
    if (type == FieldDataType::FLOAT32 || data_type_size(type) == 0) {
      return false;
    }
    uint32_t word = detail::load_unsigned(data, offset, type);
    word = (value != 0.0) ? (word | bitmask) : (word & ~bitmask);
    detail::store_unsigned(data, offset, type, word);
    return true;
  }

  switch (type) {
    case FieldDataType::FLOAT32: detail::store<float>(data, offset, static_cast<float>(value));
      break;
    case FieldDataType::INT8: detail::store(data, offset, detail::saturate<int8_t>(value)); break;
    case FieldDataType::UINT8: detail::store(data, offset, detail::saturate<uint8_t>(value)); break;
    case FieldDataType::INT16: detail::store(data, offset, detail::saturate<int16_t>(value)); break;
    case FieldDataType::UINT16: detail::store(data, offset, detail::saturate<uint16_t>(value));
      break;
    case FieldDataType::INT32: detail::store(data, offset, detail::saturate<int32_t>(value)); break;
    case FieldDataType::UINT32: detail::store(data, offset, detail::saturate<uint32_t>(value));
      break;
    default:
      return false;
  }
  return true;
}

/**
 * @brief Decodes a raw value from the payload (inverse of encode_value).
 * For bitmask fields returns 1.0 if any masked bit is set, 0.0 otherwise.
 */
inline double decode_value(
  const uint8_t * data, size_t offset, FieldDataType type, uint32_t bitmask = 0)
{
  if (bitmask != 0) {
    return (detail::load_unsigned(data, offset, type) & bitmask) ? 1.0 : 0.0;
  }
  switch (type) {
    case FieldDataType::FLOAT32: return detail::load<float>(data, offset);
    case FieldDataType::INT8: return detail::load<int8_t>(data, offset);
    case FieldDataType::UINT8: return detail::load<uint8_t>(data, offset);
    case FieldDataType::INT16: return detail::load<int16_t>(data, offset);
    case FieldDataType::UINT16: return detail::load<uint16_t>(data, offset);
    case FieldDataType::INT32: return detail::load<int32_t>(data, offset);
    case FieldDataType::UINT32: return detail::load<uint32_t>(data, offset);
    default: return std::numeric_limits<double>::quiet_NaN();
  }
}

}  // namespace reseq_hardware
