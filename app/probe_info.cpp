// SPDX-License-Identifier: Apache-2.0
#include "probe_info.hpp"

#include <cstdio>

namespace XrDap
{

const char* ProbeSerial()
{
  static char serial[sizeof(kSerialPrefix) + 2 * kChipUidSize] = {};
  if (serial[0] == '\0')
  {
    int n = std::snprintf(serial, sizeof(serial), "%s", kSerialPrefix);
    for (uint32_t i = 0; i < kChipUidSize; ++i)
    {
      n += std::snprintf(serial + n, sizeof(serial) - static_cast<size_t>(n), "%02X",
                         ChipUid()[i]);
    }
  }
  return serial;
}

}  // namespace XrDap
