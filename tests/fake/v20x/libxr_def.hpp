// SPDX-License-Identifier: Apache-2.0
#pragma once

// 主机测试里代替 LibXR 的错误码，只有 V208Radio 用到的部分。 / Stands in for LibXR's
// error codes in the host tests, with only what V208Radio uses.

namespace LibXR
{

enum class ErrorCode
{
  OK,
  BUSY,
  OUT_OF_RANGE
};

}  // namespace LibXR
