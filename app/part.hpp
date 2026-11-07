// SPDX-License-Identifier: Apache-2.0
#pragma once

// A part of a board that the probe's main loop serves (boards/<name>/board.cpp lists
// them): it may do work every round of the loop and answer control port requests.
//
// 板子上由探针主循环服务的部件（boards/<name>/board.cpp 列出）：
// 每轮主循环可以做事，也可以应答控制口请求。

#include <cstdint>

namespace XrDap
{

class Part
{
 public:
  /// @brief 主循环每轮调用一次 / Called once every round of the main loop.
  /// @return 还有事要做、主循环不应等待时为 true / true while there is work and the main
  ///         loop should not wait.
  virtual bool Poll() { return false; }

  /// @brief 应答一个控制口请求 / Answer a control port request.
  /// @return 是本部件的操作、应答已填好时为 true / true when the op is this part's and
  ///         the answer is filled in.
  virtual bool Control(const uint8_t*, uint16_t, uint8_t*, uint16_t, uint16_t&)
  {
    return false;
  }

 protected:
  ~Part() = default;
};

}  // namespace XrDap
