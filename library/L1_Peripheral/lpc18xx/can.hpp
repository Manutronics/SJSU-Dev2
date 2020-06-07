#pragma once

#include "L1_Peripheral/lpc40xx/can.hpp"

namespace sjsu
{
namespace lpc18xx
{
// The LPC40xx driver is compatible with the lpc18xx peripheral
using ::sjsu::lpc40xx::Can;
}  // namespace lpc18xx
}  // namespace sjsu
