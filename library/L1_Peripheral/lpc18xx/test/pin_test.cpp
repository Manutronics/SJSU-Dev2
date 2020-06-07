// Test for Pin class.
#include "L0_Platform/lpc18xx/lpc18xx.h"
#include "L1_Peripheral/lpc18xx/pin.hpp"
#include "L4_Testing/testing_frameworks.hpp"
#include "utility/debug.hpp"

namespace sjsu::lpc18xx
{
EMIT_ALL_METHODS(Pin);

TEST_CASE("Testing lpc18xx Pin", "[lpc18xx-pin]") {}
}  // namespace sjsu::lpc18xx
