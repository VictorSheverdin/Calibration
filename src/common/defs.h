#pragma once

#include <limits>

static const auto DOUBLE_EPS = std::numeric_limits<double>::epsilon();
static const int VIMBA_ORIGINAL_FRAME_SIZE = 2048;

enum class VimbaDecimationType { WHOLE = 1, HALF = 2, QUARTER = 4, EIGHTH = 8 };

