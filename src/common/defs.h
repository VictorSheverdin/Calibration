#pragma once

#include <limits>

static const auto DOUBLE_EPS = std::numeric_limits< double >::epsilon();
static const auto FLOAT_EPS = std::numeric_limits< float >::epsilon();

static const auto DOUBLE_MAX = std::numeric_limits< double >::max();
static const auto DOUBLE_MIN = std::numeric_limits< double >::min();

static const auto FLOAT_MAX = std::numeric_limits< float >::max();
static const auto FLOAT_MIN = std::numeric_limits< float >::min();

static const int VIMBA_ORIGINAL_FRAME_SIZE = 2048;

enum class VimbaDecimationType { WHOLE = 1, HALF = 2, QUARTER = 4, EIGHTH = 8 };

