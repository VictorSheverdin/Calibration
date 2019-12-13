#pragma once

#include <limits>

#include <VimbaCPP/Include/VimbaCPP.h>

static const auto DOUBLE_EPS = std::numeric_limits< double >::epsilon();
static const auto FLOAT_EPS = std::numeric_limits< float >::epsilon();

static const auto DOUBLE_MAX = std::numeric_limits< double >::max();
static const auto DOUBLE_MIN = std::numeric_limits< double >::min();

static const auto FLOAT_MAX = std::numeric_limits< float >::max();
static const auto FLOAT_MIN = std::numeric_limits< float >::min();

static const int VIMBA_ORIGINAL_FRAME_SIZE = 2048;

static const double MAX_REPROJECTION_ERROR = 2.0;

static const int MIN_PNP_POINTS_COUNT = 10;

enum class VimbaDecimationType { WHOLE = 1, HALF = 2, QUARTER = 4, EIGHTH = 8 };

static const VmbInt64_t ACTION_DEVICE_KEY = 1;
static const VmbInt64_t ACTION_GROUP_KEY = 1;
static const VmbInt64_t ACTION_GROUP_MASK = 1;

