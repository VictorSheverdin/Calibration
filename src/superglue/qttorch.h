#pragma once 

#ifdef QT_VERSION
	#undef slots
#endif
#include "torch/torch.h"
#ifdef QT_VERSION
	#define slots Q_SLOTS
#endif
