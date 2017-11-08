// Copyright 2014-2015 Isis Innovation Limited and the authors of gSLICr

#pragma once
#include "../gSLICr_defines.h"
#include <vector>

#define SPIXEL_INFO_MAX_NUM_NEIGHBORS 10
namespace gSLICr
{
	namespace objects
	{
		struct spixel_info
		{
			Vector2f center;
			Vector4f color_info;
			int id;
			int no_pixels;
			int neighbors[SPIXEL_INFO_MAX_NUM_NEIGHBORS]; // TODO(gizatt): I'm setting an upper bound on # of neighbors here...
																		  // It'll happen rarely, but if anyone makes connectivity assumptions,
																		  // one could possibly break them.

		};
	}

	typedef ORUtils::Image<objects::spixel_info> SpixelMap;
}