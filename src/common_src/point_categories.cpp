#include "point_categories.hpp"

namespace common
{
	
		PointCategory operator++(PointCategory& c)
		{
			c = static_cast<PointCategory>((c+1) % PointCategory::NUM_CATEGORIES);
			return c;
		}	

		PointCategory operator++(PointCategory& c, int)
		{
			PointCategory result = c;
			++c;
			return result;
		}

}
