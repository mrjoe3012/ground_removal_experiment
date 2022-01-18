#include "point_categories.hpp"

namespace point_categories
{
	
		PointCategory operator++(PointCategory& c)
		{
			return static_cast<PointCategory>((c+1) % PointCategory::NUM_CATEGORIES);
		}	

		PointCategory operator++(PointCategory& c, int)
		{
			PointCategory result = c;
			++c;
			return result;
		}

}
