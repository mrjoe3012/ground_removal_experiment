#include "point_categories.hpp"

namespace common
{


		std::unordered_map<PointCategory, CategoryColour> categoryColours = 
		{
			{PointCategory::Unassigned, std::make_tuple(255,255,255)},
			{PointCategory::Cone, std::make_tuple(255,0,0)},
			{PointCategory::Ground, std::make_tuple(0,255,0)},
		};


	
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
