#include "point_categories.hpp"

namespace common
{


		std::unordered_map<PointCategory, CategoryColour> categoryColours = 
		{
			{PointCategory::Unassigned, std::make_tuple(255,255,255)},
			{PointCategory::Cone, std::make_tuple(255,0,0)},
			{PointCategory::Ground, std::make_tuple(0,255,0)},
		};

		std::unordered_map<PointCategory, std::string> categoryNames = 
		{
			{PointCategory::Unassigned, "Unassigned"},
			{PointCategory::Cone, "Cone"},
			{PointCategory::Ground, "Ground"},
		};
	
}
