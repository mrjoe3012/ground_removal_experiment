#pragma once
#include <tuple>
#include <unordered_map>
#include <string>

// namespace shared by various modules which defines
// different point categories and their corresponding
// RGB colour value
namespace common
{
	typedef std::tuple<int, int, int> CategoryColour;

	enum PointCategory
	{
		Unassigned,
		Cone,
		Ground,
		NUM_CATEGORIES // number of categories which are defined
	};	


	// Utility function for advancing enum value (0->1->2->0...)
	PointCategory operator++(PointCategory&);
	PointCategory operator++(PointCategory&, int);
	

	// colours associated with each point category
	// (will be seen in visualizer and written to
	// output .pcd)
	extern std::unordered_map<PointCategory, CategoryColour> categoryColours;

	extern std::unordered_map<PointCategory, std::string> categoryNames;

}
