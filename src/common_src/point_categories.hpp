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
	};	


	// colours associated with each point category
	// (will be seen in visualizer and written to
	// output .pcd)
	extern std::unordered_map<PointCategory, CategoryColour> categoryColours;

	extern std::unordered_map<PointCategory, std::string> categoryNames;

}
