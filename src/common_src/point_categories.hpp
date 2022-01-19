#pragma once
#include <tuple>

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
	const CategoryColour
		COLOUR_Unassigned = std::make_tuple(0,0,0),
		COLOUR_Cone = std::make_tuple(255,0,0),
		COLOUR_Ground = std::make_tuple(0,255,0);
		


}