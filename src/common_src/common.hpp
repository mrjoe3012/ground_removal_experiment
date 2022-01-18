#pragma once

#include "point_categories.hpp"

#include <fstream>

namespace common
{
	// returns false if the file is not accessible (doesn't exist / in use)
	bool checkFile(const std::string&);
}
