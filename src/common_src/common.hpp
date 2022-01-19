#pragma once

#include "point_categories.hpp"
#include "debug_out.hpp"

#include <fstream>
#include <string>
#include <cstdarg>
#include <memory>

namespace common
{
	// returns false if the file is not accessible (doesn't exist / in use)
	bool checkFile(const std::string&);

	// returns std::string using sprintf style formatting.
	std::string fstring(const std::string& format, ...);
}
