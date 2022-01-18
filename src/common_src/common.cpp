#include "common.hpp"

namespace common
{
	bool checkFile(const std::string& path)
	{
		std::ifstream stream(path);
		return stream.good();
	}
}
