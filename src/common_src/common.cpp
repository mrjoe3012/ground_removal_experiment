#include "common.hpp"
#include <iostream>

namespace common
{
	bool checkFile(const std::string& path)
	{
		std::ifstream stream(path);
		return stream.good();
	}

	std::string fstring(const std::string& format, ...)
	{
		char* buffer = nullptr;

		std::va_list args;

		va_start(args, format);
		size_t bufferSize = std::vsnprintf(nullptr, 0, format.c_str(), args);
		va_end(args);

		buffer = new char[bufferSize+1];

		va_start(args, format);
		std::vsnprintf(buffer, bufferSize+1, format.c_str(), args);
		va_end(args);

		std::string result(buffer);
		delete[] buffer;

		return result;
	}
}
