#pragma once

#include <ostream>
#include <iostream>

namespace common
{
	// singleton output stream, which allows us to disable the outputs
	// if we don't want a verbose program
	// - also a way for us to extend how we output debug messages,
	// the class can be extended to write debug output to a file instead of cout,
	// for example.
	class DebugOut : private std::streambuf, public std::ostream
	{
		public:
			bool enabled = true;

			static DebugOut& instance();

		private:
			DebugOut();

			static DebugOut* _instance;

			int overflow(int c);

			// sends data to std::cout
			void toCout(int c);
	};
}
