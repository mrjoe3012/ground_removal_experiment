#include "common.hpp"

using common::DebugOut;

int main()
{
	DebugOut& debug = DebugOut::instance();

	debug << "Hello from algorithm simulation!!" << std::endl;

	return 0;
}
