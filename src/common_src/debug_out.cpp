#include "debug_out.hpp"

namespace common
{
	DebugOut* DebugOut::_instance = nullptr;

	DebugOut::DebugOut()
	:
	std::ostream(this)
	{
	}

	DebugOut& DebugOut::instance()
	{
		if(!_instance)
			_instance = new DebugOut();
		return *_instance;
	}

	int DebugOut::overflow(int c)
	{
		if(enabled)
		{
			toCout(c);
		}
		return 0;
	}

	void DebugOut::toCout(int c)
	{
		char ch = static_cast<char>(c);
		std::cout.put(c);
	}
}
