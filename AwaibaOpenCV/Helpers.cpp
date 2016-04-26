#include <windows.h> // Sleep
#include <stdint.h>

#include <iostream>

#define _CRTDBG_MAP_ALLOC

#include <stdlib.h>
#include <crtdbg.h>
#include "awcorecpp.h"
#include <sstream>

#include "Helpers.h"

//using namespace std;
using namespace Core;



template<typename T, typename precision=float>
struct rnd
{ 
	T _scale, _offset;

	/// CTor.
	rnd(const T &scale, const T &offset) : _scale(scale), _offset(offset) {} 

	/// Creator.
	T operator()() const 
	{
		return _offset + static_cast<T>(
			static_cast<precision>(std::rand()) / static_cast<precision>(RAND_MAX) * static_cast<precision>(_scale));
	}
};

/// Convert timer in milliseconds to readable form.
::std::string Ms2Str(ArgbFrame::time_type time)
{
	::std::ostringstream ss;
	ss << 
		(time / 60000) << "m " <<
		((time / 1000) % 60) << "s " <<
		(time % 1000) << "ms";
	return ss.str();
}
