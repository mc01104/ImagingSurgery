#include <windows.h> // Sleep
#include <stdint.h>

#include <iostream>

#define _CRTDBG_MAP_ALLOC

#include <stdlib.h>
#include <crtdbg.h>
#include "awcorecpp.h"
#include <sstream>

//using namespace std;
using namespace Core;

/// Create random values of type T in range offset + (0..scale].
template<typename T, typename precision=float>
struct rnd;

/// Convert timer in milliseconds to readable form.
::std::string Ms2Str(ArgbFrame::time_type time);
