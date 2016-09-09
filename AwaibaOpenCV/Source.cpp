#include <windows.h> // Sleep
#include <stdint.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#define _CRTDBG_MAP_ALLOC

#include <stdlib.h>
#include <crtdbg.h>

#include <algorithm>
#include <functional>
#include "awcorecpp.h"
#include "Helpers.h"
#include <conio.h>
#include <sstream>
#include <thread>

#include "Camera_processing.h"

#include "CImg.h"
#include "helper_parseopts.h"

using namespace cimg_library;

#define cimg_use_png

using namespace Core;
using namespace cv;


int main( int argc, char** argv )
{

	int period = 20; // period = images per heart cycle period
	bool sendContact = false; // send contact only and not ratio

	if(cmdOptionExists(argv, argv+argc, "-period"))
    {
		char * s_period = getCmdOption(argv, argv + argc, "-period");
		try { period  = atoi(s_period); }
		catch ( const std::exception & e ) { period = 20;}
    }

	if(cmdOptionExists(argv, argv+argc, "-sendContact"))
    {
		sendContact = true;
    }


	Camera_processing test(period, sendContact);

}