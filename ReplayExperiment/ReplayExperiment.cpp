// ReplayExperiment.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "ReplayEngine.h"

int _tmain(int argc, _TCHAR* argv[])
{
	::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-05-23_bypass_cardioscopy/Videos_2017-05-23/2017-05-23_13-38-31";
	ReplayEngine engine("test.txt", img_path);

	engine.run();

	return 0;
}

