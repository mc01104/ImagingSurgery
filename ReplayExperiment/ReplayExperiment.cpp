// ReplayExperiment.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "ReplayEngine.h"
#include "FileUtils.h"
#include "classifier.h"

int _tmain(int argc, _TCHAR* argv[])
{
	::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-05-23_bypass_cardioscopy/Videos_2017-05-23/2017-05-23_13-34-09";
	::std::string path_to_classifier = "../Export_executables/SVM_params_surgery/output_";

	BagOfFeatures contact_classifier;
	contact_classifier.load(path_to_classifier);

	ReplayEngine engine(checkPath(img_path + "/test.txt"), img_path);
	engine.setClassifier(contact_classifier);

	engine.run();

	return 0;
}

