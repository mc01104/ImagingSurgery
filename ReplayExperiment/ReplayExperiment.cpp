// ReplayExperiment.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "ReplayEngine.h"
#include "FileUtils.h"
#include "classifier.h"

int _tmain(int argc, _TCHAR* argv[])
{
	//// ------- LINE DETECTION ----------- ///////
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-05-23_bypass_cardioscopy/Videos_2017-05-23/2017-05-23_13-34-09";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-05-23_bypass_cardioscopy/Videos_2017-05-23/2017-05-23_13-38-09";
	::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-05-23_bypass_cardioscopy/Videos_2017-05-23/2017-05-23_13-21-08";

	//// ------- WALL SEGMENTATION ----------- ///////
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-05-11_bypass_cardioscopy/Videos_2017-05-11/2017-05-11_15-10-53";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-05-11_bypass_cardioscopy/Videos_2017-05-11/2017-05-11_15-15-59";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-05-11_bypass_cardioscopy/Videos_2017-05-11/2017-05-11_15-22-27";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-05-11_bypass_cardioscopy/Videos_2017-05-11/2017-05-11_15-24-28";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-05-11_bypass_cardioscopy/Videos_2017-05-11/2017-05-11_15-29-14";

	::std::string path_to_classifier = "../Export_executables/SVM_params_surgery/output_";

	BagOfFeatures contact_classifier;
	contact_classifier.load(path_to_classifier);

	ReplayEngine engine(checkPath(img_path + "/data.txt"), img_path);
	engine.setClassifier(contact_classifier);
	engine.setStatus(ReplayEngine::LINE_DETECTION);
	engine.run();

	return 0;
}

