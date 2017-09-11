// ReplayExperiment.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <fstream>

#include "ReplayEngine.h"
#include "FileUtils.h"
#include "classifier.h"
#include "ValveModel.h"

#include "CTRFactory.h"
#include "CTR.h"

void testVectorOperations();
void testBuildingModel();
void testJointConversion();
void testMapFunctions();

#define __NEW_VERSION__

int _tmain(int argc, _TCHAR* argv[])
{

	//// ------- WALL SEGMENTATION ----------- ///////
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-05-11_bypass_cardioscopy/Videos_2017-05-11/2017-05-11_15-10-53";	// OK
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-05-11_bypass_cardioscopy/Videos_2017-05-11/2017-05-11_15-15-59";	// OK
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-05-11_bypass_cardioscopy/Videos_2017-05-11/2017-05-11_15-22-27";	// OK
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-05-11_bypass_cardioscopy/Videos_2017-05-11/2017-05-11_15-24-28";	// OK
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-05-11_bypass_cardioscopy/Videos_2017-05-11/2017-05-11_15-29-14";	// OK
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-07-06_bypass_cardioscopy/Videos_2017-07-06/2017-07-06_13-32-10";	// OK
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-08-15_bypass_cardioscopy/Videos_2017-08-15/2017-08-15_12-43-50";	// no tissue detection
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-08-15_bypass_cardioscopy/Videos_2017-08-15/2017-08-15_14-40-02";	// OK
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-08-24_bypass_cardioscopy/Videos_2017-08-24/2017-08-24_13-06-34";	// OK with mask
	//// --------------------------------------////////


	//// ------- CIRCUM ----------- ///////
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-08-15_bypass_cardioscopy/Videos_2017-08-15/2017-08-15_13-57-11";	// OK 
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-08-15_bypass_cardioscopy/Videos_2017-08-15/2017-08-15_13-50-58";		// loses it when it goes to the margin

	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-08-24_bypass_cardioscopy/Videos_2017-08-24/2017-08-24_14-00-09";	
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-08-24_bypass_cardioscopy/Videos_2017-08-24/2017-08-24_12-58-07";	
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-08-24_bypass_cardioscopy/Videos_2017-08-24/2017-08-24_13-49-13";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-07-06_bypass_cardioscopy/Videos_2017-07-06/2017-07-06_15-15-33";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-09-07_bypass_cardioscopy/Videos_2017-09-07/2017-09-07_15-54-40";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-07-06_bypass_cardioscopy/Videos_2017-07-06/2017-07-06_15-17-56";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/sink_line_detection/2017-09-09_17-25-47";
	
	::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-09-07_bypass_cardioscopy/Videos_2017-09-07/2017-09-07_12-52-16";

	::std::string path_to_classifier = "../Export_executables/SVM_params_surgery/output_";

	BagOfFeatures contact_classifier;
	contact_classifier.load(path_to_classifier);

	ReplayEngine engine(checkPath(img_path + "/data.txt"), img_path);
	engine.setClassifier(contact_classifier);
	engine.setStatus(ReplayEngine::WALL_DETECTION); 
	engine.run();

	//testMapFunctions();
	return 0;
}

void testJointConversion()
{
	CTR* robot = CTRFactory::buildCTR("");
	
	double relative_conf[5] = {0, 0, 30, 0, 10};
	double rotation[3];
	double translation[3];

	MechanicsBasedKinematics::RelativeToAbsolute(robot, relative_conf, rotation, translation);

	PrintCArray(translation, 3);

}

void testVectorOperations()
{
	::Eigen::MatrixXd data(3,3);

	data << 1, 0, 0,
		    0, 1, 0,
			0, 0, 1;

	::std::cout << "unmodified matrix" << ::std::endl;
	::std::cout << data << ::std::endl;

	::std::cout << "remove first column" << ::std::endl;
	removeColumn(data, 0);
	::std::cout << data << ::std::endl;

	::std::cout << "remove first row" << ::std::endl;
	removeRowEigen(data, 1);
	::std::cout << data << ::std::endl;

	::Eigen::VectorXd tmp(2);
	tmp << 21, 40;

	appendRowEigen(data, tmp);
	::std::cout << "appending data" << ::std::endl;
	::std::cout << data << ::std::endl;

	popFirstRowEigen(data);
	::std::cout << "poping first tow" << ::std::endl;
	::std::cout << data << ::std::endl;
}

void testBuildingModel()
{
	::std::vector<::std::string> dataStr = ReadLinesFromFile("points_for_circle_fitting_rand.txt");

	::std::ofstream circleStream("circle_fitting_results.txt");


	::std::vector<::std::string>::iterator it = dataStr.begin();
	::std::vector<double> tmpData;

	ValveModel model;
	double* center;
	double radius = 0;

	for (it; it != dataStr.end(); ++it)
	{
		tmpData = DoubleVectorFromString(*it);
		if(!model.updateModel(tmpData[0], tmpData[1], tmpData[2]))
			continue;

		center = model.getCenter();
		radius = model.getRadius();

		circleStream << center[0] << "\t" << center[1] << "\t" << center[2] << "\t" << radius << ::std::endl;
	}

	circleStream.close();
}

void testMapFunctions()
{
	::std::string testStr = "george 1 tom 2 john 3";
	::std::map<::std::string, double> mapV = createMapFromKeyValuePairs(testStr);

	::std::cout << "test map creation" << ::std::endl;
	::std::cout << mapV["george"] << " " << mapV["tom"] << " " << mapV["john"] << ::std::endl;

	::std::cout << "check stream operation" << ::std::endl;
	::std::cout << mapV << ::std::endl;
}