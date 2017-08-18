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

int _tmain(int argc, _TCHAR* argv[])
{
	//// ------- LINE DETECTION ----------- ///////
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-05-23_bypass_cardioscopy/Videos_2017-05-23/2017-05-23_13-34-09";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-05-23_bypass_cardioscopy/Videos_2017-05-23/2017-05-23_13-38-09";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-05-23_bypass_cardioscopy/Videos_2017-05-23/2017-05-23_13-21-08";

	//// ------- WALL SEGMENTATION ----------- ///////
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-05-11_bypass_cardioscopy/Videos_2017-05-11/2017-05-11_15-10-53";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-05-11_bypass_cardioscopy/Videos_2017-05-11/2017-05-11_15-15-59";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-05-11_bypass_cardioscopy/Videos_2017-05-11/2017-05-11_15-22-27";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-05-11_bypass_cardioscopy/Videos_2017-05-11/2017-05-11_15-24-28";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-05-11_bypass_cardioscopy/Videos_2017-05-11/2017-05-11_15-29-14";

	//spurious detection -> going right
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-07-06_bypass_cardioscopy/Videos_2017-07-06/2017-07-06_13-32-10";

	// check transition
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-07-06_bypass_cardioscopy/Videos_2017-07-06/2017-07-06_13-32-10";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-07-06_bypass_cardioscopy/Videos_2017-07-06/2017-07-06_13-48-07";
	
	// circum
	::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-08-15_bypass_cardioscopy/Videos_2017-08-15/2017-08-15_13-57-11";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-08-15_bypass_cardioscopy/Videos_2017-08-15/2017-08-15_13-50-58";
	
	::std::string path_to_classifier = "../Export_executables/SVM_params_surgery/output_";

	BagOfFeatures contact_classifier;
	contact_classifier.load(path_to_classifier);

	ReplayEngine engine(checkPath(img_path + "/data.txt"), img_path);
	engine.setClassifier(contact_classifier);
	engine.setStatus(ReplayEngine::LINE_DETECTION);
	engine.run();
	//testVectorOperations();

	//testBuildingModel();
	//testJointConversion();


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