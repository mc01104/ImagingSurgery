// ReplayExperiment.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <fstream>

#include "ReplayEngine.h"
#include "FileUtils.h"
#include "classifier.h"
#include "ValveModel.h"
#include "IncrementalValveModel.h"
#include "Registration.h"
#include "Utilities.h"
#include "CTRFactory.h"
#include "CTR.h"

#include "OpenCVClock.h"

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSTLReader.h>

#include <thread>
#include "TubeVisualizer.h"
#include "robotVisualizer.h"

void testVectorOperations();
void testBuildingModel();
void testJointConversion();
void testMapFunctions();
void testBenchtopDetection();
bool testLeakDetection();
void testMultipleWires();
void testIncrementalModel();
int testReplayEngine();
void testRegistration();
void testClock();
void testArrow();
void testTube();
void testRobotVisulization();
void testHeartVisualization();
double computeClockDistance(double c1, double c2);
void testClockDistance();

#define __NEW_VERSION__

enum LINE_MODE
{
	LSQ,
	HOUGH,
	RANSAC
};

int _tmain(int argc, _TCHAR* argv[])
{
	//testMapFunctions();
	//testBenchtopDetection();
	//testLeakDetection();
	//testMultipleWires();
	testReplayEngine();
	//testClockDistance();
	//testIncrementalModel();
	//testRegistration();
	//testClock();

	//testArrow();
	//testTube();
	//testRobotVisulization();

	//testHeartVisualization();
}

void testClockDistance()
{
	double c1 = 12.8;
	double c2 = 0.0;

	::std::cout << computeClockDistance(c2, c1) << ::std::endl;

}


double
computeClockDistance(double c1, double c2)
{

	double distance = 0;
	double d1 = 0, d2 = 0;

	d1 = ::std::abs(c1 - c2);
	d2 = ::std::abs(12 + (c2 - c1));

	distance = ::std::min(d1, d2);

	(distance > 12 ? distance -= 12: distance);
	
	return  distance;	

}

void testHeartVisualization()
{  

	std::string inputFilename = "Zygote_Heart.stl";
 
  vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
  reader->SetFileName(inputFilename.c_str());
  reader->Update();
 
  // Visualize
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(reader->GetOutputPort());
 
  vtkSmartPointer<vtkActor> actor =  vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
 
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
 
  renderer->AddActor(actor);
  renderer->SetBackground(.3, .6, .3); // Background color green
 
  renderWindow->Render();
  renderWindowInteractor->Start();

}

void testRobotVisulization()
{
	CTR* robot = CTRFactory::buildCTR("");
	MechanicsBasedKinematics* kinematics = new MechanicsBasedKinematics(robot, 100);
	kinematics->ActivateIVPJacobian();

	RobotVisualizer rVis(*kinematics);
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();

	vtkSmartPointer<vtkRenderWindow> renWindow = vtkSmartPointer<vtkRenderWindow>::New();
	renWindow->AddRenderer(renderer);

	vtkSmartPointer<vtkRenderWindowInteractor> iRen = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	iRen->SetRenderWindow(renWindow);

	rVis.registerVisualizer(renderer);

	double configuration[5] = {0, 0, 35, 0, 0};
	rVis.update(configuration);

	iRen->Initialize();
	renWindow->Render();

	iRen->Start();
}

void testArrow()
{
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();

	double startPoint[3] = {0, 0, 0}, endPoint[3] = {0, 0, 1}, color[3] = {1, 0, 0};
	ArrowVisualizer arrow(startPoint, endPoint, color);

	vtkSmartPointer<vtkRenderWindow> renWindow = vtkSmartPointer<vtkRenderWindow>::New();
	renWindow->AddRenderer(renderer);

	vtkSmartPointer<vtkRenderWindowInteractor> iRen = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	iRen->SetRenderWindow(renWindow);

	renderer->AddActor(arrow.getActor());
	
	iRen->Initialize();
	renWindow->Render();

}

void testTube()
{
	CTR* robot = CTRFactory::buildCTR("");
	MechanicsBasedKinematics* kinematics = new MechanicsBasedKinematics(robot, 100);
	kinematics->ActivateIVPJacobian();

	double configuration[5] = {0, 0, 35, 0, 0};
	double rotation[3] = {0};
	double translation[3] = {0};
	MechanicsBasedKinematics::RelativeToAbsolute(robot, configuration, rotation, translation);

	kinematics->ComputeKinematics(rotation, translation);

	int numOfPoints = 100;
	::std::vector<double> s = linspace(0, robot->GetLength(), numOfPoints);
	::std::vector<SE3> frames(s.size());
	kinematics->GetBishopFrame(s, frames);

	::std::vector<double> point(3);
	::std::vector<::std::vector<double>> points;
	for (int i = 0; i < frames.size(); ++i)
	{
		point[0] = frames[i].GetPosition()[0];
		point[1] = frames[i].GetPosition()[1];
		point[2] = frames[i].GetPosition()[2];
		points.push_back(point);
	}

	point[0] = frames.back().GetPosition()[0] + frames.back().GetZ()[0] * 20;
	point[1] = frames.back().GetPosition()[1] + frames.back().GetZ()[1] * 20;
	point[2] = frames.back().GetPosition()[2] + frames.back().GetZ()[2] * 20;

	points.push_back(point);
	double color[3] = {45.0/255.0, 98.0/255.0, 183.0/255.0};
	double radius = 1.0;
	TubeVisualizer tube(color, radius);

	tube.update(points);

	double colorOuter[3] = {94.0/255.0, 116.0/255.0, 150.0/255.0};
	double outerRadius = 2;
	TubeVisualizer outerTube(colorOuter, outerRadius);

	::std::vector<::std::vector<double>> outerPoints;

	s = linspace(0, robot->GetLength(), 100);

	::std::vector<bool> mask(3); 
	
	for (int i = 0; i < frames.size(); ++i)
	{
		for (int j = 0; j < 3; ++j)
			mask[j] = false;

		point[0] = frames[i].GetPosition()[0];
		point[1] = frames[i].GetPosition()[1];
		point[2] = frames[i].GetPosition()[2];
		outerPoints.push_back(point);

		robot->GetExistingTubes(s[i], mask);

		if (!mask[1])
			break;
	}
	outerTube.update(outerPoints);

	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renWindow = vtkSmartPointer<vtkRenderWindow>::New();
	renWindow->AddRenderer(renderer);

	vtkSmartPointer<vtkRenderWindowInteractor> iRen = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	iRen->SetRenderWindow(renWindow);

	renderer->AddActor(tube.getActor());
	renderer->AddActor(outerTube.getActor());

	iRen->Initialize();
	renWindow->Render();
	iRen->Start();

}

void testClock()
{
	bool videoOn = false;
	OpenCVClock clockCV;
	::cv::VideoWriter video;
	if (videoOn)
		 video = ::cv::VideoWriter("clock.avi", ::cv::VideoWriter::fourcc('M','P','E','G'), 20, ::cv::Size(500, 250));
	
	::std::string img_path = "green_circle.png";
	
	::cv::Mat img;
	double time;
	for (int i = 0; i < 100000; ++i)
	{
		img = ::cv::imread(img_path);

		time = i * 0.01;
		if (i == 100)
			clockCV.setRegistrationOffset(1, 0);
		if (i == 200)
			clockCV.setRegistrationOffset(1, 0);
		if (i == 300)
			clockCV.setRegistrationOffset(1, 0);

		clockCV.update(img, time);

		::std::cout << time << ::std::endl;
		::cv::imshow("img", img);
		::cv::waitKey(10);

		if (videoOn)
			video.write(img);

	}
	if (videoOn)
		video.release();
}

void testRegistration()
{
	RegistrationHandler reg;

	::std::string base_folder = "Z:/Public/Data/Cardioscopy_project/Registration mark/";

	::std::string experimentStr = "blue_wire_tip_07";
	::std::string img_path = base_folder + experimentStr;

	::cv::VideoWriter video = ::cv::VideoWriter("registration_" + experimentStr + ".avi", ::cv::VideoWriter::fourcc('M','P','E','G'), 20, ::cv::Size(500, 250));

	::std::vector<::std::string> imList;
	int num = getImList(imList, img_path);
	std::sort(imList.begin(), imList.end(), numeric_string_compare);	

	double error = 0;
	int offset = 0;
	::cv::Mat img, img_rec, img_double;
	for (int i = 0; i < imList.size() - offset; ++i)
	{
		img = ::cv::imread(checkPath(img_path + "/" + imList[i + offset]));

		reg.processImage(img, error, img_rec);

		::cv::hconcat(img, img_rec, img_double);
		::cv::imshow("img", img_double); 
		::cv::waitKey(1);

		video.write(img_double);

		::std::cout << i << ::std::endl;
	}

	video.release();
}

int testReplayEngine()
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
	
	// test green line exvivo
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-09-12_exvivotest/2017-09-12_14-58-17";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-09-12_exvivotest/2017-09-12_14-54-17";
	

	// test leak detection
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-09-14_bypass_cardioscopy/2017-09-14_15-16-00";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-05-11_bypass_cardioscopy/Videos_2017-05-11/2017-05-11_15-44-19";
	
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-10-12_bypass_cardioscopy/Videos_2017-10-12/2017-10-12_12-30-33";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-10-12_bypass_cardioscopy/Videos_2017-10-12/2017-10-12_12-31-12";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-10-12_bypass_cardioscopy/Videos_2017-10-12/2017-10-12_12-33-19";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-10-12_bypass_cardioscopy/Videos_2017-10-12/2017-10-12_12-41-10";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-10-12_bypass_cardioscopy/Videos_2017-10-12/2017-10-12_12-43-14";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-10-12_bypass_cardioscopy/Videos_2017-10-12/2017-10-12_13-03-41";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-10-12_bypass_cardioscopy/Videos_2017-10-12/2017-10-12_13-06-02";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-10-12_bypass_cardioscopy/Videos_2017-10-12/2017-10-12_13-07-24";

	// switching directions
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-10-12_bypass_cardioscopy/Videos_2017-10-12/2017-10-12_13-39-12";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-10-19_bypass_cardioscopy/Videos_2017-10-19/2017-10-19_15-11-37";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-10-19_bypass_cardioscopy/Videos_2017-10-19/2017-10-19_14-04-54";

	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-11-09_bypass_cardioscopy/Videos_2017-11-09/2017-11-09_12-14-23";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-11-09_bypass_cardioscopy/Videos_2017-11-09/2017-11-09_15-19-25";

	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2018-01-04_bypass_cardioscopy/Videos_2018-01-04/2018-01-04_11-35-45";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2018-01-04_bypass_cardioscopy/Videos_2018-01-04/2018-01-04_11-46-55";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2018-01-04_bypass_cardioscopy/Videos_2018-01-04/2018-01-04_12-49-22";
	
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2018-01-11_bypass_cardioscopy/Videos_2018-01-11/2018-01-11_12-19-31";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2018-01-11_bypass_cardioscopy/Videos_2018-01-11/2018-01-11_12-21-04";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2018-01-11_bypass_cardioscopy/Videos_2018-01-11/2018-01-11_12-26-46";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2018-01-11_bypass_cardioscopy/Videos_2018-01-11/2018-01-11_12-34-51";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2018-01-11_bypass_cardioscopy/Videos_2018-01-11/2018-01-11_13-09-06";

	::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2018-01-16_exvivo/2018-01-16_15-54-44";
	
	
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/Registration mark/blue_wire_tip_07";
	
	::std::string path_to_classifier = "../Export_executables/SVM_params_surgery/output_";

	BagOfFeatures contact_classifier;
	contact_classifier.load(path_to_classifier);

	ReplayEngine engine(checkPath(img_path + "/data.txt"), img_path);
	engine.setClassifier(contact_classifier);
	engine.setStatus(ReplayEngine::LINE_DETECTION); 
	engine.run();

	//testMapFunctions();
	//testBenchtopDetection();
	//testLeakDetection();
	//testMultipleWires();
	//testIncrementalModel();

	//double a = 2.336;
	//::std::cout << std::setprecision(50) << a << ::std::endl;
	//::std::cout << std::setprecision(50) <<::std::pow(a, 4) << ::std::endl;
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

void testBenchtopDetection()
{
	LineDetector lineDetector;

	::std::string imageName = "green_circle.png";

    ::cv::Mat image;
    image = ::cv::imread(imageName.c_str(), ::cv::IMREAD_COLOR); // Read the file

    if( image.empty() )                      
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return;
    }

	::cv::Vec4f line;
	::cv::Vec2f centroid;
	lineDetector.processImageDemo(image, line, centroid, false, 10, LineDetector::MODE::CIRCUM);

	::cv::line(image, ::cv::Point(centroid[0] - 100 * line[0], centroid[1] - 100 * line[1]), ::cv::Point(centroid[0] + 100 * line[0], centroid[1] + 100 * line[1]), ::cvScalar(0, 0, 255), 2);
    ::cv::imshow( "Display window", image );   
	::cv::waitKey();


}

bool testLeakDetection()
{
	LeakDetector leakDetector;
	LineDetector lineDetector;
	BagOfFeatures bof;
	::std::string path_to_classifier = "../Export_executables/SVM_params_surgery/output_";
	bof.load(path_to_classifier);

	//::std::string pathToVideo = "Z:/Public/Data/Cardioscopy_project/2016-09-08_Bypass_cardioscopy/awaiba_videos_0908/2016-09-08_12-58-08.avi";
	//::std::string pathToVideo = "Z:/Public/Data/Cardioscopy_project/2016-09-08_Bypass_cardioscopy/awaiba_videos_0908/2016-09-08_13-05-08.avi";
	//::std::string pathToVideo = "Z:/Public/Data/Cardioscopy_project/2016-10-06_bypass_cardioscopy/Videos_2016-10-06/2016-10-06_13-21-56.avi";
	//::std::string pathToVideo = "Z:/Public/Data/Cardioscopy_project/2016-10-06_bypass_cardioscopy/Videos_2016-10-06/2016-10-06_13-32-37.avi";
	//::std::string pathToVideo = "Z:/Public/Data/Cardioscopy_project/2016-10-06_bypass_cardioscopy/Videos_2016-10-06/2016-10-06_13-41-27.avi";
	//::std::string pathToVideo = "Z:/Public/Data/Cardioscopy_project/2016-10-06_bypass_cardioscopy/Videos_2016-10-06/2016-10-06_13-45-18.avi";
	//::std::string pathToVideo = "Z:/Public/Data/Cardioscopy_project/2016-11-10_bypass_cardioscopy/Videos_2016-11-10/2016-11-10_10-35-56.avi";
	//::std::string pathToVideo = "Z:/Public/Data/Cardioscopy_project/2016-11-10_bypass_cardioscopy/Videos_2016-11-10/2016-11-10_10-58-44.avi";
	::std::string pathToVideo = "Z:/Public/Data/Cardioscopy_project/test_leak_detection/2017-09-14_13-26-03.mp4";
	
	
	::std::vector<::std::string> result;
	result = splitString(pathToVideo, '/');
	result = splitString(result.back(), '.');
	::std::string output_filename = result[0] + "_leaks_detected.avi";

	::cv::VideoWriter video(output_filename, ::cv::VideoWriter::fourcc('M','P','E','G'), 30, ::cv::Size(250, 250));
	
	::cv::VideoCapture cap(pathToVideo);

  // Check if camera opened successfully
  if(!cap.isOpened())
  {
    ::std::cout << "Error opening video stream or file" << endl;
    return false;
  }

  ::cv::Mat img;     
  ::cv::Vec4f line;
  ::cv::Vec2f centroid;

  while(1)
  {
 
    // Capture frame-by-frame
    cap >> img;
  
    // If the frame is empty, break immediately
    if (img.empty())
      break;
 
	int x = 0, y = 0;
	float response = 0;
	bof.predict(img, response);
	// if valve is detected -> apply leak detection
	//if (lineDetector.processImage(img, line,centroid, false, 5, LineDetector::MODE::TRANSITION)) // this is often more reliably detecting the valve than the classifier
	//	leakDetector.processImage(img, x, y);
	if (response == 1)
		leakDetector.processImage(img, x, y);

    ::cv::imshow( "Frame", img );
	video.write(img);

	// Press  ESC on keyboard to exit
    char c=(char) ::cv::waitKey(25);
    if(c==27)
      break;
  }
  
  // When everything done, release the video capture object
  cap.release();
  video.release();
  // Closes all the frames
  ::cv::destroyAllWindows();
     
  return true;
}


void	testMultipleWires()
{
	LineDetector lineDetector;

	LINE_MODE mode = HOUGH;

	// load images
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-09-25_16-36-09";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/test_green_wire_detection/2017-09-27_15-43-21";
	//::std::string img_path = "Z:/Public/Data/Cardioscopy_project/2017-12-22_white_sutures/2017-12-22_15-24-01";
	::std::string img_path = "F:/2018-01-03_16-40-19";
	
	::std::vector<::std::string> imList;
	int count = getImList(imList, checkPath(img_path + "/" ));
	::std::cout << "Loaded: " << count << " images" << ::std::endl;
	std::sort(imList.begin(), imList.end(), numeric_string_compare);	

	::cv::Vec4f line;
	::cv::Vec2f centroid;

	::cv::Mat img, thres;
	::cv::Mat img_double;

	// Hough Transform outputs vector of lines (multiple detection
	::std::vector<::cv::Vec4i> lines;
	
	thres = ::cv::Mat::zeros(250, 250, CV_8UC1);

	::std::string path_to_classifier = "../Export_executables/SVM_params_surgery/output_";

	BagOfFeatures contact_classifier;
	contact_classifier.load(path_to_classifier);

	::cv::VideoWriter video(GetDateString() + ".avi", ::cv::VideoWriter::fourcc('M','P','E','G'), 30, ::cv::Size(2 * 250, 250));
	bool lineDetected = false;
	float response = 0;

	int counter = 0;

	for (auto& t : imList)
	{
		lines.clear();
		
		img = ::cv::imread(checkPath(img_path + "/" + t));
		
		contact_classifier.predict(img, response);

		if (response == 1)
			::std::cout << "contact detected in frame:" << counter++ << ::std::endl;
		switch(mode)
		{
			case LSQ:
				lineDetected = lineDetector.processImageDemo(img, line, centroid, false, 5, ::LineDetector::MODE::CIRCUM, thres);
				break;
			case HOUGH:
				lineDetected = lineDetector.processImageDemoHoughProbabilistic(img, lines, centroid, thres, line);
				break;
			case RANSAC:
				lineDetected = lineDetector.processImageDemoRANSAC(img, line, centroid, thres);
				break;
		}
		if (lineDetected)
		{
			if (mode == HOUGH)
			{
				for( size_t i = 0; i < lines.size(); i++ )
				{
					::cv::Vec4i l = lines[i];
					::cv::line( img, ::cv::Point(l[0], l[1]),::cv::Point(l[2], l[3]), ::cv::Scalar(255,0,0), 3, CV_AA);
				}

			}
			
			::cv::line(img, ::cv::Point(centroid[0] - line[0] * 100, centroid[1] - line[1] * 100), ::cv::Point(centroid[0] + line[0] * 100, centroid[1] + line[1] * 100),
											::cv::Scalar(0, 0, 255), 2);

		}

		//::std::cout << "alpha:"  << line[0] << ", beta:" << line[1] << ::std::endl;

		::cv::hconcat(img, thres, img_double);
		::cv::imshow("img", img_double); 
		video.write(img_double);
		::cv::waitKey();
	}

	video.release();

}



void testIncrementalModel()
{
	IncrementalValveModel model;
	::Eigen::Vector3d point;
	double cPosition;

	model.setRegistrationRotation(180);
	model.getClockfacePosition(1, 0, 0, cPosition, point);
	model.clockfaceToWorldPosition(6, point);

	::std::cout << "clockface position: " << cPosition << ::std::endl;
	::std::cout << "time 6 corresponds to:" << point.transpose() << ::std::endl;

	model.clockfaceToWorldPosition(5, point);
	::std::cout << "time 5 corresponds to:" << point.transpose() << ::std::endl;

	::std::vector<::Eigen::Vector3d> leaks;
	model.getLeakPosition(leaks);

	for (int i = 0; i < 3; ++i)
		::std::cout << "leak " << num2str(i) << ":" << leaks[i].transpose() << ::std::endl;
	//::std::vector<::std::string> dataStr = ReadLinesFromFile(filename);
	//::std::vector<double> tmpData;

	//double duration = 0;
	//double duration_max = 0;
	//for (int i = 0; i < dataStr.size(); ++i)	
	//{
	//	tmpData = DoubleVectorFromString(dataStr[i]);
	//    auto start = std::chrono::system_clock::now();
	//	model.updateModel(tmpData[0], tmpData[1], tmpData[2]);
	//	auto end = std::chrono::system_clock::now();
 //
	//	std::chrono::duration<double> elapsed_seconds = end-start;
	//	std::time_t end_time = std::chrono::system_clock::to_time_t(end);
 //
	//	//std::cout << "finished computation at " << std::ctime(&end_time)
	//	//		  << "elapsed time: " << elapsed_seconds.count() << "s\n";

	//	duration += elapsed_seconds.count();
	//	if (duration > duration_max)
	//		duration_max = duration;
	//	//::std::cout << "point " << i << ::std::endl;
	//	//::std::cout << "radius: " << model.getRadius() << ::std::endl;

	//	//double center[3] = {0};
	//	//model.getCenter(center);
	//	//::std::cout << "center: ";
	//	//PrintCArray(center, 3);

	//	//double normal[3] = {0};
	//	//model.getNormal(normal);
	//	//::std::cout << "normal: ";
	//	//PrintCArray(normal, 3);
	//}
	//::std::cout << "avg duration:" << duration/dataStr.size() << ::std::endl;
	//::std::cout << "max duration:" << duration_max << ::std::endl;
	//::std::cout << "radius: " << model.getRadius() << ::std::endl;

	//double center[3] = {0};
	//model.getCenter(center);
	//::std::cout << "center: ";
	//PrintCArray(center, 3);

	//double normal[3] = {0};
	//model.getNormal(normal);
	//::std::cout << "normal: ";
	//PrintCArray(normal, 3);

	////::Eigen::Vector3d point(0, 10, 0), velocity(1, 0, 0);

	////int direction = model.getDirectionOfMotion(point, velocity);

	////::std::cout << (direction == 0 ? "clockwise" : "counter-clockwise") << ::std::endl;
}