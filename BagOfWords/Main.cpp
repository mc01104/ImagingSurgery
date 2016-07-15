#include <opencv2\opencv.hpp>
#include <opencv2\xfeatures2d\nonfree.hpp>
#include <opencv2\features2d.hpp>
#include <opencv2/ml.hpp>

#include <iostream> 
#include <stdexcept> 
#include <algorithm>
#include <deque>

#include "Main.h"
#include "BOW_lowlevel.h"
#include "FileUtils.h"
#include "Network_force.h"
#include "helper_parseopts.h"


/*bool testBOW(std::string path, BOW_l bow)
{

	::std::vector<::std::string> imList;

	int count = getImList(imList,path);

	for(int i=0; i<imList.size();i++)
	{
		float response = 0.0;
		std::string filepath = path + "\\" + imList[i];
		if (bow.predictBOW(filepath,response)) ::std::cout << "Response : " << response << ::std::endl;
		else ::std::cout << "Error in prediction" << ::std::endl;
	}
	return true;
}*/



bool testBOW(std::string path, BOW_l bow, bool visualization = false)
{

	
	::std::vector<::std::string> imList;
	int count = getImList(imList,path);
	std::sort(imList.begin(), imList.end(), numeric_string_compare);

	std::deque<float> values(5,0);
	float average_val = 0.0;

	::std::vector<::std::string> classes = bow.getClasses();

	for(int i=0; i<imList.size();i++)
	{
		float response = 0.0;
		std::string filepath = path + "\\" + imList[i];
		if (bow.predictBOW(filepath,response)) 
		{
			if (classes[(int) response] == "Free") response = 0.0;
			else response = 1.0;

			float popped = values.front();
			values.pop_front();
			values.push_back(response);

			average_val = (average_val*5.0 - popped + response)/5.0;

			if(visualization)
			{
				::cv::Mat img = ::cv::imread(filepath);

				::cv::putText(img,cv::String(::std::to_string(average_val).c_str()),cv::Point(10,50),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(0,255,0));
				::cv::imshow("Image", img);
				::cv::waitKey(0);
			}


		}
		else ::std::cout << "Error in prediction" << ::std::endl;
	}
	return true;
}





/*
int main( int argc, char** argv )
{

	std::string base_folder = "M:\\Public\\Data\\Cardioscopy_project\\ContactDetection_data\\";

	std::string train_path = base_folder + "train\\";

	std::string test_path_contact = base_folder + "test\\Contact\\";
	std::string test_path_free =  base_folder + "test\\Free\\";

	::std::string test_path_surgery =  base_folder + "..\\2016-05-26_Bypass_Cardioscopy\\Awaiba_Surgery_20160526\\2016-05-26_14-10-11\\";

	std::string output_path = base_folder + "output_";

	std::string ip = "192.168.0.12";

	float gain = 3.0;


	if(cmdOptionExists(argv, argv+argc, "-i"))
    {
		char * inputfile = getCmdOption(argv, argv + argc, "-i");
		test_path_surgery  = ::std::string(inputfile);
    }

	if(cmdOptionExists(argv, argv+argc, "-s"))
    {
		char * outputfile = getCmdOption(argv, argv + argc, "-s");
		output_path  = ::std::string(outputfile);
    }

	if(cmdOptionExists(argv, argv+argc, "-ip"))
    {
		char * s_ip = getCmdOption(argv, argv + argc, "-ip");
		ip  = ::std::string(s_ip);
    }

	if(cmdOptionExists(argv, argv+argc, "-g"))
    {
		char * s_gain = getCmdOption(argv, argv + argc, "-g");
		try { gain  = atof(s_gain); }
		catch ( const std::exception & e ) { gain = 3.0;}
    }

	if(cmdOptionExists(argv, argv+argc, "-h"))
    {
		std::cout << "Command line options are: \n -i for input path of image files \n -s for basepath of saved SVM files \n -ip to set the IP address of the server \n -g to set the force gain" << ::std::endl;
		return 0;
    }

	

	BOW_l bow;

	//if (bow.trainBOW(train_path))
	//{
	//	bow.SaveToFile(output_path);

	//	/*::std::cout << "Test with Contact" << ::std::endl;
	//	testBOW(test_path_contact,bow);

	//	::std::cout << "Test with Free file" << ::std::endl;
	//	testBOW(test_path_free,bow);
	//}

	//::std::cout << "Load from file test" << ::std::endl;
	//if (bow.LoadFromFile(output_path)) 
	//{
	//	::std::cout << "Test with Contact" << ::std::endl;
	//	testBOW(test_path_contact,bow, true);

	//	::std::cout << "Test with Free file" << ::std::endl;
	//	testBOW(test_path_free,bow, true);

	//	testBOW(test_path_surgery,bow, true);
	//}
	//else 
	//{
	//	::std::cout << "Error in BOW loading" << ::std::endl;
	//}

	Network_force testForce(output_path,test_path_surgery);
	testForce.setForceGain(gain);
	testForce.runThreads();

	system("pause");
	return 0;
}*/