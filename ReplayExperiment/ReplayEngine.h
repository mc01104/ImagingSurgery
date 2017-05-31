# pragma once
#include <iostream>
#include <deque>
#include <mutex>

#include "CTR.h"
#include "CTRFactory.h"
#include "MechanicsBasedKinematics.h"
#include "LieGroup.h"
#include "LineDetection.h"

#include <opencv2/core/core.hpp>

class ReplayEngine
{
		::std::string		dataFilename;
		::std::string		pathToImages;

		::std::mutex		robot_mutex;
		::std::mutex		img_mutex;

		CTR*							robot;
		MechanicsBasedKinematics*		kinematics;

		double				joints[5];
		::std::vector<SE3>  frames;

		::cv::Mat			img;
		::std::vector<::std::string> imList;
		::std::deque<::std::string> imQueue;

		LineDetector		lineDetector;
public:	

		ReplayEngine(const ::std::string& dataFilename, const ::std::string& pathToImages);

		~ReplayEngine();

		const ::std::string& getDataPath() const {return dataFilename;};

		const ::std::string& getpathToImages()  const {return pathToImages;};

		void run();

	private:
		static void simulate(void* tData);

		static void displayRobot(void* tData);

		static void vtkRender(void* tData);

		static void networkPlot(void* tData);

		void updateRobot(const double jointValues[], ::std::vector<SE3>& frames);

		void getFrames(::std::vector<SE3>& frames) 
		{
			frames.clear();

			for (int i = 0; i < this->frames.size(); ++i)
				frames.push_back(this->frames[i]);
		};

		void setFrames(::std::vector<SE3>& frames) 
		{
			
			this->frames.clear();	
			for (int i = 0; i < frames.size(); ++i)
				this->frames.push_back(frames[i]);
		};


		void setJoints(double joints[]);

		void initializeOrigin();

		void popNextImage();

		void getCurrentImage(::cv::Mat& im);

		void processDetectedLine(const ::cv::Vec4f& line, ::cv::Mat& img , ::cv::Vec2f& centroid);
};