# pragma once
#include <iostream>
#include <deque>
#include <mutex>

#include "CTR.h"
#include "CTRFactory.h"
#include "MechanicsBasedKinematics.h"
#include "LieGroup.h"
#include "LineDetection.h"
#include "ModelBasedLineEstimation.h"
#include "WallSegmentation.h"
#include "FilterLibrary.h"
#include "LeakDetection.h"
#include "IncrementalValveModel.h"

#include "Classifier.h"

#include <opencv2/core/core.hpp>

#include <vtkPolyLineSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRegularPolygonSource.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>
#include <vtkLineSource.h>

class ReplayEngine
{
		
		::std::string		dataFilename;
		::std::string		pathToImages;

		::std::string		pathToClassifier;

		::std::mutex		robot_mutex;
		::std::mutex		img_mutex;

		CTR*							robot;
		MechanicsBasedKinematics*		kinematics;
		double							velocityCommand[2];
		double							robot_rotation;
		double							imageInitRotation;
		double							actualPosition[3];

		float				contactCurr;
		float				contactPrev;
		::Eigen::Vector2d	centroidEig2;
		double				joints[5];
		::std::vector<SE3>  frames;

		::cv::Mat			img;
		::std::vector<::std::string> imList;
		::std::deque<::std::string> imQueue;

		bool				lineDetected;
		LineDetector		lineDetector;
		RecursiveFilter::MovingAverageFilter		r_filter;
		RecursiveFilter::RecursiveMovingAverage		theta_filter;
		RecursiveFilter::DirectionMovingAverageFilter theta_filter_complex;

		ModelBasedLineEstimation	modelBasedLine;
		LineDetector			m_dummyLine;
		LeakDetector			m_leakDetector;

		::Eigen::Vector2d	m_valve_tangent_prev;
		::Eigen::Vector2d	m_velocity_prev;

		BagOfFeatures		bof;

		WallSegmentation	wallDetector;
		bool				wallDetected;

		int					offset;

		::std::vector<int> contact;
		::std::vector<int> contact_filtered;

		RecursiveFilter::MedianFilter filter;

		vtkSmartPointer<vtkRegularPolygonSource> circleSource;
		vtkSmartPointer<vtkPolyDataMapper> mapperCircle;
		vtkSmartPointer<vtkActor> actorCircle;

		vtkSmartPointer<vtkSphereSource> leakSource1;
		vtkSmartPointer<vtkPolyDataMapper> mapperleak1;
		vtkSmartPointer<vtkActor> actorleak1;

		vtkSmartPointer<vtkSphereSource> leakSource2;
		vtkSmartPointer<vtkPolyDataMapper> mapperleak2;
		vtkSmartPointer<vtkActor> actorleak2;

		vtkSmartPointer<vtkSphereSource> leakSource3;
		vtkSmartPointer<vtkPolyDataMapper> mapperleak3;
		vtkSmartPointer<vtkActor> actorleak3;

		vtkSmartPointer<vtkSphereSource> pointOnCircleSource;
		vtkSmartPointer<vtkPolyDataMapper> pointOnCircleMapper;
		vtkSmartPointer<vtkActor> pointOnCircleActor;

		vtkSmartPointer<vtkLineSource> lineSource;
		vtkSmartPointer<vtkPolyDataMapper> lineMapper;
		vtkSmartPointer<vtkActor> lineActor;

		vtkSmartPointer<vtkLineSource> robotAxisSource;
		vtkSmartPointer<vtkPolyDataMapper> robotAxisMapper;
		vtkSmartPointer<vtkActor> RobotAxisActor;

		int		counter;

		bool	new_version;

		::Eigen::Vector2d tangent_prev;

		IncrementalValveModel iModel;
public:
		enum STATUS {LINE_DETECTION, WALL_DETECTION, LEAK_DETECTION} status;
		enum WALL_TO_FOLLOW {LEFT, TOP, BOTTOM};

public:	

		ReplayEngine(const ::std::string& dataFilename, const ::std::string& pathToImages);

		~ReplayEngine();

		const ::std::string& getDataPath() const {return dataFilename;};

		const ::std::string& getpathToImages()  const {return pathToImages;};

		void setClassifier(const ::BagOfFeatures& classifier)
		{
			this->bof = classifier;
		};

		void run();

		void setStatus(::ReplayEngine::STATUS status) {this->status = status;};

	private:
		static void simulate(void* tData);

		static void displayRobot(void* tData);

		static void vtkRender(void* tData);

		static void networkPlot(void* tData);

		bool updateRobot(const double jointValues[], ::std::vector<SE3>& frames);

		void getFrames(::std::vector<SE3>& frames) 
		{
			frames.clear();

			for (int i = 0; i < this->frames.size(); ++i)
				frames.push_back(this->frames[i]);
		};

		void getTipPosition(double position[3]);

		void setFrames(::std::vector<SE3>& frames) 
		{
			
			this->frames.clear();	
			for (int i = 0; i < frames.size(); ++i)
				this->frames.push_back(frames[i]);
		};

		void setJoints(double joints[]);

		void initializeOrigin();

		void initializeRobotAxis();

		void popNextImage();

		void getCurrentImage(::cv::Mat& im);

		void processDetectedLine(const ::cv::Vec4f& line, ::cv::Mat& img , ::cv::Vec2f& centroid, ::Eigen::Vector2d& centroidEig, ::Eigen::Vector2d& tangentEig,  ::Eigen::Vector2d& tangentEigFiltered);

		// this simulates the line-following controller
		void applyVisualServoingController(const ::Eigen::Vector2d& centroid, const ::Eigen::Vector2d& tangent, ::Eigen::Vector2d& commandedVelocity);

		// this simulates the wall-following controller
		void applyVisualServoingController(int x, int y, ::Eigen::Vector3d& commandedVelocity, ReplayEngine::WALL_TO_FOLLOW wall = WALL_TO_FOLLOW::LEFT);

		void detectLine(::cv::Mat& imag);

		void detectWall(::cv::Mat& img, int& x, int& y);

		void detectWall(::cv::Mat& img);

		void detectLeak(::cv::Mat& img);

		bool checkTransition();

		void getInnerTubeRotation(double& rotation);

		void initializeValveModel();

		void updateRobotPositionModel(double fourier[3]);

		void checkTangentDirection(::Eigen::Vector2d& tangentEig);

		void followLeft(int x, int y, ::Eigen::Vector3d& commandedVelocity);
		void followTop(int x, int y, ::Eigen::Vector3d& commandedVelocity);
		void followBottom(int x, int y, ::Eigen::Vector3d& commandedVelocity);

		void plotCommandedVelocities(const ::cv::Mat& img, const ::Eigen::Vector2d& centroidEig, const ::Eigen::Vector2d& tangentEig);

		void computePointOnValve(::Eigen::Vector3d& centroidOnValve, const ::Eigen::Vector2d& channelCenter, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal);

		void initializeLeaks();

};