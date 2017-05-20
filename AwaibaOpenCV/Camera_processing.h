// System includes
#include <windows.h> // Sleep
#include <stdint.h>
#include <stdlib.h>
#include <thread>
#include <mutex>
#include <queue>
#include <deque>
#include <numeric> 

 #include <vtkAutoInit.h>
 VTK_MODULE_INIT(vtkInteractionStyle);
 VTK_MODULE_INIT(vtkRenderingOpenGL);
#include <vtkObject.h>
#include <vtkRegularPolygonSource.h>
#include "Queue.h"
#include "BOW_lowlevel.h"
#include "classifier.h"
#include "SharedMutex.h"

// VTK
#include <vtkSmartPointer.h>
#include <vtkLine.h>
#include <vtkCellArray.h>
#include <vtkParametricSpline.h>
#include <vtkParametricFunctionSource.h>
#include <vtkTubeFilter.h>
#include <vtkLineSource.h>
#include <vtkPolyLineSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyLine.h>
#include <vtkActor.h>
#include <vtkCubeSource.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkProperty.h>
#include <vtkCommand.h>
#include <vtkPlaneSource.h>
#include <vtkCamera.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkRegularPolygonSource.h>
#include <vtkSphereSource.h>
#include <vtkArrowSource.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTransform.h>
#include <vtkVertexGlyphFilter.h>

// OpenCV includes
#include <opencv2/core/core.hpp>

// Awaiba includes
#define _CRTDBG_MAP_ALLOC
#include "awcorecpp.h"

#include "LieGroup.h"
#include "FilterLibrary.h"
#include "LineDetection.h"

using namespace Core;
using namespace cv;
//using namespace std;

struct ImgBuf {
  Mat img;
  Core::ArgbFrame::time_type timestamp;
  std::vector<double> robot_joints;
} ;


//class Filter;
//void vtkKeyboardCallback(vtkObject* caller, long unsigned int eventId, void* clientData, void* callDat);
class Camera_processing {

private:


	/********** Members private *********/
	bool m_running; 
	bool m_record; // recording images
	bool m_teleop; // robot teleoperation is ON/OFF
	bool m_newdir; // flag for saying that a savedir should be created
	bool m_rotateImage;
	bool m_points;
	std::vector<double> m_configuration; // most recent robot configuration
	Queue<ImgBuf> m_ImgBuffer; // images and timestamp buffer

	::std::string m_saveDir;
	::std::string m_imgDir;

	// Camera management
    ::std::string m_board;
	bool m_ControlLED;
	Core::Manager m_Manager;
	Core::PipeConfig m_pipeConfig;
	Core::SensorSource m_AwaibaSensorSrc;
	Core::Sensor m_sensor;
	bool m_OK;
	float m_brightness;

	// OpenCV RGB frame
	Mat RgbFrame;
	float g_r, g_g, g_b; // white balance gains
	float rotation; //initial rotation of the image
	std::string ipaddress;
	bool renderShape;
	double robot_rotation;
	bool newImg;
	bool newImg_force;
	bool m_network;
	std::vector<SE3> m_SolutionFrames;

	// mutex for image sharing between threads
	::std::mutex mutex_img;
	::std::mutex mutex_img_force;
	::std::mutex mutex_robotjoints;
	::std::mutex mutex_teleop;
	::std::mutex mutex_robotshape;
	::std::mutex m_mutex_force;

	// shared mutex for multiple readers and one writer, protecting the last read image
	SharedMutex m_mutex_sharedImg;

	// Force estimation variables
	BOW_l m_bow;
	BagOfFeatures m_bof;

	float m_contact;
	float m_force_gain;
	::cv::KalmanFilter m_kalman;
	bool m_outputForce;
	std::vector<float> m_KFParams; // Kalman filter gains, process and measure covariance
	float m_heartFreq; // estimated heart frequency
	float m_imFreq; 
	int m_FramesPerHeartCycle;
	bool m_sendContact;
	float m_contact_response;

	float m_contactAvgOverHeartCycle;
	bool m_contactMeasured;
	std::deque<float> m_contactBuffer;
	
	// George -> attempting implementation for frequency estimation
	::std::deque<float> m_contactBufferFiltered;
	int					m_maxBufferSize;
	RecursiveFilter::Filter*				m_filter;
	int					m_heartFreqInSamples;
	bool				m_estimateFreq;
	double				m_cameraFrameRate;
	RecursiveFilter::Filter*				m_freqFilter;

	std::deque<float> m_durations;
	float m_measured_period;

	// these are recieved through network
	double				m_normal[3];
	double				m_center[3];
	double				m_radius;
	double				m_target[6];
	bool				m_input_freq_received;
	bool				m_input_plane_received;
	double				m_input_frequency;
	bool				m_circumnavigation;
	/********** Functions private *********/
	
	// circumnavigation parameters
	double				m_centroid[2];
	double				m_tangent[2];
	bool				m_linedetected;

	double				m_centroidImageFrame[2];
	double				m_tangentImageFrame[2];

	// Thread functions
    void acquireImages(void);
	void displayImages(void);
	void computeForce(void);
	void recordImages(void);
	bool networkKinematics(void);
	void robotDisplay(void);
	void vtkRender(void);
	void OnLinePlot(void);
	void initializeValveDisplay();
	// keyboard input processing
	void processInput(char key);
	void displayValve(double normal[3], double center[3], double radius);
	void updateRobotTargetVisualization(double targetPosition[3]);
	void initializeTarget();
	void initializeApex();

	void computeCircumnavigationParameters(const ::cv::Mat& img);
	// camera management functions
	void changeExposure(float delta);

	bool createSaveDir();

	void		updateHeartFrequency();
	void		parseNetworkMessage(::std::vector<double>& msg);
	vtkSmartPointer<vtkSphereSource> sphereSource;
	vtkSmartPointer<vtkRegularPolygonSource> circleSource;
	vtkSmartPointer<vtkPolyDataMapper> mapperCircle;
	vtkSmartPointer<vtkActor> actorCircle;

	vtkSmartPointer<vtkArrowSource> arrowSource;
	vtkSmartPointer<vtkMatrix4x4> matrix;
	vtkSmartPointer<vtkTransform> transform;
	vtkSmartPointer<vtkTransformPolyDataFilter> transformPD;
	vtkSmartPointer<vtkPolyDataMapper> mapperArrow; 
	vtkSmartPointer<vtkActor> actorArrow;

	vtkSmartPointer<vtkPoints> points;
	vtkSmartPointer<vtkPolyData> pointsPolydata;
	vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter;
	vtkSmartPointer<vtkPolyData> polydata;
	vtkSmartPointer<vtkPolyDataMapper> mapperPoints;
	vtkSmartPointer<vtkActor> actorPoints; 

	::std::vector<double> pointsOnValve;

	vtkSmartPointer<vtkPolyData> linesPolyData;
	vtkSmartPointer<vtkPoints> pts;
	vtkSmartPointer<vtkLine> line0;
	vtkSmartPointer<vtkCellArray> lines;

	LineDetector		m_linedetector;
	RecursiveFilter::RecursiveMovingAverage	m_radius_filter;
	RecursiveFilter::RecursiveMovingAverage	m_theta_filter;

	double		apex_coordinates[5];
	bool		m_apex_initialized;

	vtkSmartPointer<vtkRegularPolygonSource> apexSource;

public:
	void updatePoints();
	void addArrow(double normal[3], double center[3]);
	void updateArrowOrientation(double normal[3], vtkSmartPointer<vtkMatrix4x4> matrix);
	// Constructor and destructor
	Camera_processing(int period, bool sendContact);
	~Camera_processing();

	// Accessors
	::std::vector<float> getWhiteBalance();
	void setWhiteBalance(float r, float g, float b);
	void setControlLED(bool LED);
	bool getControlLED();

	void InitForceEstimator(::std::string svm_base_path, float force_gain=3.0, float processNoiseCov=0.5, float measureCov = 0.5);
	void UpdateForceEstimator(const ::cv::Mat& img);
	float PredictForce();

	void initializeArrow();
};

