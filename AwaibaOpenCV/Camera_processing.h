// System includes
#include <windows.h> // Sleep
#include <stdint.h>
#include <stdlib.h>
#include <thread>
#include <mutex>
#include <queue>

 #include <vtkAutoInit.h>
 VTK_MODULE_INIT(vtkInteractionStyle);
 VTK_MODULE_INIT(vtkRenderingOpenGL);

#include "Queue.h"
#include "BOW_lowlevel.h"

// OpenCV includes
#include <opencv2/core/core.hpp>

// Awaiba includes
#define _CRTDBG_MAP_ALLOC
#include "awcorecpp.h"

#include "LieGroup.h"

using namespace Core;
using namespace cv;
//using namespace std;

struct ImgBuf {
  Mat img;
  Core::ArgbFrame::time_type timestamp;
  std::vector<double> robot_joints;
} ;

class Camera_processing {

private:


	/********** Members private *********/
	bool m_running; 
	bool m_record; // recording images
	bool m_teleop; // robot teleoperation is ON/OFF
	bool m_newdir; // flag for saying that a savedir should be created
	bool m_rotateImage;

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

	std::vector<SE3> m_SolutionFrames;

	// mutex for image sharing between threads
	::std::mutex mutex_img;
	::std::mutex mutex_img_force;
	::std::mutex mutex_robotjoints;
	::std::mutex mutex_teleop;
	::std::mutex mutex_robotshape;
	::std::mutex m_mutex_force;

	// Force estimation variables
	BOW_l m_bow;
	float m_contact;
	float m_force_gain;
	::cv::KalmanFilter m_kalman;
	bool m_outputForce;

	/********** Functions private *********/
	
	// Thread functions
    void acquireImages(void);
	void displayImages(void);
	void computeForce(void);
	void recordImages(void);
	bool networkKinematics(void);
	void robotDisplay(void);
	void vtkRender(void);

	// keyboard input processing
	void processInput(char key);

	// camera management functions
	void changeExposure(float delta);

	bool createSaveDir();

public:

	// Constructor and destructor
	Camera_processing();
	~Camera_processing();

	// Accessors
	::std::vector<float> getWhiteBalance();
	void setWhiteBalance(float r, float g, float b);
	void setControlLED(bool LED);
	bool getControlLED();

	void InitForceEstimator(::std::string svm_base_path, float force_gain=3.0, float processNoiseCov=0.5);
	void UpdateForceEstimator(::cv::Mat img);
	float PredictForce();
};