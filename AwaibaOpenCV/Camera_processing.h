// System includes
#include <windows.h> // Sleep
#include <stdint.h>
#include <stdlib.h>
#include <thread>
#include <mutex>

#include <queue>

#include "Queue.h"

// OpenCV includes
#include <opencv2/core/core.hpp>

// Awaiba includes
#define _CRTDBG_MAP_ALLOC
#include "awcorecpp.h"

using namespace Core;
using namespace cv;
//using namespace std;

struct ImgBuf {
  Mat img;
  Core::ArgbFrame::time_type timestamp;
} ;

class Camera_processing {

private:


	/********** Members private *********/
	bool m_running;
	bool m_record;

	Queue<ImgBuf> m_ImgBuffer; // images and timestamp buffer

	::std::string saveDir;
	::std::string imgDir;

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
	double robot_rotation;
	bool newImg;

	// mutex for image sharing between threads
	::std::mutex mutex_img;

	/********** Functions private *********/
	
	// Thread functions
    void acquireImages(void);
	void displayImages(void);
	void recordImages(void);
	bool networkKinematics(void);

	// keyboard input processing
	void processInput(char key);

	// camera management functions
	void changeExposure(float delta);

public:

	// Constructor and destructor
	Camera_processing();
	~Camera_processing();

	// Accessors
	::std::vector<float> getWhiteBalance();
	void setWhiteBalance(float r, float g, float b);
	void setControlLED(bool LED);
	bool getControlLED();

};