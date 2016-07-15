#define _WINSOCKAPI_
#include <windows.h> // Sleep

#include <opencv2\opencv.hpp>
#include <opencv2\xfeatures2d\nonfree.hpp>
#include <opencv2\features2d.hpp>
#include <opencv2/ml.hpp>

#include <iostream> 
#include <stdexcept> 
#include <algorithm>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <thread>
#include <mutex>
#include <queue>
#include <deque>

// Winsock includes for network
#include <winsock2.h>
#include <ws2tcpip.h>
#include "targetver.h"
#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "27015"

#include "Main.h"
#include "BOW_lowlevel.h"
#include "FileUtils.h"


class Network_force
{
public:
	Network_force(::std::string svm_base_path,::std::string image_path);
	~Network_force();

	void Network_force::runThreads();
	void setBOW(BOW_l bow);

	void setSource(std::string path);
	void setForceGain(float gain);

private:
	BOW_l m_bow;
	std::string m_imagepath;

	float m_contact;
	float m_force_gain;
	std::string m_ipaddress;

	::std::mutex m_mutex_force;

	bool networkForce(void);
	bool processImages(void);

	bool m_running;

	::cv::KalmanFilter m_kalman;
};
