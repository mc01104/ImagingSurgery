#include "LieGroup.h"
#include "Utilities.h"

// System includes
#define _WINSOCKAPI_ // define this before including windows.h for avoiding winsock function redefinitions causing errors
#include <windows.h> 

//#define WIN32_LEAN_AND_MEAN

// general includes
#include <stdint.h>
#include <stdlib.h>
#include <thread>
#include <mutex>
#include <iostream>
#include <algorithm>
#include <functional>
#include <conio.h>
#include <sstream>
#include <queue>
#include <chrono>
#include <iomanip>
#include <crtdbg.h>

// Winsock includes for network
#include <winsock2.h>
#include <ws2tcpip.h>
#include "targetver.h"
#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "27015"

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Awaiba includes
#define _CRTDBG_MAP_ALLOC
#include "awcorecpp.h"
#include "CImg.h"
using namespace cimg_library;
#define cimg_use_png


// Project includes
#include "Camera_processing.h"
#include "CSV_reader.h"
//#include "LieGroup.h"
//#include "Utilities.h"
#include "HTransform.h"
#include "MechanicsBasedKinematics.h"
#include "CTRFactory.h"

using namespace Core;
using namespace cv;
//using namespace std;

// Constructor and destructor
Camera_processing::Camera_processing() : m_Manager(Manager::GetInstance(0))
{
	// Animate CRT to dump leaks to console after termination.
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

	m_running = true;
	m_record = false;

	m_board="NanoUSB2";
	m_ControlLED=false;

	m_OK = false;
	newImg = false;


	// Parse options in camera.csv file
	// TODO: handle errors better and do not fallback to default config
	ParseOptions op = ParseOptions("./camera_info.csv");

	if (op.getStatus())
	{
		::std::cout << "Successfully parsed camera info file" << endl;
		saveDir = op.getSaveDir();
		g_r = op.getWhiteBalance()[0];
		g_g = op.getWhiteBalance()[1];
		g_b = op.getWhiteBalance()[2];
		rotation = op.getRotation();
	}
	else
	{
		::std::cout << "Information could not be parsed from camera info data, reverting to default" << endl;
		saveDir = "C:\\AwaibaData\\";
		g_r = 1.0f;
		g_g = 0.99f;
		g_b = 1.29f;
		rotation = 0.0f;
	}

	robot_rotation = 0.0;

	::std::stringstream ss;
	time_t rawTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	ss << saveDir << std::put_time(std::localtime(&rawTime), "%Y-%m-%d_%H-%M-%S") << "\\";
	imgDir = ss.str();

	CreateDirectoryA(imgDir.c_str(), NULL) ;

	// All errors are reported as std::exception.
	try
	{
		// Cleanup is automatically done when destructed (auto generated)
		int device = 0;
		const int NumSensors = 1;

		m_brightness = 1.2f;

		//m_Manager = Manager::GetInstance(device);
		m_Manager.SetBinFile("nanousb2_fpga_v06.bin");
		m_Manager.SetSensor(3); // usb(nano usb2)

		m_Manager.SetSource(m_AwaibaSensorSrc);
		Core::Sensor sensor1(2, 3,1, 2, 1, 1.8f); // gain, offset, exposure, vref_cds, vrst_pixel, digipot_level
		sensor1.DAC_DSTEP1 = -0.05;
		sensor1.DAC_DSTEP2 = -0.05;
		sensor1.DAC_DREGEN_REG = 0x111;
		sensor1.LedState = false;
		
		m_Manager.ConfigureSensor(sensor1);
		Sleep(1000);
		m_Manager.ConfigureSensor(sensor1);
		m_sensor = sensor1;

		m_pipeConfig.BadPixelReplacement.Enable = true;
		m_pipeConfig.BadPixelReplacement.Threshold = 50;
		m_pipeConfig.SkipFrames.Enable = true;
		m_pipeConfig.BlackLevelCorrection = false;
		m_pipeConfig.WhiteLevelCorrection = false;
		m_pipeConfig.DeMosaic.Enable = true;
		m_pipeConfig.Brightness = m_brightness;
		m_pipeConfig.PixelLinearization = false;
		m_Manager.SetPipeConfig(m_pipeConfig);

		m_AwaibaSensorSrc.Start(device);

		
		/*** Automatic Exposure Control Registers ***/
		m_Manager.SetFPGAData(0x00500000,0x02010200);

		Mat RgbFrame = Mat(250,250,CV_8UC3);

		m_OK = true;

		::std::thread t_acquire (&Camera_processing::acquireImages, this);
		::std::thread t_display (&Camera_processing::displayImages, this);
		::std::thread t_record (&Camera_processing::recordImages, this);
		::std::thread t_network (&Camera_processing::networkKinematics, this);

		t_acquire.join();
		t_display.join();
		t_record.join();
		t_network.join();

	}
	catch(const std::exception &ex)
	{
		::std::cout << "Exception raised: " << ex.what() << ::std::endl;
		m_OK = false;
	}
}

Camera_processing::~Camera_processing()
{
	
}

// Accessors
::std::vector<float> Camera_processing::getWhiteBalance()
{
	::std::vector<float> temp;
	temp.push_back(g_r);
	temp.push_back(g_g);
	temp.push_back(g_b);
	return temp;
}

void Camera_processing::setWhiteBalance(float r, float g, float b)
{
	g_r = r;
	g_g = g;
	g_b = b;
}

void Camera_processing::setControlLED(bool LED) { m_ControlLED = LED;}

bool Camera_processing::getControlLED() { return m_ControlLED;}



// Prcess input keyboard
void Camera_processing::processInput(char key)
{
	switch(key)
	{
	case 27:
		m_running=false;
		break;
	case 'r':
		m_record = !m_record;
		break;

	case '1':
		changeExposure(-0.2f);
		break;
	case '2':
		changeExposure(0.2f);
		break;
	case '3':
		changeExposure(0.0f);
		break;
	}
}


// Exposure management
void Camera_processing::changeExposure(float delta)
{
	if (delta==0.0) m_brightness = 1.2f;
	// The awaiba autoexposure is using the brightness parameter in the pipeline
	m_brightness = m_brightness + delta;
	if (m_brightness < 0.0) m_brightness = 0.0f;
	if (m_brightness > 4.0) m_brightness = 4.0f;

	m_pipeConfig.Brightness = m_brightness;

	m_Manager.SetPipeConfig(m_pipeConfig);
}



// Thread functions
void Camera_processing::acquireImages(void )
{
	// Read until we get a valid frame.
	ArgbFrame argbFrame1(m_Manager.GetFrameDimensions());
	RawFrame rawFrame1(m_Manager.GetFrameDimensions());

	Mat bufImg = Mat(250,250,CV_8UC3);

	while(m_running)
	{
		try
		{
			/*while (!MyManager.GetNextFrame(&argbFrame1, &rawFrame1)) Sleep(1);*/
					
			if(m_Manager.GetNextFrame(&argbFrame1, &rawFrame1))
			{
				unsigned char rData [250*250];
				unsigned char gData [250*250];
				unsigned char bData [250*250];

				//Get the pixels from the rawFrame to show to the user
				for (int i=0; i<250*250;i++)
				{
					rData[i] = ((argbFrame1.Begin()._Ptr[i] & 0x00FF0000)>>16);
					gData[i] = ((argbFrame1.Begin()._Ptr[i] & 0x0000FF00)>>8);
					bData[i] = ((argbFrame1.Begin()._Ptr[i] & 0x000000FF));
				}

				Mat R = Mat(250, 250, CV_8U, rData);
				Mat G = Mat(250, 250, CV_8U, gData);
				Mat B = Mat(250, 250, CV_8U, bData);

				// White balancing
				R = g_r*R;
				G = g_g*G;
				B = g_b*B;

				std::vector<cv::Mat> array_to_merge ;

				array_to_merge.push_back(B);
				array_to_merge.push_back(G);
				array_to_merge.push_back(R);

				mutex_img.lock();
				cv::merge(&array_to_merge[0], array_to_merge.size(), RgbFrame);
				newImg = true;
				if (m_record)
				{
					ImgBuf el;
					el.img = RgbFrame.clone();
					el.timestamp = argbFrame1.GetTimeStamp();
					m_ImgBuffer.push(el);
				}
				mutex_img.unlock();

				

			}
			else
			{
				Sleep(1);
			}
		}
		catch(const std::exception &ex){}
	}
}


void Camera_processing::displayImages(void)
{
	Mat frame = Mat(250,250,CV_8UC3);
	Mat frame_rotated = Mat(250,250,CV_8UC3);
	char key;
	::std::cout << "Start Display" << ::std::endl;
	namedWindow( "Display", 0 );

	bool display = false;

	Point center = Point( frame.cols/2, frame.rows/2 );
    Mat rot_mat = getRotationMatrix2D( center, rotation - robot_rotation*180.0/3.141592, 1.0 );

	while(m_running)
	{
		mutex_img.lock();
		if (newImg)
		{
			newImg = false;
			display = true;
			RgbFrame.copyTo(frame);
		}
		mutex_img.unlock();

		if (display)
		{
			display = false;
			rot_mat = getRotationMatrix2D( center, rotation - robot_rotation*180.0/3.141592, 1.0 );
			warpAffine( frame, frame_rotated, rot_mat, frame_rotated.size() );
			cv::imshow( "Display", frame_rotated );
			key = waitKey(1);
			processInput(key);
		}
	}
}

void Camera_processing::recordImages(void)
{
	Mat frame;
	ArgbFrame::time_type timestamp = 0;

	::std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(3);

	std::chrono::milliseconds ms(10);

	while(m_running)
	{
		ImgBuf element;

		if(m_record)
		{
			if ( m_ImgBuffer.tryPop(element))
			{
				frame = element.img;
				timestamp = element.timestamp;

				::std::string filename = imgDir + std::to_string(timestamp) + ".png";
				try {
					imwrite(filename, frame, compression_params);
				}
				catch (runtime_error& ex) 
				{
					::std::cout << "Exception converting image to PNG format:" <<  ex.what() << ::std::endl;
				}
			}
		}
	}

	::std::cout << "Finished recording data" << ::std::endl;
}



bool Camera_processing::networkKinematics(void)
{
	/**********
	Initialize the kinematics computing
	**********/
	CTR* robot = CTRFactory::buildCTR("");
	MechanicsBasedKinematics* kinematics = new MechanicsBasedKinematics(robot, 100); // the integration grid consists of 100 points (increase if you have convergence problems)


	/**********
	Declare and initialize connection socket
	**********/
	WSADATA wsaData;
    SOCKET ConnectSocket = INVALID_SOCKET;
    struct addrinfo *result = NULL,
                    *ptr = NULL,
                    hints;
    char *sendbuf = "this is a test";
    char recvbuf[DEFAULT_BUFLEN];
    int iResult;
    int recvbuflen = DEFAULT_BUFLEN;

	iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return 1;
    }

	ZeroMemory( &hints, sizeof(hints) );
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    // Resolve the server address and port
    iResult = getaddrinfo("192.168.0.3", DEFAULT_PORT, &hints, &result);
    if ( iResult != 0 ) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return 1;
    }

	// Attempt to connect to an address until one succeeds
    for(ptr=result; ptr != NULL ;ptr=ptr->ai_next) {

        // Create a SOCKET for connecting to server
        ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype, 
            ptr->ai_protocol);
        if (ConnectSocket == INVALID_SOCKET) {
            printf("socket failed with error: %ld\n", WSAGetLastError());
            WSACleanup();
            return 1;
        }

        // Connect to server.
        iResult = connect( ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
        if (iResult == SOCKET_ERROR) {
            closesocket(ConnectSocket);
            ConnectSocket = INVALID_SOCKET;
            continue;
        }
        break;
    }

    freeaddrinfo(result);

    if (ConnectSocket == INVALID_SOCKET) {
        printf("Unable to connect to server!\n");
        WSACleanup();
        return 1;
    }


	do {

		/*****
		Receive data through the network
		*****/
        iResult = recv(ConnectSocket, recvbuf, recvbuflen, 0);

		//::std::cout << iResult << ::std::endl;

        //if ( iResult > 0 )
        //    printf("Bytes received: %d\n", iResult);
        //else if ( iResult == 0 )
        //    printf("Connection closed\n");
        //else
        //    printf("recv failed with error: %d\n", WSAGetLastError());

		/*****
		Solve the kinematcs
		*****/

		// Convert the received data to a vector of doubles
		::std::string conf_str(recvbuf);
		::std::vector<double> configuration = DoubleVectorFromString(conf_str);

		// Convert the received the configuration to comply with the definition of the mechanics based kinematics implementation
		double rotation[3] = {0};
		double translation[3] = {0};
		MechanicsBasedKinematics::RelativeToAbsolute(robot, &configuration[0], rotation, translation);

		// compute kinematics and get tip rotation of innermost tube
		kinematics->ComputeKinematics(rotation, translation);
		double tipRotation = kinematics->GetInnerTubeRotation();
		double baseRotation = rotation[2];
	
		robot_rotation = tipRotation;
		
		/*****
		Acknowledge good reception of data to network for preparing next transmission
		*****/
		iResult = send( ConnectSocket, "haha", 5, 0 );
		//::std::cout << iResult << "send" << ::std::endl;

    } while( (iResult > 0) && m_running);

    // cleanup
    closesocket(ConnectSocket);
    WSACleanup();

    return 0;
}