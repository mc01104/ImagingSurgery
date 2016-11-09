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

// VTK includes 
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



// Project includes
#include "Camera_processing.h"
#include "CSV_reader.h"
#include "HTransform.h"
#include "MechanicsBasedKinematics.h"
#include "CTRFactory.h"
#include "FilterLibrary.h"

using namespace Core;
using namespace cv;
//using namespace std;


// VTK global variables (only way to get a thread running ...)

::std::mutex mutex_vtkRender;


#define VTK_CREATE(type, name) \
    vtkSmartPointer<type> name = vtkSmartPointer<type>::New()

VTK_CREATE(vtkRenderer, renDisplay3D);
VTK_CREATE(vtkRenderWindow, renderwindowDisplay3D);
VTK_CREATE(vtkRenderWindowInteractor, irenDisplay3D);
vtkInteractorStyleTrackballCamera *styleDisplay3D = vtkInteractorStyleTrackballCamera::New();


class CommandSubclass2 : public vtkCommand
{
public:
    vtkTypeMacro(CommandSubclass2, vtkCommand);

    static CommandSubclass2 *New()
    {
        return new CommandSubclass2;
    }

    void Execute(vtkObject *caller, unsigned long vtkNotUsed(eventId), void *vtkNotUsed(callData))
    {
        vtkRenderWindowInteractor *iren = static_cast<vtkRenderWindowInteractor*>(caller);
        iren->Render();
    }
};



// Constructor and destructor
Camera_processing::Camera_processing(int period, bool sendContact) : m_Manager(Manager::GetInstance(0)), m_FramesPerHeartCycle(period), m_sendContact(sendContact)
{
	// Animate CRT to dump leaks to console after termination.
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

	// George
	m_maxBufferSize = 100;
	m_filter = new MedianFilter(5);

	m_running = true;
	m_record = false;
	m_teleop = false;
	m_newdir = true; 
	//m_outputForce = false;
	m_outputForce = true;
	m_rotateImage = true;

	m_board="NanoUSB2";
	m_ControlLED=false;

	m_OK = false;
	newImg = false;
	newImg_force = false;

	::std::string svm_base_folder = "./SVM_params/";

	// Parse options in camera.csv file
	// TODO: handle errors better and do not fallback to default config
	ParseOptions op = ParseOptions("./camera_info.csv");

	if (op.getStatus())
	{
		::std::cout << "Successfully parsed camera info file" << endl;
		m_saveDir = op.getSaveDir();
		g_r = op.getWhiteBalance()[0];
		g_g = op.getWhiteBalance()[1];
		g_b = op.getWhiteBalance()[2];
		rotation = op.getRotation();
		ipaddress = op.getIPAddress();
		renderShape = op.getRenderShape();
		svm_base_folder = op.getSVMDir();

		m_KFParams = op.getKFParams();

	}
	else
	{
		::std::cout << "Information could not be parsed from camera info data, reverting to default" << endl;
		m_saveDir = "C:\\AwaibaData\\";
		g_r = 1.0f;
		g_g = 0.99f;
		g_b = 1.29f;
		rotation = 0.0f;
		ipaddress = std::string("192.168.0.2");

		m_KFParams.push_back(0.5);
		m_KFParams.push_back(0.5);
	}

	::std::cout << "Initializing Force estimator ..." << ::std::endl;
	InitForceEstimator(svm_base_folder + "output_", 3.0, m_KFParams[0], m_KFParams[1]);
	::std::cout << "Force estimator initialized" << ::std::endl;
	
	m_estimateFreq = true;
	m_measured_period = 0.0;
	robot_rotation = 0.0;

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

		if (renderShape)
		{
			::std::thread t_acquire (&Camera_processing::acquireImages, this);
			::std::thread t_display (&Camera_processing::displayImages, this);
			::std::thread t_force (&Camera_processing::computeForce, this);
			::std::thread t_record (&Camera_processing::recordImages, this);
			::std::thread t_network (&Camera_processing::networkKinematics, this);
			::std::thread t_vtk (&Camera_processing::robotDisplay, this);
			::std::thread t_vtk_render (&Camera_processing::vtkRender, this);

			t_acquire.join();
			t_display.join();
			t_force.join();
			t_record.join();
			t_network.join();
			t_vtk_render.join();
			t_vtk.join();
		}

		else
		{
			::std::thread t_acquire (&Camera_processing::acquireImages, this);
			//::std::thread t_display (&Camera_processing::displayImages, this);
			//::std::thread t_force (&Camera_processing::computeForce, this);
			//::std::thread t_record (&Camera_processing::recordImages, this);
			::std::thread t_network (&Camera_processing::networkKinematics, this);

			t_acquire.join();
			//t_display.join();
			//t_force.join();
			//t_record.join();
			t_network.join();
		}

		::std::cout << "All threads exited. Closing..." << ::std::endl;
		Sleep(1000);

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



bool Camera_processing::createSaveDir()
{
	::std::stringstream ss;
	time_t rawTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	ss << m_saveDir << std::put_time(std::localtime(&rawTime), "%Y-%m-%d_%H-%M-%S") << "\\";
	m_imgDir = ss.str();
	return CreateDirectoryA(m_imgDir.c_str(), NULL) ;
}


// Prcess input keyboard
void Camera_processing::processInput(char key)
{
	switch(key)
	{
	case 27:
		m_running=false;
		break;
	case 'r':
		if (m_record) m_newdir = true;
		m_record = !m_record;
		break;
	case 'f':
		//m_outputForce = !m_outputForce;
		m_estimateFreq = !m_estimateFreq;
		break;
	case 'o':
		m_rotateImage =  !m_rotateImage;
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

	//Mat bufImg = Mat(250,250,CV_8UC3);

	unsigned char rData [250*250];
	unsigned char gData [250*250];
	unsigned char bData [250*250];

	Mat R = Mat(250, 250, CV_8U);
	Mat G = Mat(250, 250, CV_8U);
	Mat B = Mat(250, 250, CV_8U);
	
	::std::vector<Mat> array_to_merge;
	array_to_merge.push_back(B);
	array_to_merge.push_back(G);
	array_to_merge.push_back(R);

	array_to_merge[0].data = bData;
	array_to_merge[1].data = gData;
	array_to_merge[2].data = rData;

	while(m_running)
	{
		auto start_rec = std::chrono::high_resolution_clock::now();					
		try
		{
			/*while (!MyManager.GetNextFrame(&argbFrame1, &rawFrame1)) Sleep(1);*/
			
			if(m_Manager.GetNextFrame(&argbFrame1, &rawFrame1))
			{
				//unsigned char rData [250*250];
				//unsigned char gData [250*250];
				//unsigned char bData [250*250];

				//Get the pixels from the rawFrame to show to the user
				for (int i=0; i<250*250;i++)
				{
					rData[i] = ((argbFrame1.Begin()._Ptr[i] & 0x00FF0000)>>16);
					gData[i] = ((argbFrame1.Begin()._Ptr[i] & 0x0000FF00)>>8);
					bData[i] = ((argbFrame1.Begin()._Ptr[i] & 0x000000FF));
				}

				//Mat R = Mat(250, 250, CV_8U, rData);
				//Mat G = Mat(250, 250, CV_8U, gData);
				//Mat B = Mat(250, 250, CV_8U, bData);

				array_to_merge[0] *= g_b;			
				array_to_merge[1] *= g_g;			
				array_to_merge[2] *= g_r;			

				// White balancing
				//R = g_r*R;
				//G = g_g*G;
				//B = g_b*B;

				//std::vector<cv::Mat> array_to_merge ;

				//array_to_merge.push_back(B);
				//array_to_merge.push_back(G);
				//array_to_merge.push_back(R);

				//mutex_robotjoints.lock();
				//std::vector<double> configuration = m_configuration;
				//mutex_robotjoints.unlock();

				//m_mutex_sharedImg.writeLock();
				//mutex_img.lock();
				cv::merge(&array_to_merge[0], array_to_merge.size(), RgbFrame);
				//newImg = true;
				//newImg_force = true;
				//m_mutex_sharedImg.writeUnLock();


				
				if ((! RgbFrame.empty()) && (m_outputForce)) 
					UpdateForceEstimator(RgbFrame);

				//if (m_record)
				//{
				//	ImgBuf el;
				//	el.img = RgbFrame.clone();
				//	el.timestamp = argbFrame1.GetTimeStamp();
				//	el.robot_joints = configuration;
				//	m_ImgBuffer.push(el);
				//}
				//mutex_img.unlock();

				auto stop_rec = std::chrono::high_resolution_clock::now();					
				auto duration = std::chrono::duration_cast<::std::chrono::microseconds> (stop_rec - start_rec);
				//::std::cout << duration.count()/1000.0 << ::std::endl;
			}
			else
			{
				Sleep(1);
			}
		}
		catch(const std::exception &ex){::std::cout << "exception" << ::std::endl;}
	}

	::std::cout << "Images Acquisition Thread exited successfully" << ::std::endl;
}


void Camera_processing::displayImages(void)
{
	Mat frame = Mat(250,250,CV_8UC3);
	Mat frame_rotated = Mat(250,250,CV_8UC3);
	char key;
	::std::cout << "Start Display" << ::std::endl;
	namedWindow( "Display", 0 );

	bool display = false;
	bool rec = false;
	bool teleop = false;

	Point center = Point( frame.cols/2, frame.rows/2 );
    Mat rot_mat = getRotationMatrix2D( center, rotation - robot_rotation*180.0/3.141592, 1.0 );

	while(m_running)
	{
		
		m_mutex_sharedImg.readLock();
		if (newImg)
		{
			newImg = false;
			display = true;
			rec = m_record;
			RgbFrame.copyTo(frame);
		}
		m_mutex_sharedImg.readUnLock();
		

		mutex_teleop.lock();
		teleop = m_teleop;
		mutex_teleop.unlock();

		if (display)
		{

			display = false;
			double rot = robot_rotation;
			if (!m_rotateImage) rot = 0.0;

			rot_mat = getRotationMatrix2D( center, rotation - rot*180.0/3.141592, 1.0 );
			warpAffine( frame, frame_rotated, rot_mat, frame_rotated.size() );

			if (rec) // draw a red circle on frame when recording
				cv::circle( frame_rotated, Point( 240, 10 ), 5, Scalar( 0, 0, 255 ),  -1);
			
			if (teleop) // draw a green circle on frame when teleoperating
				cv::circle( frame_rotated, Point( 220, 10 ), 5, Scalar( 0, 255, 0 ),  -1);

			cv::imshow( "Display", frame_rotated );
			key = waitKey(1);
			processInput(key);
		}
	}

	::std::cout << "Images Display Thread exited successfully" << ::std::endl;
}



void Camera_processing::computeForce(void)
{
	Mat frame = Mat(250,250,CV_8UC3);

	bool computeForce = false;

	::std::cout << "Start Force processing thread" << ::std::endl;

	while(m_running)
	{
		
		m_mutex_sharedImg.readLock();
		if (newImg_force)
		{
			RgbFrame.copyTo(frame);
			computeForce = newImg_force;
			newImg_force = false;
		}
		m_mutex_sharedImg.readUnLock();

		if (computeForce)
		{
			computeForce = false;
			if ((! frame.empty()) && (m_outputForce)) UpdateForceEstimator(RgbFrame); //this is a bug -> you need to use frame not RgbFrame
		}
		
	}

	::std::cout << "Force processing Thread exited successfully" << ::std::endl;

}

void Camera_processing::recordImages(void)
{
	Mat frame;
	ArgbFrame::time_type timestamp = 0;
	std::vector<double> robot_joint;

	::std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(3);

	auto start_record = std::chrono::high_resolution_clock::now();

	while(m_running)
	{
		ImgBuf element;

		if ( m_ImgBuffer.tryPop(element))
		{

			// create a new directory if necessary
			if(m_newdir) 
			{
				createSaveDir();
				start_record = std::chrono::high_resolution_clock::now();
				m_newdir = false;
			}

			// if recording gets too long, create a new directory to avoid too many packed images in a single one
			auto now = std::chrono::high_resolution_clock::now();
			auto duration_minutes = std::chrono::duration_cast<std::chrono::minutes>(now - start_record);
			if (duration_minutes.count()>=4) 
			{
				m_newdir = true;
				start_record = now;
			}

			frame = element.img;
			timestamp = element.timestamp;
			robot_joint = element.robot_joints;

			::std::string filename = m_imgDir + std::to_string(timestamp) + ".png";
			::std::string filename_joints = m_imgDir + std::to_string(timestamp) + ".txt";

			try {

				if (!imwrite(filename, frame, compression_params)) 
				{
					m_record = false;
					throw std::runtime_error ("Could not write to file !");
				}

				if (robot_joint.size()>0)
				{
					ofstream joints_file;
					joints_file.open (filename_joints);
					for(std::vector<double>::const_iterator i = robot_joint.begin(); i != robot_joint.end(); ++i) {
							joints_file << *i << ',';
					}
					joints_file << '\n';
					joints_file.close();
				}
			}
			catch (runtime_error& ex) 
			{
				::std::cout << "Exception in image recording:" <<  ex.what() << ::std::endl;
			}
		}
	}

	::std::cout << "Recording Data Thread exited successfully" << ::std::endl;
}



bool Camera_processing::networkKinematics(void)
{
	/**********
	Initialize the kinematics computing
	**********/
	CTR* robot = CTRFactory::buildCTR("");
	MechanicsBasedKinematics* kinematics = new MechanicsBasedKinematics(robot, 100); // the integration grid consists of 100 points (increase if you have convergence problems)

	int npoints = 35;
	std::vector<double> robot_arclength;
	std::vector<SE3> SolutionFrames(npoints);

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
    iResult = getaddrinfo(ipaddress.c_str(), DEFAULT_PORT, &hints, &result);
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
	std::cout << "Successfully connected to server" << std::endl;



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

		mutex_teleop.lock();
		m_teleop = ( configuration.size()>0 ? (bool) configuration.back() : false); //TODO: implementation bugin here maybe. Check with Georgios how is the data sent from the CTR program in non-teleop mode
		mutex_teleop.unlock();

		// Convert the received the configuration to comply with the definition of the mechanics based kinematics implementation
		double rotation[3] = {0};
		double translation[3] = {0};
		MechanicsBasedKinematics::RelativeToAbsolute(robot, &configuration[0], rotation, translation);

		// compute kinematics and get tip rotation of innermost tube
		if (kinematics->ComputeKinematics(rotation, translation))
		{
			robot_rotation = kinematics->GetInnerTubeRotation();
			double smax = robot->GetLength();

			robot_arclength.clear();
			SolutionFrames.clear();
			for (int i = 0; i<npoints;i++) robot_arclength.push_back((1.0*i)/npoints*smax);

			kinematics->GetBishopFrame(robot_arclength, SolutionFrames);

			mutex_robotshape.lock();
			m_SolutionFrames = SolutionFrames;
			mutex_robotshape.unlock();


			mutex_robotjoints.lock();
			m_configuration = configuration;
			mutex_robotjoints.unlock();
		}

		float force = 0.0;
		bool newMeasurement = false;

		if (m_outputForce)
		{
			m_mutex_force.lock();
			//PredictForce();
			force = m_contactAvgOverHeartCycle;
			newMeasurement = m_contactMeasured;
			m_contactMeasured = false;
			m_mutex_force.unlock();
		}

		

		char s_force[5]; 
		sprintf(s_force,"%.2f",force);


		/*****
		Acknowledge good reception of data to network for preparing next transmission
		*****/
		if (newMeasurement) iResult = send( ConnectSocket, s_force, 5, 0 );
		else iResult = send( ConnectSocket, "NOF", 5, 0 );

    } while( (iResult > 0) && m_running);

    // cleanup
    closesocket(ConnectSocket);
    WSACleanup();

	::std::cout << "Kinematics Thread exited successfully" << ::std::endl;
    return 0;
}



void Camera_processing::robotDisplay(void)
{

	// populate Points with dummy data for initialization
	unsigned int npts = 1;
	vtkSmartPointer<vtkPolyLineSource> lineSource = vtkSmartPointer<vtkPolyLineSource>::New();
	for (unsigned int i=0;i<npts;i++)
		lineSource->SetPoint(i, i,i,i);


	vtkSmartPointer<vtkTubeFilter> tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
	tubeFilter->SetInputConnection(lineSource->GetOutputPort());
	tubeFilter->SetRadius(0.9); 
	tubeFilter->SetNumberOfSides(30);
	tubeFilter->Update();

	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> tubeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	tubeMapper->SetInputConnection(tubeFilter->GetOutputPort());
	vtkSmartPointer<vtkActor> tubeActor = vtkSmartPointer<vtkActor>::New();
	tubeActor->GetProperty()->SetColor(0.0,0.0,1.0);
	tubeActor->SetMapper(tubeMapper);

	renDisplay3D->AddActor(tubeActor);

	auto start = std::chrono::high_resolution_clock::now();
	while(m_running)
	{
		auto duration_s = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
		if (duration_s.count()>=50) 
		{
			try
			{
				start = std::chrono::high_resolution_clock::now();

				mutex_robotshape.lock();
				std::vector<SE3> SolutionFrames = m_SolutionFrames;
				mutex_robotshape.unlock();


				npts = SolutionFrames.size();

				if (npts>2)
				{
					lineSource->SetNumberOfPoints(npts); // set new data
					for (unsigned int i=0;i<npts;i++)
						lineSource->SetPoint(i, SolutionFrames[i].GetPosition()[0],SolutionFrames[i].GetPosition()[1], SolutionFrames[i].GetPosition()[2]);
				}
				else
				{
					lineSource->SetNumberOfPoints(2); // set new data
					lineSource->SetPoint(0, 0.0,0.0,0.0);
					lineSource->SetPoint(1, 0.0,0.0,1.0);
				}
			}
			catch (runtime_error& ex) 
			{
				::std::cout << "Exception in vtk display:" <<  ex.what() << ::std::endl;
			} 
		}

	}
	::std::cout << "VTK Robot Display Thread exited successfully" << ::std::endl;

}

void Camera_processing::vtkRender(void)
{
	renderwindowDisplay3D->AddRenderer(renDisplay3D);
	renderwindowDisplay3D->Render();
	irenDisplay3D->SetRenderWindow(renderwindowDisplay3D);
	irenDisplay3D->Initialize();


	vtkSmartPointer<vtkCubeSource> cubeSource = vtkSmartPointer<vtkCubeSource>::New();
	cubeSource->SetCenter(0.0,0.0,125.0);
	cubeSource->SetXLength(150.0);
	cubeSource->SetYLength(150.0);
	cubeSource->SetZLength(250.0);

	vtkSmartPointer<vtkPolyDataMapper> mapper =  vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(cubeSource->GetOutputPort());
 
	vtkSmartPointer<vtkActor> actor =  vtkSmartPointer<vtkActor>::New();
	actor->GetProperty()->SetColor(0.5,0.5,0.5);
	actor->GetProperty()->SetOpacity(0.35);
	actor->SetMapper(mapper);

	renDisplay3D->AddActor(actor);



	vtkSmartPointer<vtkCubeSource> cubeSource2 = vtkSmartPointer<vtkCubeSource>::New();
	cubeSource2->SetCenter(12.5,0.0,0.0);
	cubeSource2->SetXLength(25.0);
	cubeSource2->SetYLength(1.0);
	cubeSource2->SetZLength(1.0);

	vtkSmartPointer<vtkPolyDataMapper> mapper2 =  vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper2->SetInputConnection(cubeSource2->GetOutputPort());
 
	vtkSmartPointer<vtkActor> actor2 =  vtkSmartPointer<vtkActor>::New();
	actor2->GetProperty()->SetColor(1.0,0.0,0.0);
	actor2->GetProperty()->SetOpacity(0.65);
	actor2->SetMapper(mapper2);

	renDisplay3D->AddActor(actor2);



	vtkSmartPointer<vtkCubeSource> cubeSource3 = vtkSmartPointer<vtkCubeSource>::New();
	cubeSource3->SetCenter(0.0,12.5,0.0);
	cubeSource3->SetXLength(1.0);
	cubeSource3->SetYLength(25.0);
	cubeSource3->SetZLength(1.0);

	vtkSmartPointer<vtkPolyDataMapper> mapper3 =  vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper3->SetInputConnection(cubeSource3->GetOutputPort());
 
	vtkSmartPointer<vtkActor> actor3 =  vtkSmartPointer<vtkActor>::New();
	actor3->GetProperty()->SetColor(0.0,1.0,0.0);
	actor3->GetProperty()->SetOpacity(0.65);
	actor3->SetMapper(mapper3);

	renDisplay3D->AddActor(actor3);


	vtkSmartPointer<CommandSubclass2> timerCallback = vtkSmartPointer<CommandSubclass2>::New();
	irenDisplay3D->AddObserver ( vtkCommand::TimerEvent, timerCallback );
	irenDisplay3D->CreateRepeatingTimer(100);

	irenDisplay3D->Start();


	::std::cout << "VTK Rendering Thread exited successfully" << ::std::endl;
}


/******************
Force estimation functions
*//////////////////

void Camera_processing::InitForceEstimator(::std::string svm_base_path, float force_gain, float processNoiseCov, float measureCov)
{
	try
	{

		m_bow.LoadFromFile(svm_base_path);

		m_kalman = ::cv::KalmanFilter(1,1);
		m_force_gain = force_gain;
		cv::setIdentity(m_kalman.measurementMatrix, cv::Scalar::all(measureCov));
		cv::setIdentity(m_kalman.processNoiseCov, cv::Scalar::all(processNoiseCov));

		::std::cout <<"Set Kalman filter gains to " << processNoiseCov << " (process), and " << measureCov << " (measure)" << std::endl;

		// Initialization of variables for image framerate and heart frequency
		m_imFreq = 40.0;
		m_heartFreq = 2.0;
		m_contactMeasured = false;
		m_heartFreqInSamples = m_imFreq/m_heartFreq;
	}
	catch (std::exception& e)
	{
		::std::cout << "Error loading Force parameters data" << e.what() << std::endl;
	}

}

void Camera_processing::updateHeartFrequency()
{
	
	::std::vector<float> tmp(m_contactBufferFiltered.size() - 1);
	diff(m_contactBufferFiltered, tmp);

	::std::vector<int> ind;
	find_all(tmp.begin(), tmp.end(), 1.0, ind);

	::std::vector<int> tmp2;
	if (ind.size() < 1)
		return;
	tmp2.resize(ind.size() - 1);

	diff<int>(ind, tmp2);

	m_FramesPerHeartCycle = (int) 2.0 * static_cast<int>(::std::accumulate(tmp2.begin(), tmp2.end(), 0.0))/tmp2.size();
	::std::cout << "estimation: " << 2.0 * static_cast<int>(::std::accumulate(tmp2.begin(), tmp2.end(), 0.0))/tmp2.size() << ::std::endl;
}

void Camera_processing::UpdateForceEstimator(const ::cv::Mat& img)
{
	::std::vector<::std::string> classes = m_bow.getClasses();
	float response = 0.0;
	
	if (m_bow.predictBOW(img,response)) 
	{
		//::std::cout << "in force estimator" << ::std::endl;
		if (classes[(int) response] == "Free") response = 0.0;
		else response = 1.0;

		m_contactBuffer.push_back(response);
		
		m_contactBufferFiltered.push_back(m_filter->step(response));

		if (m_contactBufferFiltered.size() > m_maxBufferSize)
			m_contactBufferFiltered.pop_front();

		if (m_contactBufferFiltered.size() < m_heartFreqInSamples - 1)
		{
			m_mutex_force.lock();
			m_contactAvgOverHeartCycle = 0.0;
			m_contactMeasured = true;
			m_mutex_force.unlock();
		}
		else if (m_contactBufferFiltered.size() > m_FramesPerHeartCycle) 
		{
			
			float sum = std::accumulate(m_contactBufferFiltered.rbegin(),m_contactBufferFiltered.rbegin() + this->m_FramesPerHeartCycle,0.0);

			m_mutex_force.lock();
			m_contactAvgOverHeartCycle = sum/m_FramesPerHeartCycle;
			//::std::cout << response << "," << m_contactAvgOverHeartCycle << ::std::endl;

			if (m_sendContact) m_contactAvgOverHeartCycle = response;
			m_contactMeasured = true;
			m_mutex_force.unlock();

		}

		if (m_estimateFreq && m_contactBufferFiltered.size() > 50)
			this->updateHeartFrequency();

		//::std::cout << "estimated heart frequency in samples: " << m_FramesPerHeartCycle << ::std::endl;

		//if (m_contactBuffer.size() < m_imFreq/m_heartFreq - 1.0)
		//{
		//	m_mutex_force.lock();
		//	m_kalman.correct(cv::Mat(1,1,CV_32FC1,cv::Scalar(response)));
		//	m_contactAvgOverHeartCycle = 0.0;
		//	m_contactMeasured = true;
		//	m_mutex_force.unlock();
		//}
		//else
		//{
		//	if (m_contactBuffer.size() > m_imFreq/m_heartFreq) 
		//		m_contactBuffer.pop_front();

		//	float sum = std::accumulate(m_contactBuffer.begin(),m_contactBuffer.end(),0.0);

		//	m_mutex_force.lock();
		//	m_kalman.correct(cv::Mat(1,1,CV_32FC1,cv::Scalar(response)));
		//	//m_contactAvgOverHeartCycle = sum/m_contactBuffer.size();
		//	m_contactAvgOverHeartCycle = response;
		//	m_contactMeasured = true;
		//	m_mutex_force.unlock();
		//}

		
	}
	else ::std::cout << "Problem with BOW" << ::std::endl;
}

float Camera_processing::PredictForce()
{
	cv::Mat prediction = m_kalman.predict();
	return prediction.at<float>(0,0);
}