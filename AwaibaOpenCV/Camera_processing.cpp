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
#define DEFAULT_BUFLEN 1024
#define DEFAULT_PORT "27015"
#define DEFAULT_PORT_PLOT "27016"
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
#include <vtkPlaneSource.h>
#include <vtkCamera.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkRegularPolygonSource.h>
#include <vtkSphereSource.h>
#include <vtkArrowSource.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTransform.h>
#include <vtkMath.h>
#include <vtkCallbackCommand.h>
#include "vtkKeyboardInteractionStyle.h"


// Project includes
#include "Camera_processing.h"
#include "CSV_reader.h"
#include "HTransform.h"
#include "MechanicsBasedKinematics.h"
#include "CTRFactory.h"
#include "FilterLibrary.h"

using namespace Core;
using namespace cv;
using namespace RecursiveFilter;

//#define __DESKTOP_DEVELOPMENT__
//#define	__BENCHTOP__


// VTK global variables (only way to get a thread running ...)
::std::mutex mutex_vtkRender;
#define VTK_CREATE(type, name) \
    vtkSmartPointer<type> name = vtkSmartPointer<type>::New()

VTK_CREATE(vtkRenderer, renDisplay3D);
VTK_CREATE(vtkRenderWindow, renderwindowDisplay3D);
VTK_CREATE(vtkRenderWindowInteractor, irenDisplay3D);
VTK_CREATE(KeyPressInteractorStyle, irenDisplay3DStyle);

VTK_MODULE_INIT(vtkRenderingFreeType)

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
		try
		{
			vtkRenderWindowInteractor *iren = static_cast<vtkRenderWindowInteractor*>(caller);
			iren->Render();
		}
		catch (Exception& e)
		{
			::std::cout << e.what() << ::std::endl;
		}
    }
};



// Constructor and destructor 
Camera_processing::Camera_processing(int period, bool sendContact) : m_Manager(Manager::GetInstance(0)), m_FramesPerHeartCycle(period), m_sendContact(sendContact)
	, m_radius_filter(3), m_theta_filter(4), m_wall_detector(), m_leak_detection_active(false), circStatus(CW), m_valveModel(), m_registrationHandler(&m_valveModel),
	wall_followed(IncrementalValveModel::WALL_FOLLOWED::LEFT), manualRegistration(false), m_clock(), reg_detected(false), realClockPosition(-1.0), manualRegistered(false),
	counterLine(0), m_contact_filtered(0), normal(0, 0, 1), tmpCentroid(0, 0)
{
	// Animate CRT to dump leaks to console after termination.
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	m_points = false;
	// George
	m_maxBufferSize = 10000;
	m_filter = new MedianFilter(5);
	m_freqFilter = new MovingAverageFilter(5);
	m_input_freq_received = false;
	m_network = false;
	registrationOffset = 0;

	clockFollowed = 0;

	image_center = ::Eigen::Vector2d(125, 125);
	center.x = image_center(0);
	center.y = image_center(1);
	displacement = ::Eigen::Vector2d(0, 250);;

	// apex visualization
	apexSource  = vtkSmartPointer<vtkRegularPolygonSource>::New();
	apexMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	apexMapper->SetInputConnection(apexSource->GetOutputPort());
	apexActor = vtkSmartPointer<vtkActor>::New();
	apexActor->SetMapper(apexMapper);
	apexActor->GetProperty()->SetOpacity(0.2);
	apexActor->GetProperty()->SetColor(1,0,0);
	renDisplay3D->AddActor(apexActor);

	// these are not currently rendered
	this->initializeLeaks();

	this->initializeRobotAxis();

	m_running = true;
	m_record = false;
	m_teleop = false;
	m_newdir = true; 
	//m_outputForce = false;
	m_outputForce = true;
	m_rotateImage = true;
	m_input_frequency = 80;
	m_input_plane_received = false;
	this->m_configuration.resize(5);
	m_board="NanoUSB2";
	m_ControlLED=false;
	m_contact_response = 0;
	m_contact_response_prev = 0;
	m_OK = false;
	newImg = false;
	newImg_force = false;
	m_use_automatic_transition = true;
	for (int i = 0 ;i < 3; ++i)
		robot_position[i] = desired_vel[i] = m_model_robot_position[i] = 0;

	inner_tube_rotation = 0;
	// channel center //
	//// scope 1
	//m_channel_center(0) = 37;
	//m_channel_center(1) = 102;

	//// scope 2
	//m_channel_center(0) = 213;
	//m_channel_center(1) = 102;

	//// scope 3
	//m_channel_center(0) = 180;
	//m_channel_center(1) = 70;

	//// scope 4
	//m_channel_center(0) = 51;
	//m_channel_center(1) = 133;

	//// scope 5
	//m_channel_center(0) = 152;
	//m_channel_center(1) = 151;

	// scope 6
	m_channel_center(0) = 120;
	m_channel_center(1) = 110;

	//// scope 7
	//m_channel_center(0) = 86;
	//m_channel_center(1) = 118;

	m_registrationHandler.setWorkingChannel(m_channel_center);

	m_use_original_line_transition = false;
	m_use_green_line_transition = true;

	::std::string svm_base_folder = "./SVM_params/";
	m_linedetected = false;

	// circumnavigation
	m_circumnavigation = false;
	m_apex_to_valve = false;
	//m_use_online_model = false;
	m_show_line = false;
	// Parse options in camera.csv file
	// TODO: handle errors better and do not fallback to default config
	
	m_contact_gain = 0;
	m_contact_D_gain = 0;
	m_contact_I_gain = 0;
	m_contact_desired_ratio = 0;
	m_is_control_active = 0;
	m_breathing = 0;
	m_centroid[0] = 0;
	m_centroid[1] = 0;

	m_tangent[0] = 0;
	m_tangent[1] = 0;

	m_commanded_vel[0] = 0;
	m_commanded_vel[1] = 0;

	m_centroid_apex_to_valve[0] = 0;
	m_centroid_apex_to_valve[1] = 0;
	ParseOptions op = ParseOptions("./camera_info.csv");

	if (op.getStatus())
	{
		::std::cout << "Successfully parsed camera info file" << endl;
		m_saveDir = op.getSaveDir();
#ifndef __DESKTOP_DEVELOPMENT__
		g_r = op.getWhiteBalance()[0];
		g_g = op.getWhiteBalance()[1];
		g_b = op.getWhiteBalance()[2];
#endif
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

//#ifndef __DESKTOP_DEVELOPMENT__
	::std::cout << "Initializing Force estimator ..." << ::std::endl;
#ifdef __BENCHTOP__
	svm_base_folder = "SVM_params_bench\\";
#endif
	InitForceEstimator(svm_base_folder + "output_", 3.0, m_KFParams[0], m_KFParams[1]);
	::std::cout << "Force estimator initialized" << ::std::endl;
//#endif
	m_modelBasedLine.setInitImageRotation(this->rotation);
	m_estimateFreq = false;
	m_measured_period = 0.0;
	robot_rotation = 0.0;
	// recompute from recorded data
	//m_cameraFrameRate = 48.0;
	m_cameraFrameRate = 46.0;

	// All errors are reported as std::exception.
	try
	{
#ifndef __DESKTOP_DEVELOPMENT__
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
		//m_sensor.DoLineCorrectionChange = true;


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

		m_apex_initialized = false;
		m_centroid_apex_to_valve[0] = 0;
		m_centroid_apex_to_valve[1] = 0;
		m_wall_detected = false;
		m_state_transition = false;
		//*** Automatic Exposure Control Registers ***/
		//m_Manager.SetFPGAData(0x00500000,0x02010200);
		m_Manager.SetFPGAData(0x00500000,0x42010200);
		
		

#endif
		Mat RgbFrame = Mat(250,250,CV_8UC3);

		m_OK = true;

		if (renderShape)
		{
#ifndef __DESKTOP_DEVELOPMENT__
			::std::thread t_acquire (&Camera_processing::acquireImages, this);
			::std::thread t_display (&Camera_processing::displayImages, this);
//			::std::thread t_force (&Camera_processing::computeForce, this);//
			::std::thread t_record (&Camera_processing::recordImages, this);
#endif
			::std::thread t_network (&Camera_processing::networkKinematics, this);
			::std::thread t_vtk (&Camera_processing::robotDisplay, this);
			::std::thread t_vtk_render (&Camera_processing::vtkRender, this);
			//::std::thread t_plot(&Camera_processing::OnLinePlot, this);

#ifndef __DESKTOP_DEVELOPMENT__
			t_acquire.join();
			t_display.join();
//			t_force.join();
			t_record.join();
#endif
			t_network.join();
			t_vtk_render.join();
			t_vtk.join();
			//t_plot.join();
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
	case 'a':
		initializeApex();
		break;
	case 't':
		m_use_automatic_transition = !m_use_automatic_transition;
		if (m_use_automatic_transition)
			::std::cout << "automatic transition is activated" << ::std::endl;
		else
			::std::cout << "automatic transition is deactivated" << ::std::endl;
		break;
	case 's':
		m_show_line = !m_show_line;
		if (m_show_line)
			::std::cout << "predicted tangent is activated" << ::std::endl;
		else
			::std::cout << "predicted tangent is deactivated" << ::std::endl;
		break;
	case 'r':
		if (m_record) m_newdir = true;
		m_record = !m_record;
		break;
	case 'c':
		m_sendContact = !m_sendContact;
		if (m_sendContact)
			::std::cout << "contact mode is on"  << ::std::endl;
		else
			::std::cout << "contact-ratio mode is on" << ::std::endl;
		break;
	case 'i':
		m_valveModel.resetModel();
		this->m_clock.reset();
		this->m_registrationHandler.reset();
		this->counterLine = 0;
		this->realClockPosition = -1.0;
		this->reg_detected = false;
		::std::cout << "model was reset" << ::std::endl;
		break;
	case 'f':
		//m_estimateFreq = !m_estimateFreq;
		//if (m_estimateFreq)
		//{
		//	::std::cout << "frequency estimation is switched on" << ::std::endl;
		//	m_freqFilter->resetFilter();
		//}
		//break;
		this->manualRegistration = !this->manualRegistration;
		::std::cout << "automatic registration is switched " << (this->manualRegistration ? "off" : "on") << ::std::endl;
		this->m_valveModel.resetRegistration();

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
	case '4':
		m_use_green_line_transition = !m_use_green_line_transition;
		checkTransitionState();
		break;
	case '5':
		m_use_original_line_transition = !m_use_original_line_transition;
		checkTransitionState();
		break;
	case '6':
		circStatus = CW;
		::std::cout << "operator is moving CW" << ::std::endl;
		break;
	case '7':
		circStatus = CCW;
		::std::cout << "operator is moving CCW" << ::std::endl;
		break;
	case 'l':
		m_leak_detection_active = !m_leak_detection_active;
		::std::cout << "leak detection is " << (m_leak_detection_active ? "activated" : "deactivated") << ::std::endl;
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
	
	auto start_rec = std::chrono::high_resolution_clock::now();

	while(m_running)
	{

		try
		{
			
			if(m_Manager.GetNextFrame(&argbFrame1, &rawFrame1))
			{

				//Get the pixels from the rawFrame to show to the user
				for (int i = 0; i < 250*250; ++i)
				{
					rData[i] = ((argbFrame1.Begin()._Ptr[i] & 0x00FF0000) >> 16);
					gData[i] = ((argbFrame1.Begin()._Ptr[i] & 0x0000FF00) >> 8);
					bData[i] = ((argbFrame1.Begin()._Ptr[i] & 0x000000FF));
				}
				array_to_merge[0] *= g_b;			
				array_to_merge[1] *= g_g;			
				array_to_merge[2] *= g_r;			

				mutex_robotjoints.lock();
				std::vector<double> configuration = m_configuration;
				mutex_robotjoints.unlock();

				cv::merge(&array_to_merge[0], array_to_merge.size(), RgbFrame);
				newImg = true;

				if(!RgbFrame.empty())
				{
					auto stop_rec = std::chrono::high_resolution_clock::now();					
					auto duration = std::chrono::duration_cast<::std::chrono::microseconds> (stop_rec - start_rec);
					double durInSec = ((double)  duration.count()) / 1.0e06;
			
					//m_cameraFrameRate = 1.0/durInSec;

					start_rec = stop_rec;
				}
				
				if ((! RgbFrame.empty()) && (m_outputForce)) 
					UpdateForceEstimator(RgbFrame);

				mutex_img.lock();
				if (m_record)
				{
					ImgBuf el;
					el.img = RgbFrame.clone();
					el.timestamp = argbFrame1.GetTimeStamp();
					el.robot_joints = configuration;
					m_ImgBuffer.push(el);
				}
				mutex_img.unlock();
			}
			else
			{
				Sleep(1);
			}

		}
		catch(const std::exception &ex){::std::cout << ex.what() << ::std::endl;}

	}

	::std::cout << "Images Acquisition Thread exited successfully" << ::std::endl;
}


void Camera_processing::displayImages(void)
{
	::std::cout << "Start Display" << ::std::endl;

	Mat frame = Mat(250, 250, CV_8UC3);
	Mat frame_rotated = Mat(250, 250, CV_8UC3);

	char key;
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
			if (this->m_sendContact)
				::std::cout << "contact measurements:" << this->m_contact_response << " (raw), " << this->m_contact_filtered << " (filtered)" << ::std::endl; 

			if (m_circumnavigation)
				this->computeCircumnavigationParameters(frame);
			else if (m_apex_to_valve)
				this->computeApexToValveParameters(frame);
			else // maybe put initialization code here
			{
				m_theta_filter.resetFilter();							// this is not the proper place
				m_radius_filter.resetFilter();

				m_state_transition = false;
				this->detected_valve.clear();

				m_linedetected = false;
				m_wall_detected = false;

				this->realClockPosition = -1.0;
				this->counterLine = 0;

			}

			int x = 0, y = 0;
			bool leak_detected = false;
			if (m_leak_detection_active)
				leak_detected = this->detectLeaks(frame, x, y);
			

			display = false;

			double rot = robot_rotation;
			
			if (!m_rotateImage) 
				rot = 0.0;

			rot_mat = getRotationMatrix2D( center, rotation - rot * 180.0/3.141592, 1.0 );

			warpAffine( frame, frame_rotated, rot_mat, frame_rotated.size() );

			if (leak_detected)
				cv::circle( frame_rotated, Point(y, 250-x), 5, ::cv::Scalar( 0, 0, 0 ),  -1);

			if (rec) // draw a red circle on frame when recording
				cv::circle( frame_rotated, Point( 240, 10 ), 5, ::cv::Scalar( 0, 0, 255 ),  -1);
			
			if (teleop) // draw a green circle on frame when teleoperating
				cv::circle( frame_rotated, Point( 220, 10 ), 5, ::cv::Scalar( 0, 255, 0 ),  -1);

			if (m_apex_to_valve)
				this->plotCommandedVelocities(frame_rotated);

			if (m_circumnavigation)
				this->plotCommandedVelocitiesCircum(frame_rotated);

			if (m_circumnavigation)
			{
				this->computeClockfacePosition();			// this updates the clock position based on measurements
				this->m_clock.update(frame_rotated, this->realClockPosition);
			}
			double width = 50, height = 50;
			::cv::Rect rec = ::cv::Rect(this->regPointCV.x - 0.5 * width, this->regPointCV.y - 0.5 * height, width, height);
			if (this->reg_detected)
				::cv::rectangle(frame_rotated, rec, ::cv::Scalar(0, 0, 255), 2);
			
			this->reg_detected = false;

			//::cv::Point p1 = ::cv::Point(this->line_to_plot[0] - 50 * this->m_tangent[0], this->line_to_plot[1] - 50 * this->m_tangent[1]);
			//::cv::Point p2 = ::cv::Point(this->line_to_plot[0] + 50 * this->m_tangent[0], this->line_to_plot[1] + 50 * this->m_tangent[1]);
			//::cv::line(frame_rotated, p1, p2, ::cv::Scalar(0, 255, 0), 1);

			cv::imshow( "Display", frame_rotated );

			frame_rotated.copyTo(image_annotated);

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
	Mat frame, rotatedFrame;
	ArgbFrame::time_type timestamp = 0;
	std::vector<double> robot_joint;

	::std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(3);

	auto start_record = std::chrono::high_resolution_clock::now();

	::std::ofstream bundle;
	::cv::VideoWriter video;

    Mat rot_mat;
	::cv::Mat video_image;
	while(m_running)
	{
		ImgBuf element;

		if ( m_ImgBuffer.tryPop(element))
		{

			// create a new directory if necessary
			if(m_newdir) 
			{
				createSaveDir();
				if (video.isOpened())
					video.release();

				if (bundle.is_open())
				{
					bundle.flush();
					bundle.close();
				}
				bundle.open(m_imgDir + "data.txt");
				video.open(m_imgDir + "video.avi", ::cv::VideoWriter::fourcc('M','P','E','G'), 46, ::cv::Size(250, 250));

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

				//if (robot_joint.size() > 0)
				//{
				//	ofstream joints_file;
				//	joints_file.open (filename_joints);
				//	for(std::vector<double>::const_iterator i = robot_joint.begin(); i != robot_joint.end(); ++i) {
				//			joints_file << *i << ",";
				//	}
				//	joints_file << m_input_frequency << "," << m_contactAvgOverHeartCycle << "," << m_contact_response << ",";
				//	
				//	for (int i = 0; i < 3; ++i)
				//		joints_file << m_model_robot_position[i] << ", ";

				//	joints_file << m_contact_gain << ", " << m_contact_D_gain << ", " << m_contact_I_gain << ", " << m_is_control_active << ", " << m_contact_desired_ratio << ", " << m_breathing;
				//	joints_file << '\n';
				//	joints_file.close();
				//}

				if (bundle.is_open())
				{
					bundle << timestamp << ", ";
					for(std::vector<double>::const_iterator i = robot_joint.begin(); i != robot_joint.end(); ++i) {
							bundle << *i << ", ";
					}
					bundle << m_input_frequency << "," << m_contactAvgOverHeartCycle << "," << m_contact_response << ",";
					
					for (int i = 0; i < 3; ++i)
						bundle << m_model_robot_position[i] << ", ";

					bundle << m_contact_gain << ", " << m_contact_D_gain << ", " << m_contact_I_gain << ", " << m_is_control_active << ", " << m_contact_desired_ratio << ", " << m_breathing;
					bundle << '\n';
					bundle.flush();
				}


				if (video.isOpened())
				{
					this->image_annotated.copyTo(video_image);
					video.write(video_image);
				}

			}
			catch (runtime_error& ex) 
			{
				::std::cout << "Exception in image recording:" <<  ex.what() << ::std::endl;
				video.release();
				bundle.close();
			}

		}

	}
	bundle.close();
	video.release();
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
	//iResult = getaddrinfo("127.0.0.1", DEFAULT_PORT, &hints, &result);
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

	//test
	if (iResult !=0)
	{
		::std::cout << "connection error" << ::std::endl;
		return false;
	}
	else
		::std::cout << "Successfully connected to server" << std::endl;

	m_network = true;

	::std::vector<double> configuration(5);
	double rotation[3] = {0};
	double translation[3] = {0};
	Vec3 position;

	do {
		
		//Receive data through the network
        iResult = recv(ConnectSocket, recvbuf, recvbuflen, 0);

		// Convert the received data to a vector of doubles
		::std::string conf_str(recvbuf);
		configuration = DoubleVectorFromString(conf_str);
		this->parseNetworkMessage(configuration);

		// Convert the received the configuration to comply with the definition of the mechanics based kinematics implementation
		MechanicsBasedKinematics::RelativeToAbsolute(robot, &m_configuration[0], rotation, translation);

		// compute kinematics and get tip rotation of innermost tube
		if (kinematics->ComputeKinematics(rotation, translation))
		{
			robot_rotation = kinematics->GetInnerTubeRotation();
			this->inner_tube_rotation = robot_rotation;
			double smax = robot->GetLength();

			// not very efficient
			robot_arclength.clear();
			SolutionFrames.clear();
			for (int i = 0; i<npoints;i++) robot_arclength.push_back((1.0*i)/npoints*smax);

			kinematics->GetBishopFrame(robot_arclength, SolutionFrames);

			mutex_robotshape.lock();
			m_SolutionFrames = SolutionFrames;
			mutex_robotshape.unlock();
			position = SolutionFrames.back().GetPosition();
			
		}

		for (int i = 0; i < 3; ++i)
			this->robot_position[i] = position[i];

		float force = 0.0;
		bool newMeasurement = false;

		if (m_outputForce)
		{
			m_mutex_force.lock();
			force = m_contactAvgOverHeartCycle;
			newMeasurement = m_contactMeasured;
			m_contactMeasured = false;
			m_mutex_force.unlock();
		}

		char s_force[5]; 
		::std::sprintf(s_force,"%.2f",force);

		/// create network message for circumnavigation
		::ostringstream ss;
		ss << force << " " << m_linedetected << " " << m_contact_response << " " << m_centroid[0] << " " << m_centroid[1] << " " << m_tangent[0] << " " << m_tangent[1] << " ";

		ss << m_wall_detected << " " << m_centroid_apex_to_valve[0] << " " << m_centroid_apex_to_valve[1] << " ";

		if (m_circumnavigation)
		{
			m_state_transition = false;
			this->detected_valve.clear();
		}

		ss << m_state_transition << " ";

		double clockfacePosition = -1;
		::Eigen::Vector3d point;
		if (this->m_valveModel.isInitialized())
			this->m_valveModel.getClockfacePosition(this->m_model_robot_position[0], this->m_model_robot_position[1], this->m_model_robot_position[2], clockfacePosition, point);

		ss << this->realClockPosition << " ";

		ss << this->m_valveModel.isRegistered() << " ";

		if (m_apex_initialized)
			ss << "1" << " " << apex_coordinates[0] << " "  << apex_coordinates[1] << " " << apex_coordinates[2] << " " << apex_coordinates[3] << " " <<  apex_coordinates[4] << " ";
		else
			ss << "0";
		ss << ::std::endl;
		//::std::cout << ss.str().c_str() << ::std::endl;
		/*****
		Acknowledge good reception of data to network for preparing next transmission
		*****/
		if (newMeasurement) 
			iResult = send( ConnectSocket, ss.str().c_str(),  ss.str().size() + 1, 0 );
		else 
			iResult = send( ConnectSocket, "NOF", 5, 0 );

    } while( (iResult > 0) && m_running);

    // cleanup
    closesocket(ConnectSocket);
    WSACleanup();

	::std::cout << "Kinematics Thread exited successfully" << ::std::endl;
	m_network = false;
    return 0;
}

void Camera_processing::parseNetworkMessage(::std::vector<double>& msg)
{
	// first get the configuration
	this->mutex_robotjoints.lock();
	::std::copy(msg.begin(), msg.begin() + 5, this->m_configuration.begin());
	this->mutex_robotjoints.unlock(); 

	this->mutex_teleop.lock();
	this->m_teleop = msg[5];
	this->mutex_teleop.unlock();

	this->m_input_frequency = msg[6];

	if (this->m_input_frequency < 0)
		this->m_input_frequency= 80;


	// need to add plane stuff
	this->mutex_robotshape.lock();
	memcpy(m_target, &msg.data()[7], 3 * sizeof(double));

	// new flag for circumnavigation
	m_circumnavigation = msg.data()[10];
	m_apex_to_valve = msg.data()[11];

	this->m_FramesPerHeartCycle = msg.data()[12]* 60 * m_cameraFrameRate/m_input_frequency;

	memcpy(this->m_model_robot_position, &msg.data()[13], 3 * sizeof(double));
	robot_positionEig = ::Eigen::Map<::Eigen::Vector3d> (this->m_model_robot_position, 3);

	this->m_contact_gain = msg.data()[16];
	this->m_contact_D_gain = msg.data()[17];
	this->m_contact_I_gain = msg.data()[18];
	this->m_is_control_active = msg.data()[19];
	this->m_contact_desired_ratio = msg.data()[20];
	this->m_breathing = msg.data()[21];

	this->m_commanded_vel[0] = msg.data()[22];
	this->m_commanded_vel[1] = msg.data()[23];
	//::std::cout << "vels in network: " << this->m_commanded_vel[0] << " " << this->m_commanded_vel[1] << ::std::endl;
	int tmp = msg.data()[24];
	switch (tmp)
	{
		case -1:
			this->wall_followed = IncrementalValveModel::WALL_FOLLOWED::LEFT;
			break;
		case -2:
			this->wall_followed = IncrementalValveModel::WALL_FOLLOWED::TOP;
			break;
		case -3:
			this->wall_followed = IncrementalValveModel::WALL_FOLLOWED::BOTTOM;
			break;
		default:
			this->wall_followed = IncrementalValveModel::WALL_FOLLOWED::USER;
			this->clockFollowed = tmp;
			break;
	}

	this->registrationOffset = msg.data()[25];
	this->m_valveModel.setInitialOffset(this->registrationOffset);
	this->m_clock.setInitialOffset(this->registrationOffset/30.0);

	m_input_plane_received = msg.data()[26];
	if (m_input_plane_received)
	{
		memcpy(m_normal, &msg.data()[27], 3 * sizeof(double));
		memcpy(m_center, &msg.data()[30], 3 * sizeof(double));
		m_radius = msg.data()[33];

		pointsOnValve.clear();
		int num_of_points = msg.data()[34];
		for (int i = 0; i < 3 * num_of_points; ++i)
			pointsOnValve.push_back(msg[35+i]);
	}
	this->mutex_robotshape.unlock();

}

void Camera_processing::initializeValveDisplay()
{
	circleSource = vtkSmartPointer<vtkRegularPolygonSource>::New();
	circleSource->SetNumberOfSides(50);
	circleSource->SetRadius(0);						
	double tmp[3] = {0, 0, 0};
	circleSource->SetCenter(tmp);				
	circleSource->SetNormal(tmp);

	mapperCircle = vtkSmartPointer<vtkPolyDataMapper>::New();

	mapperCircle->SetInputConnection(circleSource->GetOutputPort());;
	actorCircle =	vtkSmartPointer<vtkActor>::New();
	actorCircle->SetMapper(mapperCircle);
	actorCircle->GetProperty()->SetColor(0, 1, 0);
	actorCircle->GetProperty()->SetEdgeColor(0,1,0);
	actorCircle->GetProperty()->SetEdgeVisibility(1);
	actorCircle->GetProperty()->SetOpacity(0.3);
	renDisplay3D->AddActor(actorCircle);


	circleSourceOnLine = vtkSmartPointer<vtkRegularPolygonSource>::New();
	circleSourceOnLine->SetNumberOfSides(10);
	circleSourceOnLine->SetRadius(0);						
	circleSourceOnLine->SetCenter(tmp);				
	circleSourceOnLine->SetNormal(tmp);

	mapperCircleOnLine = vtkSmartPointer<vtkPolyDataMapper>::New();

	mapperCircleOnLine->SetInputConnection(circleSourceOnLine->GetOutputPort());;
	actorCircleOnline =	vtkSmartPointer<vtkActor>::New();
	actorCircleOnline->SetMapper(mapperCircleOnLine);
	actorCircleOnline->GetProperty()->SetColor(1, 0, 0);
	actorCircleOnline->GetProperty()->SetEdgeColor(1, 0, 0);
	actorCircleOnline->GetProperty()->SetEdgeVisibility(1);
	actorCircleOnline->GetProperty()->SetOpacity(0.4);
	renDisplay3D->AddActor(actorCircleOnline);


	// initializing vtk structure for points-on-valve visualization
	points = vtkSmartPointer<vtkPoints>::New();
	points->InsertNextPoint (0.0, 0.0, 0.0);
	pointsPolydata = vtkSmartPointer<vtkPolyData>::New();
	pointsPolydata->SetPoints(points);
 
	vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	vertexFilter->SetInputData(pointsPolydata);
    vertexFilter->Update();
 
	polydata = vtkSmartPointer<vtkPolyData>::New();
	polydata->ShallowCopy(vertexFilter->GetOutput());
  
	mapperPoints = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapperPoints->SetInputData(polydata);
 
	vtkSmartPointer<vtkActor> actorPoints =  vtkSmartPointer<vtkActor>::New();
	actorPoints->SetMapper(mapperPoints);
	actorPoints->GetProperty()->SetPointSize(5);
	renDisplay3D->AddActor(actorPoints);


}

void Camera_processing::displayValve(double normal[3], double center[3], double radius)
{
	// valve visualization
	circleSource->SetRadius(radius);						
	circleSource->SetCenter(center);				
	circleSource->SetNormal(normal);

	// line instead of arrow - TBD
	addArrow(normal, center);

	updatePoints();
	vertexFilter->Update();
	polydata->ShallowCopy(vertexFilter->GetOutput());
	
}

void Camera_processing::initializeTarget()
{
	// target visualization
	double targetPosition[3] = {12,12, 40};
	sphereSource  = vtkSmartPointer<vtkSphereSource>::New();
	sphereSource->SetCenter(targetPosition);
	sphereSource->SetRadius(1);
	vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	sphereMapper->SetInputConnection(sphereSource->GetOutputPort());
	vtkSmartPointer<vtkActor> sphereActor = vtkSmartPointer<vtkActor>::New();
	sphereActor->SetMapper(sphereMapper);
	sphereActor->GetProperty()->SetOpacity(0.2);
	sphereActor->GetProperty()->SetColor(1,0,0);
	renDisplay3D->AddActor(sphereActor);

}

void Camera_processing::robotDisplay(void)
{

	this->initializeTarget();
	this->initializeValveDisplay();
	this->initializeArrow();

	double target[6] = {0};

	bool planeReceived = false;
	double center[3] = {0};
	double radius;
	double normal[3] = {0};

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
	::Eigen::Vector3d tmp;
	::std::vector<double> s;
	double error[3] = {0};
	double actualPosition[3] = {0};

	::std::vector<::Eigen::Vector3d> leaks;
	while(m_running)
	{
		memcpy(actualPosition, this->m_model_robot_position, 3 * sizeof(double));
		auto duration_s = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
		if (duration_s.count()>=50) 
		{
			try
			{
				start = std::chrono::high_resolution_clock::now();

				mutex_robotshape.lock();
				std::vector<SE3> SolutionFrames = m_SolutionFrames;
				memcpy(target, m_target, 6 * sizeof(double));
				memcpy(center, m_center, 3 * sizeof(double));
				memcpy(normal, m_normal, 3 * sizeof(double));
				radius = m_radius;
				planeReceived = m_input_plane_received;
				mutex_robotshape.unlock();


				npts = SolutionFrames.size();

				if (npts>2)
				{
					//lineSource->SetNumberOfPoints(npts + 1); //  to add the straight segment
					//for (unsigned int i = 0; i < npts; i++)
					//	lineSource->SetPoint(i, SolutionFrames[i].GetPosition()[0],SolutionFrames[i].GetPosition()[1], SolutionFrames[i].GetPosition()[2]);
					//for (int i = 0; i < 3; ++i)
					//	tmp[i] = SolutionFrames[npts-1].GetPosition()[i] + 20*SolutionFrames[npts-1].GetZ()[i];  // remove hardcoded 20;
					//lineSource->SetPoint(npts, tmp[0], tmp[1], tmp[2]);
					for (int i = 0; i < 3; ++i)
						error[i] = actualPosition[i] - (SolutionFrames.back().GetPosition()[i] + 20*SolutionFrames[npts-1].GetZ()[i]); 
					s = linspace(0, 1, npts+2);
					lineSource->SetNumberOfPoints(npts + 1); //  to add the straight segment
					for (unsigned int i = 0; i < npts; i++)
						lineSource->SetPoint(i, SolutionFrames[i].GetPosition()[0] + s[i] * error[0],SolutionFrames[i].GetPosition()[1]  + s[i] * error[1], SolutionFrames[i].GetPosition()[2] + s[i] * error[2]);
					for (int i = 0; i < 3; ++i)
						tmp[i] = SolutionFrames[npts-1].GetPosition()[i] + 20*SolutionFrames[npts-1].GetZ()[i] +  error[i];  // remove hardcoded 20;
					lineSource->SetPoint(npts, tmp[0], tmp[1], tmp[2]);

				}
				else
				{
					lineSource->SetNumberOfPoints(2); // set new data
					lineSource->SetPoint(0, 0.0,0.0,0.0);
					lineSource->SetPoint(1, 0.0,0.0,10.0);
				}
				
				if (planeReceived)
				{
					this->displayValve(normal, center, radius);
					planeReceived = false;
				}
				
				if (m_network)
					this->updateRobotTargetVisualization(target);

				this->mutex_robotshape.lock();
				this->m_input_plane_received = planeReceived;
				this->mutex_robotshape.unlock();


				if (this->m_valveModel.isInitialized())
				{
					this->m_valveModel.getCenter(center);
					radius = this->m_valveModel.getRadius();
					this->m_valveModel.getNormal(normal);
					this->circleSourceOnLine->SetCenter(center);
					this->circleSourceOnLine->SetRadius(radius);
					this->circleSourceOnLine->SetNormal(normal);
				}
				else
					this->circleSourceOnLine->SetRadius(0.0);

			}
			catch (runtime_error& ex) 
			{
				::std::cout << "Exception in vtk display:" <<  ex.what() << ::std::endl;
			} 
		}

	}
	::std::cout << "VTK Robot Display Thread exited successfully" << ::std::endl;

}

void Camera_processing::updateRobotTargetVisualization(double targetPosition[3])
{
	this->sphereSource->SetCenter(targetPosition);
}

void Camera_processing::vtkRender(void)
{
	
	renderwindowDisplay3D->AddRenderer(renDisplay3D);
	
	irenDisplay3D->SetInteractorStyle(irenDisplay3DStyle);
	irenDisplay3D->SetRenderWindow(renderwindowDisplay3D);
	irenDisplay3D->Initialize();

	renDisplay3D->ResetCamera();
	renDisplay3D->Render();

	////////////////////////////////////////////////////////////////////////////////////
	//This is an approximation of the reachable workspace
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
	///////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////////////////////////////////////
	//Axes at beginning of robot
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


	////////////////////////////////////////////////////////////////////////////////////
	//Axes at beginning of robot
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
	/////////////////////////////////////////////////////////////////////////////////////


	vtkSmartPointer<CommandSubclass2> timerCallback = vtkSmartPointer<CommandSubclass2>::New();
	irenDisplay3D->AddObserver ( vtkCommand::TimerEvent, timerCallback );
	irenDisplay3D->CreateRepeatingTimer(50);
	
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
#ifdef  __BENCHTOP__
		m_bow.LoadFromFile(svm_base_path);
#else
		m_bof.load(svm_base_path);
#endif
		m_bof.load(svm_base_path);
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
	if (tmp2.size() < 1)
		return;
	m_FramesPerHeartCycle = (int) 2.0 * static_cast<int>(::std::accumulate(tmp2.begin(), tmp2.end(), 0.0))/tmp2.size();

	::std::cout << "heart frequency:" << m_freqFilter->step(60 * m_cameraFrameRate/(m_FramesPerHeartCycle * 0.5)) << "BPM" << ::std::endl;
}

void Camera_processing::UpdateForceEstimator(const ::cv::Mat& img)
{
#ifdef __BENCHTOP__
	::std::vector<::std::string> classes = m_bow.getClasses();
#else
	::std::vector<::std::string> classes = m_bof.getClasses();
#endif

	float response = 0.0;

#ifdef __BENCHTOP__
	if (m_bow.predictBOW(img,response)) 
#else
	if (m_bof.predict(img,response)) 
#endif
	{

		if (classes[(int) response] == "Free") 
			response = 0.0;
		else 
			response = 1.0;

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
			
			float sum = std::accumulate(m_contactBufferFiltered.rbegin(), m_contactBufferFiltered.rbegin() + this->m_FramesPerHeartCycle, 0.0);

			m_mutex_force.lock();

			m_contactAvgOverHeartCycle = sum/m_FramesPerHeartCycle;

			m_contact_response = response;
			m_contact_filtered = m_contactBufferFiltered.back();

			if (m_sendContact) 
				m_contactAvgOverHeartCycle = response;

			m_contactMeasured = true;

			m_mutex_force.unlock();

		}

	}
	else 
		::std::cout << "Problem with BOW" << ::std::endl;
}

float Camera_processing::PredictForce()
{
	cv::Mat prediction = m_kalman.predict();
	return prediction.at<float>(0,0);
}


void Camera_processing::OnLinePlot()
{

	WSADATA wsaData;
    int iResult;

    SOCKET ListenSocket = INVALID_SOCKET;
    SOCKET ClientSocket = INVALID_SOCKET;

    struct addrinfo *result = NULL;
    struct addrinfo hints;

    int iSendResult;
    char recvbuf[DEFAULT_BUFLEN];
    int recvbuflen = DEFAULT_BUFLEN;


    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return;
    }

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;

    // Resolve the server address and port
    iResult = getaddrinfo(NULL, DEFAULT_PORT_PLOT, &hints, &result);
    if ( iResult != 0 ) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return;
    }

    // Create a SOCKET for connecting to server
    ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (ListenSocket == INVALID_SOCKET) {
        printf("socket failed with error: %ld\n", WSAGetLastError());
        freeaddrinfo(result);
        WSACleanup();
        return;
    }

    // Setup the TCP listening socket // this conflicts with using namespace std in LieGroup -> FIX IT!
    iResult = ::bind( ListenSocket, result->ai_addr, (int)result->ai_addrlen);
    if (iResult == SOCKET_ERROR) {
        printf("bind failed with error: %d\n", WSAGetLastError());
        freeaddrinfo(result);
        closesocket(ListenSocket);
        WSACleanup();
        return;
    }

    freeaddrinfo(result);

    iResult = listen(ListenSocket, SOMAXCONN);
    if (iResult == SOCKET_ERROR) {
        printf("listen failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        return;
    }

    // Accept a client socket
    ClientSocket = accept(ListenSocket, NULL, NULL);
    if (ClientSocket == INVALID_SOCKET) {
        printf("accept failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        return;
    }

    // No longer need server socket
    closesocket(ListenSocket);
	int counter = 0;
	
    do {
		::std::ostringstream ss;
		//ss << sin(2 * M_PI * (double) counter/100);
		//ss << "sin,cos," << sin(2 * M_PI * (double) counter/100) << "," << cos(2 * M_PI * (double) counter/100);
		//data[18] = 20 + 1 * sin(2*M_PI * (double) counter/400);
		ss <<"frequency,CR,tipZ," << m_input_frequency << "," << m_contactAvgOverHeartCycle << "," << m_SolutionFrames.back().GetPosition()[2];
		//for(int i = 0; i < 22; ++i)
		//	ss << data[i] << " ";
		//counter++;
		// send data
        iSendResult = send( ClientSocket, ss.str().c_str(),  ss.str().size() + 1, 0 );
        if (iSendResult == SOCKET_ERROR) {
            printf("send failed with error: %d\n", WSAGetLastError());
            closesocket(ClientSocket);
            WSACleanup();
            return;
        }
        else if (iSendResult == 0)
            printf("Connection closing...\n");
 
		iResult = recv(ClientSocket, recvbuf, DEFAULT_BUFLEN, 0);

    } while (iResult > 0);

    closesocket(ClientSocket);
    WSACleanup();

	OnLinePlot();
    return;

}

void Camera_processing::updatePoints()
{
	//vtkSmartPointer<vtkPoints> tmpPoints = vtkSmartPointer<vtkPoints>::New();
	points->Reset();
	points->Squeeze();
	double tmpPoint[3] = {0};
	for (int i = 0; i < pointsOnValve.size(); i += 3)
	{
		tmpPoint[0] = pointsOnValve[i];
		tmpPoint[1] = pointsOnValve[i + 1];
		tmpPoint[2] = pointsOnValve[i + 2];
		points->InsertNextPoint(tmpPoint);
	}
	//points->DeepCopy(tmpPoints);
}

void Camera_processing::updateArrowOrientation(double normal[3], vtkSmartPointer<vtkMatrix4x4> matrix)
{
	::Eigen::Vector3d normalEig = ::Eigen::Map<::Eigen::Vector3d> (normal, 3);
	::Eigen::Vector3d z_vectorEig(0, 0, 1);


	::Eigen::Vector3d axis = normalEig.cross(z_vectorEig);
	double theta = ::std::acos(normalEig.dot(z_vectorEig)/normalEig.norm());
	::Eigen::AngleAxis<double> aa(theta, axis);
	::Eigen::MatrixXd rot = aa.toRotationMatrix();

	for(int i = 0; i < 3; ++i)
		for(int j = 0; j < 3; ++j)
			 matrix->SetElement(i, j, rot(i,j));

	
}

void Camera_processing::addArrow(double normal[3], double center[3])
{
	// Create three points
	double p0[3], p1[3];
	memcpy(p0, center, 3 * sizeof(double));

	for(int i = 0; i < 3; ++i)
		p1[i] = p0[i] - normal[i] * 10;
 
	// Create a vtkPoints container and store the points in it


	pts->InsertNextPoint(p0);
	pts->InsertNextPoint(p1);
 
	// Add the points to the polydata container
	linesPolyData->SetPoints(pts); 
	// Create the first line (between Origin and P0)

	line0->GetPointIds()->SetId(0, 0); // the second 0 is the index of the Origin in linesPolyData's points
	line0->GetPointIds()->SetId(1, 1); // the second 1 is the index of P0 in linesPolyData's points

	// Create a vtkCellArray container and store the lines in it

	lines->InsertNextCell(line0);
 
	// Add the lines to the polydata container
	linesPolyData->SetLines(lines);
	mapperArrow->SetInputData(linesPolyData);

}

void Camera_processing::initializeArrow()
{
	// arrow initialization
	linesPolyData = vtkSmartPointer<vtkPolyData>::New();
	pts = vtkSmartPointer<vtkPoints>::New();
	line0 = vtkSmartPointer<vtkLine>::New();
	lines = vtkSmartPointer<vtkCellArray>::New();
	mapperArrow = vtkSmartPointer<vtkPolyDataMapper>::New();
	actorArrow = vtkSmartPointer<vtkActor>::New();
	actorArrow->SetMapper(mapperArrow);
	renDisplay3D->AddActor(actorArrow);	
}

void Camera_processing::computeCircumnavigationParametersDebug(const ::cv::Mat& img)
{
	m_centroid[0] = 110;
	m_centroid[1] = 125;

	m_tangent[0] = 0;
	m_tangent[1] = 1;

	m_linedetected = true;
}

void Camera_processing::computeCircumnavigationParameters(const ::cv::Mat& img)
{

	m_linedetected = false;
	this->m_valveModel.setInitialOffset(this->registrationOffset);
	// make sure the initialized valve is correct based on which wall we followed
	this->m_valveModel.setWallFollowingState(this->wall_followed);
	if (this->wall_followed == IncrementalValveModel::WALL_FOLLOWED::USER)
		this->m_valveModel.setFollowedClockPosition(this->clockFollowed);

	this->detectLine(img);

	this->postProcessLine(img);

	this->updateModel();

	this->updateRegistration(img);

}

void Camera_processing::initializeApex()
{
	mutex_robotshape.lock();
	SE3 tipFrame = m_SolutionFrames.back();
	mutex_robotshape.unlock();

	for (int i = 0; i < m_configuration.size(); ++i)
		apex_coordinates[i] = this->m_configuration[i];

	double apex_position[3] = {0};

	//for (int i = 0; i < 3; ++i)
	//	apex_position[i] = tipFrame.GetPosition()[i] + 20.0 * tipFrame.GetZ()[i];	// compensate for the robot's straight segment

	memcpy(apex_position, this->m_model_robot_position, 3 * sizeof(double));

	double normal[3] = {0, 0, 1};
	
	apexSource->SetRadius(5);						
	apexSource->SetCenter(apex_position);				
	apexSource->SetNormal(normal);

	m_apex_initialized = true;
}

void Camera_processing::computeApexToValveParametersDebug(const ::cv::Mat& img)
{
	m_centroid_apex_to_valve[0] = 125;
	m_centroid_apex_to_valve[1] = 100;
}

void Camera_processing::computeApexToValveParameters(const ::cv::Mat& img)
{
	// check for valve
	::cv::Vec2f centroid2;
	::cv::Vec4f line2;

	if (m_linedetector.processImage(img, line2, centroid2, false, 10, LineDetector::MODE::TRANSITION) && m_use_original_line_transition)
	{
		this->detected_valve.push_back(true);
		//::std::cout <<"in 1" <<::std::endl;
	}
	else if (m_linedetector.processImage(img, line2, centroid2, false, 10, LineDetector::MODE::CIRCUM) && m_use_green_line_transition)
	{
		this->detected_valve.push_back(true);
				//::std::cout <<"in 2" <<::std::endl;
	}
	if (this->detected_valve.size() > 5 && this->m_use_automatic_transition)
	{
		::std::cout << "transition" << ::std::endl;
		this->m_state_transition = true;
	}

	int x = 0, y = 0;
	this->m_wall_detected = this->m_wall_detector.processImage(img, x, y, true);

	// adjust for the cropping
	::Eigen::Vector2d centroidEig;
	centroidEig(0) = x;
	centroidEig(1) = y;

	::Eigen::Vector2d image_center((int) img.rows/2, (int) img.rows/2);

	// apply rotation to compensate image initial rotation + robot's 3 tube rotation
	Mat frame_rotated2 = Mat(250,250,CV_8UC3);
	Point center = Point(img.cols/2, img.rows/2 );
    Mat rot_mat = getRotationMatrix2D(center,  rotation - robot_rotation * 180.0/3.141592, 1.0 );
	warpAffine(img, frame_rotated2, rot_mat, frame_rotated2.size() );

	::Eigen::Matrix3d rot1 = RotateZ(rotation * M_PI/180.0 - robot_rotation);
	centroidEig = rot1.block(0, 0, 2, 2).transpose()* (centroidEig - image_center) + image_center;

	// last transformation to align image frame with robot frame for convenience
	::Eigen::Vector2d displacement(0, img.rows);
	::Eigen::Matrix3d rot = RotateZ( -90 * M_PI/180.0);

	centroidEig = rot.block(0, 0, 2, 2).transpose() * centroidEig - rot.block(0, 0, 2, 2).transpose() * displacement;
	memcpy(m_centroid_apex_to_valve, centroidEig.data(), 2 * sizeof(double));

}


void	Camera_processing::checkTransitionState()
{
	if (m_use_green_line_transition && m_use_original_line_transition)
		::std::cout << " using both line detection methods for transition" << ::std::endl;
	else if (m_use_green_line_transition)
		::std::cout << " using green line detection for transition" <<::std::endl;
	else if (m_use_original_line_transition)
		::std::cout << " using original line detection for transition" << ::std::endl;
}


void Camera_processing::plotCommandedVelocitiesCircum(const ::cv::Mat& img)
{
	::Eigen::Vector2d orig_vel = ::Eigen::Map<::Eigen::Vector2d> (m_commanded_vel, 2);
	//::std::cout << "vels: " << this->m_commanded_vel[0] << " " << this->m_commanded_vel[1] << ::std::endl;
	//::std::cout << " centroid" << this->m_centroid[0] << " " << this->m_centroid[1] << ::std::endl;
	centroidEigPlot = ::Eigen::Map<::Eigen::Vector2d> (this->m_centroid, 2) - this->image_center;
	centroidEigPlot.normalize();
	
	double lambda_centering = (centroidEigPlot.transpose() * orig_vel);
	double plotting_scale = 50;

	centering_vel = plotting_scale * lambda_centering * centroidEigPlot;

	double lambda_tangent = (tangentEig.transpose() * orig_vel);
	tangent_vel = plotting_scale * lambda_tangent * tangentEig;

	// change velocities back to image frame
	rot_plot = RotateZ( -90 * M_PI/180.0);
	centering_vel = rot_plot.block(0, 0, 2, 2) * centering_vel;
	tangent_vel = rot_plot.block(0, 0, 2, 2) * tangent_vel;
	///::std::cout << tangent_vel.transpose() << " vels" << ::std::endl;
	::cv::arrowedLine(img, ::cv::Point(img.rows/2, img.cols/2), ::cv::Point(img.rows/2 + centering_vel[0], img.cols/2 + centering_vel[1]), ::cv::Scalar(48, 237, 255), 2);
	::cv::arrowedLine(img, ::cv::Point(img.rows/2, img.cols/2), ::cv::Point(img.rows/2 + tangent_vel[0], img.cols/2 +tangent_vel[1]), ::cv::Scalar(214, 226, 72), 2);
}


void Camera_processing::plotCommandedVelocities(const ::cv::Mat& img)
{
	::Eigen::Vector2d orig_vel = ::Eigen::Map<::Eigen::Vector2d> (m_commanded_vel, 2);

	// change velocities back to image frame
	::Eigen::Matrix3d rot = RotateZ( -90 * M_PI/180.0);
	orig_vel = rot.block(0, 0, 2, 2) * orig_vel;

	double kappa = 20.0;
	::cv::arrowedLine(img, ::cv::Point(img.rows/2, img.cols/2), ::cv::Point(img.rows/2 + kappa *  orig_vel(0), img.cols/2 + kappa * orig_vel(1)), ::cv::Scalar(0, 255, 255), 2);
	
}


bool Camera_processing::detectLeaks(const ::cv::Mat& img, int& x, int& y)
{
	::std::vector<::cv::Point> potentialLeaks;
	this->m_leakdetector.processImage(img, potentialLeaks);

	// rotate leaks to the world frame
	for (int i = 0; i < potentialLeaks.size(); ++i)
		this->imageToWorldFrame(potentialLeaks[i]);

	if (potentialLeaks.size() <= 0)
		return false;

	// get the tangent velocity and circumnavigation direction
	::Eigen::Vector2d tangent_vel;
	this->getTangentVelocity(tangent_vel);

	::cv::Point centroid;
	this->m_linedetector.getCentroid(img, centroid);
	this->imageToWorldFrame(centroid);


	::cv::Point tangent_vel_cv(tangent_vel(0),tangent_vel(1));

	// reject leaks in the inside of the valve
	::std::vector<::cv::Point> leaks;
	double cross_product;
	::cv::Point tmp;
	for (int i = 0; i < potentialLeaks.size(); ++i)
	{
		tmp = potentialLeaks[i];
		tmp.x = tmp.x - centroid.x;
		tmp.y = tmp.y - centroid.y;
		cross_product = tangent_vel_cv.cross(tmp);
		if (cross_product < 0 && this->circStatus == CW)
			continue;
		else if (cross_product > 0 && this->circStatus == CCW);
			continue;

		leaks.push_back(potentialLeaks[i]);
	}

	if (leaks.size() <= 0)
		return false;

	postProcessLeaks(leaks, x, y);

	return true;
}

void Camera_processing::getTangentVelocity(::Eigen::Vector2d& vel)
{
	// compute the two orthogonal velocity components

	::Eigen::Vector2d im_center(125, 125);
	::Eigen::Vector2d orig_vel = ::Eigen::Map<::Eigen::Vector2d> (m_commanded_vel, 2);
	::Eigen::Vector2d centroidEig = ::Eigen::Map<::Eigen::Vector2d> (m_centroid, 2);
	::Eigen::Vector2d tangentEig = ::Eigen::Map<::Eigen::Vector2d> (m_tangent, 2);

	centroidEig = centroidEig - im_center;
	centroidEig.normalize();
	double lambda_centering = (centroidEig.transpose() * orig_vel);
	double plotting_scale = 50;
	::Eigen::Vector2d centering_vel = plotting_scale * lambda_centering * centroidEig;

	double lambda_tangent = (tangentEig.transpose() * orig_vel);
	vel =  lambda_tangent * tangentEig;

}

void Camera_processing::imageToWorldFrame(::cv::Point& point)
{
	::Eigen::Vector2d image_center(125, 125); // not general -> fix!
	::Eigen::Matrix3d rot1 = RotateZ(rotation * M_PI/180.0 - robot_rotation);
	::Eigen::Vector2d pointEig(point.x, point.y);
	pointEig = rot1.block(0, 0, 2, 2).transpose()* (pointEig - image_center) + image_center;

	::Eigen::Vector2d displacement(0, 250);   // not general -> fix!
	::Eigen::Matrix3d rot = RotateZ( -90 * M_PI/180.0);

	pointEig = rot.block(0, 0, 2, 2).transpose() * pointEig - rot.block(0, 0, 2, 2).transpose() * displacement;

	point.x = pointEig(0);
	point.y = pointEig(1);

}

void Camera_processing::postProcessLeaks(::std::vector<::cv::Point>& leaks, int& x, int& y)
{
	if (leaks.size() == 1)
	{
		x = leaks.front().x;
		y = leaks.front().y;

		return;
	}

	::Eigen::Vector2d leak;
	double distance = 0;
	double distance_min = 10000000;
	int ind = 0;
	// which leak should we keep?
	for (int i = 0; i < leaks.size(); ++i)
	{
		leak(0) = leaks[i].x;
		leak(1) = leaks[i].y;

		distancePointToLine(leak, ::Eigen::Map<::Eigen::Vector2d> (m_centroid, 2), ::Eigen::Map<::Eigen::Vector2d> (m_tangent, 2), distance);
		if (distance < distance_min)
		{
			ind = i;
			distance_min = distance;
		}
	}

	x = leaks[ind].x;
	y = leaks[ind].y;

}

void
Camera_processing::computePointOnValve(::Eigen::Vector3d& robot_position, ::Eigen::Vector3d& centroidOnValve, const ::Eigen::Vector2d& channelCenter, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal)
{
	DP = centroidOnValve.segment(0, 2) - channelCenter;   // in pixels
	DP /= 26.67;

	rotation_valve = RotateZ(imageInitRotation * M_PI/180.0 - innerTubeRotation);
	DP = rotation_valve.block(0, 0, 2, 2).transpose()* DP;

	rotation_valve = RotateZ( -90 * M_PI/180.0);
	DP = rotation_valve.block(0, 0, 2, 2).transpose()* DP; // in world frame in mm

	tmp.segment(0, 2) = DP;
	tmp(2) = 0;
	tmp = tmp - tmp.dot(normal) * normal;

	centroidOnValve = robot_position + tmp;
}

void Camera_processing::initializeLeaks()
{
	leakSource1 = vtkSmartPointer<vtkSphereSource>::New();
	leakSource1->SetRadius(0);						
	double tmp[3] = {0, 0, 0};
	leakSource1->SetCenter(tmp);				


	mapperleak1 = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapperleak1->SetInputConnection(leakSource1->GetOutputPort());

	actorleak1 =	vtkSmartPointer<vtkActor>::New();
	actorleak1->SetMapper(mapperleak1);
	actorleak1->GetProperty()->SetColor(255, 1, 1);
	actorleak1->GetProperty()->SetOpacity(0.3);

	//renDisplay3D->AddActor(actorleak1);

	leakSource2 = vtkSmartPointer<vtkSphereSource>::New();
	leakSource2->SetRadius(0);						
	leakSource2->SetCenter(tmp);				

	mapperleak2 = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapperleak2->SetInputConnection(leakSource2->GetOutputPort());

	actorleak2 =	vtkSmartPointer<vtkActor>::New();
	actorleak2->SetMapper(mapperleak2);
	actorleak2->GetProperty()->SetColor(0, 255, 1);
	actorleak2->GetProperty()->SetOpacity(0.3);

	//renDisplay3D->AddActor(actorleak2);

	leakSource3 = vtkSmartPointer<vtkSphereSource>::New();
	leakSource3->SetRadius(0);						
	leakSource3->SetCenter(tmp);				

	mapperleak3 = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapperleak3->SetInputConnection(leakSource3->GetOutputPort());

	actorleak3 =	vtkSmartPointer<vtkActor>::New();
	actorleak3->SetMapper(mapperleak3);
	actorleak3->GetProperty()->SetColor(0, 1, 255);
	actorleak3->GetProperty()->SetOpacity(0.3);

	//renDisplay3D->AddActor(actorleak3);


}

void
Camera_processing::initializeRobotAxis()
{
	this->robotAxisSource = vtkSmartPointer<vtkLineSource>::New();
	robotAxisSource->SetPoint1(0.0, 0.0, 0.0);
	robotAxisSource->SetPoint2(0.0, 0.0, 200.0);

	// Create a mapper and actor
	this->robotAxisMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	this->robotAxisMapper->SetInputConnection(this->robotAxisSource->GetOutputPort());
	
	this->RobotAxisActor = vtkSmartPointer<vtkActor>::New();
	this->RobotAxisActor->SetMapper(this->robotAxisMapper);
	
	this->RobotAxisActor->GetProperty()->SetColor(255, 255, 255);
	this->RobotAxisActor->GetProperty()->SetLineStipplePattern(0xf0f0);
	this->RobotAxisActor->GetProperty()->SetLineWidth(1.5);

	renDisplay3D->AddActor(this->RobotAxisActor);

}

void
Camera_processing::cameraToImage(::cv::Point& point)
{
	::Eigen::Vector2d image_center(125, 125); // not general -> fix!
	::Eigen::Matrix3d rot1 = RotateZ(this->rotation * M_PI/180.0 - robot_rotation);
	::Eigen::Vector2d pointEig(point.x, point.y);
	pointEig = rot1.block(0, 0, 2, 2).transpose()* (pointEig - image_center) + image_center;

	point.x = pointEig(0);
	point.y = pointEig(1);

}


void 
Camera_processing::computeClockfacePosition()
{
	if (!this->m_linedetected)
		return;

	double angle = atan2(this->m_tangent[1], this->m_tangent[0]);

	(angle < 0 ? angle += 2 * M_PI : angle);

	angle *= 180.0/M_PI;		

	if (angle >= 180)
		angle -= 180;


	double clockAngle1 = 0, clockAngle2 = 0;
	clockAngle1 = 270.0 + angle;
	clockAngle2 = 90 + angle;

	::Eigen::Vector3d point;
	double clockfacePosition = -1.0;

	if (this->m_valveModel.isInitialized())
		this->m_valveModel.getClockfacePosition(this->m_model_robot_position[0], this->m_model_robot_position[1], this->m_model_robot_position[2], clockfacePosition, point);
	else
		clockfacePosition = this->getInitialClockPosition();
	
	double offset = 0.0;
	//if (this->m_valveModel.isRegistered())
		offset = this->m_valveModel.getRegistrationOffset();

	clockAngle1 -= offset;
	clockAngle2 -= offset;

	double c1 = clockAngle1 / 30.0;
	double c2 = clockAngle2 / 30.0;
	
	if (c1 > 12) c1 -= 12;
	if (c2 > 12) c2 -= 12;

	double d1;
	double d2;

	if (this->counterLine == 0)
	{
		d1 = this->computeClockDistance(clockfacePosition, c1);
		d2 = this->computeClockDistance(clockfacePosition, c2);
	}
	else
	{
		d1 = this->computeClockDistance(this->realClockPosition, c1);
		d2 = this->computeClockDistance(this->realClockPosition, c2);
	}


	this->counterLine++;

	if (d1 < d2) 
		this->realClockPosition = c1;
	else 
		this->realClockPosition = c2;
}


double
Camera_processing::computeClockDistance(double c1, double c2)
{

	double distance = 0;
	double d1 = 0, d2 = 0, d3 = 0;

	d1 = ::std::abs(c1 - c2);
	d2 = ::std::abs(12 + (c2 - c1));
	d3 = ::std::abs(12 + (c1 - c2));

	distance = ::std::min(d1, d2);
	distance = ::std::min(distance, d3);

	(distance > 12 ? distance -= 12: distance);

	return  distance;

}


double 
Camera_processing::getInitialClockPosition()
{
	switch (this->wall_followed)
	{
		case IncrementalValveModel::WALL_FOLLOWED::LEFT:
			return 9.0;
			break;
		case IncrementalValveModel::WALL_FOLLOWED::TOP:
			return 12;
			break;
		case IncrementalValveModel::WALL_FOLLOWED::BOTTOM:
			return 6.0;
			break;
		default:
			return this->wall_followed;			
			break;
	}
}


void
Camera_processing::updateRegistration(const ::cv::Mat& img)
{
	this->m_valveModel.getNormal(this->normal);
	
	double regError = 0;
	if (this->manualRegistration && !this->manualRegistered)
	{
		this->m_valveModel.setRegistrationRotation(this->registrationOffset);

		this->manualRegistered = true;
	}
	else if(!this->manualRegistration)
	{
		bool regFlag = this->m_registrationHandler.processImage(img, this->centroid_unrotated , this->inner_tube_rotation, (double) this->rotation, this->normal, this->realClockPosition, regError);
		if (regFlag)
		{
			::std::cout << "in registration" << ::std::endl;

			double marker = this->m_registrationHandler.getRecentMarker();

			this->m_clock.setRegistrationOffset(regError/30.0, marker);

			this->m_valveModel.setRegistrationRotation(regError);				
		}

	}

	this->reg_detected = m_registrationHandler.getRegDetected();
	if (this->reg_detected)
	{
		this->m_registrationHandler.getCentroid(regPointCV);
		this->cameraToImage(regPointCV);
	}

}

void 
Camera_processing::updateModel()
{
	if (m_contact_filtered == 1 && m_linedetected)
	{
		centroidOnValve.segment(0, 2) = this->centroid_unrotated;

		computePointOnValve(robot_positionEig, centroidOnValve, this->m_channel_center, this->inner_tube_rotation, this->rotation, this->normal);

		centroidOnValve(2) = this->m_model_robot_position[2];

		this->m_valveModel.updateModel(centroidOnValve(0), centroidOnValve(1), centroidOnValve(2), this->realClockPosition);
	}

}

void 
Camera_processing::postProcessLine(const ::cv::Mat& img)
{
	if (m_linedetected)
	{
		// bring to polar coordinated to perform filtering and then move back to point + tangent representation
		double r, theta;	
		nearestPointToLine(this->image_center, this->centroidEig, this->tangentEig, this->closest_point);
		cartesian2DPointToPolar(this->closest_point.segment(0, 2) - this->image_center, r, theta);

		// filter
		r = m_radius_filter.step(r);
		//theta = m_theta_filter.step(theta);			

		//bring back to centroid-tangent
		this->centroidEig(0) = r * cos(theta);
		this->centroidEig(1) = r * sin(theta);
		computePerpendicularVector(this->centroidEig, this->tangentEig);

		this->centroidEig += this->image_center;

		// find closest point from center to line -> we will bring that point to the center of the images - I think this is redundant
		double lambda = (image_center - centroidEig).transpose() * tangentEig;
		centroidEig += lambda * tangentEig;

		this->rot1 = RotateZ(this->rotation * M_PI/180.0 - this->robot_rotation);
		this->centroidEig = this->rot1.block(0, 0, 2, 2).transpose()* (this->centroidEig - this->image_center) + this->image_center;
		this->tangentEig = this->rot1.block(0, 0, 2, 2).transpose()* this->tangentEig;
		}

	// only for visualization -> needs to be in old frame
	Mat frame_rotated2 = Mat(250,250,CV_8UC3);
	Point center = Point(img.cols/2, img.rows/2 );
    Mat rot_mat = getRotationMatrix2D(center,  rotation - robot_rotation * 180.0/3.141592, 1.0 );
	warpAffine(img, frame_rotated2, rot_mat, frame_rotated2.size() );

	if (m_linedetected)
	{
		::cv::line( frame_rotated2, ::cv::Point(centroidEig(0), centroidEig(1)), ::cv::Point(centroidEig(0)+tangentEig(0)*100, centroidEig(1)+tangentEig(1)*100), ::cv::Scalar(0, 255, 0), 2, CV_AA);
		::cv::line( frame_rotated2, ::cv::Point(centroidEig(0), centroidEig(1)), ::cv::Point(centroidEig(0)+tangentEig(0)*(-100), centroidEig(1)+tangentEig(1)*(-100)), ::cv::Scalar(0, 255, 0), 2, CV_AA);
		::cv::circle(frame_rotated2, ::cv::Point(centroidEig[0], centroidEig[1]), 5, ::cv::Scalar(255,0,0));
	}
	::cv::imshow("line detection", frame_rotated2);
	::cv::waitKey(1);

	this->line_to_plot[0] = centroidEig(0);
	this->line_to_plot[1] = centroidEig(1);
	this->line_to_plot[2] = tangentEig(0);
	this->line_to_plot[3] = tangentEig(1);
	if (m_linedetected)
	{
	// last transformation to align image frame with robot frame for convenience
	this->rot2 = RotateZ( -90 * M_PI/180.0);
	this->centroidEig = this->rot2.block(0, 0, 2, 2).transpose() * this->centroidEig - this->rot2.block(0, 0, 2, 2).transpose() * this->displacement;
	this->tangentEig = this->rot2.block(0, 0, 2, 2).transpose() * this->tangentEig;

	memcpy(m_centroid, centroidEig.data(), 2 * sizeof(double));
	memcpy(m_tangent, tangentEig.data(), 2 * sizeof(double));
	}
}

void 
Camera_processing::detectLine(const ::cv::Mat& img)
{
	if (m_contact_filtered == 1)
	{
#ifdef __BENCHTOP__
		m_linedetected = m_modelBasedLine.stepBenchtop(m_model_robot_position, desired_vel, img, inner_tube_rotation, line, centroid);
#else
		m_linedetected = m_linedetector.processImage(img, line, centroid, false, 5, LineDetector::MODE::CIRCUM);
#endif
	}

	// if there is no line we don't update control parameters and model
	if (!m_linedetected)
		return;

	this->centroidEig(0) = this->centroid[0];
	this->centroidEig(1) = this->centroid[1];
	this->tangentEig[0] = this->line[0];
	this->tangentEig[1] = this->line[1];
	this->tangentEig.normalize();
	//::std::cout << " centroid in line detection" << this->centroid[0] << " " << this->centroid[1] << ::std::endl;

	this->centroid_unrotated(0) = this->centroidEig(0);
	this->centroid_unrotated(1) = this->centroidEig(1);
}