#include "stdafx.h"
#define _WINSOCKAPI_ // define this before including windows.h for avoiding winsock function redefinitions causing errors
#include <Windows.h>
#include <thread>

#include "ReplayEngine.h"

#include "Utilities.h"
#include "FileUtils.h"
#include "HTransform.h"

#include <vtkTubeFilter.h>
#include <vtkSmartPointer.h>
#include <vtkPolyLineSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyle.h>
#include <vtkCommand.h>
#include <vtkCallbackCommand.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkCubeSource.h>

#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
#include <vtkAVIWriter.h>

#include "vtkAutoInit.h" 
VTK_MODULE_INIT(vtkRenderingOpenGL); // VTK was built with vtkRenderingOpenGL2
VTK_MODULE_INIT(vtkInteractionStyle);

#define VTK_CREATE(type, name) \
    vtkSmartPointer<type> name = vtkSmartPointer<type>::New()

VTK_CREATE(vtkRenderer, renDisplay3D);
VTK_CREATE(vtkRenderWindow, renderwindowDisplay3D);
VTK_CREATE(vtkRenderWindowInteractor, irenDisplay3D);
VTK_CREATE(vtkInteractorStyleTrackballCamera, irenDisplay3DStyle);

// Winsock includes for network
#include <winsock2.h>
#include <ws2tcpip.h>
#include "targetver.h"
#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "27015"
#define DEFAULT_PORT_PLOT "27016"

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
		catch (::std::exception& e)
		{
			::std::cout << e.what() << ::std::endl;
		}
    }
};



ReplayEngine::ReplayEngine(const ::std::string& dataFilename, const ::std::string& pathToImages)
	: dataFilename(dataFilename), pathToImages(pathToImages), r_filter(3), theta_filter(3, &angularDistanceMinusPItoPI),
	lineDetected(false), robot_rotation(0), imageInitRotation(-90), lineDetector(), wallDetector(), wallDetected(false),
	filter(5)
{
	robot = CTRFactory::buildCTR("");
	kinematics = new MechanicsBasedKinematics(robot, 100);
	kinematics->ActivateIVPJacobian();

	int count = getImList(imList, checkPath(pathToImages + "/" ));
	std::sort(imList.begin(), imList.end(), numeric_string_compare);	

	this->offset = 200;

	for (int i = 0 + this->offset; i < count; ++i)
		imQueue.push_back(imList[i]);

	r_filter.resetFilter();
	theta_filter.resetFilter();

	velocityCommand[0] = 0;
	velocityCommand[1] = 0;	

	filter.resetFilter();
	
}

ReplayEngine::~ReplayEngine()
{
	delete robot;
	delete kinematics;
}

void ReplayEngine::run()
{
	::std::thread simulation_thread(&ReplayEngine::simulate, this);
	::std::thread robot_display_thread(&ReplayEngine::displayRobot, this);
	::std::thread rendering_thread(&ReplayEngine::vtkRender, this);
	//::std::thread network_thread(&ReplayEngine::networkPlot, this);
	
	simulation_thread.join();
	robot_display_thread.join();
	rendering_thread.join();
	//network_thread.join();
	
}

void ReplayEngine::simulate(void* tData)
{
	//::cv::VideoWriter video("line_detection_3.avi", ::cv::VideoWriter::fourcc('M','P','E','G'), 20, ::cv::Size(250, 250));

	ReplayEngine* tDataSim = reinterpret_cast<ReplayEngine*> (tData);

	::std::vector<::std::string> dataStr = ReadLinesFromFile(tDataSim->getDataPath());

	::std::vector<double> tmpData;

	::std::vector<::std::string>::const_iterator it = dataStr.begin() + tDataSim->offset;
	int counter = 0;
	::std::vector<SE3> frames;

	::cv::Vec4f line;
	::cv::Vec2f centroid;

	bool solved = false;
	::cv::Mat tmpImage;

	float response = 0;

	for(it; it != dataStr.end(); ++it)
	{
		tmpData = DoubleVectorFromString(*it);

		tDataSim->robot_mutex.lock();

		tDataSim->setJoints(tmpData.data());
		solved = tDataSim->updateRobot(tmpData.data(), frames);
		tDataSim->setFrames(frames);
		tDataSim->robot_rotation = tDataSim->kinematics->GetInnerTubeRotation();

		tDataSim->robot_mutex.unlock();

		tDataSim->img_mutex.lock();
		tDataSim->popNextImage();
		tDataSim->getCurrentImage(tmpImage);
		tDataSim->img_mutex.unlock();

		switch(tDataSim->status)
		{
			case LINE_DETECTION:
				tDataSim->detectLine(tmpImage);
				break;
			case WALL_DETECTION:
				tDataSim->detectWall(tmpImage);
				if(tDataSim->checkTransition())
					tDataSim->setStatus(LINE_DETECTION);
				break;
		}
		::cv::imshow("Display", tmpImage);
		//video.write(tmpImage);
		::cv::waitKey(1);  

	}

	::std::cout << "Exiting Simulation Thread" << ::std::endl;
	//video.release();
}

void ReplayEngine::displayRobot(void* tData)
{

	ReplayEngine* tDataDisplayRobot = reinterpret_cast<ReplayEngine*> (tData);

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
	tubeActor->SetMapper(tubeMapper);

	renDisplay3D->AddActor(tubeActor);

	auto start = std::chrono::high_resolution_clock::now();
	::Eigen::Vector3d tmp;

	::std::vector<SE3> robotFrames;
	while(1)
	{
		auto duration_s = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
		if (duration_s.count()>=50) 
		{
			try
			{
				start = std::chrono::high_resolution_clock::now();

				tDataDisplayRobot->getFrames(robotFrames);
				npts = robotFrames.size();

				if (npts>2)
				{
					lineSource->SetNumberOfPoints(npts + 1); //  to add the straight segment
					for (unsigned int i = 0; i < npts; i++)
						lineSource->SetPoint(i, robotFrames[i].GetPosition()[0],robotFrames[i].GetPosition()[1], robotFrames[i].GetPosition()[2]);
					for (int i = 0; i < 3; ++i)
						tmp[i] = robotFrames[npts-1].GetPosition()[i] + 20*robotFrames[npts-1].GetZ()[i];  // remove hardcoded 20;
					lineSource->SetPoint(npts, tmp[0], tmp[1], tmp[2]);
				}
				else
				{
					lineSource->SetNumberOfPoints(2); // set new data
					lineSource->SetPoint(0, 0.0,0.0,0.0);
					lineSource->SetPoint(1, 0.0,0.0,10.0);
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

void ReplayEngine::vtkRender(void* tData)
{

	ReplayEngine* localEngine = reinterpret_cast<ReplayEngine*> (tData);

	renderwindowDisplay3D->AddRenderer(renDisplay3D);
	
	irenDisplay3D->SetInteractorStyle(irenDisplay3DStyle);
	irenDisplay3D->SetRenderWindow(renderwindowDisplay3D);
	irenDisplay3D->Initialize();

	renDisplay3D->ResetCamera();
	renDisplay3D->Render();

	vtkSmartPointer<CommandSubclass2> timerCallback = vtkSmartPointer<CommandSubclass2>::New();
	irenDisplay3D->AddObserver ( vtkCommand::TimerEvent, timerCallback );
	irenDisplay3D->CreateRepeatingTimer(100);

	localEngine->initializeOrigin();

	//vtkSmartPointer<vtkWindowToImageFilter> screenCaptureFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
	//screenCaptureFilter->SetInput(renderwindowDisplay3D);
	//screenCaptureFilter->SetInputBufferTypeToRGBA();
	//screenCaptureFilter->ReadFrontBufferOff();
	//screenCaptureFilter->Update();

	//vtkSmartPointer<vtkAVIWriter> writer = vtkSmartPointer<vtkAVIWriter>::New();
	//writer->SetInputConnection(screenCaptureFilter->GetOutputPort());
	//writer->SetFileName("test.avi");
	//writer->Write();

	irenDisplay3D->Start();

	::std::cout << "VTK Rendering Thread exited successfully" << ::std::endl;
}

void ReplayEngine::networkPlot(void* tData)
{
	ReplayEngine* tDataNet = reinterpret_cast<ReplayEngine*> (tData);
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
	double localVel[2] = {0};
    do {
		::std::ostringstream ss;
		
		tDataNet->robot_mutex.lock();
		memcpy(localVel, tDataNet->velocityCommand, 2 * sizeof(double));
		tDataNet->robot_mutex.unlock();

		ss << "vel_x,vel_y," << localVel[0]<< "," << localVel[1];

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

	tDataNet->networkPlot(tDataNet);
    return;

}

bool ReplayEngine::updateRobot(const double jointValues[], ::std::vector<SE3>& frames)
{
	memcpy(this->joints, jointValues, 5 * sizeof(double));
	
	double rotation[3];
	double translation[3];

	MechanicsBasedKinematics::RelativeToAbsolute(this->robot, this->joints, rotation, translation);

	bool solved = this->kinematics->ComputeKinematics(rotation, translation);
	
	if (!solved)
		return false;
	
	double smax = robot->GetLength();
	::std::vector<double> arcLength;
	int nPoints = 30;
	frames.clear();

	for (int i = 0; i < nPoints; ++i)
		arcLength.push_back((1.0 * i)/nPoints * smax);

	kinematics->GetBishopFrame(arcLength, frames);

	return true;
}

void ReplayEngine::setJoints(double joints[])
{
	memcpy(this->joints, joints, 5 * sizeof(double));
}

void ReplayEngine::initializeOrigin()
{
	//Axes at beginning of robot
	vtkSmartPointer<vtkCubeSource> cubeSource2 = vtkSmartPointer<vtkCubeSource>::New();
	cubeSource2->SetCenter(12.5,0.0,0.0);
	cubeSource2->SetXLength(25.0);
	cubeSource2->SetYLength(1.0);
	cubeSource2->SetZLength(1.0);

	vtkSmartPointer<vtkPolyDataMapper> mapper2 =  vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper2->SetInputConnection(cubeSource2->GetOutputPort());
 
	vtkSmartPointer<vtkActor> actor2 =  vtkSmartPointer<vtkActor>::New();
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
	//actor3->GetProperty()->SetColor(0.0,1.0,0.0);
	//actor3->GetProperty()->SetOpacity(0.65);
	actor3->SetMapper(mapper3);

	renDisplay3D->AddActor(actor3);
	/////////////////////////////////////////////////////////////////////////////////////
}

void ReplayEngine::popNextImage()
{
	::std::string path = checkPath(this->getpathToImages() + "/"  + this->imQueue.front()); 
	this->imQueue.pop_front();
	//::std::cout << path << ::std::endl;
	this->img = ::cv::imread(path);

}

void ReplayEngine::getCurrentImage(::cv::Mat& im)
{
	im = this->img;
}

void ReplayEngine::processDetectedLine(const ::cv::Vec4f& line, ::cv::Mat& img , ::cv::Vec2f& centroid, ::Eigen::Vector2d& centroidEig, ::Eigen::Vector2d& tangentEig)
{
	
	centroidEig(0) = centroid[0];
	centroidEig(1) = centroid[1];

	tangentEig[0] = line[0];
	tangentEig[1] = line[1];
	tangentEig.normalize();

	::Eigen::Vector2d image_center((int) img.rows/2, (int) img.rows/2);

	// bring to polar coordinated to perform filtering and then move back to point + tangent representation
	double r, theta;
	::Eigen::VectorXd closest_point;
	nearestPointToLine(image_center, centroidEig, tangentEig, closest_point);
	cartesian2DPointToPolar(closest_point.segment(0, 2) - image_center, r, theta);

	// filter
	r = this->r_filter.step(r);
	theta = this->theta_filter.step(theta);

	//bring back to centroid-tangent
	centroidEig(0) = r * cos(theta);
	centroidEig(1) = r * sin(theta);

	computePerpendicularVector(centroidEig, tangentEig);
	centroidEig += image_center;

	// find closest point from center to line -> we will bring that point to the center of the images
	double lambda = (image_center - centroidEig).transpose() * tangentEig;
	centroidEig += lambda * tangentEig;

	::Eigen::Matrix3d rot1 = RotateZ(this->imageInitRotation * M_PI/180.0 - this->robot_rotation);
	centroidEig = rot1.block(0, 0, 2, 2).transpose()* (centroidEig - image_center) + image_center;
	tangentEig = rot1.block(0, 0, 2, 2).transpose()* tangentEig;

}

void ReplayEngine::applyVisualServoingController(const ::Eigen::Vector2d& centroid, const ::Eigen::Vector2d& tangent, ::Eigen::Vector2d& commandedVelocity)
{
	if (!this->lineDetected)
	{
		commandedVelocity.setZero();
		return;
	}

	// later this needs to be computed from the plane normal
	::Eigen::Matrix3d rot = ::Eigen::Matrix3d::Identity();

	::Eigen::Vector2d error, imageCenter;
	imageCenter << 125, 125;

	double gain = 0.3;
	error = imageCenter - centroid;
	error /= 26.27;
	error *= -gain;

	error -= gain * tangent;

	commandedVelocity = error;

}

void ReplayEngine::detectLine(::cv::Mat& img)
{
		float response = 0;
		this->bof.predict(img, response);

		this->lineDetected = false;
		
		::Eigen::Vector2d centroidEig, tangentEig, velCommand;
		if (response == 1)
		{
			::cv::Vec4f line;
			::cv::Vec2f centroid;
			this->lineDetected = this->lineDetector.processImage(img, line, centroid);
		
			if (this->lineDetected)
				this->processDetectedLine(line, img, centroid, centroidEig, tangentEig);
		}

		this->applyVisualServoingController(centroidEig, tangentEig, velCommand);
		this->robot_mutex.lock();
		memcpy(this->velocityCommand, velCommand.data(), 2 * sizeof(double));
		this->robot_mutex.unlock();

		::cv::Point center = ::cv::Point(img.cols/2, img.rows/2 );
		::cv::Mat rot_mat = getRotationMatrix2D(center, this->imageInitRotation - this->robot_rotation * 180.0/3.141592, 1.0 );
		warpAffine(img, img, rot_mat, img.size() );

		if (this->lineDetected)
		{
			::cv::line( img, ::cv::Point(centroidEig(0), centroidEig(1)), ::cv::Point(centroidEig(0)+tangentEig(0)*100, centroidEig(1)+tangentEig(1)*100), ::cv::Scalar(0, 255, 0), 2, CV_AA);
			::cv::line( img, ::cv::Point(centroidEig(0), centroidEig(1)), ::cv::Point(centroidEig(0)+tangentEig(0)*(-100), centroidEig(1)+tangentEig(1)*(-100)), ::cv::Scalar(0, 255, 0), 2, CV_AA);
			::cv::circle(img, ::cv::Point(centroidEig[0], centroidEig[1]), 5, ::cv::Scalar(255,0,0));
		}

}

void ReplayEngine::detectWall(::cv::Mat& img)
{
	float response = 0;
	this->bof.predict(img, response);

	::cv::Point center = ::cv::Point(img.cols/2, img.rows/2 );
	::cv::Mat rot_mat = getRotationMatrix2D(center, this->imageInitRotation - this->robot_rotation * 180.0/3.141592, 1.0 );
	warpAffine(img, img, rot_mat, img.size() );

	::Eigen::Vector3d velCommand;	
	velCommand.setZero();
	::cv::Vec4f line;
	::cv::Vec2f centroid;

	int x, y;
	this->wallDetected = this->wallDetector.processImage(img, x, y, true, center.x, center.y, 125);

	this->applyVisualServoingController(x, y,velCommand);

	this->robot_mutex.lock();
	memcpy(this->velocityCommand, velCommand.data(), 3 * sizeof(double));
	this->robot_mutex.unlock();

	this->contact.push_back(response);
	this->contact_filtered.push_back(filter.step(response));

}

void ReplayEngine::applyVisualServoingController(int x, int y, ::Eigen::Vector3d& commandedVelocity)
{
	if (!this->wallDetected)
	{
		commandedVelocity.setZero();
		return;
	}

	double scaling_factor = 26;
	double Kp = 2.0/scaling_factor;
	::Eigen::Vector2d error, imageCenter;
	if (x >= 100)
		commandedVelocity[1] = Kp * (x - 100);
	else if (x <= 20)
		commandedVelocity[1] = -Kp * (x - 20);

	// forward velocity
	commandedVelocity[2] = 2.0;    // mm/sec

}


bool 
ReplayEngine::checkTransition()
{
	if (this->contact_filtered.size() < 20)
		return false;

	int contact_frames = ::std::count(this->contact_filtered.rbegin(), this->contact_filtered.rbegin() + 20, 1);

	double pct =  ((double) contact_frames)/20.0;

	if (pct > 0.3)
		return true;
}