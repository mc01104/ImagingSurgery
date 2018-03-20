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
#include <vtkExtractVOI.h>

#include "vtkAutoInit.h" 
#include "vtkKeyboardInteractionStyle.h"

VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkRenderingFreeType); // VTK was built with vtkRenderingOpenGL2



#define VTK_CREATE(type, name) \
    vtkSmartPointer<type> name = vtkSmartPointer<type>::New()

VTK_CREATE(vtkRenderer, renDisplay3D);
VTK_CREATE(vtkRenderWindow, renderwindowDisplay3D);
VTK_CREATE(vtkRenderWindowInteractor, irenDisplay3D);
VTK_CREATE(KeyPressInteractorStyle, irenDisplay3DStyle);

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
	: dataFilename(dataFilename), pathToImages(pathToImages), r_filter(10), theta_filter(1, &angularDistanceMinusPItoPI),
	lineDetected(false), robot_rotation(0), imageInitRotation(+90), wallDetector(), wallDetected(false),
	filter(5), theta_filter_complex(4), new_version(true), contactCurr(0), contactPrev(0), centroidEig2(0, 0),
	m_clock(), reg_detected(false), clockPosition(-1.0), realClockPosition(-1), contact_ratio(0),
	lineDetector(), m_registrationHandler(&iModel)
{
	robot = CTRFactory::buildCTR("");
	kinematics = new MechanicsBasedKinematics(robot, 100);
	kinematics->ActivateIVPJacobian();

	int count = getImList(imList, checkPath(pathToImages + "/" ));
	std::sort(imList.begin(), imList.end(), numeric_string_compare);	

	//this->offset = 3240;
	this->offset = 0;

	//::std::vector<::std::string> dataStr = ReadLinesFromFile(this->getDataPath());
	//::std::vector<double> tmpData = DoubleVectorFromString(dataStr[this->offset], ',');
	//memcpy(this->actualPosition, &tmpData.data()[9], 3 * sizeof(double));

	for (int i = this->offset; i < count; ++i)
		imQueue.push_back(imList[i]);

	r_filter.resetFilter();
	theta_filter.resetFilter();
	theta_filter_complex.resetFilter();
	
	velocityCommand[0] = 0;
	velocityCommand[1] = 0;	

	filter.resetFilter();

	circleSource = vtkSmartPointer<vtkRegularPolygonSource>::New();
	mapperCircle = vtkSmartPointer<vtkPolyDataMapper>::New();
	actorCircle =	vtkSmartPointer<vtkActor>::New();
	
	pointOnCircleSource = vtkSmartPointer<vtkSphereSource>::New();
	pointOnCircleMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	pointOnCircleActor =  vtkSmartPointer<vtkActor>::New();

	//imgVTK = vtkSmartPointer<vtkImageData>::New();

	this->initializeLeaks();

	//for (int i = 0; i < 3; ++i)
	//	actualPosition[i] = 0.0;

	m_valve_tangent_prev[0] = -1;
	m_valve_tangent_prev[1] = 0;

	for (int i = 0; i < 2; ++i)
		m_velocity_prev[i] = 0.0;	

	counter = 0;
	double jointsTmp[5] = {0, 0, 35, 0, 0};
	this->setJoints(jointsTmp);

	this->initializeRobotAxis();

	pausedByUser = true;

	this->writer =  vtkSmartPointer<vtkPNGWriter>::New();

	readyToView = false;
	previous_velocity.setOnes();
}

ReplayEngine::~ReplayEngine()
{
	delete robot;
	delete kinematics;
	delete robotVis;
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
	::std::string filename = GetDateString() + "_new_reg_marker.avi";

	::cv::VideoWriter video(filename, ::cv::VideoWriter::fourcc('M','P','E','G'), 20, ::cv::Size(250, 250));

	ReplayEngine* tDataSim = reinterpret_cast<ReplayEngine*> (tData);

	::std::vector<::std::string> dataStr = ReadLinesFromFile(tDataSim->getDataPath());

	::std::vector<double> tmpData;

	::std::vector<::std::string>::const_iterator it = dataStr.begin() + tDataSim->offset;
	
	::std::vector<SE3> frames;

	::cv::Vec4f line;
	::cv::Vec2f centroid;

	bool solved = false;
	::cv::Mat tmpImage;

	float response = 0;
	char key;

	::std::string filename_base = "./images/";
	::std::string counterStr;

	for(it; it != dataStr.end(); ++it)
	{


		if (tDataSim->new_version)
			tmpData = DoubleVectorFromString(*it, ',');
		else 
			tmpData = DoubleVectorFromString(*it);

		tDataSim->robot_mutex.lock();

		if (tDataSim->new_version)
		{
			tDataSim->setJoints(&tmpData.data()[1]);
			solved = tDataSim->updateRobot(&tmpData.data()[1], frames);
			memcpy(tDataSim->actualPosition, &tmpData.data()[9], 3 * sizeof(double));
			tDataSim->contactCurr = tmpData[8];
			tDataSim->contact_ratio = tmpData[7];
			tDataSim->contact_filtered_med = tDataSim->filter.step(tmpData[8]);
		}
		else
		{
			tDataSim->setJoints(tmpData.data());
			solved = tDataSim->updateRobot(tmpData.data(), frames);
			memcpy(tDataSim->actualPosition, &tmpData.data()[8], 3 * sizeof(double));
			tDataSim->contact_ratio = tmpData[6];
		}
	
			tDataSim->setFrames(frames);
			tDataSim->robot_rotation = tDataSim->kinematics->GetInnerTubeRotation();

		if (tDataSim->new_version)
			tDataSim->updateRobotPositionModel(&tmpData.data()[9]);
		else
			tDataSim->updateRobotPositionModel(&tmpData.data()[8]);

		tDataSim->robot_mutex.unlock();

		tDataSim->img_mutex.lock();
		tDataSim->popNextImage();
		tDataSim->getCurrentImage(tmpImage);
		tDataSim->img_mutex.unlock();

		switch(tDataSim->status)
		{
			case LINE_DETECTION:
				tDataSim->detectLine(tmpImage);
				tDataSim->computeClockfacePosition();
				tDataSim->m_clock.update(tmpImage, tDataSim->realClockPosition);

				break;
			case WALL_DETECTION:
				tDataSim->detectWall(tmpImage);
				if(tDataSim->checkTransition())
					tDataSim->setStatus(LINE_DETECTION);
				break;
			case LEAK_DETECTION:
				tDataSim->detectLeak(tmpImage);
				break;
		}

		//::std::cout << tDataSim->counter << ::std::endl;
		//::std::cout << "clockface position:" << tDataSim->realClockPosition << ::std::endl;
		tDataSim->counter++;

		double clockfacePosition = -1;
		::Eigen::Vector3d point;
		//if (tDataSim->iModel.isInitialized())
		//{
		//	tDataSim->iModel.getClockfacePosition(tDataSim->actualPosition[0], tDataSim->actualPosition[1], tDataSim->actualPosition[2], clockfacePosition, point);
		//	tDataSim->computeClockfacePosition();
		//if (tDataSim->realClockPosition >= 0)
		//	tDataSim->m_clock.update(tmpImage, tDataSim->realClockPosition);
		////}

		double width = 50, height = 50;
		//if (tDataSim->contactCurr)
		//	::cv::putText(tmpImage, "contact", ::cv::Point(170, 180), ::cv::HersheyFonts::FONT_HERSHEY_PLAIN, 1, ::cv::Scalar(255, 255, 255), 2);

		//if (tDataSim->contact_filtered_med)
		//	::cv::putText(tmpImage, "filtered", ::cv::Point(170, 150), ::cv::HersheyFonts::FONT_HERSHEY_PLAIN, 1, ::cv::Scalar(255, 0, 255), 2);

		::cv::Rect rec = ::cv::Rect(tDataSim->regPointCV.x - 0.5 * width, tDataSim->regPointCV.y - 0.5 * height, width, height);
		if (tDataSim->reg_detected)
			::cv::rectangle(tmpImage, rec, ::cv::Scalar(0, 0, 255), 2);
			
		tDataSim->reg_detected = false;
		tDataSim->plotCommandedVelocities(tmpImage, tDataSim->centroid, tDataSim->tangent);
		::cv::putText(tmpImage, ::std::to_string(tDataSim->counter + tDataSim->offset) + "/" + ::std::to_string(dataStr.size()), ::cv::Point(170, 40), ::cv::HersheyFonts::FONT_HERSHEY_PLAIN, 1, ::cv::Scalar(255, 255, 255), 2);
		::cv::imshow("Display", tmpImage);

		if (tDataSim->pausedByUser)
			key = ::cv::waitKey();
		else
			key = ::cv::waitKey(1);

		tDataSim->processKeyboardInput(key);

		video.write(tmpImage);

	}

	::std::cout << "Exiting Simulation Thread" << ::std::endl;
	video.release();
}

void ReplayEngine::displayRobot(void* tData)
{

	ReplayEngine* tDataDisplayRobot = reinterpret_cast<ReplayEngine*> (tData);

	// initialize valve normal
	double start3[3] = {0, 0, 0}, end[3] = {0, 0, 1}, color[3] = {1, 0, 0};
	tDataDisplayRobot->valveNormal = new ArrowVisualizer(start3, end, color);

	tDataDisplayRobot->robotVis = new RobotVisualizer(*tDataDisplayRobot->kinematics);
	double configuration[5] = {0, 0, 35, 0, 0};
	tDataDisplayRobot->robotVis->update(configuration, tDataDisplayRobot->actualPosition);
	tDataDisplayRobot->robotVis->registerVisualizer(renDisplay3D);
	//::std::vector<vtkSmartPointer<vtkActor>> actors;
	//tDataDisplayRobot->robotVis->getActors(actors);

	//// populate Points with dummy data for initialization
	//unsigned int npts = 1;
	//vtkSmartPointer<vtkPolyLineSource> lineSource = vtkSmartPointer<vtkPolyLineSource>::New();
	//for (unsigned int i=0;i<npts;i++)
	//	lineSource->SetPoint(i, i,i,i);

	//vtkSmartPointer<vtkTubeFilter> tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
	//tubeFilter->SetInputConnection(lineSource->GetOutputPort());
	//tubeFilter->SetRadius(0.9);                  
	//tubeFilter->SetNumberOfSides(30);
	//tubeFilter->Update();

	//// Create a mapper and actor
	//vtkSmartPointer<vtkPolyDataMapper> tubeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	//tubeMapper->SetInputConnection(tubeFilter->GetOutputPort());
	//vtkSmartPointer<vtkActor> tubeActor = vtkSmartPointer<vtkActor>::New();
	//tubeActor->SetMapper(tubeMapper);
	//
	//renDisplay3D->AddActor(tubeActor);
	renDisplay3D->AddActor(tDataDisplayRobot->valveNormal->getActor());

	//for (int i = 0; i < 3; ++i)
	//	renDisplay3D->AddActor(actors[i]);

	auto start = std::chrono::high_resolution_clock::now();
	::Eigen::Vector3d tmp;

	double center[3] = {0};
	double radius;
	double normal[3] = {0};

	double p1[3] = {0, 0, 0};
	double p2[3] = {0, 0, 0};

	::std::vector<SE3> robotFrames;
	double actualPosition[3] = {0};
	::std::vector<double> s;
	double error[3] = {0};

	::std::vector<::Eigen::Vector3d> leaks;

	tDataDisplayRobot->screenCaptureFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
	tDataDisplayRobot->screenCaptureFilter->SetInput(renderwindowDisplay3D);
	tDataDisplayRobot->screenCaptureFilter->SetInputBufferTypeToRGBA();
	tDataDisplayRobot->screenCaptureFilter->ReadFrontBufferOff();
	//tDataDisplayRobot->screenCaptureFilter->Update();

	tDataDisplayRobot->writer->SetInputConnection(tDataDisplayRobot->screenCaptureFilter->GetOutputPort());
	tDataDisplayRobot->readyToView = true;
	int counter = 0;
	while(1)
	{
		memcpy(actualPosition, tDataDisplayRobot->actualPosition, 3  * sizeof(double));
		auto duration_s = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
		if (duration_s.count()>=50) 
		{
			try
			{
				start = std::chrono::high_resolution_clock::now();
				tDataDisplayRobot->robotVis->update(tDataDisplayRobot->joints, tDataDisplayRobot->actualPosition);

				tDataDisplayRobot->getFrames(robotFrames);
				//npts = robotFrames.size();

				//if (npts>2)
				//{
				//	for (int i = 0; i < 3; ++i)
				//		error[i] = actualPosition[i] - (robotFrames.back().GetPosition()[i] + 20*robotFrames[npts-1].GetZ()[i]); 
				//	s = linspace(0, 1, npts+1);
				//	lineSource->SetNumberOfPoints(npts + 1); //  to add the straight segment
				//	for (unsigned int i = 0; i < npts; i++)
				//		lineSource->SetPoint(i, robotFrames[i].GetPosition()[0] + s[i] * error[0],robotFrames[i].GetPosition()[1]  + s[i] * error[1], robotFrames[i].GetPosition()[2] + s[i] * error[2]);
				//	for (int i = 0; i < 3; ++i)
				//		tmp[i] = robotFrames[npts-1].GetPosition()[i] + 20*robotFrames[npts-1].GetZ()[i] + s.back() * error[i];  // remove hardcoded 20;
				//	lineSource->SetPoint(npts, tmp[0], tmp[1], tmp[2]);
				//}
				//else
				//{
				//	lineSource->SetNumberOfPoints(2); // set new data
				//	lineSource->SetPoint(0, 0.0,0.0,0.0);
				//	lineSource->SetPoint(1, 0.0,0.0,10.0);
				//}
				
				tDataDisplayRobot->iModel.getCenter(center);
				radius = tDataDisplayRobot->iModel.getRadius();
				tDataDisplayRobot->iModel.getNormal(normal);
				tDataDisplayRobot->circleSource->SetCenter(center);
				tDataDisplayRobot->circleSource->SetRadius(radius);
				tDataDisplayRobot->circleSource->SetNormal(normal);

				double endPoint[3] = {0};
				for (int i = 0; i < 3; ++i)
					endPoint[i] = center[i] - normal[i] * 10;
				tDataDisplayRobot->valveNormal->updateArrow(center, endPoint);

				tDataDisplayRobot->iModel.getNearestPointOnCircle(tDataDisplayRobot->actualPosition, center);
				tDataDisplayRobot->pointOnCircleSource->SetCenter(center);
				tDataDisplayRobot->pointOnCircleSource->SetRadius(2);

				//tDataDisplayRobot->screenCaptureFilter->Update();

				//tDataDisplayRobot->iModel.getLeakPosition(leaks);
				//tDataDisplayRobot->leakSource1->SetCenter(leaks[0].data());
				//tDataDisplayRobot->leakSource1->SetRadius(2);

				//tDataDisplayRobot->leakSource2->SetCenter(leaks[1].data());
				//tDataDisplayRobot->leakSource2->SetRadius(2);

				//tDataDisplayRobot->leakSource3->SetCenter(leaks[2].data());
				//tDataDisplayRobot->leakSource3->SetRadius(2);

				//bool line = tDataDisplayRobot->modelBasedLine.getTangent(p1, p2);

				//if (line)
				//{
				//	tDataDisplayRobot->lineSource->SetPoint1(p1);
				//	tDataDisplayRobot->lineSource->SetPoint2(p2);
				//}
						//get vtk image and write it on the disk
			::std::string counterStr = ::std::to_string(counter++);
			::std::string vtkFilename = "./images/vtk_" + counterStr + ".png";
			tDataDisplayRobot->writer->SetFileName(vtkFilename.c_str());


			

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
	//if (localEngine->readyToView)
	//	localEngine->writer->Write();
	renDisplay3D->ResetCamera();
	renDisplay3D->Render();

	vtkSmartPointer<CommandSubclass2> timerCallback = vtkSmartPointer<CommandSubclass2>::New();
	irenDisplay3D->AddObserver ( vtkCommand::TimerEvent, timerCallback );
	irenDisplay3D->CreateRepeatingTimer(20);

	localEngine->initializeOrigin();
	localEngine->initializeValveModel();

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
	double contact_ratio = 0;
    do {
		::std::ostringstream ss;
		
		tDataNet->robot_mutex.lock();
		memcpy(localVel, tDataNet->velocityCommand, 2 * sizeof(double));
		contact_ratio = tDataNet->contact_ratio;
		tDataNet->robot_mutex.unlock();
		
		//ss << "vel_x,vel_y,CR," << localVel[0]<< "," << localVel[1] <<","<< contact_ratio;

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
	actor2->GetProperty()->SetColor(1,0,0);
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
}

void ReplayEngine::popNextImage()
{
	::std::string path = checkPath(this->getpathToImages() + "/"  + this->imQueue.front()); 
	this->imQueue.pop_front();

	this->img = ::cv::imread(path);

}

void ReplayEngine::getCurrentImage(::cv::Mat& im)
{
	im = this->img;
}

void ReplayEngine::processDetectedLine(const ::cv::Vec4f& line, ::cv::Mat& img , ::cv::Vec2f& centroid, ::Eigen::Vector2d& centroidEig, ::Eigen::Vector2d& tangentEig, ::Eigen::Vector2d& tangentEigFiltered)
{
	double theta_previous = 0;
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
	//double angle = acos(tangentEig.transpose() * tangent_prev);
	//::std::cout << tangentEig.transpose() * tangent_prev << ::std::endl;
	// filter
	r = this->r_filter.step(r);
	
	//theta = this->theta_filter_complex.step(theta); 

	//::Eigen::Vector2d tmp2(cos(theta_previous), sin(theta_previous));
	//::Eigen::Vector2d tmp3(cos(theta), sin(theta));

	//double angle = acos(tmp2.transpose() * tmp3);

	//if (angle > 60 * M_PI/180.0)
	//	::std::cout << "counter:" << this->counter << "       " << angle * 180/M_PI << ::std::endl;
	//::std::cout <<   ((theta - theta_previous) * 180/M_PI)    << ::std::endl;

	//bring back to centroid-tangent
	centroidEig(0) = r * cos(theta);
	centroidEig(1) = r * sin(theta);

	computePerpendicularVector(centroidEig, tangentEigFiltered);
	centroidEig += image_center;

	// find closest point from center to line -> we will bring that point to the center of the images
	double lambda = (image_center - centroidEig).transpose() * tangentEigFiltered;
	centroidEig += lambda * tangentEigFiltered;

	tangent_prev = tangentEigFiltered;
	::Eigen::Matrix3d rot1 = RotateZ(this->imageInitRotation * M_PI/180.0 - this->robot_rotation);
	centroidEig = rot1.block(0, 0, 2, 2).transpose()* (centroidEig - image_center) + image_center;
	tangentEig = rot1.block(0, 0, 2, 2).transpose()* tangentEigFiltered;

	tangentEigFiltered = rot1.block(0, 0, 2, 2).transpose()* tangentEigFiltered;
	centroid[0] = centroidEig[0];
	centroid[1] = centroidEig[1];
	//::std::cout << (centroidEig - image_center).transpose() * tangentEig << ::std::endl;;
	
}

void ReplayEngine::applyVisualServoingController(const ::Eigen::Vector2d& centroid, const ::Eigen::Vector2d& tangent, ::Eigen::Vector2d& commandedVelocity)
{
	if (!this->lineDetected)
	{
		commandedVelocity.setZero();
		return;
	}

	// required only in simulation!!!!
	// last transformation to align image frame with robot frame for convenience
	// --------------------------------------------//
	::Eigen::Vector2d displacement(0, img.rows);
	::Eigen::Matrix3d rot = RotateZ( -90 * M_PI/180.0);

	::Eigen::Vector2d centroidEig = rot.block(0, 0, 2, 2).transpose() * centroid - rot.block(0, 0, 2, 2).transpose() * displacement;
	::Eigen::Vector2d tangentEig = rot.block(0, 0, 2, 2).transpose() * tangent;
	// --------------------------------------------//

	this->centroid = centroidEig;
	this->tangent  = tangentEig;
	this->tangent.normalize();

	//this->checkTangentDirection(tangentEig);
	::Eigen::Vector2d error, imageCenter;
	imageCenter << 125, 125;

	double gain = 1;
	error = imageCenter - centroidEig;
	
	//if (error.norm() < 50)
	//	error.setZero();

	error /= 26.27;
	error *= -gain;
	//error.setZero();
	error += gain * tangentEig;
	::Eigen::Vector3d vel(0, 0, 0);
	vel.segment(0, 2) = error;
 	this->checkDirection(vel);

	commandedVelocity = vel.segment(0, 2);
	//double tmp = m_valve_tangent_prev.transpose() * tangentEig;
	//tmp /= (m_valve_tangent_prev.norm() * tangentEig.norm());

	//double angle = ::std::acos(tmp) * 180/M_PI;
	//m_valve_tangent_prev = tangentEig;
	memcpy(this->velocityCommand, commandedVelocity.data(), 2 * sizeof(double));

	m_velocity_prev(0) = commandedVelocity(0);
	m_velocity_prev(1) = commandedVelocity(1);
}

void ReplayEngine::detectLine(::cv::Mat& img)
{
		this->lineDetected = false;
		
		double position[3] = {0, 0, 0};
		
		// get inner tube rotation
		double innerTubeRotation = 0;
		this->robot_mutex.lock();
		this->getInnerTubeRotation(innerTubeRotation);
		this->robot_mutex.unlock();

		// get the valve normal
		::Eigen::Vector3d normal(0, 0, 1);
		double normal_[3], center_[3];
		iModel.getNormal(normal_);
		iModel.getCenter(center_);
		normal = ::Eigen::Map<::Eigen::Vector3d> (normal_, 3);

		// robot position based on the Fourier model
		double regError = 0;
		::Eigen::Vector3d robot_positionEig = ::Eigen::Map<::Eigen::Vector3d> (this->actualPosition, 3);

		::cv::Vec4f line;
		::Eigen::Vector2d centroidEig, centroidEig2, tangentEig, velCommand,  tangentEigFiltered;
		//if (this->contactCurr == 1)
		{

			::cv::Vec2f centroid, centroid2;

			this->lineDetected = this->lineDetector.processImage(img, line, centroid, true, 5, LineDetector::MODE::CIRCUM);

			if (this->lineDetected)
			{
				centroidEig2(0) = centroid[0];
				centroidEig2(1) = centroid[1];
			}

			if (this->lineDetected)
				this->processDetectedLine(line, img, centroid, centroidEig, tangentEig, tangentEigFiltered);

		}
		this->applyVisualServoingController(centroidEig, tangentEigFiltered, velCommand);

		::Eigen::Vector3d centroidOnValve(0, 0, 0);
		::Eigen::Vector2d channelCenter(72, 135);

		// store original centroid for adding points to the model
		this->iModel.setFollowedClockPosition(this->getInitialPositionOnValve());
		if (this->contactCurr == 1 && this->lineDetected)
		{
			this->computeClockfacePosition();
			::std::cout << "robot clockface position:" << this->realClockPosition << ::std::endl;
			centroidOnValve.segment(0, 2) = centroidEig2;
			computePointOnValve(centroidOnValve, channelCenter, innerTubeRotation, imageInitRotation, normal);
			centroidOnValve(2) = this->actualPosition[2];

			this->iModel.updateModel(centroidOnValve(0), centroidOnValve(1),centroidOnValve(2), this->realClockPosition);
		}

		
		::Eigen::Vector2d regCentroid;
		this->m_registrationHandler.setWorkingChannel(channelCenter);

		this->m_registrationHandler.setRegDetected(false);

		if (this->lineDetected)
		{
			if (this->m_registrationHandler.processImage(img, centroidEig2 , innerTubeRotation, this->imageInitRotation, normal, this->realClockPosition, regError))
			{
				::std::cout << "clock position: " << this->realClockPosition << ::std::endl;
				::std::cout << "in registration" << ::std::endl;

				double marker = this->m_registrationHandler.getRecentMarker();

				this->m_clock.setRegistrationOffset(regError/30.0, marker);

				this->iModel.setRegistrationRotation(regError);
			}

		}

		this->reg_detected = m_registrationHandler.getRegDetected();

		if (this->reg_detected)
		{
			this->m_registrationHandler.getCentroid(regCentroid);
			regPointCV.x = regCentroid(0);
			regPointCV.y = regCentroid(1);
			this->cameraToImage(regPointCV);

		}


		this->robot_mutex.lock();
		memcpy(this->velocityCommand, velCommand.data(), 2 * sizeof(double));
		this->robot_mutex.unlock();
	
		::cv::imshow("unrotated", img);

		::cv::Point center = ::cv::Point(img.cols/2, img.rows/2 );
		::cv::Mat rot_mat = getRotationMatrix2D(center,this->imageInitRotation - this->robot_rotation * 180.0/3.141592, 1.0 );
		warpAffine(img, img, rot_mat, img.size() );

		if (this->lineDetected)
		{
			::cv::line( img, ::cv::Point(centroidEig(0), centroidEig(1)), ::cv::Point(centroidEig(0)+tangentEig(0)*100, centroidEig(1)+tangentEig(1)*100), ::cv::Scalar(0, 255, 0), 2, CV_AA);
			::cv::line( img, ::cv::Point(centroidEig(0), centroidEig(1)), ::cv::Point(centroidEig(0)+tangentEig(0)*(-100), centroidEig(1)+tangentEig(1)*(-100)), ::cv::Scalar(0, 255, 0), 2, CV_AA);
			::cv::circle(img, ::cv::Point(centroidEig[0], centroidEig[1]), 5, ::cv::Scalar(255,0,0));

			::cv::line( img, ::cv::Point(centroidEig(0), centroidEig(1)), ::cv::Point(centroidEig(0)+tangentEig(0)*100, centroidEig(1)+tangentEig(1)*100), ::cv::Scalar(255, 255, 0), 2, CV_AA);
			::cv::line( img, ::cv::Point(centroidEig(0), centroidEig(1)), ::cv::Point(centroidEig(0)+tangentEig(0)*(-100), centroidEig(1)+tangentEig(1)*(-100)), ::cv::Scalar(255, 255, 0), 2, CV_AA);


		}

}

void ReplayEngine::detectWall(::cv::Mat& img)
{
	float response = 0;
	this->bof.predict(img, response);

	::cv::Vec4f line1;
	::cv::Vec2f centroid4;
	//if (this->m_dummyLine.processImage(img, line1, centroid4, false, 0, LineDetector::MODE::TRANSITION))
	//	::std::cout << "valve detected" << ::std::endl;


	::cv::Point center = ::cv::Point(img.cols/2, img.rows/2 );
	::cv::Mat rot_mat = getRotationMatrix2D(center, this->imageInitRotation - this->robot_rotation * 180.0/3.141592, 1.0 );
	warpAffine(img, img, rot_mat, img.size() );

	::Eigen::Vector3d velCommand;	
	velCommand.setZero();
	::cv::Vec4f line;
	::cv::Vec2f centroid;

	int x=0, y=0;
	this->wallDetected = this->wallDetector.processImage(img, x, y, true, center.x, center.y, 125);

	this->applyVisualServoingController(x, y,velCommand, LEFT);

	//::std::cout << "servoing commands:" << velCommand.transpose() << ::std::endl;
	this->robot_mutex.lock();
	memcpy(this->velocityCommand, velCommand.data(), 3 * sizeof(double));
	this->robot_mutex.unlock();

	this->contact.push_back(response);
	this->contact_filtered.push_back(filter.step(response));


}

void ReplayEngine::applyVisualServoingController(int x, int y, ::Eigen::Vector3d& commandedVelocity, ReplayEngine::WALL_TO_FOLLOW wall)
{
	//if (!this->wallDetected)
	//{
	//	commandedVelocity.setZero();
	//	return;
	//}

	switch (wall)
	{
		case LEFT:
			followLeft(x, y, commandedVelocity);
			break;
		case TOP:
			followTop(x, y, commandedVelocity);
			break;
		case BOTTOM:
			followBottom(x, y, commandedVelocity);
			break;

	}
	//::std::cout << "x:" << x << "   y:" << y << ::std::endl;
	//::std::cout << "commanded velocities:" << commandedVelocity.transpose() << ::std::endl;
}

void ReplayEngine::followLeft(int x, int y, ::Eigen::Vector3d& commandedVelocity)
{
	//if (!this->m_wall_detected)
	//	this->m_centroid_apex[1] = 0;

	// left bias
	commandedVelocity[1] = -1;

	// forward velocity
	commandedVelocity[2] = 1;    // mm/sec
	double centerGain = 2.0;
	double scalingFactor = 26.7;
	// check controller
	if (y >= 80)
		commandedVelocity[1] += centerGain/scalingFactor * (y - 80);
	else if (y <= 20)
		commandedVelocity[1] += centerGain/scalingFactor * (y - 20);

}

void ReplayEngine::followBottom(int x, int y, ::Eigen::Vector3d& commandedVelocity)
{
	//if (!this->m_wall_detected)
	//	this->m_centroid_apex[1] = 0;

	// bottom bias
	commandedVelocity[0] = -1;

	// forward velocity
	commandedVelocity[2] = 1;    // mm/sec
	double centerGain = 2.0;
	double scalingFactor = 26.7;
	// check controller
	if (x >= 80)
		commandedVelocity[0] += centerGain/scalingFactor * (x - 80);
	else if (x <= 20)
		commandedVelocity[0] += centerGain/scalingFactor * (x - 20);

}

void ReplayEngine::followTop(int x, int y, ::Eigen::Vector3d& commandedVelocity)
{
	//if (!this->m_wall_detected)
	//	this->m_centroid_apex[1] = 0;

	// bottom bias
	commandedVelocity[0] = 1;

	// forward velocity
	commandedVelocity[2] = 1;    // mm/sec
	double centerGain = 2.0;
	double scalingFactor = 26.7;
	// check controller
	if (x <= 250 - 80)
		commandedVelocity[0] += centerGain/scalingFactor * (x - 250 + 80);
	else if (x >= 250 - 20)
		commandedVelocity[0] += centerGain/scalingFactor * (x -250 + 20);
}

bool 
ReplayEngine::checkTransition()
{

	//if (this->contact_filtered.size() < 20)
	//	return false;

	//int contact_frames = ::std::count(this->contact_filtered.rbegin(), this->contact_filtered.rbegin() + 20, 1);

	//double pct =  ((double) contact_frames)/20.0;

	//if (pct > 0.3)
	//	return true;
	return false;
}


void ReplayEngine::getTipPosition(double position[3])
{
	::std::vector<SE3> frames;
	this->getFrames(frames);

	Vec3 tmpPosition;
	tmpPosition.SetValues(0, 0, 0);
	if (frames.size() > 0)
	{
		tmpPosition = frames.back().GetPosition();
		tmpPosition += 20 * frames.back().GetZ();
	}
	for (int i = 0; i < 3; ++i)
		position[i] = tmpPosition[i];
}


void ReplayEngine::getInnerTubeRotation(double& innerTubeRotation)
{
	innerTubeRotation = this->kinematics->GetInnerTubeRotation();
}

void ReplayEngine::initializeValveModel()
{
	circleSource = vtkSmartPointer<vtkRegularPolygonSource>::New();
	circleSource->SetNumberOfSides(20);
	circleSource->SetRadius(0);						
	double tmp[3] = {0, 0, 0};
	circleSource->SetCenter(tmp);				
	circleSource->SetNormal(tmp);

	mapperCircle = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapperCircle->SetInputConnection(circleSource->GetOutputPort());

	actorCircle =	vtkSmartPointer<vtkActor>::New();
	actorCircle->SetMapper(mapperCircle);
	actorCircle->GetProperty()->SetColor(0, 1, 1);
	actorCircle->GetProperty()->SetOpacity(0.3);

	renDisplay3D->AddActor(actorCircle);

	pointOnCircleSource->SetRadius(0);						
	pointOnCircleSource->SetCenter(tmp);				

	pointOnCircleMapper->SetInputConnection(pointOnCircleSource->GetOutputPort());

	pointOnCircleActor->SetMapper(pointOnCircleMapper);
	pointOnCircleActor->GetProperty()->SetColor(1, 1, 1);
	pointOnCircleActor->GetProperty()->SetOpacity(0.3);

	renDisplay3D->AddActor(pointOnCircleActor);

	  // Create two points, P0 and P1
	  double p0[3] = {1.0, 0.0, 0.0};
	  double p1[3] = {0.0, 1.0, 0.0};
 
	  lineSource = 	vtkSmartPointer<vtkLineSource>::New();
	  lineSource->SetPoint1(p0);
	  lineSource->SetPoint2(p1);
	  lineSource->Update();
 
	  // Visualize
	  lineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	  lineMapper->SetInputConnection(lineSource->GetOutputPort());
	  lineActor = vtkSmartPointer<vtkActor>::New();
	  lineActor->SetMapper(lineMapper);
	  lineActor->GetProperty()->SetLineWidth(4);

	  renDisplay3D->AddActor(lineActor);
}


void ReplayEngine::updateRobotPositionModel(double fourier[3])
{
	memcpy(this->actualPosition, fourier, 3 * sizeof(double));
}


void ReplayEngine::checkTangentDirection(::Eigen::Vector2d& tangentEig)
{
	m_valve_tangent_prev[0] = m_valve_tangent_prev[1] = 0;

	::Eigen::Vector3d point;
	double clockPosition = -1.0;

	::Eigen::Vector3d circDirection(0, 0, 1);

	this->iModel.getClockfacePosition(this->actualPosition[0], this->actualPosition[1], this->actualPosition[2], clockPosition, point);
	this->clockPosition = clockPosition;

	double actualAngle;
	
	if (clockPosition > 0)
	{
		//actualAngle = clockPosition * 30.0;
		actualAngle = this->realClockPosition * 30;
	}
	else
		actualAngle = getInitialPositionOnValve() * 30.0;

	// compute unit vectors
	::Eigen::Vector3d p1(cos(actualAngle * M_PI/180.0), sin(actualAngle * M_PI/180.0), 0);

	// compute direction
	::Eigen::Vector3d res = circDirection.cross(p1);

	m_valve_tangent_prev = res.segment(0, 2);

	double tmp = m_valve_tangent_prev[0] * tangentEig[0] + m_valve_tangent_prev[1] * tangentEig[1];

	if (tmp < 0)
	{
		tangentEig[0] *= -1;
		tangentEig[1] *= -1;
	}
}


void ReplayEngine::detectLeak(::cv::Mat& img)
{
	float response = 0;
	this->bof.predict(img, response);

	int x = 0, y = 0;
	::cv::Vec4f line;
	::cv::Vec2f centroid;
	if (this->lineDetector.processImage(img, line,centroid, false, 10, LineDetector::MODE::TRANSITION)) // this is often more reliably detecting the valve than the classifier
	{
		::std::cout << "contact" << ::std::endl;
		this->m_leakDetector.processImage(img, x, y);
	}
	else
		::std::cout << "free" << ::std::endl;

}


void ReplayEngine::plotCommandedVelocities(const ::cv::Mat& img, const ::Eigen::Vector2d& centroid, const ::Eigen::Vector2d& tangent)
{
	::Eigen::Vector2d im_center(125, 125);
	::Eigen::Vector2d orig_vel = ::Eigen::Map<::Eigen::Vector2d> (this->velocityCommand, 2);
	Eigen::Vector2d centroidEig = centroid - im_center;

	centroidEig.normalize();

	double lambda_centering = (centroidEig.transpose() * orig_vel);
	double plotting_scale = 50;

	::Eigen::Vector2d centering_vel = plotting_scale * lambda_centering * centroidEig;

	double lambda_tangent = (tangent.transpose() * orig_vel);
	

	
	::Eigen::Vector2d tangent_vel = plotting_scale * lambda_tangent * tangent;
	centering_vel = centering_vel - lambda_tangent * orig_vel;
	tangent_vel = tangent_vel - lambda_centering * orig_vel;
	// change velocities back to image frame
	::Eigen::Matrix3d rot = RotateZ( -90 * M_PI/180.0);
	centering_vel = rot.block(0, 0, 2, 2) * centering_vel;
	tangent_vel = rot.block(0, 0, 2, 2) * tangent_vel;

	double tmp = tangent_vel.transpose() * previous_velocity;
	if (tmp < 0)
		::std::cout << "flipped" << ::std::endl;

	previous_velocity = tangent_vel;

	::cv::arrowedLine(img, ::cv::Point(img.rows/2, img.cols/2), ::cv::Point(img.rows/2 + centering_vel[0], img.cols/2 + centering_vel[1]), ::cv::Scalar(48, 237, 255), 2);
	::cv::arrowedLine(img, ::cv::Point(img.rows/2, img.cols/2), ::cv::Point(img.rows/2 + tangent_vel[0], img.cols/2 +tangent_vel[1]), ::cv::Scalar(214, 226, 72), 2);

}


void ReplayEngine::computePointOnValve(::Eigen::Vector3d& centroidOnValve, const ::Eigen::Vector2d& channelCenter, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal)
{
	::Eigen::Vector2d DP = centroidOnValve.segment(0, 2) - channelCenter;   // in pixels
	DP /= 26.67;

	::Eigen::Matrix3d rotation = RotateZ(imageInitRotation * M_PI/180.0 - innerTubeRotation);
	DP = rotation.block(0, 0, 2, 2).transpose()* DP;

	rotation = RotateZ( -90 * M_PI/180.0);
	DP = rotation.block(0, 0, 2, 2).transpose()* DP; // in world frame in mm

	::Eigen::Vector3d tmp;
	tmp.segment(0, 2) = DP;
	tmp(2) = 0;
	tmp = tmp - tmp.dot(normal) * normal;

	centroidOnValve += tmp;
	//centroidOnValve(0) += tmp(0);
	//centroidOnValve(1) += tmp(1);
}


void ReplayEngine::initializeLeaks()
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

	renDisplay3D->AddActor(actorleak1);


	leakSource2 = vtkSmartPointer<vtkSphereSource>::New();
	leakSource2->SetRadius(0);						
	leakSource2->SetCenter(tmp);				

	mapperleak2 = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapperleak2->SetInputConnection(leakSource2->GetOutputPort());

	actorleak2 =	vtkSmartPointer<vtkActor>::New();
	actorleak2->SetMapper(mapperleak2);
	actorleak2->GetProperty()->SetColor(0, 255, 1);
	actorleak2->GetProperty()->SetOpacity(0.3);

	renDisplay3D->AddActor(actorleak2);

	leakSource3 = vtkSmartPointer<vtkSphereSource>::New();
	leakSource3->SetRadius(0);						
	leakSource3->SetCenter(tmp);				

	mapperleak3 = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapperleak3->SetInputConnection(leakSource3->GetOutputPort());

	actorleak3 =	vtkSmartPointer<vtkActor>::New();
	actorleak3->SetMapper(mapperleak3);
	actorleak3->GetProperty()->SetColor(0, 1, 255);
	actorleak3->GetProperty()->SetOpacity(0.3);

	renDisplay3D->AddActor(actorleak3);


}

void ReplayEngine::initializeRobotAxis()
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

void ReplayEngine::imageToWorldFrame(::cv::Point& point)
{
	::Eigen::Vector2d image_center(125, 125); // not general -> fix!
	::Eigen::Matrix3d rot1 = RotateZ(this->imageInitRotation * M_PI/180.0 - robot_rotation);
	::Eigen::Vector2d pointEig(point.x, point.y);
	pointEig = rot1.block(0, 0, 2, 2).transpose()* (pointEig - image_center) + image_center;

	::Eigen::Vector2d displacement(0, 250);   // not general -> fix!
	::Eigen::Matrix3d rot = RotateZ( -90 * M_PI/180.0);

	pointEig = rot.block(0, 0, 2, 2).transpose() * pointEig - rot.block(0, 0, 2, 2).transpose() * displacement;

	point.x = pointEig(0);
	point.y = pointEig(1);

}


void
ReplayEngine::cameraToImage(::cv::Point& point)
{
	::Eigen::Vector2d image_center(125, 125); // not general -> fix!
	::Eigen::Matrix3d rot1 = RotateZ(this->imageInitRotation * M_PI/180.0 - robot_rotation);
	::Eigen::Vector2d pointEig(point.x, point.y);
	pointEig = rot1.block(0, 0, 2, 2).transpose()* (pointEig - image_center) + image_center;

	point.x = pointEig(0);
	point.y = pointEig(1);

}

void 
ReplayEngine::processKeyboardInput(char key)
{
	switch(key)			// we could get away with an 'if' statement but this looks cleaner if more key-events are added
	{
		case 'p':
			this->pausedByUser = !this->pausedByUser;
			break;
	}
}

int
ReplayEngine::getInitialPositionOnValve()
{
	return 7;
}


void 
ReplayEngine::computeClockfacePosition()
{

	if (!this->lineDetected)
		return;

	//::std::cout << this->tangent.transpose() << ::std::endl;

	double angle = atan2(this->tangent[1], this->tangent[0]);

	(angle < 0 ? angle += 2 * M_PI : angle);

	angle *= 180.0/M_PI;		

	if (angle >= 180)
		angle -= 2 * 180;


	double clockAngle1 = 0, clockAngle2 = 0;
	clockAngle1 = 270.0 + angle;
	clockAngle2 = 90 + angle;

	double offset = 0.0;
	if (this->iModel.isRegistered())
		offset = this->iModel.getRegistrationOffset();

	//::std::cout << "total offset:" << offset <<  ::std::endl;
	clockAngle1 -= offset;
	clockAngle2 -= offset;


	double c1 = clockAngle1 / 30.0;
	double c2 = clockAngle2 / 30.0;

	if (c1 > 12) c1 -= 12;
	if (c2 > 12) c2 -= 12;

	if (c1 < 0) c1 += 12;
	if (c2 < 0) c2 += 12;

	static int counterLine = 0;

	double d1;
	double d2;

	if (counterLine == 0)
	{
		d1 = this->computeClockDistance(this->getInitialPositionOnValve(), c1);
		d2 = this->computeClockDistance(this->getInitialPositionOnValve(), c2);
	}
	else
	{
		d1 = this->computeClockDistance(this->realClockPosition, c1);
		d2 = this->computeClockDistance(this->realClockPosition, c2);
	}


	counterLine++;



	if (d1 < d2) 
		this->realClockPosition = c1;
	else 
		this->realClockPosition = c2;
}


double
ReplayEngine::computeClockDistance(double c1, double c2)
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


void ReplayEngine::checkDirection(::Eigen::Matrix<double, 3, 1>& err)
{
	//CW : switch to -1 for CCW
 	::Eigen::Vector3d circDirection(0, 0, -1);

	//if (this->dStatus == CIRCUM_DIRECTION::CCW)
	//	circDirection *= -1.0;

	// commanded velocity
	::Eigen::Vector3d commandedVel = err.block(0, 0, 3, 1);

	// tip position
	::Eigen::Vector3d tipPosition = ::Eigen::Map<::Eigen::Vector3d> (this->actualPosition, 3);

	// axis to tip position
	double lambda = tipPosition.transpose() * ::Eigen::Vector3d(0, 0, 1);
	::Eigen::Vector3d axisToTipPositionVector = tipPosition - lambda * ::Eigen::Vector3d(0, 0, 1);
	axisToTipPositionVector.normalize();

	::Eigen::Vector3d tangent_vec;
	// compute commanded direction of rotation
	tangent_vec[0] = this->tangent[0];
	tangent_vec[1] = this->tangent[1];
	tangent_vec[2] = 0;

	double lambda_vel = commandedVel.transpose() * tangent_vec;
	::Eigen::Vector3d tangent_vel = lambda_vel * tangent_vec;
	::Eigen::Vector3d center_vel = commandedVel - tangent_vel;

	//::Eigen::Vector3d commandedDirection = axisToTipPositionVector.cross(tangent_vel);
	::Eigen::Vector3d commandedDirection = axisToTipPositionVector.cross(tangent_vel);
	commandedDirection.normalize();
	double tmp = circDirection.transpose() * commandedDirection;

	if (tmp < 0)
	{
		//::std::cout << "reverting vel->";
		tangent_vel *= -1;
	}

	//::std::cout << "cross-product [before]: " << commandedDirection(2) << ", ";

	::Eigen::Vector3d final_vel = center_vel + tangent_vel;

	commandedDirection = axisToTipPositionVector.cross(tangent_vel);
	commandedDirection.normalize();
	/*tmp = circDirection(2) * commandedDirection(2);*/
	//::std::cout << "cross-product [after]: " << commandedDirection(2) << ::std::endl;
	err.block(0, 0, 3, 1) = final_vel;
}
