#include "stdafx.h"
#include <Windows.h>
#include <thread>

#include "ReplayEngine.h"

#include "Utilities.h"
#include "FileUtils.h"

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

#include "vtkAutoInit.h" 
VTK_MODULE_INIT(vtkRenderingOpenGL); // VTK was built with vtkRenderingOpenGL2
VTK_MODULE_INIT(vtkInteractionStyle);

#define VTK_CREATE(type, name) \
    vtkSmartPointer<type> name = vtkSmartPointer<type>::New()

VTK_CREATE(vtkRenderer, renDisplay3D);
VTK_CREATE(vtkRenderWindow, renderwindowDisplay3D);
VTK_CREATE(vtkRenderWindowInteractor, irenDisplay3D);
VTK_CREATE(vtkInteractorStyleTrackballCamera, irenDisplay3DStyle);

//VTK_MODULE_INIT(vtkRenderingFreeType)

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
	: dataFilename(dataFilename), pathToImages(pathToImages)
{
	robot = CTRFactory::buildCTR("");
	kinematics = new MechanicsBasedKinematics(robot, 100);
	kinematics->ActivateIVPJacobian();

	int count = getImList(imList, checkPath(pathToImages + "/" ));
	std::sort(imList.begin(), imList.end(), numeric_string_compare);	

	for (int i = 0; i < count; ++i)
		imQueue.push_back(imList[i]);
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
	
	simulation_thread.join();
	robot_display_thread.join();
	rendering_thread.join();
	
}


void ReplayEngine::simulate(void* tData)
{
	ReplayEngine* tDataSim = reinterpret_cast<ReplayEngine*> (tData);

	::std::vector<::std::string> dataStr = ReadLinesFromFile(tDataSim->getDataPath());

	::std::vector<double> tmpData;

	::std::vector<::std::string>::const_iterator it = dataStr.begin();
	int counter = 0;
	::std::vector<SE3> frames;

	::cv::Vec4f line;
	::cv::Vec2f centroid;

	::cv::Mat tmpImage;
	for(it; it != dataStr.end(); ++it)
	{
		tmpData = DoubleVectorFromString(*it);

		tDataSim->robot_mutex.lock();

		tDataSim->setJoints(tmpData.data());
		tDataSim->updateRobot(tmpData.data(), frames);
		tDataSim->setFrames(frames);

		tDataSim->robot_mutex.unlock();

		tDataSim->img_mutex.lock();
		tDataSim->popNextImage();
		tDataSim->getCurrentImage(tmpImage);
		tDataSim->img_mutex.unlock();

		::cv::Vec4f line;
		::cv::Vec2f centroid;
		tDataSim->lineDetector.processImage(tmpImage, line, centroid);
		tDataSim->processDetectedLine(line, tmpImage, centroid);

		::cv::imshow("Display", tmpImage);
		::cv::waitKey(10);  

		::std::cout << counter++ << ::std::endl;
	}

	::std::cout << "Exiting Simulation Thread" << ::std::endl;

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

	irenDisplay3D->Start();
	::std::cout << "VTK Rendering Thread exited successfully" << ::std::endl;
}

void ReplayEngine::networkPlot(void* tData)
{
}

void ReplayEngine::updateRobot(const double jointValues[], ::std::vector<SE3>& frames)
{
	memcpy(this->joints, jointValues, 5 * sizeof(double));
	
	double rotation[3];
	double translation[3];

	MechanicsBasedKinematics::RelativeToAbsolute(this->robot, this->joints, rotation, translation);

	this->kinematics->ComputeKinematics(rotation, translation);

	double smax = robot->GetLength();
	::std::vector<double> arcLength;
	int nPoints = 30;
	frames.clear();

	for (int i = 0; i < nPoints; ++i)
		arcLength.push_back((1.0 * i)/nPoints * smax);

	kinematics->GetBishopFrame(arcLength, frames);


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

	this->img = ::cv::imread(path);

}

void ReplayEngine::getCurrentImage(::cv::Mat& im)
{
	im = this->img;
}

void ReplayEngine::processDetectedLine(const ::cv::Vec4f& line, ::cv::Mat& img , ::cv::Vec2f& centroid)
{
}