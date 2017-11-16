#include "stdafx.h"

#include <vtkRenderer.h>
#include "robotVisualizer.h"
#include "Utilities.h"


RobotVisualizer::RobotVisualizer(const MechanicsBasedKinematics& kinematics) : kinematics(kinematics)
{
	int numberOfTubes = this->kinematics.getRobot()->GetNumOfTubes();

	// not really needed when we finish the implementation
	::std::vector<::std::vector<double>> colors;
	::std::vector<double> color;
	color.push_back(94.0/255.0);
	color.push_back(116.0/255.0);
	color.push_back(150.0/255.0);

	colors.push_back(color);

	color[0] = 45.0/255.0; color[1] = 98.0/255.0; color[2] = 183.0/255.0;

	colors.push_back(color);
	colors.push_back(color);

	double radius = 2.0;
	for (int i = 0; i < numberOfTubes; ++i)
	{
		this->tubes.push_back(TubeVisualizer(colors[i].data(), radius));
		radius *= 0.8;
	}
}

RobotVisualizer::~RobotVisualizer()
{
}

void 
RobotVisualizer::update(const double* configuration)
{
	double rotation[3] = {0};
	double translation[3] = {0};

	MechanicsBasedKinematics::RelativeToAbsolute(this->kinematics.getRobot(), configuration, rotation, translation);

	this->kinematics.ComputeKinematics(rotation, translation);

	int numOfPoints = 100;
	::std::vector<double> s = linspace(0, this->kinematics.getRobot()->GetLength(), numOfPoints);
	::std::vector<SE3> frames(s.size());
	kinematics.GetBishopFrame(s, frames);

	::std::vector<double> point(3);
	::std::vector<::std::vector<double>> points;

	::std::vector<int> indices(this->tubes.size());

	for (int i = 0; i < frames.size(); ++i)
	{
		point[0] = frames[i].GetPosition()[0];
		point[1] = frames[i].GetPosition()[1];
		point[2] = frames[i].GetPosition()[2];
		points.push_back(point);
	}

	point[0] = frames.back().GetPosition()[0] + frames.back().GetZ()[0] * 20;
	point[1] = frames.back().GetPosition()[1] + frames.back().GetZ()[1] * 20;
	point[2] = frames.back().GetPosition()[2] + frames.back().GetZ()[2] * 20;

	points.push_back(point);

	for (int i = 0; i < 3; ++i)
		indices[i] = points.size();

	::std::vector<bool> mask(this->tubes.size()); 

	for (int i = 0; i < frames.size(); ++i)
	{
		for (int j = 0; j < 3; ++j)
			mask[j] = false;

		this->kinematics.getRobot()->GetExistingTubes(s[i], mask);
		
		for (int j = 0; j < this->tubes.size(); ++j)
		{
			if (!mask[j] && (indices[j] == points.size()))
				indices[j] = i - 1;
		}

	}

	// update tubes
	::std::vector<::std::vector<double>>::iterator it_begin = points.begin();
	::std::vector<::std::vector<double>>::iterator it_end = points.end();

	for (int i = 0; i < this->tubes.size(); ++i)
		this->tubes[i].update(it_begin, it_begin + indices[i]);
}


void 
RobotVisualizer::registerVisualizer(const vtkSmartPointer<vtkRenderer>& ren)
{
	for (int i = 0; i < this->tubes.size(); ++i)
		ren->AddActor(this->tubes[i].getActor());
}


void
RobotVisualizer::getActors(::std::vector<vtkSmartPointer<vtkActor>>& actors)
{
	actors.clear();
	for (int i = 0; i < 3; ++i)
		actors.push_back(this->tubes[i].getActor());
}