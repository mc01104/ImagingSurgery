#include "stdafx.h"
#include "TubeVisualizer.h"

TubeVisualizer::TubeVisualizer() : npts(40), radius(1)
{
	lineSource = vtkSmartPointer<vtkPolyLineSource>::New();
	tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
	mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	actor = vtkSmartPointer<vtkActor>::New();

	this->initialize();
}

TubeVisualizer::TubeVisualizer(double color[3], double radius) : npts(100), radius(radius)
{
	lineSource = vtkSmartPointer<vtkPolyLineSource>::New();
	tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
	mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	actor = vtkSmartPointer<vtkActor>::New();

	this->actor->GetProperty()->SetColor(color);

	this->initialize();
}

TubeVisualizer::~TubeVisualizer()
{
}

void TubeVisualizer::initialize()
{
	for (int i = 0; i < npts; ++i)
		lineSource->SetPoint(i, i,i,i);

	tubeFilter->SetInputConnection(lineSource->GetOutputPort());
	tubeFilter->SetRadius(this->radius);                  
	tubeFilter->SetNumberOfSides(30);
	tubeFilter->Update();

	mapper->SetInputConnection(tubeFilter->GetOutputPort());
	actor->SetMapper(mapper);
}

void TubeVisualizer::update(dVector& skeletonPoints)
{
	this->lineSource->SetNumberOfPoints(skeletonPoints.size());
	for (int i = 0; i < skeletonPoints.size(); ++i)
		lineSource->SetPoint(i, skeletonPoints[i][0], skeletonPoints[i][1], skeletonPoints[i][2]);
	
}