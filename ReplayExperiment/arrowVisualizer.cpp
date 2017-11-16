#include "stdafx.h"
#include "arrowVisualizer.h"

ArrowVisualizer::ArrowVisualizer()
{
	arrowSource = vtkSmartPointer<vtkArrowSource>::New();
	matrix =  vtkSmartPointer<vtkMatrix4x4>::New();
	transform = vtkSmartPointer<vtkTransform>::New();
	transformPD = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	actor = vtkSmartPointer<vtkActor>::New();
}

ArrowVisualizer::ArrowVisualizer(double startPoint[3], double endPoint[3], double color[3])
{
	arrowSource = vtkSmartPointer<vtkArrowSource>::New();
	matrix =  vtkSmartPointer<vtkMatrix4x4>::New();
	transform = vtkSmartPointer<vtkTransform>::New();
	transformPD = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	actor = vtkSmartPointer<vtkActor>::New();

	this->updateArrow(startPoint, endPoint);

	this->actor->GetProperty()->SetColor(color);
}

ArrowVisualizer::~ArrowVisualizer()
{
}

void ArrowVisualizer::updateArrow(double startPoint[3], double endPoint[3])
{
	memcpy(this->startPoint, startPoint, 3 * sizeof(double));
	memcpy(this->endPoint, endPoint, 3 * sizeof(double));
	
	this->recomputeArrowsParameters();
}

void ArrowVisualizer::setArrowColor(double color[3])
{
	this->actor->GetProperty()->SetColor(color);
}

void ArrowVisualizer::recomputeArrowsParameters()
{
	// Compute a basis
	double normalizedX[3];
	double normalizedY[3];
	double normalizedZ[3];
 
	// The X axis is a vector from start to end
	vtkMath::Subtract(this->endPoint, this->startPoint, normalizedX);
	double length = vtkMath::Norm(normalizedX);
	vtkMath::Normalize(normalizedX);
 
	// The Z axis is an arbitrary vector cross X
	double arbitrary[3];
	arbitrary[0] = vtkMath::Random(-10,10);
	arbitrary[1] = vtkMath::Random(-10,10);
	arbitrary[2] = vtkMath::Random(-10,10);
	vtkMath::Cross(normalizedX, arbitrary, normalizedZ);
	vtkMath::Normalize(normalizedZ);
 
	// The Y axis is Z cross X
	vtkMath::Cross(normalizedZ, normalizedX, normalizedY);
 
	// Create the direction cosine matrix
	this->matrix->Identity();
	for (unsigned int i = 0; i < 3; i++)
	{
		this->matrix->SetElement(i, 0, normalizedX[i]);
		this->matrix->SetElement(i, 1, normalizedY[i]);
		this->matrix->SetElement(i, 2, normalizedZ[i]);
	}    
	
	this->transform->Identity();
	this->transform->Translate(startPoint);
	this->transform->Concatenate(matrix);
	this->transform->Scale(length, length, length);
 
	// Transform the polydata
	this->transformPD->SetTransform(transform);
	this->transformPD->SetInputConnection(arrowSource->GetOutputPort());
 
#ifdef USER_MATRIX
  mapper->SetInputConnection(arrowSource->GetOutputPort());
  actor->SetUserMatrix(transform->GetMatrix());
#else
  mapper->SetInputConnection(transformPD->GetOutputPort());
#endif
  actor->SetMapper(mapper);

}
