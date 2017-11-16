# pragma once

#define USER_MATRIX

#include <vtkArrowSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkMath.h>
#include <vtkSphereSource.h>
#include <vtkProperty.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <time.h>


class ArrowVisualizer
{
		double startPoint[3];
		double endPoint[3];
		double color[3];

		vtkSmartPointer<vtkArrowSource> arrowSource;
		vtkSmartPointer<vtkMatrix4x4> matrix;
		vtkSmartPointer<vtkTransform> transform;
		vtkSmartPointer<vtkTransformPolyDataFilter> transformPD;
		vtkSmartPointer<vtkPolyDataMapper> mapper;
		vtkSmartPointer<vtkActor> actor;

	public:
		
		ArrowVisualizer();

		ArrowVisualizer(double startPoint[3], double endPoint[3], double color[3]);

		~ArrowVisualizer();

		void updateArrow(double startPoint[3], double endPoint[3]);

		void setArrowColor(double color[3]);

		vtkSmartPointer<vtkActor> getActor() {return actor;};

	protected:

		void recomputeArrowsParameters();
};