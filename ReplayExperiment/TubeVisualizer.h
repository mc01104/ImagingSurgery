#pragma once


#include <vtkTubeFilter.h>
#include <vtkPolyLineSource.h>
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


class TubeVisualizer
{
		typedef ::std::vector<::std::vector<double>> dVector;
		double color[3];

		vtkSmartPointer<vtkPolyLineSource> lineSource;
		vtkSmartPointer<vtkTubeFilter> tubeFilter;
		vtkSmartPointer<vtkPolyDataMapper> mapper;
		vtkSmartPointer<vtkActor> actor;

		int npts;

		double radius;

	public:
		
		TubeVisualizer();

		TubeVisualizer(double color[3], double radius = 1.0);

		~TubeVisualizer();

		void initialize();

		void update(dVector& skeletonPoints);

		template <typename Iterator>
		void update(Iterator it_begin, Iterator it_end)
		{
			dVector tmp(::std::distance(it_begin, it_end)); 
			::std::copy(it_begin, it_end, tmp.begin());
			this->update(tmp);
		}

		void setColor(double color[3]);

		void setNumberOfPoints(int numPoints) {this->npts = numPoints;};

		void setRadius(double radius) {this->tubeFilter->SetRadius(radius);};

		vtkSmartPointer<vtkActor> getActor() {return actor;};

};