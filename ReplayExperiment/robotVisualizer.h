#pragma once

#include "TubeVisualizer.h"
#include "MechanicsBasedKinematics.h"


class RobotVisualizer
{
		::std::vector<TubeVisualizer> tubes;

		MechanicsBasedKinematics kinematics;

	public:

		RobotVisualizer(const MechanicsBasedKinematics& kinematics);

		~RobotVisualizer();

		void update(const double* configuration, const double* actualPosition = NULL);

		void registerVisualizer(const vtkSmartPointer<vtkRenderer>& ren);

		void getActors(::std::vector<vtkSmartPointer<vtkActor>>& actors);
};