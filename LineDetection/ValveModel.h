#pragma once

#include <Eigen/dense>

class ValveModel
{
		double radius;
		::Eigen::Vector3d center;
		
		::Eigen::Vector3d v1;
		::Eigen::Vector3d v2;
		::Eigen::Vector3d v3;

		::Eigen::Matrix3d XtX;
		::Eigen::Vector3d column_sums;
		::Eigen::Vector3d column_means;
		::Eigen::Matrix3d XtXCentered;
		::Eigen::MatrixXd points;
		::Eigen::MatrixXd pointsCentered;
		::Eigen::MatrixXd projectedPoints;
		::Eigen::MatrixXd projectionMatrix;

		bool initialized;

		double maxNPoints;

		double currentParameterValue;

		::Eigen::Matrix3d	U;
	public:

		ValveModel();
		
		~ValveModel();

		bool updateModel(double x, double y, double z);

		void getTangentEstimate(double x, double y, double z,  ::Eigen::Vector3d& tangent,::Eigen::Vector3d& point_on_circle);

		void resetModel();

		bool isInitialized() {return initialized;};

		double* getCenter() {return this->center.data();};
		double	getRadius() {return this->radius;};
		double* getNormal();

		void getProjectionMatrixToPlane(::Eigen::MatrixXd& proj);

		void getClosestPointOnCircle(double x, double y, double z, ::Eigen::Vector3d& point);

	protected:

		double computeCircleParameter(double x, double y, double z);

		void projectPoints();

		void fitCircle();

		double circleObjectiveFunction(double t, const ::Eigen::Vector3d& point);	

		void computeVariance();
};

