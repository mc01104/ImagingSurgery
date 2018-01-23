#pragma once

#include <vector>
#include <Eigen/dense>

class IncrementalValveModel
{
	public:
		enum WALL_FOLLOWED
		{
			LEFT = 0,
			TOP,
			BOTTOM,
			USER
		};

	protected:

		// valve parameters
		double radius;
		::Eigen::Vector3d center;
		::Eigen::Vector3d normal;		

		::Eigen::Vector3d v1;
		::Eigen::Vector3d v2;

		::Eigen::Vector3d referencePosition;			// corresponds to three o' clock

		::std::vector<::Eigen::Vector3d> points;
		::Eigen::Matrix<double, 6, 1> errorJacobian;
		::Eigen::Matrix<double, 6, 1> x;							// concatenated circle params for optimization

		::Eigen::VectorXd xPrev;

		double	lambda;

		bool initialized;

		double maxNPoints;

		double registrationRotation;
		double totalOffset;

		bool registered;

		WALL_FOLLOWED wallFollowingState;

		int clockFollowed;

		double prevClockPosition;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		IncrementalValveModel();
		
		~IncrementalValveModel();

		bool updateModel(double x, double y, double z);

		bool updateModel(double x, double y, double z, double clockface);

		void resetModel();

		bool isInitialized() {return initialized;};

		void getCenter(double center[3]);

		void getNormal(double normal[3]);

		void getNormal(::Eigen::Vector3d& normal);

		double getRadius() {return this->radius;};

		void getModelParameters(double center[3], double normal[3], double& radius);

		void getClockfacePosition(double x, double y, double z, double& clockfacePosition, ::Eigen::Vector3d& point);

		int getDirectionOfMotion(const ::Eigen::Vector3d& point, const ::Eigen::Vector3d& velocity);

		void setRegistrationRotation(double rotation);

		double getRegistrationOffset() {return this->registrationRotation;};

		void resetRegistration(); 

		void setWallFollowingState(IncrementalValveModel::WALL_FOLLOWED state) {this->wallFollowingState = state;};

		void setFollowedClockPosition(double position) {this->clockFollowed = position;};

		void getNearestPointOnCircle(double position[3], double closestPoint[3]);

		double getNumberOfPoints() {return  this->points.size();};

		void clockfaceToWorldPosition(double angle, ::Eigen::Vector3d& point);

		void getLeakPosition(::std::vector<::Eigen::Vector3d>& leaks);

		bool isRegistered() {return this->registered;};
	protected:

		void addPoint(double x, double y, double z);
		void addPoint(double x, double y, double z, double clockPosition);

		void getNearestPointOnCircle(double x, double y, double z, ::Eigen::Vector3d& point);
		void computeAngle(const ::Eigen::Vector3d& point, double& angle);
		void angleToClockfacePosition(double angle, double& clockfacePosition);
		double computeError();
		void computeErrorJacobian();
		void updateCircleParameters();
		double computeCircleParameter(double x, double y, double z);
		double circleObjectiveFunction(double t, const ::Eigen::Vector3d& point);
		void updateReferencePosition();
		void updateCircleBaseVectors();
		void initializeCircleCenter();
		double computeSquaredDistance(const ::Eigen::Vector3d& point);
		void updateCircleOptState();
		bool pointExists(double x, double y, double z);
		double computeClockDistance(double c1, double c2);
};

