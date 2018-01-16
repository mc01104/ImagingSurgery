#include "stdafx.h"
#include <cmath>       
#include "HTransform.h"
#include "IncrementalValveModel.h"


// initial value of radius is increased on purpose to reduce clockface uncertainty due to bending in the beginning
IncrementalValveModel::IncrementalValveModel()
	: radius(19.0), center(0, 0, 0), normal(0, 0, 1), referencePosition(0, 1, 0), initialized(false), maxNPoints(200), registrationRotation(0),
	v1(0, 1, 0), v2(1, 0, 0), lambda(0.0005), wallFollowingState(LEFT), registered(false), clockFollowed(9), prevClockPosition(-1)
{
	errorJacobian.setZero();
	x.setZero();
	x(5) = this->radius/10.0;
	center /= 100;
}

IncrementalValveModel::~IncrementalValveModel()
{
}

bool 
IncrementalValveModel::updateModel(double x, double y, double z)
{
	this->addPoint(x, y, z);

	this->updateCircleParameters();

	return true;
}

bool 
IncrementalValveModel::updateModel(double x, double y, double z, double clockface)
{
	this->addPoint(x, y, z, clockface);

	this->updateCircleParameters();

	return true;
}

void 
IncrementalValveModel::addPoint(double x, double y, double z)
{
	// if this is the first point initialize the center
	if (this->pointExists(x, y, z))
		return;

	this->points.push_back(::Eigen::Vector3d(x, y, z));

	if (this->points.size() == 1)
	{
		this->initializeCircleCenter();
		this->center(2) = z;

		this->initialized = true;
	}
}

void 
IncrementalValveModel::addPoint(double x, double y, double z, double clockPosition)
{
	double d = 0;
	if (this->prevClockPosition < 0)
		this->prevClockPosition = clockPosition;
	else
	{
		d = computeClockDistance(clockPosition, this->prevClockPosition);
		if (::std::abs(d) < 1.0)
			return;
	}

	//// if this is the first point initialize the center
	//if (this->pointExists(x, y, z))
	//	return;

	this->points.push_back(::Eigen::Vector3d(x, y, z));

	this->prevClockPosition = clockPosition;

	if (this->points.size() == 1)
	{
		this->initializeCircleCenter();
		this->center(2) = z;

		this->initialized = true;
	}
}
void
IncrementalValveModel::initializeCircleCenter()
{
	// initialize center based on which wall we are following
	switch (this->wallFollowingState)
	{
	case LEFT:
		center = this->points.back() + this->radius * ::Eigen::Vector3d(0, 1, 0);
		break;
	case TOP:
		center = this->points.back() - this->radius * ::Eigen::Vector3d(1, 0, 0);
		break;
	case BOTTOM:
		center = this->points.back() + this->radius * ::Eigen::Vector3d(1, 0, 0);
		break;
	case USER:
		double angle = 0.5*  60 * this->clockFollowed;
		center = this->points.back() - this->radius * ::Eigen::Vector3d(cos(angle * M_PI/180.0), sin(angle * M_PI/180.0), 0);
		break;
	}
	
	this->x.segment(0, 3) = center/100.0;
}

void 
IncrementalValveModel::updateCircleBaseVectors()
{
	this->v1 = this->v1.eval() - this->v1.dot(this->normal) * this->normal;
	this->v1.normalize();

	this->v2 = this->v2.eval() - this->v2.dot(this->normal) * this->normal;
	this->v2.normalize();
}

void IncrementalValveModel::setRegistrationRotation(double rotation)
{
	//if (this->registered)
	//	return;
	::std::cout << "registration offset:" << rotation << ::std::endl;
	this->registrationRotation = rotation;

	::Eigen::Matrix3d rot = RotateZ(this->registrationRotation * M_PI/180.0);
	this->referencePosition = rot * this->referencePosition;

	this->registered = true;
}

// ---- NOT TESTED YET!! -------- // 
void 
IncrementalValveModel::updateReferencePosition()
{
	this->referencePosition = this->referencePosition.eval() - this->referencePosition.dot(this->normal) * this->normal;
	this->referencePosition.normalize();

}

void
IncrementalValveModel::resetModel()
{
	radius = 9.0;
	center.setZero();
	normal = ::Eigen::Vector3d(0, 0, 1);
	initialized = false;
	registrationRotation = 0.0;

	this->points.clear();
	referencePosition = ::Eigen::Vector3d(0, 1, 0);

	this->updateCircleOptState();
	
	this->v1 = ::Eigen::Vector3d(0, 1, 0);
	this->v2 = ::Eigen::Vector3d(1, 0, 0);
	this->lambda = 0.0005;
	this->wallFollowingState = ::IncrementalValveModel::LEFT;

	this->registered = false;
}

void
IncrementalValveModel::getCenter(double center[3])
{
	memcpy(center, this->center.data(), 3 * sizeof(double));
}

void
IncrementalValveModel::getNormal(double normal[3])
{
	memcpy(normal, this->normal.data(), 3 * sizeof(double));
}

void 
IncrementalValveModel::getModelParameters(double center[3], double normal[3], double& radius)
{
	memcpy(center, this->center.data(), 3 * sizeof(double));
	memcpy(normal, this->normal.data(), 3 * sizeof(double));
	radius = this->radius;
}

// ---------- TESTED -----------//
void 
IncrementalValveModel::getClockfacePosition(double x, double y, double z, double& clockfacePosition, ::Eigen::Vector3d& point)
{
	this->getNearestPointOnCircle(x, y, z, point);

	double angle = 0;
	this->computeAngle(point, angle);

	this->angleToClockfacePosition(angle, clockfacePosition);
}

void 
IncrementalValveModel::clockfaceToWorldPosition(double time, ::Eigen::Vector3d& point)
{
	double angle = 0.5*  60 * time + this->registrationRotation;        // not sure if the registration is correct
	getNearestPointOnCircle(this->center[0] + cos(angle*M_PI/180.0), this->center[1] + sin(angle*M_PI/180.0), this->center[2], point);
}

// ---------- TESTED -----------//
void
IncrementalValveModel::angleToClockfacePosition(double angle, double& clockfacePosition)   // counter clockwise angle from horizontal line (3 o'clock)
{
	 clockfacePosition = 3.0 - (1.0/30.0) * ::std::fmod(angle, 360);
	 
	 if (clockfacePosition <= 0)
		 clockfacePosition += 12;
}

// ---------- TESTED -----------//
void 
IncrementalValveModel::getNearestPointOnCircle(double x, double y, double z, ::Eigen::Vector3d& point)
{
	double t = this->computeCircleParameter(x, y, z);

	point = this->center + this->radius * ::std::cos(t) * this->v1 + this->radius * ::std::sin(t) * this->v2;
}

double
IncrementalValveModel::computeCircleParameter(double x, double y, double z)
{
	::Eigen::Vector3d point(x, y, z);

	double t1 = ::std::atan2(this->v2.transpose() * (point - this->center), this->v1.transpose() * (point - this->center));
	double t2 = t1 + M_PI;

	return (circleObjectiveFunction(t1, point) < circleObjectiveFunction(t2, point) ? t1 : t2);
}

double
IncrementalValveModel::circleObjectiveFunction(double t, const ::Eigen::Vector3d& point)
{
	return (this->center + this->radius * ::std::cos(t) * this->v1 + this->radius * ::std::sin(t) * this->v2 - point).norm();
}

// ---------- TESTED -----------//
void
IncrementalValveModel::computeAngle(const ::Eigen::Vector3d& point, double& angle)		// point should be on the circle
{
	::Eigen::Vector3d tmp = (point - this->center);
	tmp.normalize();

	double det = this->normal.dot(tmp.cross(this->referencePosition));
	double dot = this->referencePosition.dot(tmp);
	angle = atan2(det, dot); 
	(angle < 0 ? angle += 2*M_PI : angle);
	angle *= 180.0/M_PI;
}

double
IncrementalValveModel::computeError()
{
	double sum = 0;

	::std::vector<::Eigen::Vector3d>::const_iterator it = this->points.begin();
	for (it; it != this->points.end(); ++it)
		sum += computeSquaredDistance(*it);

	return sum;
}

double
IncrementalValveModel::computeSquaredDistance(const ::Eigen::Vector3d& point)
{
	::Eigen::Vector3d nearestPoint;
	this->getNearestPointOnCircle(point(0), point(1), point(2), nearestPoint);

	return (point - nearestPoint).norm();
}

void
IncrementalValveModel::computeErrorJacobian()
{
	double tmp;
	double err = 0, errPrev = 0;
	double epsilon = 0.0001;
	double invEpsilon = 1.0/epsilon;

	for (int i = 0; i < 6; ++i)
	{
		errPrev = this->computeError();
		tmp = this->x(i);

		this->x(i) += epsilon;
		this->updateCircleOptState();
		err = this->computeError();

		this->errorJacobian(i) = (err - errPrev) * invEpsilon;

		this->x(i) = tmp;
		this->updateCircleOptState();
	}
}

void 
IncrementalValveModel::updateCircleOptState()
{
	this->center = this->x.segment(0, 3) * 100;
	this->normal.segment(0, 2) = this->x.segment(3, 2);
	this->normal(2) = sqrt(1.0 - this->normal(0) * this->normal(0) - this->normal(0) * this->normal(0));
	this->radius = this->x(5) * 10.0;

	this->updateReferencePosition();

	this->updateCircleBaseVectors();
}

void
IncrementalValveModel::updateCircleParameters()
{
	if (this->points.size() < 3)
		return;

	int counter = 0; 
	::Eigen::VectorXd xPrev;

	double vectorChange = 10000;
	double errorChange = 10000;
	double errorPrev = 0, error = 0;

	while(vectorChange > 0.01 && abs(errorChange) > 0.01)
	{
		this->computeErrorJacobian();
		xPrev = x;
		errorPrev = this->computeError();
		this->x = this->x - this->lambda * this->errorJacobian;
	
		this->updateCircleOptState();
		error = this->computeError();

		if (counter++ > 20)
			return;

		if (error > errorPrev)
		{
			this->lambda *= 0.3;

			this->x = xPrev;

			this->updateCircleOptState();

			continue;
		}
		else
			this->lambda = 0.0005;

		vectorChange = (this->x - xPrev).norm();

		errorChange = error - errorPrev;


	}

}

int 
IncrementalValveModel::getDirectionOfMotion(const ::Eigen::Vector3d& point, const ::Eigen::Vector3d& velocity)
{
	::Eigen::Vector3d closestPoint;
	this->getNearestPointOnCircle(point(0), point(1), point(2), closestPoint);
	closestPoint -= this->center;
	::Eigen::Vector3d tmp = closestPoint.cross(velocity);

	return (tmp(2) > 0 ? 0 : 1);

}

void 
IncrementalValveModel::getNearestPointOnCircle(double position[3], double closestPoint[3])
{
	::Eigen::Vector3d point;
	this->getNearestPointOnCircle(position[0], position[1], position[2], point);
	memcpy(closestPoint, point.data(), 3 * sizeof(double));
}

bool 
IncrementalValveModel::pointExists(double x, double y, double z)
{
	double dst = 0;
	::Eigen::Vector3d point = ::Eigen::Vector3d(x, y, z);
	for (int i = 0; i < this->points.size(); ++i)
		if ( (this->points[i] - point).norm() < 3)				// in mm
			return true;

	return false;
}

void
IncrementalValveModel::getLeakPosition(::std::vector<::Eigen::Vector3d>& leaks)
{
	leaks.clear();
	double leakClockPositions[3] = {10, 2, 6};
	::Eigen::Vector3d tmp(center), point;

	for (int i = 0; i < 3; ++i)
	{
		this->clockfaceToWorldPosition(leakClockPositions[i], tmp);
		getNearestPointOnCircle(tmp(0), tmp(1), center[2], point);
		leaks.push_back(point);
	}
}

void
IncrementalValveModel::resetRegistration()
{
	this->registered = false; 

	this->setRegistrationRotation(0); 

	this->registered = false;

	this->referencePosition = ::Eigen::Vector3d(0, 1, 0);

	this->updateReferencePosition();
};

double 
IncrementalValveModel::computeClockDistance(double c1, double c2)
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
