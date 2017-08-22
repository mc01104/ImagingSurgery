#include "stdafx.h"
#include "ValveModel.h"
#include "Utilities.h"

ValveModel::ValveModel():
	radius(0),
	center(0, 0, 0),
	v1(1, 0, 0),
	v2(0, 1, 0),
	column_sums(0, 0, 0),
	initialized(false),
	maxNPoints(200)
{
	XtX.setZero();
	this->projectionMatrix.resize(3, 3);
}

ValveModel::~ValveModel()
{
}

// ---------TESTED--------------//
bool
ValveModel::updateModel(double x, double y, double z)
{
	::Eigen::Vector3d tmpX(x, y, z);
	if (this->points.rows() > this->maxNPoints)
		popFirstRowEigen(this->points);

	bool checkForPointInBuffer = false;
	for (int i = 0; i < this->points.rows(); ++i)
	{
		double dist = (this->points.row(i) - tmpX.transpose()).norm();
		if (dist < 4)
			checkForPointInBuffer = true;
	}

	if (!checkForPointInBuffer)
		appendRowEigen(this->points, tmpX);


	if (this->points.rows() < 6)
		return false;

	this->column_means = this->points.colwise().mean();
	this->pointsCentered = this->points.rowwise() - this->column_means.transpose();

	this->XtX = this->pointsCentered.transpose() * this->pointsCentered;

	int setting = Eigen::ComputeFullU | Eigen::ComputeFullU;
	Eigen::JacobiSVD<Eigen::Matrix3d> svd = XtX.jacobiSvd(setting);
	U = svd.matrixU();

	this->v1 = U.col(0);
	this->v2 = U.col(1);
	this->v3 = U.col(2);

	this->projectPoints();

	this->fitCircle();
	
	this->center = this->projectionMatrix.transpose() * this->center;
	this->center += this->column_means;

	this->currentParameterValue = this->computeCircleParameter(x, y, z);
	
	this->initialized = true;
	return true;
}

void
ValveModel::getTangentEstimate(double x, double y, double z, ::Eigen::Vector3d& tangent, ::Eigen::Vector3d& point_on_circle)
{
	double t = this->computeCircleParameter(x, y, z);

	tangent = -this->radius * ::std::sin(t) * this->v1 + this->radius * ::std::cos(t) * this->v2;
	if (tangent.norm() != 0)
		tangent.normalize();
	

	point_on_circle = this->center + this->radius * ::std::cos(t) * this->v1 + this->radius * ::std::sin(t) * this->v2;
}

void 
ValveModel::resetModel()
{
	radius = 0;
	center << 0, 0, 0;
	v1 << 0, 0, 0;
	v2 << 0, 0, 0;
	column_sums << 0, 0, 0;
	initialized = false;
}

// ---------TESTED------------ //
double
ValveModel::computeCircleParameter(double x, double y, double z)
{
	::Eigen::Vector3d point(x, y, z);

	double t1 = ::std::atan2(this->v2.transpose() * (point - this->center), this->v1.transpose() * (point - this->center));
	double t2 = t1 + M_PI;

	return (circleObjectiveFunction(t1, point) < circleObjectiveFunction(t2, point) ? t1 : t2);
}


void
ValveModel::projectPoints()
{
	this->projectionMatrix.setZero();
	this->projectionMatrix.row(0) = this->v1;
	this->projectionMatrix.row(1) = this->v2;
	this->projectedPoints = this->pointsCentered * this->projectionMatrix.transpose();	
}

// ----- TESTED ---- //
void
ValveModel::fitCircle()
{
	::Eigen::MatrixXd A_fit(points.rows(), 3);
	A_fit.setOnes();
	A_fit.block(0, 0, points.rows(), 2) = this->projectedPoints.block(0, 0, points.rows(), 2);
	
	::Eigen::VectorXd b_fit(points.rows(), 1);
	b_fit = -this->projectedPoints.col(0).array().square() - this->projectedPoints.col(1).array().square();

	::Eigen::VectorXd x = (A_fit.transpose() * A_fit).inverse() * A_fit.transpose() * b_fit;
	this->center[0] = -x(0)/2.0;
	this->center[1] = -x(1)/2.0;
	this->center[2] = 0;

	this->radius = ::std::sqrt((::std::pow(x(0), 2) + ::std::pow(x(1), 2))/4.0 - x(2));

	//this->computeVariance();
}

// ----- TESTED ---- //
double
ValveModel::circleObjectiveFunction(double t, const ::Eigen::Vector3d& point)
{
	return (this->center + this->radius * ::std::cos(t) * this->v1 + this->radius * ::std::sin(t) * this->v2 - point).norm();
}

void
ValveModel::getProjectionMatrixToPlane(::Eigen::MatrixXd& proj)
{
	proj.resize(3, 3);
	if (this->points.rows() < 25)
	{
		proj.setIdentity();
		return;
	}

	proj = this->U;
}

void 
ValveModel::getClosestPointOnCircle(double x, double y, double z, ::Eigen::Vector3d& point)
{
	double t = this->computeCircleParameter(x, y, z);

	point = this->center + this->radius * ::std::cos(t) * this->v1 + this->radius * ::std::sin(t) * this->v2;
}


double* 
ValveModel::getNormal()
{
	return this->v3.data();
}

void 
ValveModel::getCenter(double center[3])
{
	memcpy(center, this->center.data(), 3 * sizeof(double));
}

void 
ValveModel::getNormal(double normal[3])
{
	memcpy(normal, this->getNormal(), 3 * sizeof(double));
}
