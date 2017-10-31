#include "stdafx.h"
#include "Registration.h"
#include "HTransform.h"

RegistrationHandler::RegistrationHandler() : centroid(0, 0), workingChannel(125, 200), regPoint(0, 0, 0), registrationError(0),
	markers(12, 4, 8)
{
	model = new IncrementalValveModel();
}

RegistrationHandler::RegistrationHandler(IncrementalValveModel* model) : model(model)
{
}

RegistrationHandler::~RegistrationHandler()
{
	delete model;
}

bool
RegistrationHandler::processImage(const ::cv::Mat& img, ::Eigen::Vector3d& robot_position, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal, double& registrationError)
{
	::cv::Mat thresImage;
	bool success = this->threshold(img, thresImage);

	if (success)
		this->computeRegistrationError(robot_position, innerTubeRotation, imageInitRotation, normal);

	return success;
}

bool
RegistrationHandler::threshold(const ::cv::Mat& img, ::cv::Mat& thresholdedImg)
{
	::cv::Mat hsv;
	::cv::cvtColor(img, hsv, ::cv::COLOR_BGR2HSV);

	::std::vector<::cv::Mat> HSV_split;
	::cv::split(hsv, HSV_split);

	int l_thres = 100;
	int h_thres = 120;

	int l_thres_s = 80;
	int h_thres_s = 255;

	int l_thres_v = 136;
	int h_thres_v = 255;
	::cv::Mat mask_h, mask_s, mask_v;
	::cv::inRange(HSV_split[0], l_thres, h_thres, mask_h);
	::cv::inRange(HSV_split[1], l_thres_s, h_thres_s, mask_s);
	::cv::inRange(HSV_split[2], l_thres_v, h_thres_v, mask_v);

	::cv::Mat output;
	::cv::bitwise_and(mask_h, mask_s, output);
	::cv::bitwise_and(output, mask_v, output);

    // Apply morphological opening to remove small things
    ::cv::Mat kernel = ::cv::getStructuringElement(::cv::MORPH_ELLIPSE, ::cv::Size(5,5));
    ::cv::morphologyEx(output, output, ::cv::MORPH_OPEN,kernel);

	::cv::cvtColor(output, thresholdedImg, CV_GRAY2BGR);

	::cv::Mat bin;
	output.convertTo(bin, CV_8UC1);
	
	::std::vector< ::cv::Point> nonzero;
	::cv::findNonZero(bin, nonzero);

	
	if (nonzero.size() < 10)
		return false;

	this->computeCentroid(nonzero);

	return true;

}

void RegistrationHandler::computeCentroid(::std::vector<::cv::Point>& points)
{
	::std::vector<::cv::Point>::const_iterator it = points.begin();
	double sum_x = 0, sum_y = 0;
	for(it; it != points.end(); ++it)
	{
		sum_x += it->x;
		sum_y += it->y;
	}

	this->centroid(0) = sum_x/points.size();
	this->centroid(1) = sum_y/points.size();
}

void RegistrationHandler::computeRegistrationError(::Eigen::Vector3d& robot_position, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal)
{
	// convert centroid to world coordinates by accounting also the offset of the marker and the working channel
	this->computePointOnValve(robot_position, innerTubeRotation, imageInitRotation, normal);
	this->regPoint = robot_position;

	// find the clockface position that corresponds to this point in space
	double clockfacePosition = 0;
	::Eigen::Vector3d point;
	this->model->getClockfacePosition(this->regPoint(0), this->regPoint(1), this->regPoint(2), clockfacePosition, point);

	// compute the error
	this->computeOffset(clockfacePosition);
}


void
RegistrationHandler::computePointOnValve(::Eigen::Vector3d& robot_position, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal)
{
	::Eigen::Vector2d DP = this->centroid - this->workingChannel;   // in pixels
	DP /= 26.67;

	::Eigen::Matrix3d rotation = RotateZ(imageInitRotation * M_PI/180.0 - innerTubeRotation);
	DP = rotation.block(0, 0, 2, 2).transpose()* DP;

	rotation = RotateZ( -90 * M_PI/180.0);
	DP = rotation.block(0, 0, 2, 2).transpose()* DP; // in world frame in mm

	::Eigen::Vector3d tmp;
	tmp.segment(0, 2) = DP;
	tmp(2) = 0;
	tmp = tmp - tmp.dot(normal) * normal;

	robot_position += tmp;
}

void 
RegistrationHandler::computeOffset(double clockPosition)
{
	double min_dist = 1000, distance = 0;
	double closest_marker = clockPosition;
	for (int i = 0; i < this->markers.size(); ++i)
	{
		distance = ::std::abs(clockPosition - this->markers[i]);
		if (distance < min_dist)
		{
			min_dist = distance;
			closest_marker = this->markers[i];

		}
	}

	this->registrationError = closest_marker - clockPosition;				// check that
}

bool 
RegistrationHandler::processImageSynthetic(const ::cv::Mat& img, ::Eigen::Vector3d& robot_position, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal, double& registrationError)
{
	::cv::Mat thresImage;
	bool success = this->thresholdSynthetic(img, thresImage);

	if (success)
		this->computeRegistrationError(robot_position, innerTubeRotation, imageInitRotation, normal);

	if (success)
		registrationError = this->registrationError;
	return success;
}


bool 
RegistrationHandler::thresholdSynthetic(const ::cv::Mat& img, ::cv::Mat& thresholdedImg)
{
	::cv::Mat hsv;
	::cv::cvtColor(img, hsv, ::cv::COLOR_BGR2HSV);

	::std::vector<::cv::Mat> HSV_split;
	::cv::split(hsv, HSV_split);

	int l_thres = 163;
	int h_thres = 180;

	int l_thres_2 = 0;
	int h_thres_2 = 11;

	::cv::Mat mask_h1, mask_h2;
	::cv::inRange(HSV_split[0], l_thres, h_thres, mask_h1);
	::cv::inRange(HSV_split[0], l_thres_2, l_thres_2, mask_h2);


	::cv::Mat output;
	::cv::bitwise_or(mask_h1, mask_h2, output);
	//::cv::bitwise_and(output, mask_v, output);

    // Apply morphological opening to remove small things
    ::cv::Mat kernel = ::cv::getStructuringElement(::cv::MORPH_ELLIPSE, ::cv::Size(5,5));
    ::cv::morphologyEx(output, output, ::cv::MORPH_OPEN,kernel);

	::cv::cvtColor(output, thresholdedImg, CV_GRAY2BGR);
	::cv::imshow("marker", thresholdedImg);
	::cv::waitKey(1);

	::cv::Mat bin;
	output.convertTo(bin, CV_8UC1);
	
	::std::vector< ::cv::Point> nonzero;
	::cv::findNonZero(bin, nonzero);
	
	if (nonzero.size() < 10)
		return false;

	this->computeCentroid(nonzero);

	return true;

}