#include "stdafx.h"
#include "Registration.h"
#include "HTransform.h"


// this is not making much sense -> refactor (who has the responsibility for updating the model?
RegistrationHandler::RegistrationHandler() : centroid(0, 0), workingChannel(125, 200), regPoint(0, 0, 0), registrationError(0),
	markers(12, 4, 8), regDetected(false)
{
	model = new IncrementalValveModel();
}

RegistrationHandler::RegistrationHandler(IncrementalValveModel* model) : model(model), centroid(0, 0), workingChannel(125, 200), regPoint(0, 0, 0), registrationError(0),
	markers(12, 4, 8), regDetected(false)
{
}

// removed the delete model because it's not owned by this class -> if use the first constructor somebody needs to clean it
RegistrationHandler::~RegistrationHandler()
{
}

bool
RegistrationHandler::processImage(const ::cv::Mat& img, ::Eigen::Vector3d& robot_position, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal, double& registrationError)
{
	::cv::Mat thresImage;
	this->regDetected = false;
	bool success = this->threshold(img, thresImage);
	this->regDetected = success;

	bool newRegFound = false;
	if (success)
		newRegFound = this->computeRegistrationError(robot_position, innerTubeRotation, imageInitRotation, normal);

	if (newRegFound)
		registrationError = this->registrationError;

	return newRegFound;
}

bool
RegistrationHandler::threshold(const ::cv::Mat& img, ::cv::Mat& thresholdedImg)
{
	::cv::Mat hsv;
	::cv::cvtColor(img, hsv, ::cv::COLOR_BGR2HSV);

	::std::vector<::cv::Mat> HSV_split;
	::cv::split(hsv, HSV_split);

	int l_thres = 112;
	int h_thres = 134;

	int l_thres_s = 69;
	int h_thres_s = 172;

	int l_thres_v = 73;
	int h_thres_v = 255;

	//int l_thres = 100;
	//int h_thres = 120;

	//int l_thres_s = 80;
	//int h_thres_s = 255;

	//int l_thres_v = 136;
	//int h_thres_v = 255;

	::cv::Mat mask_h, mask_s, mask_v;
	::cv::inRange(HSV_split[0], l_thres, h_thres, mask_h);
	::cv::inRange(HSV_split[1], l_thres_s, h_thres_s, mask_s);
	::cv::inRange(HSV_split[2], l_thres_v, h_thres_v, mask_v);

	::cv::Mat output;
	::cv::bitwise_and(mask_h, mask_s, output);
	::cv::bitwise_and(output, mask_v, output);

    ::cv::Mat ow_mask = ::cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
    ::cv::circle(ow_mask, ::cv::Point(img.rows/2, img.cols/2), img.rows/2, 255,-1);
	
	::cv::bitwise_and(output, ow_mask, output);

    // Apply morphological opening to remove small things
    ::cv::Mat kernel = ::cv::getStructuringElement(::cv::MORPH_ELLIPSE, ::cv::Size(5,5));
    ::cv::morphologyEx(output, output, ::cv::MORPH_OPEN,kernel);

	::cv::cvtColor(output, thresholdedImg, CV_GRAY2BGR);

	::cv::imshow("marker", output);
	::cv::imshow("unrotated", img);

	::cv::Mat bin;
	output.convertTo(bin, CV_8UC1);
	
	::std::vector< ::cv::Point> nonzero;
	::cv::findNonZero(bin, nonzero);


 	if (nonzero.size() < 150) // used to be 500 - test
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

bool RegistrationHandler::computeRegistrationError(::Eigen::Vector3d& robot_position, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal)
{
	// convert centroid to world coordinates by accounting also the offset of the marker and the working channel
	this->computePointOnValve(robot_position, innerTubeRotation, imageInitRotation, normal);
	this->regPoint = robot_position;

	// find the clockface position that corresponds to this point in space
	double clockfacePosition = 0;
	::Eigen::Vector3d point;
	this->model->getClockfacePosition(this->regPoint(0), this->regPoint(1), this->regPoint(2), clockfacePosition, point);

	//::std::cout << "marker detected at robot's " << clockfacePosition << " o' clock" << ::std::endl;
	// compute the error
	return this->computeOffset(clockfacePosition);
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

bool 
RegistrationHandler::computeOffset(double clockPosition)
{
	double min_dist = 1000, distance = 0;
	double d1 = 0, d2 = 0;
	double closest_marker = clockPosition;
	for (int i = 0; i < this->markers.size(); ++i)
	{
		d1 = ::std::abs(this->markers[i] - clockPosition);
		d2 = ::std::abs(12 + (clockPosition - this->markers[i]));
		distance = ::std::min(d1, d2);

		if (distance < min_dist)
		{
			min_dist = distance;
			closest_marker = this->markers[i];
		}
		
	}
	//::std::cout << "closest marker:" << closest_marker << std::endl;
	auto result2 = std::find(this->visitedMarkers.begin(), this->visitedMarkers.end(), closest_marker);
 
	if (result2 != this->visitedMarkers.end())
	{
		//::std::cout << "marker: " << closest_marker << "  has already been used to register" << ::std::endl;
		return false;
	}
	this->visitedMarkers.push_back(closest_marker);

	//this->registrationError = closest_marker - clockPosition;				// check that
	double angularOffset = min_dist * 30; // in degrees

	double angle1 = 0.5*  60 * clockPosition; // + this->registrationRotation; 
	double angle2 = 0.5*  60 * closest_marker; // + this->registrationRotation; 

	::Eigen::Vector3d pRobot(cos(angle1 * M_PI/180.0), sin(angle1 * M_PI/180.0), 1);
	::Eigen::Vector3d pMarker(cos(angle2 * M_PI/180.0), sin(angle2 * M_PI/180.0), 1);

	double tmp = pMarker.cross(pRobot)[2];
	
	if (tmp < 0)
		angularOffset *= -1;

	this->registrationError = angularOffset;

	return true;
}

bool 
RegistrationHandler::processImageSynthetic(const ::cv::Mat& img, ::Eigen::Vector3d& robot_position, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal, double& registrationError)
{
	::cv::Mat thresImage;
	bool success = this->thresholdSynthetic(img, thresImage);

	bool newRegFound = false;
	if (success)
		newRegFound = this->computeRegistrationError(robot_position, innerTubeRotation, imageInitRotation, normal);

	if (newRegFound)
		registrationError = this->registrationError;

	return newRegFound;
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
	::cv::inRange(HSV_split[1], l_thres_2, l_thres_2, mask_h2);

	::cv::Mat output;
	::cv::bitwise_or(mask_h1, mask_h2, output);

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

void 
RegistrationHandler::reset()
{
	this->registrationError = 0;

	this->visitedMarkers.clear();
}