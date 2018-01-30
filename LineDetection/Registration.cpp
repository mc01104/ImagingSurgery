#include "stdafx.h"
#include "Registration.h"
#include "HTransform.h"


// this is not making much sense -> refactor (who has the responsibility for updating the model?
RegistrationHandler::RegistrationHandler() : centroid(0, 0), workingChannel(125, 120), registrationError(0),
	markers(12, 4, 8), regDetected(false), clockface(-1), offset(0, 0, 1), iter(0)
{
	model = new IncrementalValveModel();

    l_thres = 32;
	h_thres = 100;

    l_thres_s = 0;
	h_thres_s = 116;

	l_thres_v = 1;
	h_thres_v = 255;

	sliderValueHueMin = l_thres;
	sliderValueHueMax = h_thres;

	sliderValueSatMin = l_thres_s;
	sliderValueSatMax = h_thres_s;

	sliderValueValMin = l_thres_v;
	sliderValueValMax = h_thres_v;

    ow_mask = ::cv::Mat::zeros(250, 250, CV_8UC1);
    ::cv::circle(ow_mask, ::cv::Point(250.0/2, 250.0/2), 250.0/2, 255,-1);

}

RegistrationHandler::RegistrationHandler(IncrementalValveModel* model) : model(model), centroid(0, 0), workingChannel(125, 200), registrationError(0),
	markers(12, 4, 8), regDetected(false), clockface(-1), offset(0, 0, 1), iter(0)
{
    l_thres = 32;
	h_thres = 100;

    l_thres_s = 0;
	h_thres_s = 116;

	l_thres_v = 1;
	h_thres_v = 255;

	sliderValueHueMin = l_thres;
	sliderValueHueMax = h_thres;

	sliderValueSatMin = l_thres_s;
	sliderValueSatMax = h_thres_s;

	sliderValueValMin = l_thres_v;
	sliderValueValMax = h_thres_v;

    ow_mask = ::cv::Mat::zeros(250, 250, CV_8UC1);
	::cv::circle(ow_mask, ::cv::Point(250.0/2, 250.0/2), 250.0/2, 255,-1);
}

// removed the delete model because it's not owned by this class -> if use the first constructor somebody needs to clean it
RegistrationHandler::~RegistrationHandler()
{
}

bool
RegistrationHandler::processImage(const ::cv::Mat& img, ::Eigen::Vector2d& lineCentroid, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal, double clockface, double& registrationError)
{
	if (this->iter < 1)
		this->initializeSliders();

	iter++;

	if (!this->model->isInitialized())
		return false;

	if (clockface < 0)
		return false;

	this->clockface = clockface;
	
	this->regDetected = false;
	this->regDetected = this->threshold(img, thresImage);
	

	bool newRegFound = false;
	if (this->regDetected)
		newRegFound = this->computeRegistrationError(lineCentroid, innerTubeRotation, imageInitRotation, normal);

	if (newRegFound)
		registrationError = this->registrationError;

	return newRegFound;
}

bool
RegistrationHandler::processImage(const ::cv::Mat& img, double clockfacePosition, double& registrationError)
{

	if (!this->model->isInitialized())
		return false;

	::cv::Mat thresImage;
	this->regDetected = false;
	bool success = this->threshold(img, thresImage);
	this->regDetected = success;

	bool newRegFound = false;
	if (success)
		newRegFound = this->computeOffset(clockfacePosition);

	if (newRegFound)
		registrationError = this->registrationError;

	return newRegFound;
}

bool
RegistrationHandler::threshold(const ::cv::Mat& img, ::cv::Mat& thresholdedImg)
{

	::cv::cvtColor(img, hsv, ::cv::COLOR_BGR2HSV);

	::cv::split(hsv, HSV_split);

	::cv::inRange(HSV_split[0], l_thres, h_thres, mask_h);
	::cv::inRange(HSV_split[1], l_thres_s, h_thres_s, mask_s);
	::cv::inRange(HSV_split[2], l_thres_v, h_thres_v, mask_v);


	::cv::bitwise_and(mask_h, mask_s, output);
	::cv::bitwise_and(output, mask_v, output);

	::cv::bitwise_and(output, ow_mask, output);

    // Apply morphological opening to remove small things
    kernel = ::cv::getStructuringElement(::cv::MORPH_ELLIPSE, ::cv::Size(5,5));
    ::cv::morphologyEx(output, output, ::cv::MORPH_OPEN,kernel);

	::cv::cvtColor(output, thresholdedImg, CV_GRAY2BGR);

	::cv::imshow("marker", output);

	output.convertTo(bin, CV_8UC1);

	::cv::findNonZero(bin, nonzero);

 	if (nonzero.size() < 100) // used to be 500 - test
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

bool RegistrationHandler::computeRegistrationError(const ::Eigen::Vector2d lineCentroid, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal)
{

	this->computeMarkerOffset(lineCentroid, innerTubeRotation, imageInitRotation,normal);

	this->offset(2) = 0.0;
	double timeOffset = 360.0/30.0 * this->offset.norm()/(2.0 * M_PI * 9);

	double realAngle = this->clockface * 30.0;
	::Eigen::Vector3d tmp(::std::cos(realAngle * M_PI/180.0), ::std::sin(realAngle * M_PI/180.0), 0);

	this->offset(2) = 0.0;
	this->offset.normalize();

	res = tmp.cross(this->offset);
	::std::cout << "working channel:" << this->clockface << ::std::endl;
	double finalMarkerClockPosition = this->clockface;
	::std::cout << timeOffset << ::std::endl;
	(res(2) > 0 ? finalMarkerClockPosition += timeOffset : finalMarkerClockPosition -= timeOffset);
	::std::cout << "marker's estimated position:" << finalMarkerClockPosition << ::std::endl;
	return this->computeOffset(finalMarkerClockPosition);

}

void
RegistrationHandler::computePointOnValve(const ::Eigen::Vector3d& robot_position, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal)
{
	DP = this->centroid - this->workingChannel;   // in pixels
	DP /= 26.67;

	rotation = RotateZ(imageInitRotation * M_PI/180.0 - innerTubeRotation);
	DP = rotation.block(0, 0, 2, 2).transpose()* DP;

	rotation = RotateZ( -90 * M_PI/180.0);
	DP = rotation.block(0, 0, 2, 2).transpose()* DP; // in world frame in mm

	tmp.segment(0, 2) = DP;
	tmp(2) = 0;
	tmp = tmp - tmp.dot(normal) * normal;

	this->offset = tmp;
}

bool 
RegistrationHandler::computeOffset(double clockPosition)
{
	double min_dist = 1000, distance = 0;
	double d1 = 0, d2 = 0, d3 = 0;
	double closest_marker = clockPosition;
	for (int i = 0; i < this->markers.size(); ++i)
	{
		d1 = ::std::abs(this->markers[i] - clockPosition);
		d2 = ::std::abs(12 + (clockPosition - this->markers[i]));
		d3 = ::std::abs(12 + (this->markers[i] - clockPosition));
		distance = ::std::min(d1, d2);
		distance = ::std::min(distance, d3);

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
	//double tmp = pRobot.cross(pMarker)[2];
	
	if (tmp < 0)
		angularOffset *= -1;

	this->registrationError = angularOffset;

	return true;
}

bool 
RegistrationHandler::processImageSynthetic(const ::cv::Mat& img, ::Eigen::Vector3d& robot_position, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal, double& registrationError)
{
	//::cv::Mat thresImage;
	//bool success = this->thresholdSynthetic(img, thresImage);

	bool newRegFound = false;
	//if (success)
	//	newRegFound = this->computeRegistrationError(robot_position, innerTubeRotation, imageInitRotation, normal);

	//if (newRegFound)
	//	registrationError = this->registrationError;

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

	this->clockface = -1;

	this->offset = ::Eigen::Vector3d(0, 0, 1);

	this->iter = 0;

	this->centroid = ::Eigen::Vector2d(0, 0);

	this->regDetected = false;
}

bool 
RegistrationHandler::processImage(const ::cv::Mat& img, double& registrationError, ::cv::Mat& img_rec)
{
	::cv::Mat thresImage;
	this->regDetected = false;
	bool success = this->threshold(img, img_rec);
	registrationError = 0;

	return true;
}

void RegistrationHandler::onTrackbarChangeHL(int newValue, void * object)
{
	RegistrationHandler* localObj = reinterpret_cast<RegistrationHandler*> (object);

	localObj->l_thres = newValue;

}

void RegistrationHandler::onTrackbarChangeHH(int newValue, void * object)
{
	RegistrationHandler* localObj = reinterpret_cast<RegistrationHandler*> (object);

	localObj->h_thres = newValue;

}

void RegistrationHandler::onTrackbarChangeSL(int newValue, void * object)
{
	RegistrationHandler* localObj = reinterpret_cast<RegistrationHandler*> (object);

	localObj->l_thres_s = newValue;

}

void RegistrationHandler::onTrackbarChangeSH(int newValue, void * object)
{
	RegistrationHandler* localObj = reinterpret_cast<RegistrationHandler*> (object);

	localObj->h_thres_s = newValue;

}

void RegistrationHandler::onTrackbarChangeVL(int newValue, void * object)
{
	RegistrationHandler* localObj = reinterpret_cast<RegistrationHandler*> (object);

	localObj->l_thres_v = newValue;

}

void RegistrationHandler::onTrackbarChangeVH(int newValue, void * object)
{
	RegistrationHandler* localObj = reinterpret_cast<RegistrationHandler*> (object);

	localObj->h_thres_v = newValue;

}

double 
RegistrationHandler::computeOffset(double clockPosition1, double clockPosition2)
{

	double d1 = ::std::abs(clockPosition1 - clockPosition2);
	double d2 = ::std::abs(12 + (clockPosition2 - clockPosition1));
	double d3 = ::std::abs(12 + (clockPosition1 - clockPosition2));
	double distance = ::std::min(d1, d2);
	distance = ::std::min(distance, d3);

	double angle1 = 0.5*  60 * clockPosition1;
	double angle2 = 0.5*  60 * clockPosition2;

	::Eigen::Vector3d pRobot(cos(angle1 * M_PI/180.0), sin(angle1 * M_PI/180.0), 1);
	::Eigen::Vector3d pMarker(cos(angle2 * M_PI/180.0), sin(angle2 * M_PI/180.0), 1);

	//double tmp = pMarker.cross(pRobot)[2];
	double tmp = pRobot.cross(pMarker)[2];
	
	if (tmp < 0)
		distance *= -1;

	return distance;
}

void 
RegistrationHandler::computeMarkerOffset(const ::Eigen::Vector2d lineCentroid, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal)
{
	DP = this->centroid - lineCentroid;
	DP /= 26.67;

	rotation = RotateZ(imageInitRotation * M_PI/180.0 - innerTubeRotation);
	DP = rotation.block(0, 0, 2, 2).transpose()* DP;

	rotation = RotateZ( -90 * M_PI/180.0);
	DP = rotation.block(0, 0, 2, 2).transpose()* DP; // in world frame in mm

	tmp.segment(0, 2) = DP;
	tmp(2) = 0;
	tmp = tmp - tmp.dot(normal) * normal;

	this->offset = tmp;
}


void 
RegistrationHandler::initializeSliders()
{
	
	::cv::namedWindow("registration", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO | CV_GUI_EXPANDED);

	/// Create Trackbars
	::cv::createTrackbar("Hue_min", "registration", &sliderValueHueMin, 180, &RegistrationHandler::onTrackbarChangeHL, this);
	::cv::createTrackbar("Hue_max", "registration", &sliderValueHueMax, 180, &RegistrationHandler::onTrackbarChangeHH, this);

	::cv::createTrackbar("Sat_min", "registration", &sliderValueSatMin, 255, &RegistrationHandler::onTrackbarChangeSL, this);
	::cv::createTrackbar("Sat_max", "registration", &sliderValueSatMax, 255, &RegistrationHandler::onTrackbarChangeSH, this);

	::cv::createTrackbar("Val_min", "registration", &sliderValueValMin, 255, &RegistrationHandler::onTrackbarChangeVL, this);
	::cv::createTrackbar("Val_max", "registration", &sliderValueValMax, 255, &RegistrationHandler::onTrackbarChangeVH, this);

	::cv::waitKey(1);
}

