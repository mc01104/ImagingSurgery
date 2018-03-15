#pragma once

#include <Eigen/Dense>
#include "IncrementalValveModel.h"
#include <vector>
#include <opencv2/opencv.hpp>


class RegistrationHandler
{
	::Eigen::Vector2d centroid;

	::Eigen::Vector2d workingChannel;

	//::Eigen::Vector3d regPoint;

	IncrementalValveModel* model;

	double registrationError;

	::Eigen::Vector3d markers;
	::std::vector<double> visitedMarkers;

	bool isInitialized;

	bool regDetected;

	double clockface;

	int sliderValueHueMin;
	int sliderValueHueMax;

	int sliderValueSatMin;
	int sliderValueSatMax;

	int sliderValueValMin;
	int sliderValueValMax;

	int l_thres;
	int h_thres;

	int l_thres_s;
	int h_thres_s;

	int l_thres_v;
	int h_thres_v;

	::Eigen::Vector3d			offset;
	::cv::Mat					thresImage;
	::cv::Mat					hsv;
	::std::vector<::cv::Mat>	HSV_split;
	::cv::Mat					mask_h;
	::cv::Mat					mask_s;
	::cv::Mat					mask_v;
	::cv::Mat					output;
	::cv::Mat					ow_mask;
	::cv::Mat					kernel;
	::cv::Mat					bin;
	::std::vector< ::cv::Point> nonzero;
	::Eigen::Vector2d			DP;
	::Eigen::Matrix3d			rotation;

	::Eigen::Vector3d			tmp;
	::Eigen::Vector3d			res;

	int		iter;
public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		RegistrationHandler();

		RegistrationHandler(IncrementalValveModel* model);

		~RegistrationHandler();

		bool processImage(const ::cv::Mat& img, double& registrationError, ::cv::Mat& img_rec);

		bool processImage(const ::cv::Mat& img, ::Eigen::Vector2d& lineCentroid, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal, double clockface, double& registrationError);

		bool processImage(const ::cv::Mat& img, double clockfacePosition, double& registrationError);

		bool processImageSynthetic(const ::cv::Mat& img, ::Eigen::Vector3d& robot_position, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal, double& registrationError);

		bool thresholdSynthetic(const ::cv::Mat& img, ::cv::Mat& thresholdedImg);

		void getCentroid(::Eigen::Vector2d& centroid) {centroid = this->centroid;};

		void getCentroid(::cv::Point& point){point.x = this->centroid[0]; point.y = this->centroid[1];};

		void setWorkingChannel(::Eigen::Vector2d& workingChannel) {this->workingChannel = workingChannel;};

		void reset();

		bool getRegDetected(){return this->regDetected;};

		void setRegDetected(bool flag) {this->regDetected = false;};

		double getRecentMarker() {return this->visitedMarkers.back();};

		void computeMarkerOffset(const ::Eigen::Vector2d lineCentroid, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal);

	protected:
		bool threshold(const ::cv::Mat& img, ::cv::Mat& thresholdedImg);

		void computeCentroid(::std::vector<::cv::Point>& points);

		bool computeRegistrationError(const ::Eigen::Vector2d lineCentroid, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal);

		void computePointOnValve(const ::Eigen::Vector3d& robot_position, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal);

		bool computeOffset(double clockPosition);

		double computeOffset(double clockPosition1, double clockPosition2);

		// threshold callbacks
		// Hue
		static void onTrackbarChangeHL(int newValue, void * object);
		static void onTrackbarChangeHH(int newValue, void * object);

		// Saturation
		static void onTrackbarChangeSL(int newValue, void * object);
		static void onTrackbarChangeSH(int newValue, void * object);

		// Value
		static void onTrackbarChangeVL(int newValue, void * object);
		static void onTrackbarChangeVH(int newValue, void * object);

		void initializeSliders();

};