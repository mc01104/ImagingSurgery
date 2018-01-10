#pragma once

#include <Eigen/Dense>
#include "IncrementalValveModel.h"
#include <vector>
#include <opencv2/opencv.hpp>


class RegistrationHandler
{
	::Eigen::Vector2d centroid;

	::Eigen::Vector2d workingChannel;

	::Eigen::Vector3d regPoint;

	IncrementalValveModel* model;

	double registrationError;

	::Eigen::Vector3d markers;
	::std::vector<int> visitedMarkers;

	bool isInitialized;

	bool regDetected;

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

public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		RegistrationHandler();

		RegistrationHandler(IncrementalValveModel* model);

		~RegistrationHandler();

		bool processImage(const ::cv::Mat& img, double& registrationError, ::cv::Mat& img_rec);

		bool processImage(const ::cv::Mat& img, ::Eigen::Vector3d& robot_position, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal, double& registrationError);

		bool processImage(const ::cv::Mat& img, double clockfacePosition, double& registrationError);

		bool processImageSynthetic(const ::cv::Mat& img, ::Eigen::Vector3d& robot_position, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal, double& registrationError);

		bool thresholdSynthetic(const ::cv::Mat& img, ::cv::Mat& thresholdedImg);

		void getCentroid(::Eigen::Vector2d& centroid) {centroid = this->centroid;};

		void setWorkingChannel(::Eigen::Vector2d& workingChannel) {this->workingChannel = workingChannel;};

		void reset();

		bool getRegDetected(){return this->regDetected;};

		double getRecentMarker() {return this->visitedMarkers.back();};

	protected:
		bool threshold(const ::cv::Mat& img, ::cv::Mat& thresholdedImg);

		void computeCentroid(::std::vector<::cv::Point>& points);

		bool computeRegistrationError(::Eigen::Vector3d& robot_position, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal);

		void computePointOnValve(::Eigen::Vector3d& robot_position, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal);

		bool computeOffset(double clockPosition);

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

};