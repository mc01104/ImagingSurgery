#pragma once

#include <Eigen/Dense>
#include "IncrementalValveModel.h"
#include <opencv2/opencv.hpp>


class RegistrationHandler
{
	::Eigen::Vector2d centroid;

	::Eigen::Vector2d workingChannel;

	::Eigen::Vector3d regPoint;

	IncrementalValveModel* model;

	double registrationError;

	::Eigen::Vector3d markers;

	bool isInitialized;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		RegistrationHandler();

		RegistrationHandler(IncrementalValveModel* model);

		~RegistrationHandler();

		bool processImage(const ::cv::Mat& img, double& registrationError, ::cv::Mat& img_rec);
		bool processImage(const ::cv::Mat& img, ::Eigen::Vector3d& robot_position, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal, double& registrationError);
		void getCentroid(::Eigen::Vector2d& centroid) {centroid = this->centroid;};

		void setWorkingChannel(::Eigen::Vector2d& workingChannel) {this->workingChannel = workingChannel;};

	protected:
		bool threshold(const ::cv::Mat& img, ::cv::Mat& thresholdedImg);
		void computeCentroid(::std::vector<::cv::Point>& points);
		void computeRegistrationError(::Eigen::Vector3d& robot_position, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal);
		void computePointOnValve(::Eigen::Vector3d& robot_position, double innerTubeRotation, double imageInitRotation, const ::Eigen::Vector3d& normal);
		void computeOffset(double clockPosition);

};