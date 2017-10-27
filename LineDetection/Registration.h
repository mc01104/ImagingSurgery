#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>


class RegistrationHandler
{
	::Eigen::Vector2d centroid;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		RegistrationHandler();

		~RegistrationHandler();

		bool processImage(const ::cv::Mat& img, double& registrationError, ::cv::Mat& img_rec);

		void getCentroid(::Eigen::Vector3d& centroid) {centroid = this->centroid;};

	protected:
		bool threshold(const ::cv::Mat& img, ::cv::Mat& thresholdedImg);
		void computeCentroid(::std::vector<::cv::Point>& points);
};