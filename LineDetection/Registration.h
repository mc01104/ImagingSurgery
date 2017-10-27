#pragma once

#include <opencv2/opencv.hpp>


class RegistrationHandler
{
	public:

		RegistrationHandler();

		~RegistrationHandler();

		bool processImage(const ::cv::Mat& img, double& registrationError, ::cv::Mat& img_rec);

	protected:
		bool threshold(const ::cv::Mat& img, ::cv::Mat& thresholdedImg);

};