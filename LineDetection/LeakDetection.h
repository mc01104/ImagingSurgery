#pragma once
#include <opencv2/opencv.hpp>

class LeakDetector
{
		::cv::Mat currentImage;
		::cv::Mat thresholdedImage;
		::cv::Mat centroids;
	public:
		LeakDetector();

		~LeakDetector();

		void processImage(const ::cv::Mat& img, int x, int y);


	private:
		
		void thresholdImage();
		void thresholdImageHSV();
};