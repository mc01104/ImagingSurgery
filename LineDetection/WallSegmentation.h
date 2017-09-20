#pragma once

// general includes
#include <vector>
#include <string>
#include <exception>
#include <cmath>

// openCV
#include <opencv2/opencv.hpp>


class WallSegmentation
{
	public:
        WallSegmentation();

        ~WallSegmentation();

        bool processImage(::cv::Mat img, int& x, int& y, bool display=false, int cx = 108, int cy = 108, int radius = 135);

	private:

        void convertImage(const cv::Mat &img, cv::Mat& S, cv::Mat& A, ::cv::Mat& V);

        void thresholdImage(const cv::Mat &img, ::cv::Mat &output);
};
