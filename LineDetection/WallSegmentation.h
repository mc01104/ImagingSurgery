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
		int sliderValueSat;

		int sliderValueAlphaMin;
		int sliderValueAlphaMax;

		int s_thres;

		int alpha_min;
		int alpha_max;

		int counter;

	public:
        WallSegmentation();

        ~WallSegmentation();

        bool processImage(::cv::Mat img, int& x, int& y, bool display=false, int cx = 125, int cy = 125, int radius = 125);

	private:

        void convertImage(const cv::Mat &img, cv::Mat& S, cv::Mat& A, ::cv::Mat& V);

        void thresholdImage(const cv::Mat &img, ::cv::Mat &output);

		void initializeTrackbars();

		static void onTrackbarChangeS(int newValue, void * object);

		static void onTrackbarChangeAL(int newValue, void * object);

		static void onTrackbarChangeAH(int newValue, void * object);
};
