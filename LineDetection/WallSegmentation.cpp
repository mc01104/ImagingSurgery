#include "stdafx.h"
#include "WallSegmentation.h"


WallSegmentation::WallSegmentation()
{
	sliderValueSat = 98;
	sliderValueAlphaMin = 130;
	sliderValueAlphaMax = 160;
	counter = 0;
}

WallSegmentation::~WallSegmentation()
{
}

bool WallSegmentation::processImage(::cv::Mat img, int& x, int& y, bool display, int cx, int cy, int radius)
{
	if (this->counter < 1)
		this->initializeTrackbars();
 
	this->counter++;

	int numOfPoints = (int) 3.14 * radius*radius *0.05;

    // Blur  the image to remove small artifacts (color defects from camera, blood flow ..)
    ::cv::Mat img_blur;
    ::cv::GaussianBlur(img, img_blur, ::cv::Size(15,15), 0);

    // Mask the optical window using provided center and radius
    ::cv::Mat ow_mask = ::cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
    ::cv::circle(ow_mask, ::cv::Point(cx, cy), radius, 255,-1);

	::cv::Mat thresholded_mask, intermediate_img;
    img_blur.copyTo(intermediate_img, ow_mask);

    this->thresholdImage(intermediate_img, thresholded_mask);

    if (display)
    {
        ::cv::imshow("Thresholded mask", thresholded_mask);
        //::cv::imshow("Input image", img);

        ::cv::waitKey(1);
    }

    if (::cv::countNonZero(thresholded_mask) > numOfPoints)
    {
        ::cv::Moments M = ::cv::moments(thresholded_mask, true);
        x = (int) M.m10/M.m00;
        y = (int) M.m01/M.m00;

        ::cv::circle(img, ::cv::Point(x, y), 5, ::cv::Scalar(0, 255, 0), -1);
		 
		return true;
    }

    return false;
}


void WallSegmentation::thresholdImage(const cv::Mat &img, ::cv::Mat &output)
{
    output = ::cv::Mat::zeros(img.rows, img.cols, CV_8UC1);

    ::cv::Mat S, A, V;
    convertImage(img, S, A, V);

	const int thresh_S = this->sliderValueSat;
	const int min_a = this->sliderValueAlphaMin;
	const int max_a = this->sliderValueAlphaMax;

    ::cv::Mat mask_s;
    ::cv::threshold(S, mask_s, thresh_S,255, ::cv::THRESH_BINARY_INV);

    ::cv::Mat mask_a;
    ::cv::inRange(A, min_a, max_a, mask_a);

	::cv::bitwise_and(mask_s, mask_a, output); 

    // Apply morphological opening to remove small things
    ::cv::Mat kernel = ::cv::getStructuringElement(::cv::MORPH_ELLIPSE,::cv::Size(9,9));
    ::cv::morphologyEx(output,output,::cv::MORPH_OPEN,kernel);
}


void WallSegmentation::convertImage(const cv::Mat &img, cv::Mat& S, cv::Mat& A, ::cv::Mat& V)
{	
    ::cv::Mat hsv, lab;

    ::std::vector< ::cv::Mat> hsv_split;
    ::std::vector< ::cv::Mat> lab_split;

    ::cv::cvtColor(img,hsv, CV_BGR2HSV);
    ::cv::cvtColor(img,lab, CV_BGR2Lab);

    ::cv::split(hsv,hsv_split);
    ::cv::split(lab,lab_split);

    S = hsv_split[1];
	V = hsv_split[2];
    A = lab_split[1];   
}

void WallSegmentation::initializeTrackbars()
{
	::cv::namedWindow("apex-to-valve thresholds", 1);

    /// Create Trackbars
    ::cv::createTrackbar("Sat", "apex-to-valve thresholds", &sliderValueSat, 255, &WallSegmentation::onTrackbarChangeS, this);
	::cv::createTrackbar("Val_min", "apex-to-valve thresholds", &sliderValueAlphaMin, 255, &WallSegmentation::onTrackbarChangeAL, this);
	::cv::createTrackbar("Val_max", "apex-to-valve thresholds", &sliderValueAlphaMax, 255, &WallSegmentation::onTrackbarChangeAH, this);

}

void WallSegmentation::onTrackbarChangeS(int newValue, void * object)
{
	WallSegmentation* localObj = reinterpret_cast<WallSegmentation*> (object);

	localObj->s_thres = newValue;

}


void WallSegmentation::onTrackbarChangeAL(int newValue, void * object)
{
	WallSegmentation* localObj = reinterpret_cast<WallSegmentation*> (object);

	localObj->alpha_min = newValue;

}

void WallSegmentation::onTrackbarChangeAH(int newValue, void * object)
{
	WallSegmentation* localObj = reinterpret_cast<WallSegmentation*> (object);

	localObj->alpha_max = newValue;

}
