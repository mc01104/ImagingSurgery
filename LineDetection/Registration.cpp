#include "stdafx.h"
#include "Registration.h"


RegistrationHandler::RegistrationHandler()
{
}

RegistrationHandler::~RegistrationHandler()
{
}

bool
RegistrationHandler::processImage(const ::cv::Mat& img, double& registrationError, ::cv::Mat& img_rec)
{
	::cv::Mat thresImage;
	bool sucess = this->threshold(img, thresImage);

	/*::cv::Mat combinedImage;*/
	::cv::hconcat(img, thresImage, img_rec);
	::cv::imshow("img", img_rec);
	::cv::waitKey(1);
	
	return sucess;
}

bool
RegistrationHandler::threshold(const ::cv::Mat& img, ::cv::Mat& thresholdedImg)
{
	::cv::Mat hsv;
	::cv::cvtColor(img, hsv, ::cv::COLOR_BGR2HSV);

	::std::vector<::cv::Mat> HSV_split;
	::cv::split(hsv, HSV_split);

	int l_thres = 100;
	int h_thres = 120;

	int l_thres_s = 80;
	int h_thres_s = 255;

	int l_thres_v = 136;
	int h_thres_v = 255;
	::cv::Mat mask_h, mask_s, mask_v;
	::cv::inRange(HSV_split[0], l_thres, h_thres, mask_h);
	::cv::inRange(HSV_split[1], l_thres_s, h_thres_s, mask_s);
	::cv::inRange(HSV_split[2], l_thres_v, h_thres_v, mask_v);

	::cv::Mat output;
	::cv::bitwise_and(mask_h, mask_s, output);
	::cv::bitwise_and(output, mask_v, output);

    // Apply morphological opening to remove small things
    ::cv::Mat kernel = ::cv::getStructuringElement(::cv::MORPH_ELLIPSE, ::cv::Size(5,5));
    ::cv::morphologyEx(output, output, ::cv::MORPH_OPEN,kernel);

	::cv::cvtColor(output, thresholdedImg, CV_GRAY2BGR);

	::cv::Mat bin;
	output.convertTo(bin, CV_8UC1);
	
	::std::vector< ::cv::Point> nonzero;
	::cv::findNonZero(bin, nonzero);

	if (nonzero.size() < 10)
		return false;

	return true;

}