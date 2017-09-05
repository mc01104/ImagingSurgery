#include "stdafx.h"
#include "WallSegmentation.h"


WallSegmentation::WallSegmentation()
{
}

WallSegmentation::~WallSegmentation()
{
}

bool WallSegmentation::processImage(::cv::Mat img, int& x, int& y, bool display, int cx, int cy, int radius)
{

    int64 t0 = cv::getTickCount();
	int numOfPoints = (int) 3.14 * radius*radius *0.04;
    ::cv::Mat thresholded_mask, intermediate_img;

    // Blur  the image to remove small artifacts (color defects from camera, bood flow ..)
    ::cv::Mat img_blur;
    ::cv::GaussianBlur(img,img_blur,::cv::Size(15,15),0);

    // Mask the optical window using provided center and radius
    ::cv::Mat ow_mask = ::cv::Mat::zeros(img.rows,img.cols, CV_8UC1);
    ::cv::circle(ow_mask,::cv::Point(cx,cy),radius,255,-1);

    img_blur.copyTo(intermediate_img,ow_mask);
    this->thresholdImage(intermediate_img,thresholded_mask);

	int num_of_points = ::cv::countNonZero(thresholded_mask);

    if (::cv::countNonZero(thresholded_mask)>numOfPoints)
    {
        ::cv::Moments M = ::cv::moments(thresholded_mask,true);
        x = int(M.m10/M.m00);
        y = int(M.m01/M.m00);

        ::cv::circle(img, ::cv::Point(x,y),5,::cv::Scalar(0,255,0),-1);
    }

    int64 t1 = cv::getTickCount();
    double secs = (t1-t0)/cv::getTickFrequency();

    if (display)
    {
        ::cv::imshow("Thresholded mask", thresholded_mask);
        ::cv::imshow("Input image", img);

        ::cv::waitKey(1);
    }

    return true;
}


bool WallSegmentation::thresholdImage(const cv::Mat &img, ::cv::Mat &output)
{

    output = ::cv::Mat::zeros(img.rows,img.cols, CV_8UC1);

    ::cv::Mat S, A, V;
    convertImage(img,S,A, V);

    // Apply thresholds
    const int thresh_S = 64;
	//const int thresh_V = 200; -> worked first sucess
	const int thresh_V = 250;
    const int min_a = 130, max_a = 145;

    ::cv::Mat mask_s;
    ::cv::threshold(S,mask_s,thresh_S,255,::cv::THRESH_BINARY_INV);

	double min, max;
	::cv::minMaxLoc(A, &min, &max);
	//::std::cout << "min:" << min << " max:" << max << ::std::endl;

	::cv::Mat mask_v;
	::cv::threshold(V, mask_v, thresh_V, 255, ::cv::THRESH_BINARY_INV);

    ::cv::Mat mask_a;
    ::cv::inRange(A,min_a,max_a,mask_a);

	::cv::bitwise_and(mask_s, mask_v, output); 
    ::cv::bitwise_and(output,mask_a,output);

    // Apply morphological opening to remove small things
    ::cv::Mat kernel = ::cv::getStructuringElement(::cv::MORPH_ELLIPSE,::cv::Size(9,9));
    ::cv::morphologyEx(output,output,::cv::MORPH_OPEN,kernel);

    return true;
}

bool WallSegmentation::convertImage(const cv::Mat &img, cv::Mat& S, cv::Mat& A, ::cv::Mat& V)
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

    return true;
}
