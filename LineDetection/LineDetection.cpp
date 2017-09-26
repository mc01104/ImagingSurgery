#include "stdafx.h"
#include "LineDetection.h"

#define SCOPE_2

LineDetector::LineDetector()
{
	mode = MODE::CIRCUM;
}

LineDetector::~LineDetector()
{
}

bool LineDetector::processImage(::cv::Mat img, bool display, int crop)
{

    ::cv::Mat img_crop = img(::cv::Rect(crop,crop,img.cols-2*crop, img.rows-2*crop));

    ::cv::Vec4f line;

	bool lineDetected = false;


    if (this->detectLine(img_crop,line))
		lineDetected = true;


    return lineDetected;

}

bool LineDetector::processImage(::cv::Mat img, ::cv::Vec4f& line,cv::Vec2f &centroid, bool display, int crop, LineDetector::MODE mode)
{

    ::cv::Mat img_crop = img(::cv::Rect(crop,crop,img.cols-2*crop, img.rows-2*crop));

	bool lineDetected = false;

	switch (mode)
	{
		case MODE::TRANSITION:
			lineDetected = this->detectLine(img_crop,line, centroid);
			break;
		case MODE::CIRCUM:
		    lineDetected = this->detectLineAllChannels(img_crop,line, centroid);
			break;
	}

	if (lineDetected)
	{
		centroid[0] += crop;
		centroid[1] += crop;
	}

    return lineDetected;

}


void LineDetector::thresholdImage(const cv::Mat &img, ::cv::Mat &out)
{
    ::cv::Mat O2, O3;
    this->RGBtoOpponent(img, out, O2, O3); //out is the O1 layer directly

	// Threshold luminance to keep the 95th percentile
	// This avoids high-luminance parasite reflections
    double min, max;
    ::cv::Mat O3_mask;
    ::cv::minMaxLoc(O3, &min, &max);
    ::cv::threshold(O3, O3_mask, max - 0.05*(max-min), 1, ::cv::ThresholdTypes::THRESH_BINARY_INV);

	// Threshold to zero O1 pixels >0 and to 255 01 pixels <0
	::cv::threshold( out, out, 0, 255,::cv::ThresholdTypes::THRESH_BINARY_INV);
	           
    // Apply O3 mask after, not before !
    out = out.mul(O3_mask);

    ::cv::Mat O2_mask;
    ::cv::threshold(O2, O2_mask, 0, 1, ::cv::ThresholdTypes::THRESH_BINARY_INV);
	out = out.mul(O2_mask);

}

bool LineDetector::RGBtoOpponent(const ::cv::Mat &img, ::cv::Mat &O1, ::cv::Mat &O2, ::cv::Mat &O3)
{
    // ideally todo here is an assert for datatype and size of input ...

	::std::vector< ::cv::Mat> channels;
	::cv::split(img, channels);

    const int rows = img.rows;
    const int cols = img.cols;
    O1 = cv::Mat(img.size(), CV_32FC1);
    O2 = cv::Mat(img.size(), CV_32FC1);
    O3 = cv::Mat(img.size(), CV_32FC1);

    for(int r = 0;r < rows; ++r)
    {
        for(int c = 0; c < cols; ++c)
        {
            const cv::Vec3b &bgr = img.at<cv::Vec3b>(r, c);
            O1.at<float>(r,c) = (1.0f/sqrt(2)) * ((float)bgr[2] - (float)bgr[1]);
            O2.at<float>(r,c) = (1.0f/sqrt(6)) * ((float)bgr[2] + (float)bgr[1] - 2.0f * (float)bgr[0]);
            O3.at<float>(r,c) = (1.0f/sqrt(3)) * ((float)bgr[2] + (float)bgr[1] + (float)bgr[0]);
        }
    }

    // TODO: exception handling here
    return true;
}


bool LineDetector::detectLine(const ::cv::Mat img, ::cv::Vec4f &line, ::cv::Vec2f& centroid)
{	
    ::cv::Mat thresholded;
	
    ::cv::Mat thresholded_binary(img.size(),CV_8UC1);

	this->thresholdImage(img,thresholded);
 
	::cv::Mat masked_img;
    ::cv::Mat ow_mask = ::cv::Mat::zeros(img.rows,img.cols, CV_8UC1);
    ::cv::circle(ow_mask,::cv::Point(img.rows/2,img.cols/2),img.rows/2,255,-1);
	thresholded.copyTo(masked_img,ow_mask);

	masked_img.convertTo(thresholded_binary,CV_8UC1);

	::std::vector< ::cv::Point> nonzero;
	::cv::findNonZero(thresholded_binary, nonzero);

	::cv::imshow("thresholded", thresholded_binary);

	if (nonzero.size() > 400)
	{
        ::cv::fitLine(nonzero,line, CV_DIST_L2, 0, 0.01, 0.01);

		this->computeCentroid(nonzero, centroid);

		return true;
	}

	return false;
}

void LineDetector::computeCentroid(::std::vector< ::cv::Point>& nonzero, ::cv::Vec2f& centroid)
{
	float distance;
	float totalX=0.0, totalY=0.0;    
	for(int i = 0; i < nonzero.size(); ++i) 
	{
	    totalX+=nonzero[i].x;
	    totalY+=nonzero[i].y;
	}

	centroid[0] = totalX/nonzero.size();
	centroid[1] = totalY/nonzero.size();
}


// Helper function
void findNonZero(const ::cv::Mat& binary, ::std::vector< ::cv::Point> &idx) 
{
    assert(binary.cols > 0 && binary.rows > 0 && binary.channels() == 1);
    const int M = binary.rows;
    const int N = binary.cols;
    for (int m = 0; m < M; ++m)
    {
        const char* bin_ptr = binary.ptr<char>(m);
        for (int n = 0; n < N; ++n)
        {
            if (bin_ptr[n] > 0)
                idx.push_back(cv::Point(n,m));
        }
    }
}


bool LineDetector::processImageSynthetic(::cv::Mat img, ::cv::Vec4f& line,cv::Vec2f &centroid, bool display, int crop)
{

    ::cv::Mat img_crop = img(::cv::Rect(crop,crop,img.cols-2*crop, img.rows-2*crop));

	bool lineDetected = false;

    if (this->detectLineSynthetic(img_crop,line, centroid))
		lineDetected = true;

	centroid[0] += crop;
	centroid[1] += crop;

    return lineDetected;

}


void LineDetector::thresholdImageSynthetic(const cv::Mat &img, ::cv::Mat &out)
{
	::cv::Mat img_grey;
	::cv::cvtColor(img, img_grey, ::cv::ColorConversionCodes::COLOR_RGB2GRAY); 
	::cv::threshold(img_grey, out, 50, 255, ::cv::ThresholdTypes::THRESH_BINARY_INV);
}

bool LineDetector::detectLineSynthetic(const ::cv::Mat img, ::cv::Vec4f &line, ::cv::Vec2f& centroid)
{	
    ::cv::Mat thresholded;
	
    ::cv::Mat thresholded_binary(img.size(),CV_8UC1);

	this->thresholdImageSynthetic(img,thresholded);
    thresholded.convertTo(thresholded_binary,CV_8UC1);

	
    ::std::vector< ::cv::Point> nonzero;
    ::cv::findNonZero(thresholded_binary, nonzero);
    if (nonzero.size()>20)
	{
        ::cv::fitLine(nonzero,line, CV_DIST_L2, 0, 0.01, 0.01);

		this->computeCentroid(nonzero, centroid);
		::cv::line( thresholded_binary, ::cv::Point(centroid(0), centroid(1)), ::cv::Point(centroid(0)+line(0)*100, centroid(1)+line(1)*100), ::cv::Scalar(0, 255, 0), 2, CV_AA);
		::cv::line( thresholded_binary, ::cv::Point(centroid(0), centroid(1)), ::cv::Point(centroid(0)+line(0)*(-100), centroid(1)+line(1)*(-100)), ::cv::Scalar(0, 255, 0), 2, CV_AA);

		::cv::imshow("thresholded", thresholded_binary);
		::cv::waitKey(1);

		return true;
	}

	return false;
}

bool LineDetector::detectLineAllChannels(const ::cv::Mat img, cv::Vec4f &line, ::cv::Vec2f& centroid)
{
    ::cv::Mat thresholded;
	
    ::cv::Mat thresholded_binary(img.size(),CV_8UC1);

	//this->thresholdImageAllChannels(img,thresholded);
	this->thresholdImageWire(img,thresholded);
    thresholded.convertTo(thresholded_binary,CV_8UC1);
	
    ::std::vector< ::cv::Point> nonzero;
    ::cv::findNonZero(thresholded_binary, nonzero);

	::cv::Mat	 output;
	::cv::cvtColor(thresholded, output, CV_GRAY2BGR);

	::std::vector<::cv::Vec2f> lines_hough;
	if (nonzero.size() > 50)
	{
        ::cv::fitLine(nonzero,line, CV_DIST_L2, 0, 0.01, 0.01);

		this->computeCentroid(nonzero, centroid);

		::cv::line( output, ::cv::Point(line[2],line[3]), ::cv::Point(line[2]+line[0]*100,line[3]+line[1]*100), ::cv::Scalar(255, 255, 255), 2, CV_AA);
		::cv::line( output, ::cv::Point(line[2],line[3]), ::cv::Point(line[2]+line[0]*(-100),line[3]+line[1]*(-100)), ::cv::Scalar(255, 255, 255), 2, CV_AA);

		::cv::imshow("thresholded", output);
		::cv::waitKey(1);

		return true;
	}
	else 
	{
		::cv::imshow("thresholded", thresholded_binary);
		::cv::waitKey(1);

		return false;
	}

}

void LineDetector::thresholdImageAllChannels(const ::cv::Mat& img,::cv::Mat& thresholded)
{
	// threshold in HSV to remove LED brightness
	::cv::Mat V;
	this->convertImage(img, ::cv::Mat(), ::cv::Mat(), V);
	const int thresh_V = 250;
	
	::cv::Mat mask_v;
	::cv::threshold(V, mask_v, thresh_V, 255, ::cv::THRESH_BINARY_INV);

    ::cv::Mat O1, O2, O3;
    this->RGBtoOpponent(img, O1, O2, O3); 

	double min, max;
    ::cv::minMaxLoc(O1, &min, &max);
	O1 = O1 - min;
	::cv::minMaxLoc(O1, &min, &max);
	O1 /= max;
	O1 *= 255;

    ::cv::minMaxLoc(O2, &min, &max);
	O2 = O2 - min;
	::cv::minMaxLoc(O2, &min, &max);
	O2 /= max;
	O2 *= 255;

    ::cv::minMaxLoc(O3, &min, &max);
	O3 = O3 - min;
	::cv::minMaxLoc(O3, &min, &max);
	O3 /= max;
	O3 *= 255;

	// normalize image
	O1.convertTo(O1, CV_8UC1);
	O2.convertTo(O2, CV_8UC1);
	O3.convertTo(O3, CV_8UC1);

	// histogram equalization
	cv::equalizeHist(O1, O1);
	cv::equalizeHist(O2, O2);
	cv::equalizeHist(O3, O3);

	// invert image
	::cv::minMaxLoc(O1, &min, &max);
	O1 = 255 - O1;
	O2 = 255 - O2;
	O3 = 255 - O3;

	::cv::threshold(O1, O1, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	::cv::threshold(O2, O2, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	::cv::threshold(O3, O3, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

	// mask circle
    ::cv::Mat circle_img = ::cv::Mat::zeros(O1.size(), CV_32FC1);
    ::cv::circle(circle_img, ::cv::Size(O1.size[0]/2, O1.size[1]/2), 100, ::cv::Scalar(255, 255, 255), -1);
	::cv::normalize(circle_img, circle_img,1 , ::cv::NORM_L2);
	circle_img = circle_img * 255;
	circle_img.convertTo(circle_img, CV_8UC1);

	O1 = O1.mul(circle_img);
	O2 = O2.mul(circle_img);
	O3 = O3.mul(circle_img);

	// combine all channels
	::cv::Mat out;
    ::cv::bitwise_and(O1, O2, out);
    ::cv::bitwise_and(out, O3, out);

	//threshold
	::cv::threshold(out, thresholded, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

}

bool LineDetector::convertImage(const cv::Mat &img, cv::Mat& S, cv::Mat& A, ::cv::Mat& V)
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


void LineDetector::thresholdImageWire(const ::cv::Mat& img, ::cv::Mat& out)
{
	::cv::Mat hsv;
	::std::vector<::cv::Mat> hsv_split;

	::cv::cvtColor(img, hsv, CV_BGR2HSV);
	::cv::split(hsv, hsv_split);

    ::cv::Mat mask_h, mask_s, mask_v;
	const int min_h = 30, max_h = 110;
	const int min_s = 1, max_s = 255;
    ::cv::inRange(hsv_split[0] ,min_h,max_h,mask_h);
    ::cv::inRange(hsv_split[1] ,min_s,max_s,mask_s);
    //::cv::inRange(hsv_split[2] ,min_s,max_s,mask_v);

	::cv::bitwise_and(mask_s, mask_h, out); 
	//::cv::bitwise_and(out, mask_h, out);
	::cv::Mat channel_mask = ::cv::Mat::ones(img.rows, img.cols, CV_8UC1)*255;


	// mask the working channel
	// mask the working channel
	//// scope 1
	//::cv::circle(channel_mask, ::cv::Point(29, 149), 40,  0, -1);
	//// scope 2
	//::cv::circle(channel_mask, ::cv::Point(215, 149), 40,  0, -1);
	//// scope 3
	//::cv::circle(channel_mask, ::cv::Point(47, 115), 40,  0, -1);
	//// scope 4
	//::cv::circle(channel_mask, ::cv::Point(43, 116), 46,  0, -1);

	::cv::bitwise_and(out, channel_mask, out); 

    // Apply morphological opening to remove small things
    ::cv::Mat kernel = ::cv::getStructuringElement(::cv::MORPH_ELLIPSE,::cv::Size(5,5));
    ::cv::morphologyEx(out,out,::cv::MORPH_OPEN,kernel);

}


bool 
LineDetector::processImageDemo(::cv::Mat img, cv::Vec4f &line, cv::Vec2f &centroid, bool display, int crop,  LineDetector::MODE mode, ::cv::Mat& thres)
{
    ::cv::Mat img_crop = img(::cv::Rect(crop,crop,img.cols-2*crop, img.rows-2*crop));

	bool lineDetected = false;

	lineDetected = this->detectLineDemo(img, line, centroid, thres);

	if (lineDetected)
	{
		centroid[0] += crop;
		centroid[1] += crop;
	}

    return lineDetected;

}

bool LineDetector::detectLineDemo(const ::cv::Mat img, cv::Vec4f &line, ::cv::Vec2f& centroid, ::cv::Mat& thres)
{
    ::cv::Mat thresholded;
	
    ::cv::Mat thresholded_binary(img.size(), CV_8UC1);

	this->thresholdImageDemo(img, thresholded);
    thresholded.convertTo(thresholded_binary, CV_8UC1);
	
    ::std::vector< ::cv::Point> nonzero;
    ::cv::findNonZero(thresholded_binary, nonzero);

	::cv::Mat	 output;
	::cv::cvtColor(thresholded, output, CV_GRAY2BGR);
	output.copyTo(thres);

	::std::vector<::cv::Vec2f> lines_hough;
	if (nonzero.size() > 50)
	{
        ::cv::fitLine(nonzero,line, CV_DIST_L2, 0, 0.01, 0.01);

		this->computeCentroid(nonzero, centroid);

		::cv::line( output, ::cv::Point(line[2],line[3]), ::cv::Point(line[2]+line[0]*100,line[3]+line[1]*100), ::cv::Scalar(255, 255, 255), 2, CV_AA);
		::cv::line( output, ::cv::Point(line[2],line[3]), ::cv::Point(line[2]+line[0]*(-100),line[3]+line[1]*(-100)), ::cv::Scalar(255, 255, 255), 2, CV_AA);

		::cv::imshow("thresholded", output);
		::cv::waitKey(1);

		return true;
	}
	else 
	{
		::cv::imshow("thresholded", thresholded_binary);
		::cv::waitKey(1);

		return false;
	}
}


void
LineDetector::thresholdImageDemo(const ::cv::Mat& img, ::cv::Mat& out)
{
	::cv::Mat hsv;
	::std::vector<::cv::Mat> hsv_split;

	::cv::cvtColor(img, hsv, CV_BGR2HSV);
	::cv::split(hsv, hsv_split);

    ::cv::Mat mask_h, mask_s, mask_v;
	//const int min_h = 80, max_h = 98;
	const int min_h = 70, max_h = 98;
	//const int min_s = 48, max_s = 80;
	const int min_s = 70, max_s = 150;
    ::cv::inRange(hsv_split[0] ,min_h,max_h,mask_h);
    ::cv::inRange(hsv_split[1] ,min_s,max_s,mask_s);
    //::cv::inRange(hsv_split[2] ,min_s,max_s,mask_v);

	::cv::bitwise_and(mask_s, mask_h, out); 
	//::cv::bitwise_and(out, mask_h, out);
	::cv::Mat channel_mask = ::cv::Mat::ones(img.rows, img.cols, CV_8UC1)*255;

	// mask the working channel
	// scope 1
	::cv::circle(channel_mask, ::cv::Point(25, 149), 50,  0, -1);
	//::cv::circle(channel_mask, ::cv::Point(149, 45), 50,  0, -1); // (this was just for testing)
	//::cv::circle(channel_mask, ::cv::Point(46, 132), 50,  0, -1); // (this was just for testing)
	//::cv::circle(channel_mask, ::cv::Point(30, 132), 50,  0, -1); // (this was just for testing)

	//// scope 2
	//::cv::circle(channel_mask, ::cv::Point(215, 149), 40,  0, -1);
	//// scope 3
	//::cv::circle(channel_mask, ::cv::Point(47, 115), 40,  0, -1);
	//// scope 4
	//::cv::circle(channel_mask, ::cv::Point(43, 116), 46,  0, -1);

	::cv::bitwise_and(out, channel_mask, out); 

    // Apply morphological opening to remove small things
    ::cv::Mat kernel = ::cv::getStructuringElement(::cv::MORPH_ELLIPSE,::cv::Size(5,5));
    ::cv::morphologyEx(out,out,::cv::MORPH_OPEN,kernel);

}
