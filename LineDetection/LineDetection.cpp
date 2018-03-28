#include "stdafx.h"
#include "LineDetection.h"
#include <Eigen/Dense>
#include "Utilities.h"
#define SCOPE_2

const double pi = 3.1415926535897;

LineDetector::LineDetector() : counter(0)
{
	mode = MODE::CIRCUM;

	/// Initialize values
    min_h = 92;
	max_h = 135;

    min_s = 40;
	max_s = 255;

	min_v = 1;
	max_v = 220;

	sliderValueHueMin = min_h;
	sliderValueHueMax = max_h;

	sliderValueSatMin = min_s;
	sliderValueSatMax = max_s;

	sliderValueValMin = min_v;
	sliderValueValMax = max_v;
	
	int crop = 5;
	img_crop = ::cv::Mat(250 - 2  * crop, 250 - 2 * crop, CV_8UC3);
	thresholded_binary = ::cv::Mat(img_crop.size(),CV_8UC1);
	
	channel_mask = ::cv::Mat::zeros(250 - 2  * crop, 250 - 2 * crop, CV_8UC1);
	::cv::circle(channel_mask, ::cv::Point(channel_mask.rows/2.0, channel_mask.cols/2.0), channel_mask.rows/2.0,  255, -1);

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
    img_crop = img(::cv::Rect(crop,crop,img.cols-2*crop, img.rows-2*crop));

	bool lineDetected = false;

	if (this->counter < 1)
		this->initializeTrackbars();

	this->counter++;

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

	//if (nonzero.size() > 400)
	//{
 //       ::cv::fitLine(nonzero,line, CV_DIST_L2, 0, 0.01, 0.01);

	//	this->computeCentroid(nonzero, centroid);

	//	return true;
	//}
	::std::vector<::cv::Vec4i> lines;

	if (nonzero.size() > 50)
	{
        /*::cv::fitLine(nonzero,line, CV_DIST_L2, 0, 0.01, 0.01);*/
		::cv::HoughLinesP(thresholded_binary, lines, 1, 1 * pi/180.0, 70, 25, 3);
		if (lines.size() > 0)
		{
			this->averageTangentPCA(lines, line);
		}
		else 
			return false;
		//this->averageTangentPCA(lines, line);
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

    ::cv::Mat img_blur;
    ::cv::GaussianBlur(img, img_blur, ::cv::Size(15,15), 0);

	this->thresholdImageWire(img_blur, thresholded);


	int erosion_type = ::cv::MORPH_ELLIPSE; 
	int erosion_size = 3;
	::cv::Mat element = ::cv::getStructuringElement( erosion_type,
						::cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
						::cv::Point( erosion_size, erosion_size ) );

	/// Apply the erosion operation
	::cv::erode( thresholded, thresholded, element );

    thresholded.convertTo(thresholded_binary,CV_8UC1);

    ::cv::findNonZero(thresholded_binary, this->nonzero);

	::cv::cvtColor(thresholded, output, CV_GRAY2BGR);
	::cv::imshow("thresholded", output);
	::cv::waitKey(1);

	if (nonzero.size() > 50)
	{
		::cv::HoughLinesP(thresholded_binary, lines_hough, 1, 1 * pi/180.0, 30, 15, 2);

		if (lines_hough.size() <= 0)
			return false;

		this->averageTangentPCA(lines_hough, line);

		this->computeCentroid(nonzero, centroid);

		return true;
	}
	else 
		return false;
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

	::cv::cvtColor(img, hsv, CV_BGR2HSV);
	::cv::split(hsv, hsv_split);

    ::cv::inRange(hsv_split[0], min_h, max_h, mask_h);
    ::cv::inRange(hsv_split[1], min_s, max_s, mask_s);
    ::cv::inRange(hsv_split[2], min_v, max_v, mask_v);

	::cv::bitwise_and(mask_s, mask_h, out); 
	::cv::bitwise_and(mask_v, out, out);

	::cv::bitwise_and(out, channel_mask, out); 

    // Apply morphological opening to remove small things
    this->kernel = ::cv::getStructuringElement(::cv::MORPH_ELLIPSE, ::cv::Size(5,5));
    ::cv::morphologyEx(out, out, ::cv::MORPH_OPEN, this->kernel);

}


bool 
LineDetector::processImageDemo(::cv::Mat img, cv::Vec4f &line, cv::Vec2f &centroid, bool display, int crop,  LineDetector::MODE mode, ::cv::Mat& thres, LineDetector::ALGORITHM linemode)
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


bool 
LineDetector::detectLineDemoHough(const ::cv::Mat img, ::std::vector<::cv::Vec2f>& lines, ::cv::Vec2f& centroid, ::cv::Mat& thres)
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

	if (nonzero.size() > 50)
	{
		//::cv::HoughLines(thresholded_binary, lines, 1, 1 * pi/180.0, 150);
		::cv::HoughLinesP(thresholded_binary, lines, 1, 1 * pi/180.0, 150, 15, 10);
		this->computeCentroid(nonzero, centroid);

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

bool 
LineDetector::processImageDemoHough(::cv::Mat img, ::std::vector<::cv::Vec2f>& lines, ::cv::Vec2f& centroid, ::cv::Mat& thres)
{
	int crop = 5;
    ::cv::Mat img_crop = img(::cv::Rect(crop,crop,img.cols-2*crop, img.rows-2*crop));

	bool lineDetected = false;
	
	lineDetected = this->detectLineDemoHough(img, lines, centroid, thres);

	if (lineDetected)
	{
		centroid[0] += crop;
		centroid[1] += crop;
	}

    return lineDetected;

}

bool 
LineDetector::detectLineDemoHoughProbabilistic(const ::cv::Mat img, ::std::vector<::cv::Vec4i>& lines, ::cv::Vec2f& centroid, ::cv::Mat& thres)
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

	if (nonzero.size() > 50)
	{
		//::cv::HoughLines(thresholded_binary, lines, 1, 1 * pi/180.0, 150);
		::cv::HoughLinesP(thresholded_binary, lines, 1, 1 * pi/180.0, 70, 25, 3);
		this->computeCentroid(nonzero, centroid);

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

bool 
LineDetector::processImageDemoHoughProbabilistic(::cv::Mat img, ::std::vector<::cv::Vec4i>& lines, ::cv::Vec2f& centroid, ::cv::Mat& thres, ::cv::Vec4f& line)
{
	int crop = 5;
    ::cv::Mat img_crop = img(::cv::Rect(crop,crop,img.cols-2*crop, img.rows-2*crop));

	bool lineDetected = false;
	
	lineDetected = this->detectLineDemoHoughProbabilistic(img, lines, centroid, thres);
	//this->averageTangent(lines, line);
	if (lines.size() > 0)
		this->averageTangentPCA(lines, line);

	if (lineDetected)
	{
		centroid[0] += crop;
		centroid[1] += crop;
	}

    return lineDetected;

}


bool 
LineDetector::processImageDemoRANSAC(::cv::Mat img, cv::Vec4f &line, cv::Vec2f &centroid, ::cv::Mat& thres)
{
	int crop = 5;
    ::cv::Mat img_crop = img(::cv::Rect(crop,crop,img.cols-2*crop, img.rows-2*crop));

	bool lineDetected = false;
	
	lineDetected = this->detectLineDemoRANSAC(img, line, centroid, thres);

	if (lineDetected)
	{
		centroid[0] += crop;
		centroid[1] += crop;
	}

    return lineDetected;
}

bool 
LineDetector::detectLineDemoRANSAC(const ::cv::Mat img, ::cv::Vec4f& line, ::cv::Vec2f& centroid, ::cv::Mat& thres)
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

	::std::vector<::Eigen::Vector2d> tmp;
	::Eigen::Vector2d tmp_vector;

	// more efficient way to do that or even eliminate it
	for (int i = 0; i < nonzero.size(); ++i)
	{
		tmp_vector(0) = nonzero[i].x;
		tmp_vector(1) = nonzero[i].y;
		tmp.push_back(tmp_vector);
	}

	double radius = 0, angle = 0;
	double alpha = 0, beta = 0;
	if (nonzero.size() > 50)
	{
		//::cv::HoughLines(thresholded_binary, lines, 1, 1 * pi/180.0, 150);
		ransac(tmp, 2000, alpha, beta, radius, angle);
		this->computeCentroid(nonzero, centroid);

		::cv::imshow("thresholded", output);
		::cv::waitKey(1);
		line[0] = radius;

		line[1] = angle;
		line[2] = centroid[0];
		line[3] = centroid[1];
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
LineDetector::averageTangent(::std::vector<::cv::Vec4i>& lines, ::cv::Vec4f& line)
{
	double sum_x = 0, sum_y = 0;
	double x = 0, y = 0, norm = 1;
	for (int i = 0; i < lines.size(); ++i)
	{
		// compute each line direction
		x = lines[i][2] - lines[i][0];
		y = lines[i][3] - lines[i][1];
		
		norm = ::std::sqrt(x*x + y*y);
		x /= norm; y /= norm;
		sum_x += x; sum_y += y;
	}

	line[0] = sum_x/lines.size();
	line[1] = sum_y/lines.size();
}

void 
LineDetector::averageTangentPCA(::std::vector<::cv::Vec4i>& lines, ::cv::Vec4f& line)
{
	::Eigen::MatrixXd data;
	::Eigen::Vector2d tmp;
	for (int i = 0; i < lines.size(); ++i)
	{
		tmp(0) = lines[i][2] - lines[i][0];
		tmp(1) = lines[i][3] - lines[i][1];
		tmp.normalize();

		appendRowEigen(data, tmp);

		tmp(0) *= -1;
		tmp(1) *= -1;

		appendRowEigen(data, tmp);
	}

	::Eigen::JacobiSVD<::Eigen::MatrixXd> svd(data.transpose() * data, ::Eigen::ComputeThinU | ::Eigen::ComputeThinV);

	line(0) = svd.matrixU()(0, 0);
	line(1) = svd.matrixU()(1, 0);

}

bool 
LineDetector::getCentroid(const ::cv::Mat& img, ::cv::Point& centroid)
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

	if (nonzero.size() <= 0)
		return false;

	::cv::Vec2f tmp;
	this->computeCentroid(nonzero, tmp);

	centroid.x = tmp[0];
	centroid.y = tmp[1];

	return true;

}


void LineDetector::onTrackbarChangeHL(int newValue, void * object)
{
	LineDetector* localObj = reinterpret_cast<LineDetector*> (object);

	localObj->min_h = newValue;

}

void LineDetector::onTrackbarChangeHH(int newValue, void * object)
{
	LineDetector* localObj = reinterpret_cast<LineDetector*> (object);

	localObj->max_h = newValue;

}

void LineDetector::onTrackbarChangeSL(int newValue, void * object)
{
	LineDetector* localObj = reinterpret_cast<LineDetector*> (object);

	localObj->min_s = newValue;

}

void LineDetector::onTrackbarChangeSH(int newValue, void * object)
{
	LineDetector* localObj = reinterpret_cast<LineDetector*> (object);

	localObj->max_s = newValue;

}

void LineDetector::onTrackbarChangeVL(int newValue, void * object)
{
	LineDetector* localObj = reinterpret_cast<LineDetector*> (object);

	localObj->min_v = newValue;

}

void LineDetector::onTrackbarChangeVH(int newValue, void * object)
{
	LineDetector* localObj = reinterpret_cast<LineDetector*> (object);

	localObj->max_v = newValue;

}

void LineDetector::initializeTrackbars()
{
	::cv::namedWindow("line thresholds", 1);

    /// Create Trackbars
    ::cv::createTrackbar("Hue_min", "line thresholds", &sliderValueHueMin, 180, &LineDetector::onTrackbarChangeHL, this);
	::cv::createTrackbar("Hue_max", "line thresholds", &sliderValueHueMax, 180, &LineDetector::onTrackbarChangeHH, this);

    ::cv::createTrackbar("Sat_min", "line thresholds", &sliderValueSatMin, 255, &LineDetector::onTrackbarChangeSL, this);
	::cv::createTrackbar("Sat_max", "line thresholds", &sliderValueSatMax, 255, &LineDetector::onTrackbarChangeSH, this);

	::cv::createTrackbar("Val_min", "line thresholds", &sliderValueValMin, 255, &LineDetector::onTrackbarChangeVL, this);
	::cv::createTrackbar("Val_max", "line thresholds", &sliderValueValMax, 255, &LineDetector::onTrackbarChangeVH, this);

}