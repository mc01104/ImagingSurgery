#include "stdafx.h"
#include "LineDetection.h"

LineDetector::LineDetector()
{
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

	if (false)
	{
        if (lineDetected)
        {
            ::cv::line( img, ::cv::Point(line[2],line[3]), ::cv::Point(line[2]+line[0]*100,line[3]+line[1]*100), ::cv::Scalar(0, 255, 0), 2, CV_AA);
            ::cv::line( img, ::cv::Point(line[2],line[3]), ::cv::Point(line[2]+line[0]*(-100),line[3]+line[1]*(-100)), ::cv::Scalar(0, 255, 0), 2, CV_AA);
            //::cv::line( img, ::cv::Point(line[3],line[2]), ::cv::Point(line[3]+line[1]*(-50),line[2]+line[0]*(-50)), ::cv::Scalar(0, 255, 0), 2, CV_AA);
        }

        ::cv::imshow("Fitted line", img);
        ::cv::waitKey(0);
	}

    return lineDetected;

}

bool LineDetector::processImage(::cv::Mat img, ::cv::Vec4f& line,cv::Vec2f &centroid, bool display, int crop)
{

    ::cv::Mat img_crop = img(::cv::Rect(crop,crop,img.cols-2*crop, img.rows-2*crop));

	bool lineDetected = false;

  //  if (this->detectLine(img_crop,line, centroid))
		//lineDetected = true;

    if (this->detectLineAllChannels(img_crop,line, centroid))
		lineDetected = true;

	centroid[0] += crop;
	centroid[1] += crop;

    return lineDetected;

}


void LineDetector::thresholdImage(const cv::Mat &img, ::cv::Mat &out)
{
    ::cv::Mat O2,O3;
    this->RGBtoOpponent(img,out,O2,O3); //out is the O1 layer directly

	// Threshold luminance to keep the 95th percentile
	// This avoids high-luminance parasite reflections
    double min, max;
    ::cv::Mat O3_mask;
    ::cv::minMaxLoc(O3, &min, &max);
    ::cv::threshold(O3,O3_mask,max - 0.05*(max-min), 1, ::cv::ThresholdTypes::THRESH_BINARY_INV);


	// Threshold to zero O1 pixels >0 and to 255 01 pixels <0
	// 3 is threshold binary inverse
	::cv::threshold( out, out, 0, 255,::cv::ThresholdTypes::THRESH_BINARY_INV);
	           
    // Apply O3 mask after, not before !
    out = out.mul(O3_mask);

    ::cv::Mat O2_mask;
    ::cv::threshold(O2,O2_mask,0, 1, ::cv::ThresholdTypes::THRESH_BINARY_INV);
	out = out.mul(O2_mask);

}

bool LineDetector::RGBtoOpponent(const ::cv::Mat &img, ::cv::Mat &O1, ::cv::Mat &O2, ::cv::Mat &O3)
{
    // ideally todo here is an assert for datatype and size of input ...

	::std::vector< ::cv::Mat> channels;
	::cv::split(img,channels);

    const int rows = img.rows;
    const int cols = img.cols;
    O1 = cv::Mat(img.size(),CV_32FC1);
    O2 = cv::Mat(img.size(),CV_32FC1);
    O3 = cv::Mat(img.size(),CV_32FC1);

    for(int r=0;r<rows;r++)
    {
        for(int c=0;c<cols;c++)
        {
            const cv::Vec3b &bgr = img.at<cv::Vec3b>(r,c);
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
    thresholded.convertTo(thresholded_binary,CV_8UC1);
	
    ::std::vector< ::cv::Point> nonzero;
    ::cv::findNonZero(thresholded_binary, nonzero);

	::cv::namedWindow("thresholded", 0);
	::cv::imshow("thresholded", thresholded_binary);
	::cv::waitKey(1);

	if (nonzero.size()>80)
	{
        ::cv::fitLine(nonzero,line, CV_DIST_L2, 0, 0.01, 0.01);

		this->computeCentroid(nonzero, centroid);
		return true;
	}

	else return false;
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
    if (nonzero.size()>80)
	{
        ::cv::fitLine(nonzero,line, CV_DIST_L2, 0, 0.01, 0.01);

		this->computeCentroid(nonzero, centroid);
		::cv::namedWindow("thresholded", 0);
		::cv::line( thresholded_binary, ::cv::Point(centroid(0), centroid(1)), ::cv::Point(centroid(0)+line(0)*100, centroid(1)+line(1)*100), ::cv::Scalar(0, 255, 0), 2, CV_AA);
		::cv::line( thresholded_binary, ::cv::Point(centroid(0), centroid(1)), ::cv::Point(centroid(0)+line(0)*(-100), centroid(1)+line(1)*(-100)), ::cv::Scalar(0, 255, 0), 2, CV_AA);
		::cv::imshow("thresholded", thresholded_binary);
		::cv::waitKey(1);

		return true;
	}

	else return false;
}

bool LineDetector::detectLineAllChannels(const ::cv::Mat img, cv::Vec4f &line, ::cv::Vec2f& centroid)
{
    ::cv::Mat thresholded;
	
    ::cv::Mat thresholded_binary(img.size(),CV_8UC1);

	this->thresholdImageAllChannels(img,thresholded);
    thresholded.convertTo(thresholded_binary,CV_8UC1);
	
    ::std::vector< ::cv::Point> nonzero;
    ::cv::findNonZero(thresholded_binary, nonzero);

	::cv::namedWindow("thresholded", 0);
	::cv::imshow("thresholded", thresholded_binary);
	::cv::waitKey(10);

	if (nonzero.size()>80)
	{
        ::cv::fitLine(nonzero,line, CV_DIST_L2, 0, 0.01, 0.01);

		this->computeCentroid(nonzero, centroid);
		return true;
	}

	else return false;
}

void LineDetector::thresholdImageAllChannels(const ::cv::Mat& img,::cv::Mat& thresholded)
{
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
    ::cv::circle(circle_img, ::cv::Size(O1.size[0]/2, O1.size[1]/2), 125, ::cv::Scalar(255, 255, 255), -1);
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

	//::cv::imshow("O1", O1);
	//::cv::imshow("O2", O2);
	//::cv::imshow("O3", O3);
	//::cv::imshow("out", O1);
	//::cv::waitKey(10);  

}
