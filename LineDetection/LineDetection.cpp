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

	if (display)
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

    if (this->detectLine(img_crop,line, centroid))
		lineDetected = true;
	::cv::namedWindow( "line", 0 );

	if (true)
	{
        if (lineDetected)
        {
            ::cv::line( img, ::cv::Point(line[2],line[3]), ::cv::Point(line[2]+line[0]*100,line[3]+line[1]*100), ::cv::Scalar(0, 255, 0), 2, CV_AA);
            ::cv::line( img, ::cv::Point(line[2],line[3]), ::cv::Point(line[2]+line[0]*(-100),line[3]+line[1]*(-100)), ::cv::Scalar(0, 255, 0), 2, CV_AA);
            //::cv::line( img, ::cv::Point(line[3],line[2]), ::cv::Point(line[3]+line[1]*(-50),line[2]+line[0]*(-50)), ::cv::Scalar(0, 255, 0), 2, CV_AA);
      ::cv::imshow("line", img);
    ::cv::waitKey(1);
		}

 
	}

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
    ::cv::threshold(O2,O2_mask,-50, 1, ::cv::ThresholdTypes::THRESH_BINARY_INV);
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
	//::cv::invert(thresholded, thresholded);
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

