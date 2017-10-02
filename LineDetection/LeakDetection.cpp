#include "stdafx.h"
#include "LeakDetection.h"


LeakDetector::LeakDetector()
{
}

LeakDetector::~LeakDetector()
{
}

void 
LeakDetector::processImage(const ::cv::Mat& img, int x, int y)
{
	img.copyTo(this->currentImage);
	::cv::Mat hsv;
	::std::vector<::cv::Mat> hsv_split;

	::cv::cvtColor(this->currentImage, hsv, CV_BGR2HSV);
	::cv::split(hsv, hsv_split);
	this->thresholdImageHSV();

	::cv::Mat labels;
	 ::cv::Mat g;
	int num = ::cv::connectedComponentsWithStats(this->thresholdedImage, labels, g, centroids);
	//::std::cout << centroids << ::std::endl;
	::cv::Mat H, S, V;
	H = hsv_split[0];
	S = hsv_split[1];
	V = hsv_split[2];

	::std::vector<::cv::Point> centers;
	int max_area = 0;
	int ind = 0;
	for (int i = 1; i < num ; ++i)
	{
		if (g.at<int>(i, ::cv::CC_STAT_AREA) > 200 && g.at<int>(i, ::cv::CC_STAT_AREA) < 200000)
		{

			centers.push_back(::cv::Point(centroids.at<double>(i, 0), centroids.at<double>(i, 1)));
			
			//int h_value = (int) H.at<uchar>((int) centroids.at<double>(i, 0), (int) centroids.at<double>(i, 1));
			//int s_value = (int) S.at<uchar>((int) centroids.at<double>(i, 0), (int) centroids.at<double>(i, 1));
			//int v_value = (int) V.at<uchar>((int) centroids.at<double>(i, 0), (int) centroids.at<double>(i, 1));
			//::std::cout << "H: " << h_value << ", S:" << s_value << ", V:" << v_value << ::std::endl;
			if (g.at<int>(i, ::cv::CC_STAT_AREA) > max_area)
				ind = centers.size()-1;
		}
		//::std::cout << "centroid: " << i << " has area:" << g.at<int>(i, ::cv::CC_STAT_AREA) << ::std::endl;
	}
	

	//x = centers[0].x;
	//y = centers[0].y;
	//for (int i = 0; i < centers.size(); ++i)
	//	::cv::circle(img, ::cv::Point(centers[i].x, centers[i].y), 5, ::cv::Scalar(0,255,0), -1);
	//::std::cout << centers.size() << ::std::endl;
	if(centers.size() > 0)
		::cv::circle(img, ::cv::Point(centers[ind].x, centers[ind].y), 5, ::cv::Scalar(0,255,0), -1);
	//::cv::imshow("unrotated", this->currentImage);
	::cv::imshow("leak_detection", this->thresholdedImage);
}

	
void 
LeakDetector::thresholdImage()
{
	::std::vector<::cv::Mat> rgb_split;
	::cv::split(this->currentImage, rgb_split);
	::cv::inRange(rgb_split[2], 180, 215, this->thresholdedImage);
}


void
LeakDetector::thresholdImageHSV()
{
	::cv::Mat hsv;
	::std::vector<::cv::Mat> hsv_split;

	::cv::cvtColor(this->currentImage, hsv, CV_BGR2HSV);
	::cv::split(hsv, hsv_split);

    ::cv::Mat mask_h1, mask_h2, mask_s, mask_v;
	const int min_h1 = 160, max_h1 = 180;
	const int min_h2 = 0, max_h2 = 20;
	const int min_s = 85, max_s = 200;
	const int min_v = 165, max_v = 230;

	::cv::inRange(hsv_split[0], min_h1, max_h1, mask_h1);
    ::cv::inRange(hsv_split[0], min_h2, max_h2, mask_h2);

    ::cv::inRange(hsv_split[1], min_s, max_s, mask_s);
    ::cv::inRange(hsv_split[2], min_v, max_v, mask_v);


	::cv::bitwise_or(mask_h1, mask_h2, this->thresholdedImage); 
	::cv::bitwise_and(this->thresholdedImage, mask_s, this->thresholdedImage);
	::cv::bitwise_and(mask_v, this->thresholdedImage, this->thresholdedImage);
	::cv::Mat channel_mask = ::cv::Mat::ones(this->currentImage.rows, this->currentImage.cols, CV_8UC1)*255;


	// mask the working channel
	// mask the working channel
	//// scope 1
	//::cv::circle(channel_mask, ::cv::Point(29, 149), 40,  0, -1);
	//// scope 2
	//::cv::circle(channel_mask, ::cv::Point(215, 149), 40,  0, -1);
	//// scope 3
	//::cv::circle(channel_mask, ::cv::Point(47, 115), 40,  0, -1);
	// scope 4
	//::cv::circle(channel_mask, ::cv::Point(43, 116), 50,  0, -1);
	//::cv::circle(channel_mask, ::cv::Point(60, 115), 30,  0, -1);
	//::cv::circle(channel_mask, ::cv::Point(46, 115), 30,  0, -1);
	::cv::circle(channel_mask, ::cv::Point(116, 46), 30,  0, -1);
	//::cv::circle(channel_mask, ::cv::Point(20, 115), 30,  0, -1);
	

	::cv::bitwise_and(this->thresholdedImage, channel_mask, this->thresholdedImage); 

    // Apply morphological opening to remove small things
    ::cv::Mat kernel = ::cv::getStructuringElement(::cv::MORPH_ELLIPSE,::cv::Size(25,25));
    ::cv::morphologyEx(this->thresholdedImage, this->thresholdedImage, ::cv::MORPH_OPEN, kernel);
    ::cv::morphologyEx(this->thresholdedImage, this->thresholdedImage, ::cv::MORPH_CLOSE, kernel);

}