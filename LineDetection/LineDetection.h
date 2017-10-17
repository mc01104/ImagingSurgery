// general includes
#include <vector>
#include <string>
#include <exception>
#include <cmath>

// openCV
#include <opencv2/opencv.hpp>


class LineDetector
{
	public:
		enum MODE
		{
			TRANSITION,
			CIRCUM
		} mode;

		enum ALGORITHM
		{
			LSQ,
			HOUGH,
			RANSAC
		} lineMode;

		LineDetector();
		~LineDetector();
        bool processImage(::cv::Mat img, bool display=false, int crop = 12);
		bool processImage(::cv::Mat img, cv::Vec4f &line, cv::Vec2f &centroid, bool display=false, int crop = 30,  LineDetector::MODE mode = LineDetector::MODE::CIRCUM);
		bool processImageSynthetic(::cv::Mat img, ::cv::Vec4f& line,cv::Vec2f &centroid, bool display=false, int crop = 20);
		bool processImageDemo(::cv::Mat img, cv::Vec4f &line, cv::Vec2f &centroid, bool display=false, int crop = 30,  LineDetector::MODE mode = LineDetector::MODE::CIRCUM, ::cv::Mat& thres = ::cv::Mat(), LineDetector::ALGORITHM linemode = LineDetector::ALGORITHM::LSQ);
		bool processImageDemoHough(::cv::Mat img, ::std::vector<::cv::Vec2f>& lines, ::cv::Vec2f& centroid, ::cv::Mat& thres);
		bool processImageDemoHoughProbabilistic(::cv::Mat img, ::std::vector<::cv::Vec4i>& lines, ::cv::Vec2f& centroid, ::cv::Mat& thres, ::cv::Vec4f& line);
		bool processImageDemoRANSAC(::cv::Mat img, cv::Vec4f &line, cv::Vec2f &centroid, ::cv::Mat& thres);
		bool getCentroid(const ::cv::Mat& img, ::cv::Point& centroid);
private:
        bool RGBtoOpponent(const ::cv::Mat &img, ::cv::Mat &O1, ::cv::Mat &O2, ::cv::Mat &O3);
        bool detectLine(const ::cv::Mat img, cv::Vec4f &line, ::cv::Vec2f& centroid = ::cv::Vec2f());
		void thresholdImage(const cv::Mat &img, ::cv::Mat &out);
		void computeCentroid(::std::vector< ::cv::Point>& nonzero, ::cv::Vec2f& centroid);

		// for benchtop testing
		void thresholdImageSynthetic(const cv::Mat &img, ::cv::Mat &out);
        bool detectLineSynthetic(const ::cv::Mat img, cv::Vec4f &line, ::cv::Vec2f& centroid = ::cv::Vec2f());

		bool detectLineAllChannels(const ::cv::Mat img, cv::Vec4f &line, ::cv::Vec2f& centroid = ::cv::Vec2f());
		void thresholdImageAllChannels(const ::cv::Mat& img,::cv::Mat& thresholded);
		bool convertImage(const cv::Mat &img, cv::Mat& S, cv::Mat& A, ::cv::Mat& V);

		void thresholdImageWire(const ::cv::Mat& img, ::cv::Mat& out);
		void thresholdImageDemo(const ::cv::Mat& img, ::cv::Mat& out);
		bool detectLineDemo(const ::cv::Mat img, cv::Vec4f &line, ::cv::Vec2f& centroid, ::cv::Mat& thres);
		bool detectLineDemoHough(const ::cv::Mat img, ::std::vector<::cv::Vec2f>& lines, ::cv::Vec2f& centroid, ::cv::Mat& thres);
		bool detectLineDemoHoughProbabilistic(const ::cv::Mat img, ::std::vector<::cv::Vec4i>& lines, ::cv::Vec2f& centroid, ::cv::Mat& thres);
		bool detectLineDemoRANSAC(const ::cv::Mat img, ::cv::Vec4f& line, ::cv::Vec2f& centroid, ::cv::Mat& thres);

		void averageTangent(::std::vector<::cv::Vec4i>& lines, ::cv::Vec4f& line);
		void averageTangentPCA(::std::vector<::cv::Vec4i>& lines, ::cv::Vec4f& line);
};


// helper function to find nonzero elements in a Mat
void findNonZero(const cv::Mat& binary, std::vector<cv::Point> &idx);
