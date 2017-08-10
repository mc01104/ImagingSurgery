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
		LineDetector();
		~LineDetector();
        bool processImage(::cv::Mat img, bool display=false, int crop = 12);
		bool processImage(::cv::Mat img, cv::Vec4f &line, cv::Vec2f &centroid, bool display=false, int crop = 0);
		bool processImageSynthetic(::cv::Mat img, ::cv::Vec4f& line,cv::Vec2f &centroid, bool display=false, int crop = 20);

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

};


// helper function to find nonzero elements in a Mat
void findNonZero(const cv::Mat& binary, std::vector<cv::Point> &idx);
