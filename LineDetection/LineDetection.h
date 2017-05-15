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
		bool processImage(::cv::Mat img, cv::Vec4f &line, cv::Vec2f &centroid, bool display=false, int crop = 12);

	private:
        bool RGBtoOpponent(const ::cv::Mat &img, ::cv::Mat &O1, ::cv::Mat &O2, ::cv::Mat &O3);
        bool detectLine(const ::cv::Mat img, cv::Vec4f &line, ::cv::Vec2f& centroid = ::cv::Vec2f());
		void thresholdImage(const cv::Mat &img, ::cv::Mat &out);
		void computeCentroid(::std::vector< ::cv::Point>& nonzero, ::cv::Vec2f& centroid);
};


// helper function to find nonzero elements in a Mat
void findNonZero(const cv::Mat& binary, std::vector<cv::Point> &idx);