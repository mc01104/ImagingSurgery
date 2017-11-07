#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

class OpenCVClock
{
		::cv::Point clockCenter;

		double	radius;

		typedef ::std::pair<::cv::Point, ::cv::Point> Tick;
		::std::vector<Tick> ticks;

		double hourClockHand;

		::cv::Vec4f hourVector;

		double registrationOffset;

		Tick registrationMark;

		int regCounter;
	public:

		OpenCVClock();

		OpenCVClock(::cv::Point& clockCenter, double radius, double currentTime);

		~OpenCVClock();

		void setCurrentTime(double time);

		void setRegistrationOffset(double offset);

		void update(::cv::Mat& img, double time);

	private:

		void updateTicks();

		void updateRegistration();

		void updateClockHands();

		void createTickLines();

		void renderRegistrationCounter(::cv::Mat& img);

};