#include "stdafx.h"

#define _USE_MATH_DEFINES
#include <cmath>

#include "OpenCVClock.h"


OpenCVClock::OpenCVClock() 
	: clockCenter(50, 50), radius(20), hourClockHand(12), registrationOffset(0), regCounter(0), initialOffset(0)
{
	this->updateTicks();
}

OpenCVClock::OpenCVClock(::cv::Point& clockCenter, double radius, double currentTime)
{
	this->clockCenter = clockCenter;
	
	this->radius = radius;

	this->hourClockHand = currentTime;

	this->registrationOffset = 0;

	this->updateTicks();
	
	this->regCounter = 0;
}

OpenCVClock::~OpenCVClock()
{
}

void 
OpenCVClock::setCurrentTime(double time)
{
	this->hourClockHand = time;

	this->updateClockHands();
}

void 
OpenCVClock::setRegistrationOffset(double offset, double marker)
{
	this->visited_markers.push_back(marker);

	this->registrationOffset += offset;

	this->regCounter++;

	this->updateRegistration();

}

void 
OpenCVClock::update(::cv::Mat& img, double time)
{
	this->setCurrentTime(time + this->registrationOffset);
	this->updateRegistration();

	// draw circle for clock body
	::cv::circle(img, this->clockCenter, this->radius, ::cv::Scalar(244, 69, 66), 2);

	// draw the hour marks
	::std::vector<Tick>::const_iterator it = this->ticks.begin();
	for (it; it != this->ticks.end(); ++it)
		::cv::line(img, it->first, it->second, ::cv::Scalar(244, 69, 66), 2);

	// draw clock hand
	::cv::arrowedLine(img, this->clockCenter, ::cv::Point(this->hourVector[0], this->hourVector[1]), ::cv::Scalar(229, 244, 65), 2);

	// draw registration mark
	::cv::line(img, this->registrationMark.first, this->registrationMark.second, ::cv::Scalar(0, 0, 255), 2);

	this->renderRegistrationCounter(img);
}

void 
OpenCVClock::updateTicks()
{
	this->ticks.clear();
	::cv::Point point;
	Tick tmpTick;
	for (int i = 0; i < 12; ++i)
	{
		point.x = this->clockCenter.x + 0.9 * this->radius * cos(i * 30.0 * M_PI/180.0);
		point.y = this->clockCenter.y + 0.9 * this->radius * sin(i * 30.0 * M_PI/180.0);
		tmpTick.first = point;

		point.x = this->clockCenter.x + 1.1 * this->radius * cos(i * 30.0 * M_PI/180.0);
		point.y = this->clockCenter.y + 1.1 * this->radius * sin(i * 30.0 * M_PI/180.0);
		tmpTick.second = point;

		this->ticks.push_back(tmpTick);
	}
}

void 
OpenCVClock::updateRegistration()
{
	double angle = this->registrationOffset * 30 - 90; 
	::cv::Point point;
	point.x = this->clockCenter.x + 0.9 * this->radius * cos(angle * M_PI/180.0);
	point.y = this->clockCenter.y + 0.9 * this->radius * sin(angle * M_PI/180.0);
	this->registrationMark.first = point;

	point.x = this->clockCenter.x + 1.1 * this->radius * cos(angle * M_PI/180.0);
	point.y = this->clockCenter.y + 1.1 * this->radius * sin(angle * M_PI/180.0);
	this->registrationMark.second = point;
}

void 
OpenCVClock::updateClockHands()
{
	double angle = this->hourClockHand * 30 - 90;
	hourVector[0] = this->clockCenter.x + this->radius * cos(angle * M_PI/180.0);
	hourVector[1] = this->clockCenter.y + this->radius * sin(angle * M_PI/180.0);
}

void 
OpenCVClock::renderRegistrationCounter(::cv::Mat& img)
{
	double markRadius = 3;
	if (this->regCounter == 1)
	{
		::cv::circle(img, ::cv::Point(this->clockCenter.x + 1.5 * this->radius, this->clockCenter.y - (this->radius - markRadius))
			, markRadius, ::cv::Scalar(255, 0, 0), -1);
		::cv::putText(img, ::std::to_string(this->visited_markers[this->regCounter - 1]), 
			::cv::Point(this->clockCenter.x + 2.0 * this->radius, this->clockCenter.y - (this->radius - 2 * markRadius)),
			::cv::FONT_HERSHEY_PLAIN, 1.0, ::cv::Scalar(255, 0, 0));
	}
	else if (this->regCounter == 2)
	{
		::cv::circle(img, ::cv::Point(this->clockCenter.x + 1.5 * this->radius, this->clockCenter.y - (this->radius - markRadius))
			, markRadius, ::cv::Scalar(255, 0, 0), -1);
		::cv::circle(img, ::cv::Point(this->clockCenter.x + 1.5 * this->radius, this->clockCenter.y - 2 * markRadius)
			, markRadius, ::cv::Scalar(255, 0, 0), -1);
		::cv::putText(img, ::std::to_string(this->visited_markers[this->regCounter - 2]), 
			::cv::Point(this->clockCenter.x + 2.0 * this->radius, this->clockCenter.y - (this->radius - 2 * markRadius)),
			::cv::FONT_HERSHEY_PLAIN, 1.0, ::cv::Scalar(255, 0, 0));
		::cv::putText(img, ::std::to_string(this->visited_markers[this->regCounter - 1]), 
			::cv::Point(this->clockCenter.x + 2.0 * this->radius, this->clockCenter.y),
			::cv::FONT_HERSHEY_PLAIN, 1.0, ::cv::Scalar(255, 0, 0));

	}
	else if (this->regCounter == 3)
	{
		::cv::circle(img, ::cv::Point(this->clockCenter.x + 1.5 * this->radius, this->clockCenter.y - (this->radius - markRadius))
			, markRadius, ::cv::Scalar(255, 0, 0), -1);
		::cv::circle(img, ::cv::Point(this->clockCenter.x + 1.5 * this->radius, this->clockCenter.y)
			, markRadius, ::cv::Scalar(255, 0, 0), -1);
		::cv::circle(img, ::cv::Point(this->clockCenter.x + 1.5 * this->radius, this->clockCenter.y + (this->radius - markRadius))
			, markRadius, ::cv::Scalar(255, 0, 0), -1);
		::cv::putText(img, ::std::to_string(this->visited_markers[this->regCounter - 3]), 
			::cv::Point(this->clockCenter.x + 2.0 * this->radius, this->clockCenter.y - (this->radius - 2 * markRadius)),
			::cv::FONT_HERSHEY_PLAIN, 1.0, ::cv::Scalar(255, 0, 0));
		::cv::putText(img, ::std::to_string(this->visited_markers[this->regCounter - 2]), 
			::cv::Point(this->clockCenter.x + 2.0 * this->radius, this->clockCenter.y),
			::cv::FONT_HERSHEY_PLAIN, 1.0, ::cv::Scalar(255, 0, 0));

		::cv::putText(img, ::std::to_string(this->visited_markers[this->regCounter - 1]), 
			::cv::Point(this->clockCenter.x + 2.0 * this->radius, this->clockCenter.y + (this->radius - 2 * markRadius)),
			::cv::FONT_HERSHEY_PLAIN, 1.0, ::cv::Scalar(255, 0, 0));

	}
}

void OpenCVClock::setInitialOffset(double initialOffset)
{
	double tmp = this->registrationOffset - this->initialOffset;
	this->initialOffset = initialOffset;

	this->registrationOffset = tmp + this->initialOffset;

	this->updateRegistration();

}

void OpenCVClock::reset()
{
		this->registrationOffset = 0;

		this->regCounter = 0;

		this->updateRegistration();

		this->visited_markers.clear();

		this->initialOffset  = 0;
}