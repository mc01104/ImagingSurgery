#include "FilterLibrary.h"
#include "Utilities.h"

#include <algorithm>
using namespace RecursiveFilter;

Filter::Filter(int windowSize) : 
	windowSize(windowSize),
	counter(0)

{
}

Filter::~Filter()
{
}


MedianFilter::MedianFilter(int windowSize)
	: Filter(windowSize)
{
	this->sortedData.resize(windowSize);
}

MedianFilter::~MedianFilter()
{
}

double 
MedianFilter::step(double incomingValue)
{
	this->updateDataBuffer(incomingValue);
	
	if (this->data.size() >= this->windowSize)
		return this->computeMedian();

	return incomingValue;

}

void
Filter::updateDataBuffer(double incomingValue)
{

	if (this->data.size() < this->windowSize)
		this->data.push_back(incomingValue);
	else
	{
		this->data.pop_front();
		this->data.push_back(incomingValue);
	}
}

void
Filter::resetFilter()
{
	this->data.clear();
}

double 
MedianFilter::computeMedian()
{
	::std::copy(this->data.begin(), this->data.end(), this->sortedData.begin());
	::std::sort(this->sortedData.begin(), this->sortedData.end());

	return this->sortedData[static_cast<int> ((this->windowSize - 1)/2)];
}

MovingAverageFilter::MovingAverageFilter(int windowSize)
	:Filter(windowSize)
{
}

MovingAverageFilter::~MovingAverageFilter()
{
}

double 
MovingAverageFilter::step(double incomingValue)
{
	this->updateDataBuffer(incomingValue);
	
	if (this->data.size() >= this->windowSize)
		return computeAverage();

	return incomingValue;

}

double
MovingAverageFilter::computeAverage()
{
	double sum =  0;
	for(::std::deque<double>::iterator it = data.begin(); it != data.end(); ++it)
		sum += *it;
	return sum/data.size();
}


AngularMovingAverageFilter::AngularMovingAverageFilter(int windowSize)	: MovingAverageFilter(windowSize)
{
}

AngularMovingAverageFilter::~AngularMovingAverageFilter()
{
}

double
AngularMovingAverageFilter::computeAverage()
{
	//do stuff here
	double sum_X = 0, sum_Y = 0;
	for (int i = 0; i < this->data.size(); ++i)
	{
		sum_X += ::std::cos(data[i]);
		sum_Y += ::std::sin(data[i]);
	}

	return ::std::atan2(sum_Y/this->data.size(), sum_X/this->data.size());

}

double
AngularMovingAverageFilter::step(double incomingValue)
{
	return MovingAverageFilter::step(incomingValue);
}

RecursiveMovingAverage::RecursiveMovingAverage(int windowSize, double (*distance)(const double, const double))
	:MovingAverageFilter(windowSize), prevValue(0), distance_fun(distance)
{
}

RecursiveMovingAverage::~RecursiveMovingAverage()
{
}

double
RecursiveMovingAverage::step(double incomingValue)
{
	if (this->data.size() == this->windowSize)
	{
		double delta = 0;
		if (this->distance_fun)
			delta = this->distance_fun(incomingValue, this->data.back());
		else 
			delta = incomingValue - this->data.back();
		prevValue = (prevValue * this->windowSize + delta)/this->windowSize;

		this->updateDataBuffer(incomingValue);
	}
	else
		prevValue = MovingAverageFilter::step(incomingValue);

	return prevValue;
}