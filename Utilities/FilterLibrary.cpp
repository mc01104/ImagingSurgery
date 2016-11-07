#include "FilterLibrary.h"
#include "Utilities.h"

#include <algorithm>


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
MedianFilter::updateDataBuffer(double incomingValue)
{

	if (this->data.size() < this->windowSize)
		this->data.push_back(incomingValue);
	else
	{
		this->data.pop_front();
		this->data.push_back(incomingValue);
	}
}

double 
MedianFilter::computeMedian()
{
	::std::copy(this->data.begin(), this->data.end(), this->sortedData.begin());
	::std::sort(this->sortedData.begin(), this->sortedData.end());

	return this->sortedData[static_cast<int> ((this->windowSize - 1)/2)];
}



