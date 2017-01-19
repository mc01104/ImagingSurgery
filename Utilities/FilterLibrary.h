#pragma once
#include <queue>

namespace RecursiveFilter
{
	class Filter
	{
	protected:
		::std::deque<double>		data;
		int							windowSize;
		int							counter;

	public:
	
		Filter(int windowSize);
		virtual ~Filter();
		void	updateDataBuffer(double incomingValue);
		void	resetFilter();
		virtual double step(double incomingValue) = 0;
	};


	class MedianFilter : public Filter
	{
		::std::vector<double>		sortedData;
	public:
		MedianFilter(int windowSize);
		~MedianFilter();

		double		step(double incomingValue);

	private:
		//void		updateDataBuffer(double incomingValue);

		double		computeMedian();
	};


	class MovingAverageFilter : public Filter
	{
	public:
		MovingAverageFilter(int windowSize = 5);
		~MovingAverageFilter();

		double step(double incomingValue);
	private:
		double computeAverage();
	};
};