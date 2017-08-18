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
		virtual void	updateDataBuffer(double incomingValue);
		virtual void	resetFilter();
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
		double		computeMedian();
	};


	class MovingAverageFilter : public Filter
	{
	public:
		MovingAverageFilter(int windowSize = 5);
		~MovingAverageFilter();

		double step(double incomingValue);
	protected:
		virtual double computeAverage();
	};

	class AngularMovingAverageFilter : public MovingAverageFilter
	{
		public:
			AngularMovingAverageFilter(int windowSize = 5);
			~AngularMovingAverageFilter();

			double step(double incomingValue);

		protected:
			virtual double computeAverage();
	};


	class RecursiveMovingAverage : public MovingAverageFilter
	{
		double prevValue;
		double (*distance_fun)(const double, const double);
	public:
		RecursiveMovingAverage(int windowSize = 15, double (*distance)(const double, const double) = NULL);
		
		~RecursiveMovingAverage();
		
		void setDistance(double (*distance)(const double, const double)) {distance_fun = distance;};

		virtual double step(double incomingValue);

	};

};