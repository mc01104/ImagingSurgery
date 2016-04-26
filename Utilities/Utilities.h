#pragma once
#include "time.h"
#include <string>
#include <vector>
#include <iostream>
#include <Eigen/Dense>

//namespace utilities
//{
	::std::vector< ::std::string> ReadLinesFromFile(const ::std::string& pathToFile);


	::std::vector< double> DoubleVectorFromString(const ::std::string& inputString);


	template <class T>
	::std::vector<T> operator-(const ::std::vector<T>& lhs, const ::std::vector<T>& rhs)
	{
		
		if (lhs.size() != rhs.size())
			throw("this operation requires vector of the same size");

		::std::vector<T> result;

		for (size_t i = 0; i < lhs.size(); ++i)
			result.push_back(lhs[i] - rhs[i]);

		return result;
	};


	template <class T>
	::std::vector<T>& operator/=(::std::vector< T>& lhs, const ::std::vector< T>& rhs)
	{
		
		if (lhs.size() != rhs.size())
			throw("this operation requires vector of the same size");

		for (size_t i = 0; i < lhs.size(); ++i)
		{
			if (rhs[i] == 0.0)
				throw("cannot divide by zero");

			lhs[i] = lhs[i] / rhs[i];
		}

	};

	template <class T>
	::std::vector<T>& operator /= (::std::vector<T>& lhs, const double rhs)
	{
		for (size_t i = 0; i < lhs.size(); ++i)
		{
			if (rhs == 0.0)
				throw("cannot divide by zero");

			lhs[i] = lhs[i] / rhs;
		}
	}

	template <class T>
	::std::vector<T>& operator*=(::std::vector< T>& lhs, const ::std::vector< T>& rhs)
	{
		
		if (lhs.size() != rhs.size())
			throw("this operation requires vector of the same size");

		for (size_t i = 0; i < lhs.size(); ++i)
			lhs[i] = lhs[i] * rhs[i];

	};


	template <class T>
	::std::vector<T> operator/(const ::std::vector< T>& lhs, const ::std::vector< T>& rhs)
	{
		
		if (lhs.size() != rhs.size())
			throw("this operation requires vector of the same size");

		::std::vector<T> result;

		for (size_t i = 0; i < lhs.size(); ++i)
		{
			if (rhs[i] == 0.0)
				throw("cannot divide by zero");

			result.push_back(lhs[i] / rhs[i]);
		}

		return result;
	};


	template <class T>
	void PrintVector(const ::std::vector<T>& vectorToBePrinted)
	{
		for(::std::vector<T>::const_iterator it = vectorToBePrinted.begin(); it !=  vectorToBePrinted.end(); ++it)
			::std::cout << *it << " ";

		::std::cout << ::std::endl;
	};


	double Norm2(const ::std::vector< double>& doubleVector);


	::Eigen::MatrixXd PseudoInverse(const ::Eigen::MatrixXd& matrix);


	void PseudoInverse(const ::Eigen::MatrixXd& inputMatrix, ::Eigen::MatrixXd& outputMatrix, const ::Eigen::MatrixXd& weight = ::Eigen::MatrixXd());


	template <typename Derived>
	::std::ostream& operator<<(::std::ostream& os, const ::Eigen::EigenBase<Derived>& toPrint)
	{
		::Eigen::IOFormat OctaveFmt(::Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
		os << toPrint << ::std::endl;

		return os;
	}


	double NormSquared(const ::std::vector<double>& input);


	template <typename T>
	::std::ostream& operator<<(::std::ostream& os, const ::std::vector<T>& toPrint)
	{
		for (::std::vector<T>::const_iterator it = toPrint.begin(); it != toPrint.end(); ++it)
			os << *it << " ";
		os << ::std::endl;

		return os;
	};


	::std::string GetDateString();

	void PrintCArray(const double* toPrint, size_t size, ::std::ostream& os = ::std::cout);
//}
