#include "Utilities.h"
#include <iostream>
#include <fstream>
#include <sstream>

::std::vector<::std::string> ReadLinesFromFile(const ::std::string& pathToFile)
{
	::std::vector< ::std::string> linesVector;

	::std::ifstream inputFile(pathToFile.c_str());
	
	::std::string tempLine;
	while(::std::getline(inputFile, tempLine))
		linesVector.push_back(tempLine);

	return linesVector;
}


::std::vector< double> DoubleVectorFromString(const ::std::string& inputString)
{
	::std::istringstream ss(inputString);

	::std::vector<double> result;
	while(!ss.eof())
	{
		double tmp;
		ss >> tmp;
		result.push_back(tmp);
	}

	return result;
}

double Norm2(const ::std::vector< double>& doubleVector)
{
	double tmp = 0;

	for (::std::vector<double>::const_iterator it = doubleVector.begin(); it != doubleVector.end(); ++ it)
		tmp += ::std::pow(*it, 2);

	return ::std::sqrt(tmp);
}

::Eigen::MatrixXd PseudoInverse(const ::Eigen::MatrixXd& matrix)
{
	::Eigen::MatrixXd quad = matrix * matrix.transpose();
	
	if (quad.determinant() == 0)
		throw("matrix is close to singularity!!");

	return matrix.transpose() * quad.inverse();
}

void PseudoInverse(const ::Eigen::MatrixXd& inputMatrix, ::Eigen::MatrixXd& outputMatrix, const ::Eigen::MatrixXd& weight)
{
	Eigen::MatrixXd tmp;
	if (weight.size() > 0)
		tmp = inputMatrix.transpose() * weight.transpose() * weight * inputMatrix;
	else
		tmp = inputMatrix * inputMatrix.transpose();
	tmp = tmp.inverse();
	outputMatrix =  inputMatrix.transpose() * tmp;

}

double NormSquared(const ::std::vector<double>& input)
{
	double sum = 0;
	for(::std::vector<double>::const_iterator it = input.begin(); it != input.end(); ++it)
		sum += ::std::pow(*it, 2);

	return sum;
}

::std::string GetDateString()
{
  time_t rawtime;
  struct tm * timeinfo;

  char buffer [80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime (buffer,80,"%Y-%m-%d-%H-%M-%S", timeinfo);

  return ::std::string(buffer);
}


void PrintCArray(const double* toPrint, size_t size, ::std::ostream& os)
{
	for (size_t i = 0; i < size; ++i)
		os << toPrint[i] << " ";

	os << ::std::endl;

}

void nearestPointToLine(const ::Eigen::VectorXd& point, const ::Eigen::VectorXd& point_on_line, const ::Eigen::VectorXd& line_tangent, ::Eigen::VectorXd& closest_point)
{
	double lambda = (point - point_on_line).transpose() * line_tangent;
	closest_point = point_on_line + lambda * line_tangent;
}

void cartesian2DPointToPolar(const ::Eigen::Vector2d& point_cart, double& radius, double& angle)
{
	angle = ::std::atan2(point_cart(1), point_cart(0));
	radius = point_cart.norm();
}

double angularDistanceMinusPItoPI(const double angle1, const double angle2)
{
	return ::std::atan2(::std::sin(angle1 - angle2), ::std::cos(angle1 - angle2));	
}