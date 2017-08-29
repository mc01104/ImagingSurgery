#include "Utilities.h"
#include <iostream>
#include <iterator>
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

::std::vector< double> DoubleVectorFromString(const ::std::string& inputString, char delim)
{
	::std::vector<::std::string> strs = splitString(inputString, delim);
	
	::std::vector<double> result;

	for (int i = 0; i < strs.size(); ++i)
		result.push_back(atof(strs[i].c_str()));

	return result;
}

std::vector<std::string> splitString(const std::string &s, char delim) 
{
    std::vector<std::string> elems;
    splitString(s, delim, std::back_inserter(elems));
    return elems;
}

::std::vector<::std::string> splitString(const ::std::string& inputStr)
{
	::std::istringstream ss(inputStr);

	::std::vector<::std::string> result;
	while(!ss.eof())
	{
		::std::string tmp;
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

void distancePointToLine(const ::Eigen::VectorXd& point, const ::Eigen::VectorXd& point_on_line, const ::Eigen::VectorXd& line_tangent, double& distance)
{
	::Eigen::VectorXd closest_point;
	nearestPointToLine(point, point_on_line, line_tangent, closest_point);

	distance = (closest_point - point).norm();
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

bool is_not_digit(char c)
{
    return !std::isdigit(c);
}

bool numeric_string_compare(const std::string& s1, const std::string& s2)
{
    // handle empty strings...

	const std::string s1_ = s1.substr(0, s1.find_last_of("."));
	const std::string s2_ = s2.substr(0, s2.find_last_of("."));

    std::string::const_iterator it1 = s1_.begin(), it2 = s2_.begin();

    if (std::isdigit(s1_[0]) && std::isdigit(s2_[0])) {

		double n1 = 0;
		std::istringstream ss(s1_);
		ss >> n1;

		double n2 = 0;
		std::istringstream ss2(s2_);
		ss2 >> n2;

        if (n1 != n2) return n1 < n2;

        it1 = std::find_if(s1_.begin(), s1_.end(), is_not_digit);
        it2 = std::find_if(s2_.begin(), s2_.end(), is_not_digit);
    }

    return std::lexicographical_compare(it1, s1_.end(), it2, s2_.end());
}

void computePerpendicularVector(const ::Eigen::Vector2d& in_vector, ::Eigen::Vector2d& out_vector)
{
	::Eigen::Vector2d tmp(in_vector);
	tmp.normalize();

	double radius, angle;
	cartesian2DPointToPolar(tmp, radius, angle);

	if (::std::sin(angle) == 0)
	{
		out_vector(0) = 0;
		out_vector(1) = 1;
	}
	else
	{
		out_vector(0) = 1;
		out_vector(1) = -::std::cos(angle)/::std::sin(angle);
		out_vector.normalize();
	}

}

void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove)
{
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-1;

    if( colToRemove < numCols )
        matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);

    matrix.conservativeResize(numRows,numCols);
}

void removeRowEigen(Eigen::MatrixXd& matrix, unsigned int rowToRemove)
{
    unsigned int numRows = matrix.rows() - 1;
    unsigned int numCols = matrix.cols();

    if( rowToRemove < numRows )
        matrix.block(rowToRemove, 0, numRows - rowToRemove,numCols) = matrix.block(rowToRemove + 1,0,numRows - rowToRemove, numCols);

    matrix.conservativeResize(numRows,numCols);
}

void appendColumn(::Eigen::MatrixXd& matrix, ::Eigen::VectorXd& columnToAppend)
{
	assert(matrix.rows() == columnToAppend.rows());

	matrix.conservativeResize(matrix.rows(), matrix.cols() + 1);
	matrix.col(matrix.cols() - 1) = columnToAppend;
}

void appendRowEigen(::Eigen::MatrixXd& in_matrix, const ::Eigen::VectorXd& vector_to_add)
{
	if (in_matrix.size() == 0)
	{
		in_matrix.resize(1, vector_to_add.size());
		in_matrix = vector_to_add.transpose();
	}
	else
	{
		assert(in_matrix.cols() == vector_to_add.size());

		in_matrix.conservativeResize(in_matrix.rows() + 1, in_matrix.cols());
		in_matrix.row(in_matrix.rows() - 1) = vector_to_add.transpose();
	}
}

void popFirstRowEigen(::Eigen::MatrixXd& in_matrix)
{
	removeRowEigen(in_matrix, 0);
}

::std::map<::std::string, double>  createMapFromKeyValuePairs(const ::std::string& msgToParse)
{
	::std::vector<::std::string> strVector = splitString(msgToParse);

	assert(strVector.size() % 2 == 0);

	::std::map<::std::string, double> result;
	for (int i = 0; i < strVector.size(); ++i)
		result[strVector[i].c_str()] = (double) atof(strVector[++i].c_str());

	return result;
}