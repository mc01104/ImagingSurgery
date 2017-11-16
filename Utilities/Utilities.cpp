#include "Utilities.h"
#include <iostream>
#include <iterator>
#include <fstream>
#include <sstream>
#include <cmath>

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
	result.pop_back(); // HACKY
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
	//::std::cout << "size" << ::std::endl;
	//::std::cout << strVector.size() << ::std::endl;
	//::std::cout << msgToParse << ::std::endl;
	//::std::cout << ::std::endl;
	assert(strVector.size() % 2 == 0);

	::std::map<::std::string, double> result;
	for (int i = 0; i < strVector.size(); ++i)
		result[strVector[i].c_str()] = (double) atof(strVector[++i].c_str());

	return result;
}


::std::random_device rd;

::std::mt19937 mt(rd());

void ransac(const ::std::vector<::Eigen::Vector2d>& points, int max_iter, double& alpha, double& beta, double& radius, double& angle)
{
	int num_of_points = points.size();

	::std::uniform_int_distribution<int> dist(0, num_of_points - 1);

	// best params
	double dist_threshold = 10;
	double inlierRatio = 0.20;   // 0.2

	int inlierNum = 0;
	int bestInNum = 0;										// Best fitting line with largest number of inliers

	::Eigen::Vector2d line, normal_vector, tangent;
	::Eigen::VectorXd lineEig(4), cpoint(2);
	double distance = 0;
	::std::vector<int> inlierId;

	alpha = 0;
	beta = 0;
	radius = 0;
	angle = 0;
	int ind_a = 0, ind_b = 0;
	for (int i = 0; i < max_iter; ++i)
	{
		//inlierId.clear();
		inlierNum = 0;
		// select two points
		ind_a = dist(mt);
		ind_b = dist(mt);
		
		line(0) = points[ind_b](0) - points[ind_a](0);
		line(1) = points[ind_b](1) - points[ind_a](1);
		line.normalize();
		
		lineEig(0) = line(0);
		lineEig(1) = line(1);
		lineEig(2) = points[ind_b](0);
		lineEig(3) = points[ind_b](1);
		normal_vector(0) = -lineEig(1);
		normal_vector(1) = lineEig(0);

		for (int j = 0; j < points.size(); ++j)
		{
			distancePointToLine(points[j], lineEig, normal_vector, distance);

			if (distance < dist_threshold)
				inlierNum++;
				//inlierId.push_back(j);
		}
		//inlierNum = inlierId.size();
		
		if (inlierNum > (int)(inlierRatio * num_of_points) && inlierNum > bestInNum)
		{
			bestInNum = inlierNum;
			alpha = (points[ind_b](1) - points[ind_a](1))/(points[ind_b](0) - points[ind_a](0));
			beta = points[ind_b](0) - alpha * points[ind_a](0);
			nearestPointToLine(::Eigen::Vector2d(0, 0), lineEig.segment(2, 2), line.segment(0, 2), cpoint);
			cartesian2DPointToPolar(cpoint, radius, angle);
			computePerpendicularVector(cpoint, tangent);
			radius = tangent(0);
			angle = tangent(1);
		}
	}


}


void distancePointToLine(const ::Eigen::Vector2d& point, ::Eigen::VectorXd& line ,double& distance)
{

	::Eigen::Vector2d point_on_line(line[3], line[3]);
	::Eigen::Vector2d tangent(line[0], line[1]);
	distancePointToLine(point, point_on_line , tangent, distance);
}

void distancePointToLine(const ::Eigen::Vector2d& point, ::Eigen::VectorXd& line, ::Eigen::Vector2d& vertical , double& distance)
{
	
	distance = ::std::abs(vertical.transpose() * (point - line.segment(2, 2)));

}

::std::vector<double> linspace(double a, double b, int n) 
{
    ::std::vector<double> array;
    double step = (b-a) / (n-1);

    while(a <= b) {
        array.push_back(a);
        a += step;           // could recode to better handle rounding errors
    }
    return array;
}

::std::vector<double> linspace2(double a, double b, int n) 
{
    ::std::vector<double> array;
    double epsilon = 0.0001;
    double step = (b-a) / (n-1);
    if (a==b)
    {
        for (int i = 0; i < n; i++)
        {
            array.push_back(a);
        }
    }
    else if (step >= 0)
    {
        while(a <= b + epsilon)
        {
            array.push_back(a);
            a += step;           
        }       
    }
    else
    {
        while(a + epsilon >= b )
        {
            array.push_back(a);
            a += step;           
        }       
    }
    return array;
}