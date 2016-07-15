#pragma once

#include <vector>
#include <string>

#include "dirent.h"
#include "helpers_sort.h"

#include <opencv2\opencv.hpp>

bool getClassesNames(std::vector<std::string>& classes, std::string path);
int getImList(std::vector<std::string>& imList, std::string path);


template <typename T>
cv::Mat_<T> vec2cvMat_2D(std::vector< std::vector<T> > &inVec){
  int rows = static_cast<int>(inVec.size());
	int cols = static_cast<int>(inVec[0].size());

	cv::Mat_<T> resmat(rows, cols);
	for (int i = 0; i < rows; i++)
	{
		resmat.row(i) = cv::Mat(inVec[i]).t();
	}
	return resmat;
}