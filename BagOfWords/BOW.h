#pragma once

#include <opencv2\opencv.hpp>
#include <opencv2\xfeatures2d\nonfree.hpp>
#include <opencv2\features2d.hpp>
#include <opencv2/ml.hpp>

#include <vector>
#include <string>


class BOW
{
public:
	BOW();
	~BOW();

	bool LoadFromFile(::std::string path);
	bool SaveToFile(::std::string path);
	bool trainBOW(::std::string path);
	bool predictBOW(std::string path, float& response);


private:
	::cv::Ptr<::cv::FeatureDetector> m_featureDetector;
	::cv::Ptr<::cv::DescriptorExtractor> m_descriptorExtractor;
	::cv::Ptr<::cv::BOWKMeansTrainer> m_bowtrainer;
	::cv::Mat m_vocabulary;
	::cv::Ptr<::cv::DescriptorMatcher> m_descriptorMatcher;
	::cv::Ptr<::cv::BOWImgDescriptorExtractor> m_bowide;

	::cv::Ptr<::cv::ml::SVM> m_svm ;
	::cv::TermCriteria m_tc_Kmeans;
	::cv::TermCriteria m_tc_SVM;

	std::vector<std::string> m_classes;
	std::vector<std::string> m_imList;
	std::vector<std::string> m_classList;

	int m_dictionarySize;
};
