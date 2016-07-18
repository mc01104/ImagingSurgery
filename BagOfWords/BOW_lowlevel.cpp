#include "BOW_lowlevel.h"
#include "FileUtils.h"


BOW_l::BOW_l()
{

	m_featureDetector = ::cv::xfeatures2d::SURF::create();
	m_descriptorExtractor =  ::cv::xfeatures2d::SURF::create();

	m_dictionarySize = 10000;
	m_tc_Kmeans = ::cv::TermCriteria(::cv::TermCriteria::MAX_ITER + ::cv::TermCriteria::EPS,100000, 0.000001);
	int retries = 1;
	int flags = ::cv::KMEANS_PP_CENTERS;
	m_bowtrainer = new ::cv::BOWKMeansTrainer(50, m_tc_Kmeans, retries, flags);
	
	m_descriptorMatcher = ::cv::DescriptorMatcher::create("FlannBased");
	m_bowide = new ::cv::BOWImgDescriptorExtractor(m_descriptorExtractor, m_descriptorMatcher);

	m_knn = ::cv::ml::KNearest::create();
	m_knn->setAlgorithmType(::cv::ml::KNearest::KDTREE);

	m_trained = false;
}


BOW_l::~BOW_l()
{

}


/*
Loads the classifier and BOW vocabulary from files
*/
bool BOW_l::LoadFromFile(::std::string path)
{
	try
	{
		m_svm = ::cv::ml::StatModel::load<::cv::ml::SVM>(path + "SVM.xml");

		cv::FileStorage storage(path + "VOC.xml", cv::FileStorage::READ);
		storage["vocabulary"] >> m_vocabulary;
		storage.release();  


		m_bowide->setVocabulary(m_vocabulary);

		std::vector<int> v_word_labels;
		for (int i=0;i<m_vocabulary.rows;i++) v_word_labels.push_back(i);
		::cv::Mat mat_words_labels(v_word_labels);
		m_knn->setDefaultK(1);
		m_knn->train(m_vocabulary, ::cv::ml::ROW_SAMPLE, mat_words_labels);

		cv::FileStorage storage2(path + "SCALE_means.xml", cv::FileStorage::READ);
		storage2["scale_m"] >> m_scaling_means;
		storage2.release();   

		cv::FileStorage storage3(path + "SCALE_stds.xml", cv::FileStorage::READ);
		storage3["scale_stds"] >> m_scaling_stds;
		storage3.release();   


		m_classes.clear();

		cv::FileStorage storage4(path + "CLASSES.xml", cv::FileStorage::READ);
		::cv::FileNode n = storage4["classes"];                         // Read string sequence - Get node
		::cv::FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
		for (; it != it_end; ++it)
			m_classes.push_back((::std::string)*it);
		storage4.release();   

		m_trained = true;
		return true;
	}
	catch ( const std::exception & e ) 
	{
		::std::cout << "Error in Loading SVM classifier" << ::std::endl;
		::std::cout << e.what();

		m_trained = false;
		return false;
	}
}


/*
Saves the classifier and BOW vocabulary to files
*/
bool BOW_l::SaveToFile(::std::string path)
{
	try
	{
		m_svm->save(path + "SVM.xml");

		cv::FileStorage storage(path + "VOC.xml", cv::FileStorage::WRITE);
		storage << "vocabulary" << m_vocabulary;
		storage.release();   


		storage = ::cv::FileStorage(path + "SCALE_means.xml", cv::FileStorage::WRITE);
		storage << "scale_m" << m_scaling_means;
		storage.release();   

		storage = ::cv::FileStorage(path + "SCALE_stds.xml", cv::FileStorage::WRITE);
		storage << "scale_stds" << m_scaling_stds;
		storage.release();   

		storage = ::cv::FileStorage(path + "CLASSES.xml", cv::FileStorage::WRITE);
		storage << "classes" << "[";
		for (int i=0;i<m_classes.size();i++) storage<<m_classes[i];
		storage << "]"; 
		storage.release();   
		
		return true;
	}
	catch ( const std::exception & e ) 
	{
		::std::cerr << e.what();
		return false;
	}
}


::std::vector<::std::string> BOW_l::getClasses()
{
	return m_classes;
}



/*
Trains the BOW with an SVM classifier

argument path is a string pointing to the training dataset. It should contain folders each named after a class, containing positive sample images in the form of PNGs

returns true if the SVM is trained correctly
*/
bool BOW_l::trainBOW(::std::string path)
{

	// declare variables
	std::vector<cv::KeyPoint> keyPoints;
	::cv::Mat descriptors;
	::cv::Mat training_descriptors(0,m_descriptorExtractor->descriptorSize(),m_descriptorExtractor->descriptorType());
	::cv::Mat cluster_labels;

	::cv::Mat img;

	::cv::Mat labels(0, 1, CV_32FC1);
	::cv::Mat trainingData(0, m_dictionarySize, CV_32FC1);
	::cv::Mat response_hist;
	int total_samples = 0;

	// list class names 
	int class_id = 0;
	if (getClassesNames(m_classes, path))
	{
		
		for each (std::string className in m_classes)
		{
			int count = getImList(m_imList, path + className);
			for (int i=0;i<count;i++) 
			{
				m_classList.push_back(className);
				labels.push_back(class_id);
			}
			class_id++;
		}
	}

	::std::vector<int> image_number;
	// get images and compute keypoints as training descriptors
	for(int i=0; i<m_imList.size();i++)
	{
		std::string filepath = path + m_classList[i] + "\\" + m_imList[i];

		img = ::cv::imread(filepath);
		m_featureDetector->detect(img,keyPoints);
		m_descriptorExtractor->compute(img, keyPoints,descriptors);
		training_descriptors.push_back(descriptors);
		for (int j = 0; j< descriptors.rows; j++) image_number.push_back(i);
	}


	// kmeans classify
	int k = 50;
	::cv::kmeans(training_descriptors, k, cluster_labels, m_tc_Kmeans, 3, cv::KMEANS_PP_CENTERS, m_vocabulary );
	// cluster and set vocabulary into bow ide
	//m_bowtrainer->add(training_descriptors);
	//m_vocabulary = m_bowtrainer->cluster(); 

	::std::vector<float> temp(k, 0.0);
	::std::vector< ::std::vector<float> > v_im_histograms(m_imList.size(),temp);

	for(int i=0; i<cluster_labels.rows;i++)
	{
		int img_number = image_number[i];
		int word = cluster_labels.at<int>(i);

		v_im_histograms[img_number][word] ++;

	}

	::cv::Mat im_histograms = ::cv::Mat::zeros(m_imList.size(),k,CV_32FC1);
	im_histograms = vec2cvMat_2D(v_im_histograms);

	m_scaling_means.clear();
	m_scaling_stds.clear();

	// scale thehistogram columns
	for (int i=0; i<im_histograms.cols;i++)
	{
		::cv::Scalar mean;
		::cv::Scalar stdev;

		::cv::Mat col = im_histograms.col(i);

		::cv::meanStdDev(col,mean,stdev);

		col = col - mean.val[0];
		col = col / stdev.val[0];

		m_scaling_means.push_back(mean.val[0]);
		m_scaling_stds.push_back(stdev.val[0]);

	}




	m_bowide->setVocabulary(m_vocabulary);
	//// Setup training data for classifiers
	//for(int i=0; i<m_imList.size();i++)
	//{
	//	std::string filepath = path + m_classList[i] + "\\" + m_imList[i];

	//	img = ::cv::imread(filepath);
	//	m_featureDetector->detect(img,keyPoints);
	//	m_bowide->compute(img, keyPoints, response_hist);

	//	trainingData.push_back(response_hist);

	//	/*int class_id = 0;
	//	for (int j=0;j < m_classes.size();j++)
	//	{
	//		if (m_classes[j] == m_classList[i]) class_id = j;
	//	}
	//	labels.push_back(class_id);*/
	//}  

	m_svm = ::cv::ml::SVM::create();
	//m_svm->setType(::cv::ml::SVM::C_SVC);
	//m_svm->setKernel(::cv::ml::SVM::LINEAR);
	//m_svm->setTermCriteria(::cv::TermCriteria(::cv::TermCriteria::MAX_ITER,1000,0.000001));
	m_svm->setGamma(0.02);

	if (m_svm->train(im_histograms, ::cv::ml::ROW_SAMPLE,labels))
	{
		m_trained = true;
	}
	else m_trained = false;

	return m_trained;

}

/*
Does a prediction using the BOW

Arguments:
	- path is a string pointing to an image file
	- float is a reference which will take the response

Returns true if there was no runtime error during prediction
*/
bool BOW_l::predictBOW(std::string path, float& response)
{
	if (!m_trained) return false;
	::cv::Mat img = ::cv::imread(path);
	return predictBOW(img, response);
}


/*
Does a prediction using the BOW

Arguments:
	- img is a openCV Mat containing an image
	- float is a reference which will take the response

Returns true if there was no runtime error during prediction
*/
bool BOW_l::predictBOW(::cv::Mat img, float& response)
{
	if (!m_trained) return false;

	std::vector<cv::KeyPoint> keyPoints;
	::cv::Mat descriptors;

	::cv::Mat bowDescriptor;	

	::cv::Mat wordsInImg;

	m_featureDetector->detect(img, keyPoints);
	m_descriptorExtractor->compute(img, keyPoints,descriptors);

	if (descriptors.rows<1)
	{
		response = 0.0;
		return true;
	}

	try{
		// Vector quantization of words in image
		m_knn->findNearest(descriptors,1,wordsInImg);
	}
	catch (std::exception& e)
	{
		response = 0.0;
		//::std::cout << e.what();
		return false;
	}

	
	// construction of response histogram
	::std::vector<float> temp(m_vocabulary.rows, 0.0);

	for(int i=0; i<wordsInImg.rows;i++)
	{
		int word = wordsInImg.at<float>(i,0);
		temp[word] ++;
	}
	::cv::Mat response_histogram;
	::cv::transpose(::cv::Mat(temp),response_histogram); // make a row-matrix from the column one made from the vector


	// Normalization of response histogram
	for (int i=0; i<response_histogram.cols;i++)
	{
		::cv::Mat col = response_histogram.col(i);

		col = col - m_scaling_means[i];
		col = col / m_scaling_stds[i];
	}


	response = 0.0;
	try
	{
		response = m_svm->predict(response_histogram);
	}
	catch ( const std::exception & e ) 
	{
		response = 0.0;
		//::std::cout << e.what();
		return false;
	}

	return true;
}
