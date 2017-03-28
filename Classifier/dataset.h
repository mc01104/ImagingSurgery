#ifndef __DATASET_H__
#define __DATASET_H__

// general includes
#include <vector>
#include <string>
#include <exception>

// opencv
#include <opencv2/opencv.hpp>

/**
  * @brief: Dataset class for image classifier
  * @author: George & Ben
  */
class Dataset
{

        protected:

            ::std::vector< ::std::string> m_classes;
            ::std::vector< ::std::string> m_imList;
            ::std::string m_mainPath;
            ::std::vector< ::cv::Mat> m_images;
            ::std::vector<int> m_labels;
            bool m_initialized;

	public:
		
                // Constructor/destructor
                Dataset();
                Dataset(const ::std::string& path);
                ~Dataset();

                // accessors
                ::std::vector< ::std::string> getClasses()  const {return m_classes;};
                ::std::vector< ::std::string> getImagesList()  const {return m_imList;};
                ::std::vector< ::cv::Mat> getImages()  const {return m_images;};
                ::std::vector<int> getLabels() const {return m_labels;};
                bool isInit() const {return m_initialized;};

                // methods
                void initDataset(const ::std::string& path);
                void clear();

                bool serializeInfo(const ::std::string output_path);
                bool createFromXML(const ::std::string& path);

};


#endif	__DATASET_H__
