#include <chrono>

#include "dataset.h"

#include "Utilities.h"
#include "FileUtils.h"
#include <iomanip>
#include <ctime> 


Dataset::Dataset()
{
}

Dataset::~Dataset()
{
}

Dataset::Dataset(const ::std::string& path)
{
    initDataset(path);
}


/**
 * @brief: Initialize a dataset and load images, classes info ...
 *
 * @author: Ben & George
 *
 * \param[in] path - the path to the root directory of the dataset. Must contain subdirectories with classes names and images inside
 * */
void Dataset::initDataset(const ::std::string& path)
{
    // start by clearing all data if the dataset was initialized before
    this->clear();

    m_mainPath = checkPath(path + "/");
	
    // list class names in input path and extract images
    int class_id = 0;
    int imCount = 0;
    if (getClassesNames(m_classes, path))
    {
        for (std::string className : m_classes)
        {
            int count = getImList(m_imList, checkPath(path + "/" + className));
            for (int i=0;i<count;i++)
            {
                m_labels.push_back(class_id);
                m_images.push_back(::cv::imread(checkPath(path + "/" + className + "/" + m_imList[imCount+i]) ));
            }
            imCount += count;
            class_id++;
        }
    }

    if (m_imList.size() != m_labels.size())
            throw("Error creating the training dataset -> images and labels are not equal in number");

    m_initialized = true;
}


/**
 * @brief: Serialize dataset info into an XML files
 *
 * Saves basic info for now, the main path to the dataset files, and the time it was saved
 *
 * @author: Ben & George
 *
 * \param[in] output_path - the path in which DATASET.XML will be saved
 * \return true if saved correctly, false if an error occured or the dataset is unitialized
 * */
bool Dataset::serializeInfo(const ::std::string output_path)
{
    if (! m_initialized)
        return false;

    try
    {
        cv::FileStorage storage(checkPath(output_path + "DATASET.xml"), cv::FileStorage::WRITE);
        storage << "mainPath" << "[";
        storage << m_mainPath;
        storage << "]";

        storage << "classes" << "[";
        storage << m_classes;
        storage << "]";

        time_t rawTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        ::std::stringstream ss;
        ss << ::std::put_time(std::localtime(&rawTime), "%Y-%m-%d_%H-%M-%S");
        storage << "timestamp" << "[";
        storage << ss.str();
        storage << "]";

    }
    catch ( const std::exception & e )
    {
            ::std::cerr << e.what();
            throw("Error during dataset serialization");
            return false;
    }

    return true;
}


/**
 * @brief: Load a dataset from info contained in XML files
 *
 * Loads the filepath from the XML file and uses it to initialize a dataset object
 *
 * @author: Ben & George
 *
 * \param[in] path - the path in which DATASET.XML is
 * \return true if loaded correctly, false if an error occured
 * */
bool Dataset::createFromXML(const ::std::string& path)
{
    try
    {
        cv::FileStorage storage(checkPath(path + "DATASET.xml"), cv::FileStorage::READ);
        ::cv::FileNode n = storage["mainPath"];
        ::std::string path = ((::std::string)*n.begin());

        initDataset(path);

        return true;

    }
    catch ( const std::exception & e )
    {
            ::std::cerr << e.what();    
            throw("Error during dataset deserialization");
            return false;
    }
}




/**
 * @brief: Clears info from a dataset instance
 * *
 * @author: Ben & George
 * */
void Dataset::clear()
{
    if (m_initialized)
    {
        m_classes.clear();
        m_imList.clear();
        m_mainPath.clear();
        m_images.clear();
        m_labels.clear();
        m_mainPath = "";
        m_initialized = false;
    }
}
