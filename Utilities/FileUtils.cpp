#include "FileUtils.h"

#include <iostream> 
#include <stdexcept> 
#include <algorithm>
#include <vector>




bool getClassesNames(std::vector<std::string>& classes, std::string path)
{
	
    DIR *dir = opendir(path.c_str());
    struct dirent *entry = readdir(dir);

    while (entry != NULL)
    {
        if ( (entry->d_type == DT_DIR) && !(entry->d_name[0] =='.') )
			classes.push_back(std::string(entry->d_name));

		entry = readdir(dir);
    }

    closedir(dir);
	return true;
}


int getImList(std::vector<std::string>& imList, std::string path)
{	
	int count = 0;
    // Read files in the input folder
	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir (path.c_str())) != NULL) {
		/* print all the files and directories within directory */
		while ((ent = readdir (dir)) != NULL) {
			std::string file(ent->d_name);

			std::size_t found = file.find(".png");
			if (found!=std::string::npos) 
			{
				imList.push_back(file);
				count++;
			}
		}
		closedir (dir);
	} else {
		return count;
	}
	return count;
}



int getImagesFromPath(std::vector<cv::Mat*>& imVector, std::string path)
{
    int count = 0;
    // Read files in the input folder
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir (path.c_str())) != NULL) {
        /* print all the files and directories within directory */
        while ((ent = readdir (dir)) != NULL) {
            std::string file(ent->d_name);

            std::size_t found = file.find(".png");
            if (found!=std::string::npos)
            {
                std::string filepath = path + "/" + file;
                ::cv::Mat img = ::cv::imread(filepath);

                imVector.push_back(&img);
                count++;
            }
        }
        closedir (dir);
    } else {
        return count;
    }
    return count;
}


/**
 * @brief: Replace a substring in a string efficiently
 *
 * @author: Ben
 *
 * \param[in,out] source - the string to be modified
 * \param[in] from - the substring to be changed
 * \param[in] to - the substring to place instead of 'from'
 * */
void replaceInStr( ::std::string& source, const ::std::string& from, const ::std::string& to )
{
    ::std::string newString;
    newString.reserve( source.length() );
   ::std:: string::size_type lastPos = 0;
    ::std::string::size_type findPos;

    while( ::std::string::npos != ( findPos = source.find( from, lastPos )))
    {
        newString.append( source, lastPos, findPos - lastPos );
        newString += to;
        lastPos = findPos + from.length();
    }

    // Care for the rest after last occurrence
    newString += source.substr( lastPos );

    source.swap( newString );
}


/**
 * @brief: Check path for incorrect file separators, and replace them if needed
 *
 * @author: Ben
 *
 * \param[in] path - the input path
 * \return the corrected path
 * */
::std::string checkPath(::std::string path)
{
    const ::std::string goodPathSeparator =
    #ifdef LINUX
            "/";
    #else
            "\\";
    #endif

    const ::std::string wrongPathSeparator =
    #ifdef LINUX
            "\\";
    #else
            "/";
    #endif

    replaceInStr(path, wrongPathSeparator, goodPathSeparator);

    return path;
}












