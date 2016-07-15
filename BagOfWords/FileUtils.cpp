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
