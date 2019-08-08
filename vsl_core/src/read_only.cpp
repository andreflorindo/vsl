// Inspired from setup_assistant_widget.cpp, header_wighet.cpp, start_screen_widget.cpp, setup_screen_widget.cpp
/* Author: Andre Florindo*/

#include <read_only.h>

bool getFileContent(PathStruct& path) //<-------------  Review
{
    std::ifstream filename ("examples/simplePath.txt");

    if (filename.empty())
    {
        ROS_ERROR_NAMED("read_data", "Path is empty");
        return false;
    }
    // Open the File

    if (!filename.good())
    {
        ROS_ERROR_NAMED("rdf_loader", "Unable to load path");
        return false;
    }

    //https://stackoverflow.com/questions/46663046/save-read-double-vector-from-file-c                    //<-------------  Review
    std::vector<char> buffer{};
    std::ifstream ifs(filename, std::ios::in | std::ifstream::binary);
    std::istreambuf_iterator<char> iter(ifs);
    std::istreambuf_iterator<char> end{};
    std::copy(iter, end, std::back_inserter(buffer));
    path.trivial_vector.reserve(buffer.size() / sizeof(double));
    memcpy(&path.trivial_vector[0], &buffer[0], buffer.size());
    //Review , only reading a vector, now how to read a matrix
    // If z is not given in the file, maybe add a collumn of zeros
    filename.close();
    return true;
};

/*
bool getFileContent(const std::string& fileName, std::vector<double> & path)
{

 
	std::string str;
	// Read the next line from File until it reaches the end.
	while (std::getline(path_file, str))
	{
		// Line contains string of length > 0 then save it in vector
		if(str.size() > 0)
			path.push_back(str);
	}
	//Close The File
	path_file.close();
	return true;
}
*/