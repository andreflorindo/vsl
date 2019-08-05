#include <vsl_sreen_window.h>


void SetupScreenWidget::focusGiven()
{}

bool SetupScreenWidget::focusLost()
{
    return true;  // accept switching by default
}


namespace vsl_screen_window
{

namespace fs = boost::filesystem;

bool getFileContent(std::string fileName, std::vector<double> & path)
{
	// Open the File
	std::ifstream path_file(fileName.c_str());
 
	// Check if object is valid
	if(!path_file)
	{
		std::cerr << "Program could not open the file: "<<fileName<<std::endl;
		return false;
	}
 
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
}