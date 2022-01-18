#include "point_categorization.hpp"

namespace point_categorization
{

	PointCategorization::PointCategorization(std::string inputFileName, std::string outputFileName)
	{
		_inputFileName = inputFileName;
		_outputFileName = outputFileName;

		if(!common::checkFile(inputFileName))
			throw std::runtime_error(std::string("Unable to access file: " + inputFileName));
	}

	void PointCategorization::tryReadInputFile()
	{
		// throw a runtime err if we can't read the file
		if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(_inputFileName, *_pCloud) == -1)
		{
			throw std::runtime_error(std::string("Unable to read point cloud from: ") + _inputFileName);
		}	
	}

	void PointCategorization::setupVisualizer()
	{

	}
	
	void PointCategorization::run()
	{
		tryReadInputFile();
	}

}
