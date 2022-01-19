#include "point_categorization.hpp"

using common::DebugOut;

namespace point_categorization
{

	PointCategorization::PointCategorization(std::string inputFileName, std::string outputFileName)
		:	
		_pCloud(new pcl::PointCloud<pcl::PointXYZRGB>())
	{
		DebugOut& debugStream = DebugOut::instance();
		debugStream << "Creating program instance..." << std::endl;

		_inputFileName = inputFileName;
		_outputFileName = outputFileName;

		if(!common::checkFile(inputFileName))
			throw std::runtime_error(std::string("Unable to access file: " + inputFileName));
	}

	void PointCategorization::tryReadInputFile()
	{
		DebugOut& debugStream = DebugOut::instance();

		// throw a runtime err if we can't read the file
		if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(_inputFileName, *_pCloud) == -1)
		{
			throw std::runtime_error(std::string("Unable to read point cloud from: ") + _inputFileName);
		}	

		debugStream << common::fstring("Successfully read point cloud of width %d.",_pCloud->width) << std::endl;
	}

	void PointCategorization::initializePointCloudRGB()
	{
		using namespace common;	

		DebugOut& debug = DebugOut::instance();

		unsigned int numPointsInitialized = 0;

		const CategoryColour& unassignedColour = categoryColours[PointCategory::Unassigned];

		for(pcl::PointXYZRGB& p : *_pCloud)
		{
			// check if colour is unset (black)
			// if so, then set it to the colour corresponding
			// to an unassigned point.
			if(p.r == 0 && p.g == 0 && p.b == 0)
			{
				p.r = std::get<0>(unassignedColour), p.g = std::get<1>(unassignedColour), p.b = std::get<2>(unassignedColour);
				numPointsInitialized++;
			}
		}

		debug << fstring("Initialized %d points to 'Unassigned'.", numPointsInitialized) << std::endl;

	}

	void PointCategorization::setupVisualizer()
	{
		DebugOut& debug = DebugOut::instance();
		debug << "Setting up visualizer..." << std::endl;

		_pVisualizer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer(_inputFileName.c_str()));
		_pVisualizer->setBackgroundColor(0,0,0);
		_pVisualizer->addPointCloud(_pCloud, "input_cloud");
		_pVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input_cloud");
		_pVisualizer->addCoordinateSystem(1.0);
		_pVisualizer->initCameraParameters();
	}

	void PointCategorization::visualizerLoop()
	{
		DebugOut& debug = DebugOut::instance();
		debug << "Beginning visualizer loop..." << std::endl;

		while(!_pVisualizer->wasStopped())
			_pVisualizer->spinOnce();

		debug << "Viewer was stopped. Exiting visualizer loop." << std::endl;
	}
	
	void PointCategorization::run()
	{
		DebugOut::instance() << "Running program instance..." << std::endl;
		tryReadInputFile();
		initializePointCloudRGB();
		setupVisualizer();
		visualizerLoop();
	}

}
