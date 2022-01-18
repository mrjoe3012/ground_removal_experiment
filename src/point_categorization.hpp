#pragma once
#include "point_categories.hpp"

#include <string>
#include <stdexcept>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace point_categories;

namespace point_categorization
{

	// the program encapsulated in a class,
	// execution begins by invoking the run() function.
	class PointCategorization
	{
		public:
			PointCategorization(std::string inputFileName, std::string outputFileName);

			// begins reading the input file and running the visualizer loop
			void run();

		private:
			const float _MAX_BRUSH_SIZE = 1.5f, _MIN_BRUSH_SIZE = 0.05f;
			const float _BRUSH_SIZE_INCREMENT_SMALL = 0.05f, _BRUSH_SIZE_INCREMENT_LARGE = 0.2f;

			float _brushSize = 0.1f;
			PointCategory _currentPointCategory = PointCategory::Unassigned;

			std::string _inputFileName, _outputFileName;

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pCloud = nullptr;

			pcl::visualization::PCLVisualizer::Ptr _pVisualizer = nullptr;

			void tryReadInputFile();
			void setupVisualizer();

			// visualizer callbacks
	};
	
}
