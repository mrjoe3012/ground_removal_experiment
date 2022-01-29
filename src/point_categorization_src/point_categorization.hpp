#pragma once

#include "common.hpp"

#include "edit_stack.hpp"
#include "edit_record.hpp"

#include <string>
#include <stdexcept>
#include <cstdio>
#include <functional>
#include <cmath>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

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
			const float _MAX_BRUSH_SIZE = 1.0f, _MIN_BRUSH_SIZE = 0.05f;
			const float _BRUSH_SIZE_INCREMENT_SMALL = 0.05f, _BRUSH_SIZE_INCREMENT_LARGE = 0.2f;

			float _brushSize = 0.1f;
			common::PointCategory _currentPointCategory = common::PointCategory::Unassigned;

			std::string _inputFileName, _outputFileName;

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pCloud = nullptr;

			pcl::visualization::PCLVisualizer::Ptr _pVisualizer = nullptr;
			
			// a flag that tells us when the visualizer needs
			// to update its point cloud
			bool _cloudModified = false;

			// edit stack structure containign editrecords
			// and providing undo/redo functionality
			EditStack _editStack;

			// attempts to read input point cloud data
			void tryReadInputFile();
			// sets RGB colour to unassigned for any points
			// that have no RGB data
			void initializePointCloudRGB();
			// initializes the visualizer and its callbacks
			void setupVisualizer();
			void visualizerLoop();

			void viewerKeyboardCallback(const pcl::visualization::KeyboardEvent&);
			void viewerPointPickingCallback(const pcl::visualization::PointPickingEvent&);	

			std::vector<pcl::PointXYZRGB*> getPointsNear(const pcl::PointXYZRGB& p, float maxDistance);

			void tryUndo();
			void tryRedo();

			void categorizePointsNear(pcl::PointXYZRGB p);
			void trySavePointCloud();
			
			void setCurrentPointCategory(common::PointCategory);

			void setBrushSize(float newSize);
	};
	
}
