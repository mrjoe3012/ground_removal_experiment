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

		std::function<void(const pcl::visualization::KeyboardEvent&)> keyboardCallback = std::bind(&PointCategorization::viewerKeyboardCallback, this, std::placeholders::_1);
		std::function<void(const pcl::visualization::PointPickingEvent&)> pointPickingCallback = std::bind(&PointCategorization::viewerPointPickingCallback, this, std::placeholders::_1);

		_pVisualizer->registerKeyboardCallback(keyboardCallback);
		_pVisualizer->registerPointPickingCallback(pointPickingCallback);

	}

	void PointCategorization::visualizerLoop()
	{
		DebugOut& debug = DebugOut::instance();
		debug << "Beginning visualizer loop..." << std::endl;

		while(!_pVisualizer->wasStopped())
		{
			if(_cloudModified)
			{
				_cloudModified = false;
				_pVisualizer->updatePointCloud(_pCloud, "input_cloud");
			}
			_pVisualizer->spinOnce();
		}

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

	void PointCategorization::viewerKeyboardCallback(const pcl::visualization::KeyboardEvent& event)
	{
		using common::PointCategory;

		if(event.keyDown())
		{
			if(event.getKeySym() == "KP_1")
			{
				setCurrentPointCategory(PointCategory::Unassigned);
			}
			else if(event.getKeySym() == "KP_2")
			{
				setCurrentPointCategory(PointCategory::Ground);
			}
			else if(event.getKeySym() == "KP_3")
			{
				setCurrentPointCategory(PointCategory::Cone);
			}
			else if(event.getKeySym() == "Up")
			{
				if(event.isShiftPressed())
				{
					setBrushSize(_brushSize + _BRUSH_SIZE_INCREMENT_LARGE);
				}
				else
				{
					setBrushSize(_brushSize + _BRUSH_SIZE_INCREMENT_SMALL);
				}
			}
			else if(event.getKeySym() == "Down")
			{
				if(event.isShiftPressed())
				{
					setBrushSize(_brushSize - _BRUSH_SIZE_INCREMENT_LARGE);
				}
				else
				{
					setBrushSize(_brushSize - _BRUSH_SIZE_INCREMENT_SMALL);
				}
			}
			else if(event.getKeySym() == "S" && event.isShiftPressed())
			{
				trySavePointCloud();
			}
			else if(event.getKeySym() == "z" && event.isCtrlPressed())
			{
				tryUndo();
			}
			else if(event.getKeySym() == "y" && event.isCtrlPressed())
			{
				tryRedo();
			}
		}
	}

	void PointCategorization::viewerPointPickingCallback(const pcl::visualization::PointPickingEvent& event)
	{
		categorizePointsNear((*_pCloud)[event.getPointIndex()]);
	}

	void PointCategorization::tryUndo()
	{
		DebugOut& debug = DebugOut::instance();

		if(_editStack.canUndo())
		{
			_editStack.undo();
		}
		else
		{
			debug << "No actions to undo..." << std::endl;
		}
	}

	void PointCategorization::tryRedo()
	{
		DebugOut& debug = DebugOut::instance();

		if(_editStack.canRedo())
		{
			_editStack.redo();
		}
		else
		{
			debug << "No actions to redo..." << std::endl;
		}
	}

	std::vector<pcl::PointXYZRGB*> PointCategorization::getPointsNear(const pcl::PointXYZRGB& point, float maxDistance)
	{
		std::vector<pcl::PointXYZRGB*> points;

		float sqrMaxDistance = maxDistance*maxDistance;

		for(pcl::PointXYZRGB& p : *_pCloud)
		{
			float sqrPointDistance = std::pow(point.x-p.x,2) + std::pow(point.y-p.y,2) + std::pow(point.z-p.z,2);
			if(sqrPointDistance <= sqrMaxDistance)
				points.push_back(&p);
		}	
		
		return points;
	}

	void PointCategorization::categorizePointsNear(pcl::PointXYZRGB p)
	{
		DebugOut& debug = DebugOut::instance();

		// getting the points near our category
		std::vector<pcl::PointXYZRGB*> points = getPointsNear(p, _brushSize);
		// the name of the category we are setting to
		std::string categoryName = common::categoryNames[_currentPointCategory];

		debug << common::fstring("Assigning %d points to '%s'.", points.size(), categoryName.c_str()) << std::endl;	

		// the colour associated with out current category
		common::CategoryColour colour = common::categoryColours[_currentPointCategory];

		// store previous colours for undo functionality
		std::vector<common::CategoryColour> previousColours;

		// change the colour of each point near the target
		for(pcl::PointXYZRGB* p : points)
		{
			previousColours.push_back(std::make_tuple(p->r, p->g, p->b));
			p->r = std::get<0>(colour), p->g = std::get<1>(colour), p->b = std::get<2>(colour);
		}

		// mark cloud as modified so that it gets updated
		_cloudModified = true;

		std::function<void()> undoFunc = [this, points, previousColours]() mutable
		{
			DebugOut& debug = DebugOut::instance();
			debug << common::fstring("Undoing categorization of %d points.", points.size()) << std::endl;

			for(unsigned int i = 0; i < points.size(); i++)
			{
				pcl::PointXYZRGB* p = points[i];
				common::CategoryColour c = previousColours[i];
				p->r = std::get<0>(c), p->g = std::get<1>(c), p->b = std::get<2>(c);
			}

			_cloudModified = true;

		};

		std::function<void()> redoFunc = [this, points, colour, categoryName]() mutable
		{
			DebugOut& debug = DebugOut::instance();
			debug << common::fstring("Redoing categorization of %d points (back to %s).", points.size(), categoryName.c_str()) << std::endl;

			for(pcl::PointXYZRGB* p : points)
			{
				p->r = std::get<0>(colour), p->g = std::get<1>(colour), p->b = std::get<2>(colour);
			}

			this->_cloudModified = true;
		};

		EditRecord editRecord(undoFunc, redoFunc);

		_editStack.pushEdit(editRecord);

	}

	void PointCategorization::trySavePointCloud()
	{
		DebugOut& debug = DebugOut::instance();

		if(pcl::io::savePCDFile(_outputFileName, *_pCloud, true) == -1)
		{
			throw std::runtime_error(std::string("Unable to save point cloud to: ") + _outputFileName);
		}
		else
		{
			debug << common::fstring("Successfully saved point cloud to '%s'", _outputFileName.c_str()) << std::endl;
		}
	}

	void PointCategorization::setCurrentPointCategory(common::PointCategory c)
	{
		using namespace common;

		DebugOut& debug = DebugOut::instance();

		std::string categoryName = categoryNames[c];	

		debug << fstring("Setting current point category to '%s'.", categoryName.c_str()) << std::endl;

		_currentPointCategory = c;
	}


	void PointCategorization::setBrushSize(float newSize)
	{
		DebugOut& debug = DebugOut::instance();

		if(newSize > _MAX_BRUSH_SIZE)
			newSize = _MAX_BRUSH_SIZE;
		else if(newSize < _MIN_BRUSH_SIZE)
			newSize = _MIN_BRUSH_SIZE;

		debug << common::fstring("Setting brush size to %f.", newSize) << std::endl;

		_brushSize = newSize;
	}
}
