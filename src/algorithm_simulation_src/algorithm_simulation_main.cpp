#include <string>
#include <stdexcept>
#include <fstream>
#include <filesystem>
#include <thread>
#include <mutex>
#include <algorithm>

#include <boost/program_options.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "common.hpp"
#include "algorithm_parameter.hpp"
#include "simulation_data.hpp"
#include "ground_removal.h"


namespace po = boost::program_options;
using namespace algorithm_simulation;

using common::DebugOut;

// take the commandline arguments and resolve the input files, config file and output directory
bool handleArguments(int argc, char** argv, std::vector<std::string>& inputPointCloudPaths, std::string& configFilePath, std::string& outputPath, int& numThreads)
{
	DebugOut& debug = DebugOut::instance();

	debug << "Handling commandline arguments..." << std::endl;

	try
	{
		po::options_description desc("Options:");
		desc.add_options()
			("help", "Help and usage")
			("i", po::value<std::vector<std::string>>()->multitoken(), "Path(s) to input .pcd files")
			("o", po::value<std::string>(), "Directory for output files")
			("c", po::value<std::string>(), "Path to configuration file")
			("threads", po::value<int>()->default_value(4), "Number of threads to open when running the simulation.");

		po::variables_map vm;
		po::store(po::parse_command_line(argc, argv, desc), vm);
		po::notify(vm);

		if(vm.count("help"))
		{
			debug << desc << std::endl;
			return false;
		}
		else if(vm["threads"].as<int>() < 1)
		{
			debug << "--threads must be given a value >= 1" << std::endl;
			return false;
		}
		else if(!vm.count("i") || !vm.count("o") || !vm.count("c"))
		{
			debug << "At least one input .pcd, an output path and a configuration file must be specified." << std::endl;
			debug << desc << std::endl;
			return false;
		}
		else
		{
			inputPointCloudPaths = vm["i"].as<std::vector<std::string>>();
			configFilePath = vm["c"].as<std::string>();
			outputPath = vm["o"].as<std::string>();
			numThreads = vm["threads"].as<int>();
			debug << common::fstring("configFilePath: %s\noutputPath: %s\nnumThreads: %d\ninputPointCloudPaths: ", configFilePath.c_str(), outputPath.c_str(), numThreads) << std::endl;
			for(std::string path : inputPointCloudPaths)
				debug << path << std::endl;
			debug << std::endl;
			return true;
		}
	}
	catch(const std::exception& e)
	{
		debug << "Error: " << e.what() << std::endl;
		return false;
	}
}

// attempt to read a .cfg file containing simulation parameters.
bool readConfigFile(const std::string& path, ParameterSet& baselineSet, std::vector<AlgorithmParameter>& algorithmParameters, unsigned int& numSteps)
{

	DebugOut& debug = DebugOut::instance();

	debug << common::fstring("Reading configuration file '%s'...", path.c_str()) << std::endl;

	try
	{
		po::options_description desc("Configuration variables:");
		desc.add_options()
			("global_settings.numStepsPerParameter", po::value<unsigned int>()->required(), "")

			("default_values.tM", po::value<float>()->required(), "")
			("default_values.tMSmall", po::value<float>()->required(), "")
			("default_values.tB", po::value<float>()->required(), "")
			("default_values.tRMSE", po::value<float>()->required(), "")
			("default_values.tDPrev", po::value<float>()->required(), "")
			("default_values.tDGround", po::value<float>()->required(), "")
			("default_values.numBins", po::value<unsigned int>()->required(), "")
			("default_values.numSegments", po::value<unsigned int>()->required(), "")

			("minimum_values.tM", po::value<float>()->required(), "")
			("minimum_values.tMSmall", po::value<float>()->required(), "")
			("minimum_values.tB", po::value<float>()->required(), "")
			("minimum_values.tRMSE", po::value<float>()->required(), "")
			("minimum_values.tDPrev", po::value<float>()->required(), "")
			("minimum_values.tDGround", po::value<float>()->required(), "")
			("minimum_values.numBins", po::value<unsigned int>()->required(), "")
			("minimum_values.numSegments", po::value<unsigned int>()->required(), "")

			("maximum_values.tM", po::value<float>()->required(), "")
			("maximum_values.tMSmall", po::value<float>()->required(), "")
			("maximum_values.tB", po::value<float>()->required(), "")
			("maximum_values.tRMSE", po::value<float>()->required(), "")
			("maximum_values.tDPrev", po::value<float>()->required(), "")
			("maximum_values.tDGround", po::value<float>()->required(), "")
			("maximum_values.numBins", po::value<unsigned int>()->required(), "")
			("maximum_values.numSegments", po::value<unsigned int>()->required(), "");

		po::variables_map vm;
		po::store(po::parse_config_file(path.c_str(), desc, false), vm);
		po::notify(vm);		

		// get configuration variables:

		numSteps = vm["global_settings.numStepsPerParameter"].as<unsigned int>();

		// set the baseline set's value to the default value in the config file.
		baselineSet.tM = vm["default_values.tM"].as<float>();
		// Create an algorithm parameter object and define a function to update a ParameterSet structure with its stored value.
		AlgorithmParameter tM("tM", baselineSet.tM, vm["minimum_values.tM"].as<float>(), vm["maximum_values.tM"].as<float>(), [](float value, ParameterSet& s){s.tM = value;});

		baselineSet.tMSmall = vm["default_values.tMSmall"].as<float>();
		AlgorithmParameter tMSmall("tMSmall", baselineSet.tMSmall, vm["minimum_values.tMSmall"].as<float>(), vm["maximum_values.tMSmall"].as<float>(), [](float value, ParameterSet& s){s.tMSmall = value;});

		baselineSet.tB = vm["default_values.tB"].as<float>();
		AlgorithmParameter tB("tB", baselineSet.tB, vm["minimum_values.tB"].as<float>(), vm["maximum_values.tB"].as<float>(), [](float value, ParameterSet& s){s.tB = value;});

		baselineSet.tRMSE = vm["default_values.tRMSE"].as<float>();
		AlgorithmParameter tRMSE("tRMSE", baselineSet.tRMSE, vm["minimum_values.tRMSE"].as<float>(), vm["maximum_values.tRMSE"].as<float>(), [](float value, ParameterSet& s){s.tRMSE = value;});

		baselineSet.tDPrev = vm["default_values.tDPrev"].as<float>();
		AlgorithmParameter tDPrev("tDPrev", baselineSet.tDPrev, vm["minimum_values.tDPrev"].as<float>(), vm["maximum_values.tDPrev"].as<float>(), [](float value, ParameterSet& s){s.tDPrev = value;});

		baselineSet.tDGround = vm["default_values.tDGround"].as<float>();
		AlgorithmParameter tDGround("tDGround", baselineSet.tDGround, vm["minimum_values.tDGround"].as<float>(), vm["maximum_values.tDGround"].as<float>(), [](float value, ParameterSet& s){s.tDGround = value;});

		baselineSet.numBins = vm["default_values.numBins"].as<unsigned int>();
		AlgorithmParameter numBins("numBins", baselineSet.numBins, vm["minimum_values.numBins"].as<unsigned int>(), vm["maximum_values.numBins"].as<unsigned int>(), [](float value, ParameterSet& s){s.numBins = static_cast<unsigned int>(value);});

		baselineSet.numSegments = vm["default_values.numSegments"].as<unsigned int>();
		AlgorithmParameter numSegments("numSegments", baselineSet.numSegments, vm["minimum_values.numSegments"].as<unsigned int>(), vm["maximum_values.numSegments"].as<unsigned int>(), [](float value, ParameterSet& s){s.numSegments = static_cast<unsigned int>(value);});

		algorithmParameters = {
			tM,tMSmall,tB,tRMSE,tDPrev,tDGround,numBins,numSegments
		};

	}
	catch(const std::exception& e)
	{
		debug << "Error: " << e.what() << std::endl;
		return false;
	}

	debug << "Successfully read configuration file." << std::endl << std::endl;

	return true;

}

// attempt to read point clouds from provided .pcd files
bool readPointClouds(const std::vector<std::string>& paths, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& inputPointClouds)
{
	DebugOut& debug = DebugOut::instance();

	debug << "Reading input point clouds..." << std::endl;

	try
	{
		for(const std::string& path : paths)
		{
			debug << common::fstring("Attempting to read point cloud from '%s'...", path.c_str()) << std::endl;
			
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

			if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(path.c_str(), *pCloud) == -1)
			{
				throw std::runtime_error(common::fstring("Unable to read point cloud from '%s'.", path.c_str()));
			}

			debug << common::fstring("Successfully read point cloud of width %d.", pCloud->width) << std::endl;

			inputPointClouds.push_back(pCloud);

		}
	}
	catch(const std::exception& e)
	{
		debug << "Error: " << e.what() << std::endl;
		return false;
	}

	debug << "Successfully read input point clouds." << std::endl << std::endl;

	return true;
}

// work function for each thread that is created in simulateAlgorithm()
// each thread will step through a parameter in a reduced range and output to
// its own designated output vector
void simulateAlgorithmWorkFunction(unsigned int numSteps, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> inputPointClouds,
		ParameterSet baselineSet, AlgorithmParameter parameter,
		std::vector<SimulationData>& workPool, std::mutex& mWorkPool,
		std::mutex& mDebug)
{

	using namespace common;

	mDebug.lock();
	DebugOut& debug = DebugOut::instance();
	mDebug.unlock();

	typedef struct { float total,cone,ground,unassigned; } CloudData;
	std::vector< CloudData > cloudData;
	
	// cache the total points and points in each category for our input
	// point clouds.
	for(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pInputCloud : inputPointClouds)
	{
		float t = 0.0f, c = 0.0f, g = 0.0f, u = 0.0f;

		for(pcl::PointXYZRGB p : *pInputCloud)
		{
			t++;

			CategoryColour colour = std::make_tuple(p.r, p.g, p.b);

			if(colour == categoryColours[PointCategory::Cone])
				c++;
			else if(colour == categoryColours[PointCategory::Ground])
				g++;
			else
				u++;	
		}

		cloudData.push_back({
			.total=t, .cone=c, .ground=g, .unassigned=u
		});
	}

	ParameterSet parameterSet = baselineSet;

	parameter.value(parameter.minValue());

	int i = 0;

	// step through all of the parameter's values
	do
	{
		// keep track of how many points are removed for each category
		double totalRemoved = 0.0f, groundRemoved = 0.0f, coneRemoved = 0.0f, groundRemovedTotal = 0.0f;

		// modifies the parameter set according to the current value
		// held by AlgorithmParameter parameter
		parameter.updateParameterSet(parameterSet);

		unsigned int j = 0;

		for(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pInputCloud : inputPointClouds)
		{
			double total1 = cloudData[j].total, total2 = 0.0f;
			// use cached cloud data for the initial values
			double ground1 = cloudData[j].ground, cone1 = cloudData[j].cone, unassigned1 = cloudData[j].unassigned;
			j++;

			double ground2 = 0.0f, cone2 = 0.0f, unassigned2 = 0.0f;

			// use ground removal algorithm to generate a new point cloud
			std::unique_ptr<ground_removal::SegmentArray<pcl::PointXYZRGB>> pSegmentArray = ground_removal::assignPointsToBinsAndSegments(*pInputCloud, parameterSet.numSegments, parameterSet.numBins);

			// translate the ParameterSet object into ground_removal::AlgorithmParameters,
			// which is the structure used by the algorithm.
			// Note that technically, numBins and numSegments are not parameters and are
			// instead used in segmentation.
			ground_removal::AlgorithmParameters algorithmParams = {
				.tM = parameterSet.tM,
				.tMSmall = parameterSet.tMSmall,
				.tB = parameterSet.tB,
				.tRMSE = parameterSet.tRMSE,
				.tDPrev = parameterSet.tDPrev,
				.tDGround = parameterSet.tDGround,
			};

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pProcessedCloud = ground_removal::groundRemoval<pcl::PointXYZRGB>(*pSegmentArray, algorithmParams);

			// go through the processed cloud to determine the difference in points for each
			// category
			for(pcl::PointXYZRGB p : *pProcessedCloud)
			{
				total2++;

				CategoryColour colour = std::make_tuple(p.r, p.g, p.b);

				if(colour == categoryColours[PointCategory::Cone])
					cone2++;
				else if(colour == categoryColours[PointCategory::Ground])
					ground2++;
				else
					unassigned2++;
			}

			// accumulate the ratios (which will be averaged)

			float x = 0.0f;

			totalRemoved += (total1-total2) / total1;
			groundRemoved += (ground1-ground2) / (total1-total2);
			groundRemovedTotal += (ground1-ground2) / (ground1);
			if(cone1 > 0)
				coneRemoved += (cone1-cone2) / (cone1);

		}

		double numClouds = static_cast<double>(inputPointClouds.size());

		// calculate averages
		totalRemoved /= numClouds;
		groundRemoved /= numClouds;
		coneRemoved /= numClouds;
		groundRemovedTotal /= numClouds;

		// create final data report
		SimulationData data = {
			.parameterValue = parameter.value(),
			.averagePointsRemovedTotal = totalRemoved,
			.averageGroundPointsRemoved = groundRemoved,
			.averageGroundPointsRemovedTotal = groundRemovedTotal,
			.averageConePointsRemoved = coneRemoved,
		};

		// output data to work pool after acquiring a lock
		{
			const std::lock_guard lock(mWorkPool);
			workPool.push_back(data);
		}

	} while(parameter.stepParameter(numSteps));

}

SimulationResult simulateAlgorithm(unsigned int numSteps, int numThreads,
		const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& inputPointClouds,
		ParameterSet baselineSet, std::vector<AlgorithmParameter> algorithmParameters)
{

	using namespace common;

	SimulationResult result;

	DebugOut& debug = DebugOut::instance();
	std::mutex mDebug;

	debug << "Beginning algorithm simulation..." << std::endl;

	for(AlgorithmParameter algorithmParameter : algorithmParameters)
	{
		debug << fstring("Stepping through parameter '%s'...", algorithmParameter.name().c_str()) << std::endl;

		typedef std::shared_ptr<std::thread> ThreadPtr;
		std::vector<ThreadPtr> threadPool;

		// work pool which is written to by threads
		// needs to be sorted after it has been populated
		std::vector<SimulationData> workPool;
		std::mutex mWorkPool;

		float stepDelta = (algorithmParameter.maxValue() - algorithmParameter.minValue()) / numThreads;
		int stepsPerThread = numSteps / numThreads;

		debug << "stepDelta: " << stepDelta << std::endl;

		// create worker threads and assign them some work
		for(int i = 0; i < numThreads; i++)
		{
			AlgorithmParameter parameter = algorithmParameter;
			parameter.maxValue(parameter.minValue() + stepDelta*(i+1));
			parameter.minValue(parameter.minValue() + stepDelta*i);

			debug << fstring("Creating thread #%d, stepping from %.2f to %.2f.", i+1, parameter.minValue(), parameter.maxValue()) << std::endl;

			ThreadPtr pThread(new std::thread(simulateAlgorithmWorkFunction,
					stepsPerThread, inputPointClouds, baselineSet,
					parameter, 
					std::ref(workPool), std::ref(mWorkPool),
					std::ref(mDebug)));
			threadPool.push_back(pThread);
		}

		debug << "Finished creating threads, waiting for work to finish..." << std::endl;

		for(int i = 0; i < threadPool.size(); i++)
		{
			threadPool[i]->join();
			debug << fstring("%d threads joined.", i+1) << std::endl;
		}

		debug << "Sorting work pool..." << std::endl;

		// sort data so that it is in ascending order
		std::sort(workPool.begin(), workPool.end(), [](const SimulationData& a, const SimulationData& b){ return a.parameterValue < b.parameterValue; });

		result.push_back(std::make_pair(algorithmParameter.name(), workPool));

	}

	debug << "Finished algorithm simulation." << std::endl;

	return result;

}

bool writeResultsToCSV(std::string outputDirectoryPath, const SimulationResult& result)
{

	namespace fs = std::filesystem;

	DebugOut& debug = DebugOut::instance();

	try
	{
	
		debug << common::fstring("Writing simulation data to '%s'...", outputDirectoryPath.c_str()) << std::endl;
		// the output directory provided by the user
		fs::path outputPath = outputDirectoryPath;

		// ensure the path exists
		if(!fs::exists(outputPath))
			throw std::runtime_error(common::fstring("Bad output directory '%s'", outputDirectoryPath.c_str()));
	
		// write a csv file for each parameter
		for(std::pair<std::string, std::vector<SimulationData>> parameterData : result)
		{
			std::string parameterName = parameterData.first;
			fs::path filePath = outputPath;
			filePath += ("/" + parameterName + ".csv");
			std::vector<SimulationData>& dataList = parameterData.second;
			std::ofstream outputFile(filePath, std::ios::out);
			if(!outputFile.good())
				throw std::runtime_error(common::fstring("Unable to open/create file '%s'", filePath.c_str()));
			// first line is a header with column names
			outputFile << "value,total_removed,ground_removed,ground_removed_total,cone_removed" << std::endl;
			// main data
			for(SimulationData data : dataList)
			{
				outputFile << common::fstring("%f,%f,%f,%f,%f", data.parameterValue, data.averagePointsRemovedTotal, data.averageGroundPointsRemoved, data.averageGroundPointsRemovedTotal, data.averageConePointsRemoved) << std::endl;
			}
		}	

	}
	catch(const std::exception& e)
	{
		debug << "Error: " << e.what() << std::endl;
		return false;
	}

	debug << "Finished writing simulation data." << std::endl;

	return true;

}

int main(int argc, char* argv[])
{
	DebugOut& debug = DebugOut::instance();

	std::vector<std::string> inputPointCloudPaths;
	std::string configFilePath, outputPath;
	int numThreads = 0;

	// returning by reference
	if(!handleArguments(argc, argv, inputPointCloudPaths, configFilePath, outputPath, numThreads))
		return 0;

	ParameterSet baselineSet;
	std::vector<AlgorithmParameter> algorithmParameters;
	unsigned int numSteps;

	// returning by reference
	if(!readConfigFile(configFilePath, baselineSet, algorithmParameters, numSteps))
		return -1;

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> inputPointClouds;

	// returning by reference
	if(!readPointClouds(inputPointCloudPaths, inputPointClouds))
		return -1;

	SimulationResult simulationResult = simulateAlgorithm(numSteps, numThreads, inputPointClouds, baselineSet, algorithmParameters);

	if(!writeResultsToCSV(outputPath, simulationResult))
		return -1;

	return 0;
}
