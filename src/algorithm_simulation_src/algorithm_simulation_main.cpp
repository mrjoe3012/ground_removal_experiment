#include <string>
#include <stdexcept>

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

bool handleArguments(int argc, char** argv, std::vector<std::string>& inputPointCloudPaths, std::string& configFilePath, std::string& outputPath)
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
			("c", po::value<std::string>(), "Path to configuration file");

		po::variables_map vm;
		po::store(po::parse_command_line(argc, argv, desc), vm);
		po::notify(vm);

		if(vm.count("help"))
		{
			debug << desc << std::endl;
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
			debug << common::fstring("configFilePath: %s\noutputPath: %s\ninputPointCloudPaths: ", configFilePath.c_str(), outputPath.c_str()) << std::endl;
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

// returns a list of name-data pairs containing the algorithm results for each value
// of the variable parameter for each parameter.
SimulationResult simulateAlgorithm(unsigned int numSteps, const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& inputPointClouds,
		ParameterSet baselineSet, std::vector<AlgorithmParameter> algorithmParameters)
{
	using namespace common;

	DebugOut& debug = DebugOut::instance();

	debug << "Beginning algorithm simulation..." << std::endl;

	SimulationResult result;

	for(AlgorithmParameter parameter : algorithmParameters)
	{

		debug << fstring("Stepping through parameter '%s'...", parameter.name().c_str()) << std::endl;
		
		std::vector<SimulationData> dataList;

		parameter.value(parameter.minValue());

		ParameterSet parameterSet = baselineSet;

		float totalRemoved = 0.0f, groundRemoved = 0.0f, coneRemoved = 0.0f, unassignedRemoved = 0.0f;

		do
		{

			parameter.updateParameterSet(parameterSet);

			for(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pInputCloud : inputPointClouds)
			{
				float total1 = 0.0f, total2 = 0.0f;
				float ground1 = 0.0f, cone1 = 0.0f, unassigned1 = 0.0f;
				float ground2 = 0.0f, cone2 = 0.0f, unassigned2 = 0.0f;

				for(pcl::PointXYZRGB p : *pInputCloud)
				{
					total1++;

					CategoryColour colour = std::make_tuple(p.r, p.g, p.b);

					if(colour == categoryColours[PointCategory::Cone])
						cone1++;
					else if(colour == categoryColours[PointCategory::Ground])
						ground1++;
					else
						unassigned1++;


				}

				std::unique_ptr<ground_removal::SegmentArray<pcl::PointXYZRGB>> pSegmentArray = ground_removal::assignPointsToBinsAndSegments(*pInputCloud, parameterSet.numSegments, parameterSet.numBins);

				ground_removal::AlgorithmParameters algorithmParams = {
					.tM = parameterSet.tM,
					.tMSmall = parameterSet.tMSmall,
					.tB = parameterSet.tB,
					.tRMSE = parameterSet.tRMSE,
					.tDPrev = parameterSet.tDPrev,
					.tDGround = parameterSet.tDGround,
				};

				pcl::PointCloud<pcl::PointXYZRGB>::Ptr pProcessedCloud = ground_removal::groundRemoval<pcl::PointXYZRGB>(*pSegmentArray, algorithmParams);

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

				totalRemoved += (total1-total2) / total1;
				groundRemoved += (ground1-ground2) / (total1-total2);
				coneRemoved += (cone1-cone2) / (total1-total2);
				unassignedRemoved += (unassigned1-unassigned2) / (total1-total2);

			}

			float numClouds = static_cast<float>(inputPointClouds.size());

			totalRemoved /= numClouds;
			groundRemoved /= numClouds;
			coneRemoved /= numClouds;
			unassignedRemoved /= numClouds;

			SimulationData data = {
				.parameterValue = parameter.value(),
				.averagePointsRemovedTotal = totalRemoved,
				.averageGroundPointsRemoved = groundRemoved,
				.averageUnassignedPointsRemoved = unassignedRemoved,
				.averageConePointsRemoved = coneRemoved,
			};

			dataList.push_back(data);

			debug << fstring("{%f, %f, %f, %f, %f}", parameter.value(), totalRemoved, groundRemoved, coneRemoved, unassignedRemoved) << std::endl;

		} while(parameter.stepParameter(numSteps));	

		result.push_back(std::make_pair(parameter.name(), dataList));
	}	

	debug << "Finished simulation..." << std::endl;

	return result;

}

int main(int argc, char* argv[])
{
	DebugOut& debug = DebugOut::instance();

	std::vector<std::string> inputPointCloudPaths;
	std::string configFilePath, outputPath;

	if(!handleArguments(argc, argv, inputPointCloudPaths, configFilePath, outputPath))
		return 0;

	ParameterSet baselineSet;
	std::vector<AlgorithmParameter> algorithmParameters;
	unsigned int numSteps;

	if(!readConfigFile(configFilePath, baselineSet, algorithmParameters, numSteps))
		return -1;

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> inputPointClouds;

	if(!readPointClouds(inputPointCloudPaths, inputPointClouds))
		return -1;

	simulateAlgorithm(numSteps, inputPointClouds, baselineSet, algorithmParameters);

	return 0;
}
