#include <iostream>
#include <cstdio>
#include <string>


#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include <boost/program_options.hpp>

#include "common.hpp"
#include "point_categorization.hpp"

namespace po = boost::program_options;

std::string inputFilePath, outputFilePath;

bool handleProgramOptions(int argc, char* argv[])
{
	try
	{
		po::options_description desc("Options:");
		desc.add_options()
			("help", "Help and usage")
			("i", po::value<std::string>() , "Path to input .pcd file")
			("o", po::value<std::string>() , "Path to output.pcd file");
		
		po::variables_map vm;
		po::store(po::parse_command_line(argc, argv, desc), vm);
		po::notify(vm);

		if(vm.count("help"))
		{
			std::cout << desc << std::endl;
			return false;
		}
		else if(!vm.count("i") || !vm.count("o"))
		{
			std::cout << "An input path and output path must be specified." << std::endl;
			std::cout << desc << std::endl;
			return false;
		}
		else
		{
			inputFilePath = vm["i"].as<std::string>();
			outputFilePath = vm["o"].as<std::string>();
			return true;
		}
	}
	catch(const std::exception& e)
	{
		std::cerr << "Error: " << e.what() << std::endl;
		return false;
	}
}

int main(int argc, char* argv[])
{
	using namespace point_categorization;
	using common::DebugOut;

	DebugOut& debugStream = DebugOut::instance();

	debugStream << common::fstring("Handling %d commandline arguments...", argc) << std::endl;

	if(!handleProgramOptions(argc, argv))
	{
		return 0;
	}

	debugStream << common::fstring("inputFilePath: \"%s\"\noutputFilePath: \"%s\"", inputFilePath.c_str(), outputFilePath.c_str()) << std::endl;

	PointCategorization programInstance(inputFilePath, outputFilePath);

	programInstance.run();

	debugStream << "Shutting down..." << std::endl;

	return 0;
}
