#include <iostream>
#include <string>


#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include <boost/program_options.hpp>

#include "point_categories.hpp"
#include "point_categorization.hpp"

namespace po = boost::program_options;

using namespace point_categorization;

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
	if(!handleProgramOptions(argc, argv))
	{
		return 0;
	}

	std::cout << inputFilePath << std::endl << outputFilePath << std::endl;

	point_categories::PointCategory p = point_categories::PointCategory::Unassigned;
	std::cout << p++ << std::endl << p++ << std::endl << p++ << std::endl << p << std::endl;
	
	PointCategorization programInstance(inputFilePath, outputFilePath);

	programInstance.run();

	return 0;
}
