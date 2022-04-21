#pragma once
#include <vector>
#include <string>

namespace algorithm_simulation
{

	// holds the data that is collected for each parameter.
	// data may be collected for multiple .pcd frames, in which
	// case values are averaged.
	struct SimulationData
	{

		public:
			// the value of the variable parameter when the results were recorded.
			double parameterValue;
			// (total points removed) / (total number of points)
			double averagePointsRemovedTotal;
			// (ground points removed) / (total ground points)
			double averageGroundPointsRemoved;
			// (ground points removed) / (total points removed)
			double averageGroundPointsRemovedTotal;
			// (cone points removed) / (total points removed)
			double averageConePointsRemoved;

	};

	// simulation will return a list where each element contains the name of the parameter
	// and its corresponding list of results, where each result was collected with a specific
	// value for the parameter.
	typedef std::vector<std::pair<std::string, std::vector<SimulationData>>> SimulationResult;

}
