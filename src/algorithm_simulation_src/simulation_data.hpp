#pragma once
#include <vector>
#include <string>

namespace algorithm_simulation
{

	struct SimulationData
	{

		public:
			// the value of the variable parameter when the results were recorded.
			double parameterValue;
			// (total points removed) / (total number of points)
			double averagePointsRemovedTotal;
			// (ground points removed) / (total points removed)
			double averageGroundPointsRemoved;
			// (unassigned points removed) / (total points removed)
			double averageUnassignedPointsRemoved;
			// (cone points removed) / (total points removed)
			double averageConePointsRemoved;

	};

	typedef std::vector<std::pair<std::string, std::vector<SimulationData>>> SimulationResult;

}
