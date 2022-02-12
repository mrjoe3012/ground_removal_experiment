#pragma once
#include <vector>
#include <string>

namespace algorithm_simulation
{

	struct SimulationData
	{

		public:
			// the value of the variable parameter when the results were recorded.
			float parameterValue;
			// (total points removed) / (total number of points)
			float averagePointsRemovedTotal;
			// (ground points removed) / (total points removed)
			float averageGroundPointsRemoved;
			// (unassigned points removed) / (total points removed)
			float averageUnassignedPointsRemoved;
			// (cone points removed) / (total points removed)
			float averageConePointsRemoved;

	};

	typedef std::vector<std::pair<std::string, std::vector<SimulationData>>> SimulationResult;

}
