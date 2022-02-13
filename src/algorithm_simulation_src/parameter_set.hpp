#pragma once

namespace algorithm_simulation
{

	// represents the ground removal algorithm parameters,
	struct ParameterSet
	{
		unsigned int numBins, numSegments;
		float tM, tMSmall, tB, tRMSE, tDPrev, tDGround;
	};

}
