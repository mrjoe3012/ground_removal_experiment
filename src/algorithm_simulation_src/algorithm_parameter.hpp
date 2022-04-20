#pragma once

#include "parameter_set.hpp"

#include <functional>
#include <string>

namespace algorithm_simulation
{

	// represents a parameter used in the algorithm.
	// This class provides a way to easily change the parameter's
	// value and provides a way to alter a ParameterSet based on
	// its value.
	class AlgorithmParameter
	{

		private:
			// the function that will be specified on the parameter's creation,
			// this is so we can define how each parameter will alter the corresponding
			// parameter set, which translates directly to ground removal algorithm parameters.
			std::function<void(float value, ParameterSet&)> _updateParameterSetFunc;

			float _value, _minValue, _maxValue;

			std::string _name;

		public:
			AlgorithmParameter(std::string name, float value, float minValue, float maxValue, std::function<void(float value, ParameterSet&)> updateParameterSetFunc);

			// updates the corresponding variable in a parameter set structure
			// based on the current value of this object
			void updateParameterSet(ParameterSet&) const;

			// steps the current value of this parameter,
			// the parameter totalSteps decides how large the increment:
			// e.g if the function is called 10 times with totalSteps = 10,
			// the value will be stepped from its minimum to its maximum
			bool stepParameter(unsigned int totalSteps);

			void value(float);
			float value() const;

			std::string name() const;

			float minValue() const;
			float maxValue() const;

			void minValue(float value);
			void maxValue(float value);
	};

}
