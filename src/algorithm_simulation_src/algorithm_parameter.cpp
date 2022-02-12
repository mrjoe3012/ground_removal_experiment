#include "algorithm_parameter.hpp"

namespace algorithm_simulation
{
	AlgorithmParameter::AlgorithmParameter(std::string name, float value, float minValue, float maxValue, std::function<void(float,ParameterSet&)> f)
		:
		_name(name),
		_value(value),
		_minValue(minValue),
		_maxValue(maxValue),
		_updateParameterSetFunc(f)
	{
	}

	void AlgorithmParameter::updateParameterSet(ParameterSet& set) const
	{
		_updateParameterSetFunc(_value, set);
	}

	void AlgorithmParameter::value(float v)
	{
		_value = v;
	}

	float AlgorithmParameter::value() const
	{
		return _value;
	}

	std::string AlgorithmParameter::name() const
	{
		return _name;
	}

	float AlgorithmParameter::minValue() const
	{
		return _minValue;
	}

	float AlgorithmParameter::maxValue() const
	{
		return _maxValue;
	}

	bool AlgorithmParameter::stepParameter(unsigned int totalSteps)
	{
		float delta = (_maxValue-_minValue) / static_cast<float>(totalSteps);
		float newValue = _value + delta;

		bool overStepped = false;

		// just in case our value is somehow out of bounds
		if(newValue < _minValue)
		{
			newValue = _minValue;
			overStepped = true;
		}
		else if(newValue > _maxValue)
		{
			newValue = _maxValue;	
			overStepped = true;
		}

		_value = newValue;

		return !overStepped;
	}
}
