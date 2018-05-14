#include "MultiManipRobotParameters.h"

MultiManipRobotParameters::MultiManipRobotParameters(ShortInteger number_of_manipulators, ManipulatorParameters manipulators_parameters[], SystemParameters* base_satellite_parameters)
{
	ShortInteger i;
	num_man = number_of_manipulators;	
	base_params = new SystemParameters;
	man_params = new ManipulatorParameters[num_man];
	*base_params = *base_satellite_parameters;
	for(i=0; i<num_man; i++)
	{
		man_params[i] = manipulators_parameters[i];
	}
}

MultiManipRobotParameters::MultiManipRobotParameters(void)
{
	base_params = new SystemParameters;
	man_params = new ManipulatorParameters[1];
}

MultiManipRobotParameters::~MultiManipRobotParameters(void)
{
	delete base_params;
	delete [] man_params;
}

ShortInteger MultiManipRobotParameters::get_number_of_manipulators(void)
{
	return num_man;
}

ManipulatorParameters* MultiManipRobotParameters::get_manipulators_parameters(void)
{
	return man_params;
}

SystemParameters* MultiManipRobotParameters::get_base_satellite_parameters(void)
{
	return base_params;
}
