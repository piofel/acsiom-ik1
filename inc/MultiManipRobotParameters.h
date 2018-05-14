#ifndef __MULTIMANIPROBOTPARAMETERS_H_INCLUDED__
#define __MULTIMANIPROBOTPARAMETERS_H_INCLUDED__

#include "TypeDefinitions.h"
#include "ManipulatorParameters.h"
#include "SystemParameters.h"

class MultiManipRobotParameters
{
	private:
		ShortInteger num_man;
		ManipulatorParameters* man_params;
		SystemParameters* base_params;	
	public:
		MultiManipRobotParameters(ShortInteger number_of_manipulators, ManipulatorParameters manipulators_parameters[], SystemParameters* base_satellite_parameters);
		MultiManipRobotParameters(void);
		~MultiManipRobotParameters(void);
		ShortInteger get_number_of_manipulators(void);
		ManipulatorParameters* get_manipulators_parameters(void);
		SystemParameters* get_base_satellite_parameters(void);
};

#endif
