#ifndef __MANIPULATORPARAMETERS_H_INCLUDED__
#define __MANIPULATORPARAMETERS_H_INCLUDED__

#include "TypeDefinitions.h"
#include "SystemParameters.h"

class ManipulatorParameters
{
	private:
		ShortInteger num_dof;
		SystemParameters* links_params;
		SystemParameters* end_eff_params;	
	public:
		ManipulatorParameters(ShortInteger number_of_degrees_of_freedom, SystemParameters links_parameters[], SystemParameters* end_effector_parameters);
		ManipulatorParameters(void);
		~ManipulatorParameters(void);
		void operator=(const ManipulatorParameters& right_side);
		ShortInteger get_number_of_degrees_of_freedom(void);
		SystemParameters* get_links_parameters(void);
		SystemParameters* get_end_effector_parameters(void);
};

#endif
