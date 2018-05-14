#include "ManipulatorParameters.h"

ManipulatorParameters::ManipulatorParameters(ShortInteger number_of_degrees_of_freedom, SystemParameters links_parameters[], SystemParameters* end_effector_parameters)
{
	ShortInteger i;
	num_dof = number_of_degrees_of_freedom;
	end_eff_params = new SystemParameters;
	links_params = new SystemParameters[num_dof];
	*end_eff_params = *end_effector_parameters;
	for(i=0; i<num_dof; i++)
	{
		links_params[i] = links_parameters[i];	
	}
}

ManipulatorParameters::ManipulatorParameters(void)
{
	end_eff_params = new SystemParameters;
	links_params = new SystemParameters[1];
}

ManipulatorParameters::~ManipulatorParameters(void)
{
	delete end_eff_params;
	delete [] links_params;
}

void ManipulatorParameters::operator=(const ManipulatorParameters& right_side)
{
	ShortInteger i;
	num_dof = right_side.num_dof;
	delete end_eff_params;
	delete [] links_params;
	end_eff_params = new SystemParameters;
	*end_eff_params = *right_side.end_eff_params;
	links_params = new SystemParameters[num_dof];
	for(i=0; i<num_dof; i++)
	{
		links_params[i] = right_side.links_params[i];
	}
}

ShortInteger ManipulatorParameters::get_number_of_degrees_of_freedom(void)
{
	return num_dof;	
}

SystemParameters* ManipulatorParameters::get_links_parameters(void)
{
	return links_params;
}

SystemParameters* ManipulatorParameters::get_end_effector_parameters(void)
{
	return end_eff_params;
}
