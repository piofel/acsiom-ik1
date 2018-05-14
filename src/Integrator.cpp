#include "Integrator.h"

Integrator::Integrator(RealNumMatrix* initial_state)
{
	state = *initial_state;
}

void Integrator::integrate(RealNumMatrix* derivative, RealNumber time_differential)
{
	Errors er;
	if(state.is_the_same_size_as(derivative))
	{
		state = state + derivative->multiply_by_scalar(time_differential);
	}
	else
	{
		er.display_error(10, "Integrator::integrate");
	}
}

RealNumMatrix Integrator::get_state(void)
{
	return  state;
}
