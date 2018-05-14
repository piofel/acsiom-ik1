#ifndef __INTEGRATOR_H_INCLUDED__
#define __INTEGRATOR_H_INCLUDED__

#include "RealNumMatrix.h"

class Integrator
{
	private:
		RealNumMatrix state;
	public:
		Integrator(RealNumMatrix* initial_state);
		void integrate(RealNumMatrix* derivative, RealNumber time_differential);
		RealNumMatrix get_state(void);
};

#endif
