#ifndef __SYSTEMPARAMETERS_H_INCLUDED__
#define __SYSTEMPARAMETERS_H_INCLUDED__

#include "TypeDefinitions.h"
#include "Errors.h"

class SystemParameters
{
	private:
		UnsignShortInteger int_params_num;
		UnsignShortInteger float_params_num;
		Integer* int_params;
		RealNumber* float_params;
	public:
		SystemParameters(UnsignShortInteger number_of_int_parameters, UnsignShortInteger number_of_float_parameters, Integer int_parameters[], RealNumber float_parameters[]);
		SystemParameters(void);
		~SystemParameters(void);
		void operator=(const SystemParameters& right_side);
		UnsignShortInteger get_number_of_int_parameters(void);
		UnsignShortInteger get_number_of_float_parameters(void);
		Integer get_int_parameter(UnsignShortInteger int_parameter_number);
		RealNumber get_float_parameter(UnsignShortInteger float_parameter_number);
};

#endif
