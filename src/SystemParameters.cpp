#include "SystemParameters.h"

SystemParameters::SystemParameters(UnsignShortInteger number_of_int_parameters, UnsignShortInteger number_of_float_parameters, Integer int_parameters[], RealNumber float_parameters[])
{
	UnsignShortInteger i;
	if(number_of_int_parameters>0)
	{
		int_params_num = number_of_int_parameters;
		int_params = new Integer[number_of_int_parameters];
		for(i=0; i<int_params_num; i++)
		{
			int_params[i] = int_parameters[i];
		}	
	}
	else
	{
		int_params = new Integer[1];
		int_params_num = 0;
	}
	if(number_of_float_parameters>0)
	{
		float_params_num = number_of_float_parameters;
		float_params = new RealNumber[number_of_float_parameters];
		for(i=0; i<float_params_num; i++)
		{	
			float_params[i] = float_parameters[i];
		}
	}
	else
	{
		float_params = new RealNumber[1];
		float_params_num = 0;
	}
}

SystemParameters::SystemParameters(void)
{
	int_params = new Integer[1];
	float_params = new RealNumber[1];
	int_params_num = 0;
	float_params_num = 0;
}

SystemParameters::~SystemParameters(void)
{
	delete [] int_params;
	delete [] float_params;
}

void SystemParameters::operator=(const SystemParameters& right_side)
{
	UnsignShortInteger i;
	int_params_num = right_side.int_params_num;
	float_params_num = right_side.float_params_num;
	if(int_params_num>0)
	{
		delete [] int_params;
		int_params = new Integer[int_params_num];
		for(i=0; i<int_params_num; i++)
		{
			int_params[i] = right_side.int_params[i];
		}	
	}
	if(float_params_num>0)
	{
		delete [] float_params;
		float_params = new RealNumber[float_params_num];
		for(i=0; i<float_params_num; i++)
		{
			float_params[i] = right_side.float_params[i];
		}
	}
}

UnsignShortInteger SystemParameters::get_number_of_int_parameters(void)
{
	return int_params_num;
}

UnsignShortInteger SystemParameters::get_number_of_float_parameters(void)
{
	return float_params_num;
}

Integer SystemParameters::get_int_parameter(UnsignShortInteger int_parameter_number)
{
	//indexed from 0
	Errors er;
	Integer p;
	if(int_parameter_number>=0 && int_parameter_number<int_params_num)
	{
		p = int_params[int_parameter_number];
	}
	else
	{
		er.display_error(24, "SystemParameters::get_int_parameter");
		p=0;
	}
	return p;
}

RealNumber SystemParameters::get_float_parameter(UnsignShortInteger float_parameter_number)
{
	//indexed from 0
	Errors er;
	RealNumber p;
	if(float_parameter_number>=0 && float_parameter_number<float_params_num)
	{
		p = float_params[float_parameter_number];
	}
	else
	{
		er.display_error(24, "SystemParameters::get_float_parameter");
		p=0;
	}
	return p;
}
