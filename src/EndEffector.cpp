#include "EndEffector.h"

EndEffector::EndEffector(SystemParameters* parameters, ShortInteger master_frame_number, ShortInteger master_manipulator_number) : RigidBody(parameters, master_frame_number, master_manipulator_number)
{
	ShortInteger u, i;
	RealNumber p[3], o[3];
	SpatialTransforms st;
	for(i=0; i<3; i++)
	{
		u = parameters->get_int_parameter(i+4);
		p[i] = parameters->get_float_parameter(i+10);
		o[i] =  parameters->get_float_parameter(i+13);
		if(u == 1)
		{
			o[i] = st.deg_to_rad(o[i]);
		}
	}
	ConstantFrame f(p,o,master_frame_number, master_manipulator_number);
	main_frame = f;
}

EndEffector::EndEffector(void)
{
}

ConstantFrame* EndEffector::get_main_frame(void)
{
	return &main_frame;
}
