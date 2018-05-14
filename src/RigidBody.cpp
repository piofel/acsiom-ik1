#include "RigidBody.h"

RigidBody::RigidBody(SystemParameters* parameters, ShortInteger master_frame_number, ShortInteger master_manipulator_number)
{
	ShortInteger i, u;
	RealNumber p[3], o[3];
	SpatialTransforms st;
	mass = parameters->get_float_parameter(0);		
	for(i=0; i<3; i++)
	{
		u = parameters->get_int_parameter(i+1);
		p[i] = parameters->get_float_parameter(i+1);
		o[i] =  parameters->get_float_parameter(i+4);
		if(u == 1)
		{
			o[i] = st.deg_to_rad(o[i]);
		}
		mass_moments_of_inertia[i] = parameters->get_float_parameter(i+7);
	}
	ConstantFrame comf(p, o, master_frame_number, master_manipulator_number);
	center_of_mass_frame = comf;
}

RigidBody::RigidBody(void)
{
}

RealNumber RigidBody::get_mass(void)
{
	return mass;
}

ConstantFrame* RigidBody::get_center_of_mass_frame(void)
{
	return &center_of_mass_frame;
}

RealNumber* RigidBody::get_mass_moments_of_inertia(void)
{
	return mass_moments_of_inertia;
}

DualMatrix RigidBody::get_dual_inertia_matrix(void)
{
	// the inertia matrix is relative to the center of mass of the body
	ShortInteger c;
	RealNumMatrix r(4,4);
	RealNumMatrix d(4,4);
	r.set_to_zero();
	d.set_to_zero();
	for(c=2;c<=4;c++)
	{
		r.set_element(c,c,mass);
		d.set_element(c,c,mass_moments_of_inertia[c-2]);
	}
	DualMatrix result(r,d);
	result.set_differential_operator_in_real_part();
	return result;
}
