#include "Base.h"

Base::Base(SystemParameters* parameters) : RigidBody(parameters, 0, 0)
{
	main_frame = FrameSymbol(0,0,' ','0',' ');
}

Base::Base(void) : RigidBody()
{
	main_frame = FrameSymbol(0,0,' ','0',' ');
}

void Base::operator=(const Base& right_side)
{
	ShortInteger i;
	mass = right_side.mass;
	main_frame = right_side.main_frame;
	center_of_mass_frame = right_side.center_of_mass_frame;
	for(i=0; i<3; i++)
	{
		mass_moments_of_inertia[i] = right_side.mass_moments_of_inertia[i];
	}
}

FrameSymbol* Base::get_main_frame_symbol(void)
{
	return &main_frame;
}

Quaternion Base::get_axis_of_rotation_as_quaternion_in_base_frame(ShortInteger axis_number, RealNumMatrix* base_pose)
{
	Axes ax;
	Quaternion axis, inertial_to_base_orient;
	SpatialTransforms st;
	DualQuaternion base_to_inertial_pose;
	axis = *ax.get_axis_in_quaternion_form(axis_number);
	base_to_inertial_pose = st.translation_roll_pitch_yaw_to_dual_quaternion(base_pose);
	inertial_to_base_orient = base_to_inertial_pose.get_real_part();
	inertial_to_base_orient = inertial_to_base_orient.conjugate(); 
	axis = inertial_to_base_orient * axis * inertial_to_base_orient.conjugate();  
	return axis;
}
