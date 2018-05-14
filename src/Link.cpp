#include "Link.h"

Link::Link(SystemParameters* parameters, ShortInteger master_frame_number, ShortInteger master_manipulator_number) : RigidBody(parameters, master_frame_number, master_manipulator_number)
{
	ShortInteger u;
	SpatialTransforms st;
	preceding_link_length = parameters->get_float_parameter(10);
	u = parameters->get_int_parameter(4);
	preceding_link_twist = parameters->get_float_parameter(11);
	if(u == 1)
	{
		preceding_link_twist = st.deg_to_rad(preceding_link_twist);
	}
	joint_frame = FrameSymbol(master_frame_number, master_manipulator_number);
	joint_type = parameters->get_int_parameter(5);
}

Link::Link(void) : RigidBody()
{
}

void Link::operator=(const Link& right_side)
{
	UnsignShortInteger i;
	mass = right_side.mass;
	center_of_mass_frame = right_side.center_of_mass_frame;
	for(i=0;i<3;i++)
	{
		mass_moments_of_inertia[i] = right_side.mass_moments_of_inertia[i];
	}
	joint_frame = right_side.joint_frame;
	preceding_link_length = right_side.preceding_link_length;
	preceding_link_twist = right_side.preceding_link_twist;
	joint_type = right_side.joint_type;
}

DualQuaternion Link::calc_joint_frame(RealNumber joint_offset, RealNumber joint_angle)
{
	//calculates frame of joint i relative to joint i-1
	SpatialTransforms st;
	Axes a;
	RealNumMatrix* axis_1;
	RealNumMatrix* axis_3;
	axis_1 = a.get_axis(1);
	axis_3 = a.get_axis(3);
	/*
	Quaternion r(1,0,0,0);
	DualQuaternion rot_twist =  st.angle_axis_to_dual_quaternion(preceding_link_twist, axis_1);
	DualQuaternion trans_length =  st.distance_axis_to_dual_quaternion(preceding_link_length, axis_1, &r);
	DualQuaternion rot_joint_angle =  st.angle_axis_to_dual_quaternion(joint_angle, axis_3);
	DualQuaternion trans_joint_offset =  st.distance_axis_to_dual_quaternion(joint_offset, axis_3, &r);
	return rot_twist * trans_length * rot_joint_angle * trans_joint_offset;
	*/
	DualQuaternion screw1, screw2;
	screw1 = st.screw_transform(axis_1,preceding_link_length,preceding_link_twist);
	screw2 = st.screw_transform(axis_3,joint_offset,joint_angle);
	return screw1 * screw2;
}

RealNumber Link::get_preceding_link_length(void)
{
	return preceding_link_length;
}

RealNumber Link::get_preceding_link_twist(void)
{
	return preceding_link_twist;
}

FrameSymbol* Link::get_joint_frame_symbol(void)
{
	return &joint_frame;
}

bool Link::is_joint_redundant(void)
{
	bool result;
	if(joint_type==2 || joint_type==3)
	{
		result=true;
	}
	else
	{
		result=false;
	}
	return result;
}

bool Link::is_joint_prismatic(void)
{
	bool result;
	if(joint_type==0 || joint_type==2)
	{
		result=true;
	}
	else
	{
		result=false;
	}
	return result;
}

bool Link::is_joint_revolute(void)
{
	bool result;
	if(joint_type==1 || joint_type==3)
	{
		result=true;
	}
	else
	{
		result=false;
	}
	return result;
}
