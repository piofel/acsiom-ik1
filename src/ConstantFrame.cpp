#include "ConstantFrame.h"

ConstantFrame::ConstantFrame(RealNumber position[], RealNumber orientation[], ShortInteger master_frame_number, ShortInteger master_manipulator_number)
{
	pos = new RealNumMatrix;
	orn = new RealNumMatrix;
	dqr = new DualQuaternion;
	*pos = RealNumMatrix(3,1);
	*orn = RealNumMatrix(3,1);
	SpatialTransforms st;
	ShortInteger i;
	for(i=0; i<3; i++)
	{
		pos->set_element(i+1,1,position[i]);
		orn->set_element(i+1,1,orientation[i]);
	}
	*dqr = st.translation_roll_pitch_yaw_to_dual_quaternion(pos, orn);
	s = FrameSymbol(master_frame_number, master_manipulator_number);
}

ConstantFrame::ConstantFrame(void)
{
	pos = new RealNumMatrix;
	orn = new RealNumMatrix;
	dqr = new DualQuaternion;
	s = FrameSymbol(-9, -9);
}

ConstantFrame::~ConstantFrame(void)
{
	delete pos;
	delete orn;
	delete dqr;
}

void ConstantFrame::operator=(const ConstantFrame& right_side)
{
	s = right_side.s;
	*pos = *right_side.pos;
	*orn = *right_side.orn;
	*dqr = *right_side.dqr;
}

RealNumber ConstantFrame::get_position(ShortInteger axis_number)
{
	return pos->get_element(axis_number,1);
}

RealNumber ConstantFrame::get_orientation(ShortInteger axis_number)
{
	return orn->get_element(axis_number,1);
}

DualQuaternion ConstantFrame::get_dual_quaternion_representation(void)
{
	return *dqr;
}

RealNumMatrix* ConstantFrame::get_position(void)
{
	return pos;
}

RealNumMatrix* ConstantFrame::get_orientation(void)
{
	return orn;
}

FrameSymbol* ConstantFrame::get_frame_symbol(void)
{
	return &s;
}
