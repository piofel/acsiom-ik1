#ifndef __LINK_H_INCLUDED__
#define __LINK_H_INCLUDED__

#include "RigidBody.h"
	
class Link : public RigidBody
{
	private: 
		FrameSymbol joint_frame;
		RealNumber preceding_link_length, preceding_link_twist;
		UnsignShortInteger joint_type;
	public:
		Link(SystemParameters* parameters, ShortInteger master_frame_number, ShortInteger master_manipulator_number);
		Link(void);
		void operator=(const Link& right_side);
		DualQuaternion calc_joint_frame(RealNumber joint_offset, RealNumber joint_angle); //calculates frame of joint i relative to joint i-1
		RealNumber get_preceding_link_length(void);
		RealNumber get_preceding_link_twist(void);
		FrameSymbol* get_joint_frame_symbol(void);
		bool is_joint_redundant(void);
		bool is_joint_prismatic(void);
		bool is_joint_revolute(void);
};

#endif
