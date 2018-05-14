#ifndef __BASE_H_INCLUDED__
#define __BASE_H_INCLUDED__ 

#include "RigidBody.h"

class Base : public RigidBody
{
	private:
		FrameSymbol main_frame;
	public:
		Base(SystemParameters* parameters);
		void operator=(const Base& right_side);
		Base(void);
		FrameSymbol* get_main_frame_symbol(void);
		Quaternion get_axis_of_rotation_as_quaternion_in_base_frame(ShortInteger axis_number, RealNumMatrix* base_pose);
};

#endif

