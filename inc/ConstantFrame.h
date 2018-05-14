#ifndef __CONSTANTFRAME_H_INCLUDED__
#define __CONSTANTFRAME_H_INCLUDED__

#include "FrameSymbol.h"
#include "DualQuaternion.h"
#include "SpatialTransforms.h"

class ConstantFrame
{
	private:
		RealNumMatrix* pos;
		RealNumMatrix* orn;
		DualQuaternion* dqr;
		FrameSymbol s;
	public:
		ConstantFrame(RealNumber position[], RealNumber orientation[], ShortInteger master_frame_number, ShortInteger master_manipulator_number);
		ConstantFrame(void);
		~ConstantFrame(void);
		void operator=(const ConstantFrame& right_side);
		RealNumber get_position(ShortInteger axis_number);
		RealNumber get_orientation(ShortInteger axis_number);
		RealNumMatrix* get_position(void);
		RealNumMatrix* get_orientation(void);
		DualQuaternion get_dual_quaternion_representation(void);
		FrameSymbol* get_frame_symbol(void);
};

#endif
