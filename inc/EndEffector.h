#ifndef __ENDEFFECTOR_H_INCLUDED__
#define __ENDEFFECTOR_H_INCLUDED__

#include "SystemParameters.h"
#include "ConstantFrame.h"
#include "RigidBody.h"

class EndEffector : public RigidBody
{
	private:
		ConstantFrame main_frame;
	public:
		EndEffector(SystemParameters* parameters, ShortInteger master_frame_number, ShortInteger master_manipulator_number);
		EndEffector(void);
		ConstantFrame* get_main_frame(void);
};

#endif
