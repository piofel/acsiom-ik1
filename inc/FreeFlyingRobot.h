#ifndef __FREEFLYINGROBOT_H_INCLUDED__
#define __FREEFLYINGROBOT_H_INCLUDED__

#include "MultiManipRobot.h"

class FreeFlyingRobot : public MultiManipRobot 
{
	public:
		FreeFlyingRobot(MultiManipRobotParameters* multi_manip_robot_parameters);
		FreeFlyingRobot(void);	
		DualQuaternion velocity(FrameSymbol* frame_of_interest, FrameSymbol* frame_of_expression, DualQuaternion* frame_of_interest_to_master, DualQuaternion* frame_of_expression_to_master, RealNumMatrix* base_pose, RealNumMatrix* joint_vector, RealNumMatrix* base_pose_rate, RealNumMatrix* joint_vector_rate);
		DualQuaternion dual_momentum(RealNumMatrix* base_pose, RealNumMatrix* joint_vector, RealNumMatrix* base_pose_rate, RealNumMatrix* joint_vector_rate);
};

#endif
