#ifndef __FREEFLOATINGROBOT_H_INCLUDED__
#define __FREEFLOATINGROBOT_H_INCLUDED__

#include "MultiManipRobot.h"

class FreeFloatingRobot : public MultiManipRobot 
{
	public:
		FreeFloatingRobot(MultiManipRobotParameters* multi_manip_robot_parameters);
		FreeFloatingRobot(void);	
		RealNumMatrix base_velocity(RealNumMatrix* base_pose, RealNumMatrix* joint_vector, RealNumMatrix* joint_vector_rate, DualQuaternion* dual_momentum);
		DualQuatMatrix generalized_jacobian_matrix(RealNumMatrix* base_pose, RealNumMatrix* joint_vector);
		RealNumber condition_of_generalized_jacobian_matrix(RealNumMatrix* base_pose, RealNumMatrix* joint_vector);
		DualQuatMatrix velocity_dependent_on_initial_momentum(DualQuaternion* initial_momentum, RealNumMatrix* base_pose, RealNumMatrix* joint_vector);
		DualQuatMatrix end_effectors_velocities(RealNumMatrix* base_pose, RealNumMatrix* joint_vector, RealNumMatrix* joint_vector_rate, DualQuaternion* initial_momentum);
		RealNumMatrix inverse_velocity_kinematics(DualQuaternion* initial_momentum, DualQuatMatrix* end_effectors_velocities, RealNumMatrix* redundant_joints_velocities, RealNumMatrix* base_pose, RealNumMatrix* joint_vector);
};

#endif
