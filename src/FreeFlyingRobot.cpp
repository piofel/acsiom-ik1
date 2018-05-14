#include "FreeFlyingRobot.h"

FreeFlyingRobot::FreeFlyingRobot(MultiManipRobotParameters* multi_manip_robot_parameters) : MultiManipRobot(multi_manip_robot_parameters)
{
}

FreeFlyingRobot::FreeFlyingRobot(void) : MultiManipRobot()
{
}

DualQuaternion FreeFlyingRobot::velocity(FrameSymbol* frame_of_interest, FrameSymbol* frame_of_expression, DualQuaternion* frame_of_interest_to_master, DualQuaternion* frame_of_expression_to_master, RealNumMatrix* base_pose, RealNumMatrix* joint_vector, RealNumMatrix* base_pose_rate, RealNumMatrix* joint_vector_rate)
{
	ShortInteger cb, zb, i;
	RealNumMatrix joint_vector_rate_sm;
	DualQuaternion result, e;
	DualQuatMatrix jacobian;
	DualQuatMatrix result_dqm(1,1);
	result_dqm.set_to_zero();
	result.set_to_zero();
	zb = frame_of_interest->get_master_frame_number();
	if(zb >= 0)
	{
		jacobian = jacobian_dep_base(frame_of_interest, frame_of_expression, frame_of_interest_to_master, frame_of_expression_to_master, base_pose, joint_vector);
		result_dqm = jacobian.multiply_by_real_num_matrix(base_pose_rate);
	}
	if(zb >= 1)
	{
		cb = frame_of_interest->get_master_manipulator_number();
		joint_vector_rate_sm = get_vector_of_single_manip_from_joint_var(joint_vector_rate, cb);
		jacobian = jacobian_dep_manip(frame_of_interest, frame_of_expression, frame_of_interest_to_master, frame_of_expression_to_master, base_pose, joint_vector);
		for(i=1; i<=zb; i++)
		{
			e = jacobian.get_element(1,i);
			e = e.multiply_by_scalar_r(joint_vector_rate_sm.get_element(i,1));
			result = result + e; 
		}
	}
	result = result + result_dqm.get_element(1,1);
	return result;
}

DualQuaternion FreeFlyingRobot::dual_momentum(RealNumMatrix* base_pose, RealNumMatrix* joint_vector, RealNumMatrix* base_pose_rate, RealNumMatrix* joint_vector_rate)
{
	UnsignShortInteger c;
	RealNumber n;
	DualQuaternion result;
	DualQuatMatrix n0, n0bpr, nm, nmjvr;
	n0 = inertia_dependent_on_base(base_pose, joint_vector);
	n0bpr = n0.multiply_by_real_num_matrix(base_pose_rate);
	result = n0bpr.get_element(1,1);
	nm = inertia_dependent_on_manip(base_pose, joint_vector);
	nmjvr = nm.multiply_by_real_num_matrix(joint_vector_rate);
	result = result + nmjvr.get_element(1,1);
	return result;
}
