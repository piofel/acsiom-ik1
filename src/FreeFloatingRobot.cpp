#include "FreeFloatingRobot.h"

FreeFloatingRobot::FreeFloatingRobot(MultiManipRobotParameters* multi_manip_robot_parameters) : MultiManipRobot(multi_manip_robot_parameters)
{
}

FreeFloatingRobot::FreeFloatingRobot(void) : MultiManipRobot()
{
}

RealNumMatrix FreeFloatingRobot::base_velocity(RealNumMatrix* base_pose, RealNumMatrix* joint_vector, RealNumMatrix* joint_vector_rate, DualQuaternion* dual_momentum)
{
	DualQuatMatrix n0, nm, nmjvr;
	DualQuaternion e;
	RealNumMatrix mc, n0m, nmjvrc, diff;
	RealNumMatrix result(6,1);
	n0 = inertia_dependent_on_base(base_pose, joint_vector);
	n0m = n0.real_matrix_6();
	nm = inertia_dependent_on_manip(base_pose, joint_vector);
	nmjvr = nm.multiply_by_real_num_matrix(joint_vector_rate);
	e = nmjvr.get_element(1,1);
	nmjvrc = e.column_6();
	mc = dual_momentum->column_6();
	diff = mc - nmjvrc;
	result = n0m.solve_linear_equation(&diff);
	return result;
}

DualQuatMatrix FreeFloatingRobot::generalized_jacobian_matrix(RealNumMatrix* base_pose, RealNumMatrix* joint_vector)
{
	DualQuatMatrix jem, je0, result, s, n0, nm;
	RealNumMatrix n0m, n0i, nmm, n0inmm;
	DualQuaternion identity; 
	identity.set_to_identity();
	FrameSymbol inertial_frame_symbol(-1,-1);
	jem = jacobian_dep_manip_2(&inertial_frame_symbol, &identity, base_pose, joint_vector);
	je0 = jacobian_dep_base_2(&inertial_frame_symbol, &identity, base_pose, joint_vector);
	n0 = inertia_dependent_on_base(base_pose, joint_vector);
	n0m = n0.real_matrix_6();
	n0i = n0m.inverse();
	nm = inertia_dependent_on_manip(base_pose, joint_vector);
	nmm = nm.real_matrix_6();
	n0inmm = n0i.dot(&nmm);
	s = je0.multiply_by_real_num_matrix(&n0inmm);
	result = jem - s;
	return result;
}

DualQuatMatrix FreeFloatingRobot::velocity_dependent_on_initial_momentum(DualQuaternion* initial_momentum, RealNumMatrix* base_pose, RealNumMatrix* joint_vector)
{
	DualQuatMatrix je0, result, n0;
	RealNumMatrix n0m, n0i, mc, n0imc;
	DualQuaternion identity; 
	identity.set_to_identity();
	FrameSymbol inertial_frame_symbol(-1,-1);
	mc = initial_momentum->column_6();
	n0 = inertia_dependent_on_base(base_pose, joint_vector);
	n0m = n0.real_matrix_6();
	n0i = n0m.inverse();
	je0 = jacobian_dep_base_2(&inertial_frame_symbol, &identity, base_pose, joint_vector);
	n0imc = n0i.dot(&mc);
	result = je0.multiply_by_real_num_matrix(&n0imc);
	return result;
}

DualQuatMatrix FreeFloatingRobot::end_effectors_velocities(RealNumMatrix* base_pose, RealNumMatrix* joint_vector, RealNumMatrix* joint_vector_rate, DualQuaternion* initial_momentum)
{
	DualQuatMatrix gjm, vm, gjmjvr;
	gjm = generalized_jacobian_matrix(base_pose, joint_vector);
	vm = velocity_dependent_on_initial_momentum(initial_momentum, base_pose, joint_vector);
	gjmjvr = gjm.multiply_by_real_num_matrix(joint_vector_rate);
	return gjmjvr + vm;
}

RealNumMatrix FreeFloatingRobot::inverse_velocity_kinematics(DualQuaternion* initial_momentum, DualQuatMatrix* end_effectors_velocities, RealNumMatrix* redundant_joints_velocities, RealNumMatrix* base_pose, RealNumMatrix* joint_vector)
{
	DualQuatMatrix gjm, vm, redundant_jacobian, velocity_from_redundant_dofs, velocity_from_principal_joints;
	RealNumMatrix mgjm, igjm, m_velocity_from_principal_joints, result;
	Errors er;
	gjm = generalized_jacobian_matrix(base_pose, joint_vector);
	redundant_jacobian = extract_redundant_degrees_of_freedom(&gjm);
	if(redundant_jacobian.get_number_of_columns() == redundant_joints_velocities->get_number_of_rows())
	{
		vm = velocity_dependent_on_initial_momentum(initial_momentum, base_pose, joint_vector);
		velocity_from_redundant_dofs = redundant_jacobian.multiply_by_real_num_matrix(redundant_joints_velocities);
		velocity_from_principal_joints = *end_effectors_velocities - vm - velocity_from_redundant_dofs;
		m_velocity_from_principal_joints = velocity_from_principal_joints.real_matrix_6();
		mgjm = gjm.real_matrix_6();
		result = mgjm.solve_linear_equation(&m_velocity_from_principal_joints);
	}
	else
	{
		er.display_error(26, "FreeFloatingRobot::inverse_differential_kinematics");
	}
	return result;
}

RealNumber FreeFloatingRobot::condition_of_generalized_jacobian_matrix(RealNumMatrix* base_pose, RealNumMatrix* joint_vector)
{
	RealNumber cond;
	DualQuatMatrix gjm;
	RealNumMatrix mgjm;
	gjm = generalized_jacobian_matrix(base_pose, joint_vector);
	mgjm = gjm.real_matrix_6();
	cond = mgjm.condition_number();
	return cond;
}
