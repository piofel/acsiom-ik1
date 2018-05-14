#include "Simulator.h"

Simulator::Simulator(void)
{
	FileOperator fo;
	Errors er;
	MultiManipRobotParameters rp = fo.read_multi_manip_robot_parameters();
	float_robot = new FreeFloatingRobot[1];
	fly_robot = new FreeFlyingRobot[1];
	float_robot[0] = FreeFloatingRobot(&rp);
	fly_robot[0] = FreeFlyingRobot(&rp);
	time_differential = DEFAULT_TIME_DIFFERENTIAL;
	simulation_start_time = DEFAULT_SIMULATION_START_TIME; 
	simulation_stop_time = DEFAULT_SIMULATION_STOP_TIME;
}

void Simulator::reload_parameters(void)
{
	FileOperator fo;
	MultiManipRobotParameters rp = fo.read_multi_manip_robot_parameters();
	float_robot[0] = FreeFloatingRobot(&rp);
	fly_robot[0] = FreeFlyingRobot(&rp);
}

Simulator::~Simulator(void)
{
	delete [] fly_robot;
	delete [] float_robot;
}

FreeFlyingRobot* Simulator::get_free_flying_robot(void)
{
	return fly_robot;
}

FreeFloatingRobot* Simulator::get_free_floating_robot(void)
{
	return float_robot;
}

DualQuaternion Simulator::get_pose_of_frame_to_its_master_frame(FrameSymbol* frame, char robot_type)
{
	ShortInteger chi;
	char sp;
	Manipulator* m;
	EndEffector* e;
	DualQuaternion frame_to_master;
	ConstantFrame* eemf;
	Errors er;
	Quaternion zero(0,0,0,0);
	Quaternion one(1,0,0,0);
	DualQuaternion identity(one, zero);
	frame_to_master = identity;
	sp = frame->get_symbol_part(1);
	if(sp!='N'&&sp!='0'&&sp!='l'&&sp!='E')
	{
		er.display_error(1);
	}
	else
	{
		if(sp=='E')
		{
			chi = frame->get_master_manipulator_number();
			if(robot_type=='y')
			{
				m = fly_robot[0].get_manipulator(chi);
			}
			if(robot_type=='o')
			{
				m = float_robot[0].get_manipulator(chi);
			}
			e = m->get_end_effector();
			eemf = e->get_main_frame();
			frame_to_master = eemf->get_dual_quaternion_representation();
		}
	}
	return frame_to_master;
}

DualQuaternion Simulator::free_flyer_pose_simulation(FrameSymbol* frame_of_expression, FrameSymbol* expressed_frame)
{
	DualQuaternion result, frame_of_expression_to_master, expressed_frame_to_master;
	FileOperator fo;
	RealNumMatrix base_pose, joint_vector;
	frame_of_expression_to_master = get_pose_of_frame_to_its_master_frame(frame_of_expression, 'y');
	expressed_frame_to_master = get_pose_of_frame_to_its_master_frame(expressed_frame, 'y');
	base_pose = fo.read_base_pose_derivative(0);	
	joint_vector = fo.read_joint_vector(fly_robot);
	result = fly_robot[0].pose(frame_of_expression, expressed_frame, &frame_of_expression_to_master, &expressed_frame_to_master, &base_pose, &joint_vector);
	return result;
}

DualQuaternion Simulator::free_flyer_velocity_simulation(FrameSymbol* frame_of_interest, FrameSymbol* frame_of_expression)
{
	DualQuaternion result, frame_of_expression_to_master, frame_of_interest_to_master;
	FileOperator fo;
	RealNumMatrix base_pose, joint_vector, base_pose_rate, joint_vector_rate;
	frame_of_expression_to_master = get_pose_of_frame_to_its_master_frame(frame_of_expression, 'y');
	frame_of_interest_to_master = get_pose_of_frame_to_its_master_frame(frame_of_interest, 'y');
	base_pose = fo.read_base_pose_derivative(0);	
	joint_vector = fo.read_joint_vector(fly_robot);
	base_pose_rate = fo.read_base_pose_derivative(1);	
	joint_vector_rate = fo.read_joint_velocities(fly_robot);
	result = fly_robot[0].velocity(frame_of_interest, frame_of_expression, &frame_of_interest_to_master, &frame_of_expression_to_master, &base_pose, &joint_vector, &base_pose_rate, &joint_vector_rate);
	return result;
}

DualQuatMatrix Simulator::free_floater_velocity_simulation(void)
{
	DualQuatMatrix result;
	DualQuaternion initial_momentum;
	RealNumMatrix base_pose, joint_vector, joint_vector_rate;
	FileOperator fo;
	base_pose = fo.read_base_pose_derivative(0);	
	joint_vector = fo.read_joint_vector(float_robot);
	joint_vector_rate = fo.read_joint_velocities(float_robot);
	initial_momentum = fo.read_initial_momentum();
	result = float_robot[0].end_effectors_velocities(&base_pose, &joint_vector, &joint_vector_rate, &initial_momentum);
	return result;
}

RealNumMatrix Simulator::free_floater_inverse_velocity_kinematics_simulation(void)
{
	DualQuaternion initial_momentum;
	RealNumMatrix result, base_pose, joint_vector, redundant_joint_velocities;
	DualQuatMatrix end_effectors_velocities;
	FileOperator fo;
	base_pose = fo.read_base_pose_derivative(0);	
	joint_vector = fo.read_joint_vector(float_robot);
	initial_momentum = fo.read_initial_momentum();
	end_effectors_velocities = fo.read_end_effectors_velocities(float_robot);
	redundant_joint_velocities = fo.read_redundant_joints_velocities(float_robot);
	result = float_robot[0].inverse_velocity_kinematics(&initial_momentum, &end_effectors_velocities, &redundant_joint_velocities, &base_pose, &joint_vector);
	return result;
}

void Simulator::free_floater_trajectory_simulation(char output_type)
{
	bool first_row;
	UnsignShortInteger nm;
	RealNumber time, n;
	Errors er;
	RealNumMatrix* output_m;
	RealNumMatrix trajectories, base_pose, base_pose_rate, initial_base_pose, joint_variables_rate, joint_variables, joint_offsets_angles, initial_joint_variables, initial_joint_offsets_angles, output_tsr, end_eff_velocities_m, m, error_m;
	DualQuaternion initial_momentum;
	DualQuatMatrix end_eff_velocities, desired_end_effectors_velocities, error;
	FileOperator fo;
	RealNumMatrix redundant_joints_velocities(1,1);
	nm = float_robot[0].get_number_of_manipulators();
	RealNumMatrix initial_end_eff_pose(nm*6,1);
	initial_end_eff_pose.set_to_zero();
	initial_base_pose = fo.read_base_pose_derivative(0);	
	initial_joint_offsets_angles = fo.read_joint_vector(float_robot);
	initial_joint_variables = float_robot[0].get_joint_variables_vector_from_off_ang_vec(&initial_joint_offsets_angles);
	Integrator joint_velocity_vector_integrator(&initial_joint_variables);
	Integrator base_velocity_integrator(&initial_base_pose);
	Integrator end_eff_velocities_integrator(&initial_end_eff_pose);
	initial_momentum = fo.read_initial_momentum();
	desired_end_effectors_velocities = fo.read_end_effectors_velocities(float_robot);
	redundant_joints_velocities.set_to_zero();
	first_row = true;
	time = simulation_start_time;
	while(time<=simulation_stop_time)
	{
		joint_variables = joint_velocity_vector_integrator.get_state();
		joint_offsets_angles = float_robot[0].extend_joint_variables_vector_to_off_ang_vec(&initial_joint_offsets_angles, &joint_variables);
		base_pose = base_velocity_integrator.get_state();
		joint_variables_rate = float_robot[0].inverse_velocity_kinematics(&initial_momentum, &desired_end_effectors_velocities, &redundant_joints_velocities, &base_pose, &joint_offsets_angles);
		end_eff_velocities = float_robot[0].end_effectors_velocities(&base_pose, &joint_offsets_angles, &joint_variables_rate, &initial_momentum);
		end_eff_velocities_m = end_eff_velocities.real_matrix_6();
		switch(output_type)
		{
			case 'b':
				output_m = &base_pose;
				break;
			case 'j':
				output_m = &joint_variables;
				break;
			case 'c':
				output_m = &joint_variables_rate;
				break;
			case 'v':
				output_m = &end_eff_velocities_m;
				break;
			case 'p':
				m = end_eff_velocities_integrator.get_state();
				output_m = &m;
				break;
			case 'e':
				error = desired_end_effectors_velocities - end_eff_velocities;
				error_m = error.real_matrix_6();
				output_m = &error_m;
				break;
			case 'n':
				n = float_robot[0].condition_of_generalized_jacobian_matrix(&base_pose, &joint_offsets_angles);
				m = RealNumMatrix(1,1);
				m.set_element(1,1,n);
				output_m = &m;
				break;
			default:
				output_m = &end_eff_velocities_m;
				er.display_error(8, "Simulator::free_floater_end_effectors_trajectory_simulation");
		}
		output_tsr = convert_to_timeseries_row(output_m, time);
		if(first_row)
		{
			first_row = false;
			trajectories = output_tsr;
		}
		else
		{
			trajectories.append_row(&output_tsr, true);
		}
		base_pose_rate = float_robot[0].base_velocity(&base_pose, &joint_offsets_angles, &joint_variables_rate, &initial_momentum);
		base_velocity_integrator.integrate(&base_pose_rate, time_differential);
		joint_velocity_vector_integrator.integrate(&joint_variables_rate, time_differential);
		end_eff_velocities_integrator.integrate(&end_eff_velocities_m,time_differential);
		time = time + time_differential;
	}
	fo.save_timeseries_to_file(&float_robot[0], trajectories, output_type);
}

RealNumMatrix Simulator::convert_to_timeseries_row(RealNumMatrix* vector, RealNumber time)
{
	RealNumMatrix result;
	RealNumMatrix time_m(1,1);
	time_m.set_element(1,1,time);
	result = *vector;
	result.insert_row(&time_m,1);
	result = result.transpose();
	return result;
}
