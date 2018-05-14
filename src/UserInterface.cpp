#include "UserInterface.h"

using namespace std;

UserInterface::UserInterface(void)
{
	prev_ch = '\n';
}

UserInterface::UserInterface(int argc, char* argv[])
{
	ui_argc = argc;
	ui_argv = argv;
	prev_ch = '\n';
}

void UserInterface::display_main_menu(void)
{
	bool exit_f;
	char o;
	Errors er;
	Simulator s;
       	Simulator* simulator = &s;
	FreeFlyingRobot* ry = simulator->get_free_flying_robot();
	FreeFloatingRobot* ro = simulator->get_free_floating_robot();
	Visualization vis(ui_argc,ui_argv);
	exit_f = false;
	display_program_name();
	display_main_menu_options();
	while(!exit_f)
	{
		o = ask_option();
		switch(o)
		{
			case '0':
				exit_f = true;
				cout << "Program terminated." << endl;
				break;
			case '1':
				simulator->reload_parameters();
				ry = simulator->get_free_flying_robot();
				ro = simulator->get_free_floating_robot();
				cout << "Done. " << endl;
				display_separator();
				display_main_menu_options();
				break;
			case '2':
				display_multi_manip_robot_parameters(ry);
				display_separator();
				display_main_menu_options();
				break;
			case '3':
				display_free_flyer_kinematics(simulator,o);
				display_separator();
				display_main_menu_options();
				break;
			case '4':
				display_free_flyer_kinematics(simulator,o);
				display_separator();
				display_main_menu_options();
				break;
			case '5':
				display_free_floater_differential_kinematics(simulator);
				display_separator();
				display_main_menu_options();
				break;
			case '6':
				display_free_floater_inverse_velocity_kinematics(simulator);
				display_separator();
				display_main_menu_options();
				break;
			case '7':
				display_trajectory_simulation_control_panel(simulator);
				display_separator();
				display_main_menu_options();
				break;
			case '8':
				vis.display_multi_manip_robot();
				display_separator();
				display_main_menu_options();
				break;
			case '9':
				display_test_results_1(simulator);
				display_separator();
				display_main_menu_options();
				break;
			default:
				er.display_error(8);
				display_separator();
				display_main_menu_options();
		}
	}
}

void UserInterface::display_main_menu_options(void)
{
	cout << "Type the number of option and press Enter: " << endl;
	cout << "1. Reload robot parameters" << endl;
	cout << "2. Display robot parameters" << endl;
	cout << "3. Solve direct kinematics of a free-flying robot" << endl;
	cout << "4. Solve direct velocity kinematics of a free-flying robot" << endl;
	cout << "5. Solve direct velocity kinematics of a free-floating robot" << endl;
	cout << "6. Solve inverse velocity kinematics of a free-floating robot" << endl;
	cout << "7. Simulate trajectories of a free-floating robot during control" << endl;
	cout << "8. Display robot" << endl;
	cout << "0. Exit" << endl;
}

void UserInterface::display_quaternion(Quaternion* quaternion)
{
	RealNumber* q = quaternion->get_coefficients();	
	cout.setf(ios::fixed|ios::showpoint);
  	cout << setprecision(REAL_NUM_PRECISION);
	cout << q[0] << " + " << q[1] << " * i + " << q[2] << " * j + " << q[3] << " * k" << endl;
	cout.unsetf(ios::fixed|ios::showpoint);
  	cout << setprecision(6); // default
}

void UserInterface::display_dual_quaternion(DualQuaternion* dual_quaternion)
{
	Quaternion rp, dp;
	rp = dual_quaternion->get_real_part();
	dp = dual_quaternion->get_dual_part();
	cout << "Real part:" << endl;
	display_quaternion(&rp);
	cout << "Dual part:" << endl;
	display_quaternion(&dp);
}

void UserInterface::display_matrix(RealNumMatrix* matrix)
{
	UnsignShortInteger i, j;
	UnsignShortInteger nrow = matrix->get_number_of_rows(); 
	UnsignShortInteger ncol = matrix->get_number_of_columns();
	cout.setf(ios::fixed|ios::showpoint);
  	cout << setprecision(REAL_NUM_PRECISION);
	for(i=1; i <= nrow ; i++)
	{
		cout << "|";
		for(j=1; j <= ncol ; j++)
		{
			cout << "\t";
			cout << matrix->get_element(i,j);
		}
		cout << "\t|\n";
	}
	cout.unsetf(ios::fixed|ios::showpoint);
  	cout << setprecision(6); // default
}

void UserInterface::display_dual_quaternion_matrix(DualQuatMatrix* matrix)
{
	UnsignShortInteger i, j;
	DualQuaternion dq;
	UnsignShortInteger nrow = matrix->get_number_of_rows(); 
	UnsignShortInteger ncol = matrix->get_number_of_columns();
	for(i=1; i <= nrow ; i++)
	{
		display_short_separator();
		cout << "Row " << i << endl;
		for(j=1; j <= ncol ; j++)
		{
			display_short_separator();
			dq = matrix->get_element(i,j);
			display_dual_quaternion(&dq);
		}
	}
}

void UserInterface::display_base_parameters(Base* base)
{
	UnsignShortInteger i;
	SpatialTransforms st;
	ConstantFrame* comf = base->get_center_of_mass_frame();
	display_separator();
	cout << "Base parameters" << endl;
	display_separator();
	cout << "Mass: " << base->get_mass() << endl;
	for(i=1; i<4; i++)
	{
		cout << "Center of mass position (axis " << i << "): " << comf->get_position(i) << endl;
	}
	cout << "Roll of the frame with the origin fixed in the base center of mass: " << st.rad_to_deg(comf->get_orientation(1)) << endl; 
	cout << "Pitch of the frame with the origin fixed in the base center of mass: " << st.rad_to_deg(comf->get_orientation(2)) << endl; 
	cout << "Yaw of the frame with the origin fixed in the base center of mass: " << st.rad_to_deg(comf->get_orientation(3)) << endl; 
	for(i=0; i<3; i++)
	{
		cout << "Mass moment of inertia (axis " << i+1 << "): " << base->get_mass_moments_of_inertia()[i] << endl;
	}
}

void UserInterface::display_link_parameters(Link* link)
{
	Errors er;
	UnsignShortInteger i, link_number;
	SpatialTransforms st;
	ConstantFrame* comf = link->get_center_of_mass_frame();
	FrameSymbol* main_frame = link->get_joint_frame_symbol();
	link_number = main_frame->get_master_frame_number();
	display_separator();
	cout << "Link " << link_number << " parameters" << endl;
	display_separator();
	cout << "Joint type: ";
	if(link->is_joint_redundant())
	{
		cout << "redundant, ";
	}
	if(link->is_joint_prismatic())
	{
		cout << "prismatic";
	}
	else
	{
		cout << "revolute";
	}
	cout << ".\n";
	cout << "Length of the preceding link: " << link->get_preceding_link_length() << endl;
	cout << "Twist of the preceding link: " << st.rad_to_deg(link->get_preceding_link_twist()) << endl;
	cout << "Mass: " << link->get_mass() << endl;
	for(i=1; i<4; i++)
	{
		cout << "Center of mass position (axis " << i << "): " << comf->get_position(i) << endl;
	}
	cout << "Roll of the frame with the origin fixed in the link center of mass: " << st.rad_to_deg(comf->get_orientation(1)) << endl; 
	cout << "Pitch of the frame with the origin fixed in the link center of mass: " << st.rad_to_deg(comf->get_orientation(2)) << endl; 
	cout << "Yaw of the frame with the origin fixed in the link center of mass: " << st.rad_to_deg(comf->get_orientation(3)) << endl; 
	for(i=0; i<3; i++)
	{
		cout << "Mass moment of inertia (axis " << i+1 << "): " << link->get_mass_moments_of_inertia()[i] << endl;
	}
}

void UserInterface::display_end_effector_parameters(EndEffector* end_effector)
{
	UnsignShortInteger i, end_effector_number ;
	SpatialTransforms st;
	ConstantFrame* eef = end_effector->get_main_frame();
	ConstantFrame* comf = end_effector->get_center_of_mass_frame();
	FrameSymbol* mfs = eef->get_frame_symbol();
	end_effector_number = mfs->get_master_manipulator_number();
	display_separator();
	cout << "End effector " << end_effector_number << " parameters" << endl;
	display_separator();
	for(i=1; i<4; i++)
	{
		cout << "Position of the origin of the end-effector main frame (axis " << i << "): " << eef->get_position(i) << endl;
	}
	cout << "Roll of the end-effector main frame: " << st.rad_to_deg(eef->get_orientation(1)) << endl;
	cout << "Pitch of the end-effector main frame: " << st.rad_to_deg(eef->get_orientation(2)) << endl;
	cout << "Yaw of the end-effector main frame: " << st.rad_to_deg(eef->get_orientation(3)) << endl;
	cout << "Mass: " << end_effector->get_mass() << endl;
	for(i=1; i<4; i++)
	{
		cout << "Center of mass position (axis " << i << "): " << comf->get_position(i) << endl;
	}
	cout << "Roll of the frame with the origin fixed in the end-effector center of mass: " << st.rad_to_deg(comf->get_orientation(1)) << endl; 
	cout << "Pitch of the frame with the origin fixed in the end-effector center of mass: " << st.rad_to_deg(comf->get_orientation(2)) << endl; 
	cout << "Yaw of the frame with the origin fixed in the end-effector center of mass: " << st.rad_to_deg(comf->get_orientation(3)) << endl; 
	for(i=0; i<3; i++)
	{
		cout << "Mass moment of inertia (axis " << i+1 << "): " << end_effector->get_mass_moments_of_inertia()[i] << endl;
	}
}

void UserInterface::display_manipulator_parameters(Manipulator* manipulator)
{
	UnsignShortInteger manipulator_number;
	UnsignShortInteger ndof, i;
	Link* link;
	EndEffector* ee;
	manipulator_number = manipulator->get_manipulator_number();
	display_separator();
	cout << "Manipulator " << manipulator_number << " parameters" << endl;
	display_separator();
	ndof  = manipulator->get_number_of_degrees_of_freedom();	
	for(i=1; i<=ndof; i++)
	{
		link = manipulator->get_link(i);
		display_link_parameters(link);
	}
	ee = manipulator->get_end_effector();
	display_end_effector_parameters(ee);
}

void UserInterface::display_multi_manip_robot_parameters(MultiManipRobot* multi_manip_robot)
{
	UnsignShortInteger nm, i;
	Base* b;
	Manipulator* m;
	b = multi_manip_robot->get_base();
	display_base_parameters(b);
	nm = multi_manip_robot->get_number_of_manipulators();
	for(i=1; i<=nm; i++)
	{
		m = multi_manip_robot->get_manipulator(i);
		display_manipulator_parameters(m);
	}
}

void UserInterface::display_program_name(void)
{
	display_separator();
	display_separator();
	cout << PROGRAM_NAME << endl;
	cout << "CONTROL FOR MULTI-ARM ROBOTS" << endl;
	cout << "Development: Piotr Andrzej Felisiak" << endl;
	cout << "Supervision: Kaiyu Qin" << endl;
	display_separator();
	display_separator();
}

void UserInterface::display_separator(void)
{
	cout << "-------------------------------------------------------------------------------" << endl;
}

void UserInterface::display_short_separator(void)
{
	cout << "---------------------------------------------------" << endl;
}

char UserInterface::ask_option(void)
{
	char o;
	cout << "Option: ";
	o = cin.get();
	if(o=='\n' || o=='`')
	{
		o = cin.get();
	}
	else
	{
		if(prev_ch!='\n')
		{
			cout << prev_ch << endl;
		}
	}
	prev_ch = o;
	return o;
}

UnsignShortInteger UserInterface::ask_manipulator_number(MultiManipRobot* multi_manip_robot)
{
	UnsignShortInteger n, nm;
	Errors er;
	nm = multi_manip_robot->get_number_of_manipulators();
	cout << "Type the manipulator number and press Enter. " << endl;
	cout << "Manipulator number: ";
	cin >> n; 
	if(n<1||n>nm)
	{
		er.display_error(3);
		n = 1;
		cout << "Manipulator number set to " << n << ".\n";
	}
	return n;
}

UnsignShortInteger UserInterface::ask_joint_number(MultiManipRobot* multi_manip_robot, UnsignShortInteger manipulator_number)
{
	UnsignShortInteger n, nj;
	Errors er;
	Manipulator* m;
	m = multi_manip_robot->get_manipulator(manipulator_number);
	nj = m->get_number_of_degrees_of_freedom();
	cout << "Type the joint number and press Enter. " << endl;
	cout << "Joint number: ";
	cin >> n; 
	if(n<1||n>nj)
	{
		er.display_error(4);
		n = 1;
		cout << "Joint number set to " << n << ".\n";
	}
	return n;
}

FrameSymbol UserInterface::ask_frame(MultiManipRobot* multi_manip_robot)
{
	char o;
	UnsignShortInteger mn, jn;
	FrameSymbol fs;
	Errors er;
	Manipulator* m;
	cout << "n. inertial frame\n";
	cout << "b. base satellite main frame\n";
	cout << "j. joint frame\n";
	cout << "e. end-effector main frame\n";
	o = ask_option();
	switch(o)
	{
		case 'n':
			fs = FrameSymbol(-1, -1, 'x', 'N', 'x');
			break;
		case 'b':
			fs = FrameSymbol(0, 0, 'x', '0', 'x');
			break;
		case 'j':
			mn = ask_manipulator_number(multi_manip_robot);
			jn = ask_joint_number(multi_manip_robot, mn);
			fs = FrameSymbol(jn, mn, 'x', 'l', 'x');
			break;
		case 'e':
			mn = ask_manipulator_number(multi_manip_robot);
			m = multi_manip_robot->get_manipulator(mn);
			jn = m->get_number_of_degrees_of_freedom();
			fs = FrameSymbol(jn, mn, 'x', 'E', 'x');	
			break;
		default:
			er.display_error(8);
			fs = FrameSymbol(0, 0, 'x', '0', 'x');
			cout << "Frame set to the base satellite main frame.\n";
	}
	return fs;
}

void UserInterface::display_free_floater_inverse_velocity_kinematics(Simulator* simulator)
{
	RealNumMatrix jv;
	display_wait();
	jv = simulator->free_floater_inverse_velocity_kinematics_simulation();
	display_short_separator();
	cout << "Velocities of principal joints of all manipulators:\n";
	display_matrix(&jv);
}

void UserInterface::display_free_floater_differential_kinematics(Simulator* simulator)
{
	UnsignShortInteger nm, i;
	DualQuatMatrix v;
	DualQuaternion e;
	FreeFloatingRobot* float_robot; 
	display_wait();
	float_robot = simulator->get_free_floating_robot();
	nm = float_robot->get_number_of_manipulators();
	v = simulator->free_floater_velocity_simulation();
	for(i=1; i<=nm; i++)
	{
		e = v.get_element(i,1);
		display_short_separator();
		cout << "Velocity of end-effector number " << i << ":" << endl;
		display_velocity(&e);
	}
}

void UserInterface::display_free_flyer_kinematics(Simulator* simulator, char option)
{
	DualQuaternion dq;
	RealNumMatrix mat;
	SpatialTransforms st;
	FreeFlyingRobot* fly_robot;
	FrameSymbol frame_of_interest;
	FrameSymbol frame_of_expression;
	Errors er;
	fly_robot = simulator->get_free_flying_robot();
	cout << "Type a letter denotiong the frame of interest:\n";
	frame_of_interest = ask_frame(fly_robot);
	cout << "Type a letter denotiong the frame in which the values will be expressed:\n";
	frame_of_expression = ask_frame(fly_robot);
	switch(option)
	{
		case '3':
			dq = simulator->free_flyer_pose_simulation(&frame_of_expression, &frame_of_interest);
			mat = st.dual_quaternion_to_homogeneous_transform(&dq);
			display_matrix(&mat);
			break;
		case '4':
			dq = simulator->free_flyer_velocity_simulation(&frame_of_interest, &frame_of_expression);
			display_velocity(&dq);
			break;
		default:
			er.display_error(8);
	}
}

void UserInterface::display_wait(void)
{
	
	cout << "Please wait..." << endl; 
}

void UserInterface::display_velocity(DualQuaternion* velocity)
{
	Quaternion q;
	RealNumMatrix mat;
	cout << "Angular velocity (deg/s):\n";
	q = velocity->get_real_part();
	mat = q.get_vector_part_m();
	mat = mat.multiply_by_scalar(180/M_PI);
	display_matrix(&mat);
	cout << "Linear velocity (m/s):\n";
	q = velocity->get_dual_part();
	mat = q.get_vector_part_m();
	display_matrix(&mat);
}

void UserInterface::display_trajectory_simulation_control_panel(Simulator* simulator)
{
	char o;
	cout << "Choose the trajectory type and press ENTER:\n";
	cout << "b. base pose\n";
	cout << "j. joint variables\n";
	cout << "c. calculated joint variables rate\n";
	cout << "v. end-effectors' velocities\n";
	cout << "p. end-effectors' poses\n";
	cout << "e. end-effectors' velocities error\n";
	cout << "n. condition number of the generalized jacobian matrix\n";
	cout << "Trajectory type: ";
	cin >> o;
	display_wait();
	simulator->free_floater_trajectory_simulation(o);
	cout << "Output data written in " << OUTPUT_FILE_PATH << endl;
}

void UserInterface::display_test_results_1(Simulator* simulator)
{
	Quaternion g(1,2,3,4);
	Quaternion h(5,6,7,8);
	Quaternion r = g*h;
	display_quaternion(&r);
}
