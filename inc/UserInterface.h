#ifndef __USERINTERFACE_H_INCLUDED__
#define __USERINTERFACE_H_INCLUDED__

#include <iostream>
#include <iomanip>
#include "Simulator.h"
#include "Visualization.h"

#define REAL_NUM_PRECISION 3

class UserInterface
{
	private:
		int ui_argc;
		char** ui_argv;
		char prev_ch;
		void display_separator(void);
		void display_short_separator(void);
		void display_program_name(void);
		char ask_option(void);
		FrameSymbol ask_frame(MultiManipRobot* multi_manip_robot);
		UnsignShortInteger ask_manipulator_number(MultiManipRobot* multi_manip_robot);
		UnsignShortInteger ask_joint_number(MultiManipRobot* multi_manip_robot, UnsignShortInteger manipulator_number);
		void display_test_results_1(Simulator* simulator);
		void display_main_menu_options(void);
		void display_free_flyer_kinematics(Simulator* simulator, char option);
		void display_free_floater_differential_kinematics(Simulator* simulator);
		void display_free_floater_inverse_velocity_kinematics(Simulator* simulator);
		void display_wait(void);
		void display_trajectory_simulation_control_panel(Simulator* simulator);
	public:
		UserInterface(void);
		UserInterface(int argc, char* argv[]);
		void display_matrix(RealNumMatrix* matrix);
		void display_dual_quaternion_matrix(DualQuatMatrix* matrix);
		void display_quaternion(Quaternion* quaternion);
		void display_dual_quaternion(DualQuaternion* dual_quaternion);
		void display_main_menu(void);
		void display_base_parameters(Base* base);
		void display_link_parameters(Link* link);
		void display_end_effector_parameters(EndEffector* end_effector);
		void display_manipulator_parameters(Manipulator* manipulator);
		void display_multi_manip_robot_parameters(MultiManipRobot* multi_manip_robot);
		void display_velocity(DualQuaternion* velocity);
};

#endif
