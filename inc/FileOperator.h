#ifndef __FILEOPERATOR_H_INCLUDED__
#define __FILEOPERATOR_H_INCLUDED__

#include <cstdlib>
#include <string>
#include <sstream>
#include <fstream>
#include "MultiManipRobot.h"

#define INPUT_FILE_PATH "../../auxiliary_calculations/test_input_1/"
//#define INPUT_FILE_PATH "../input/"
#define OUTPUT_FILE_PATH "../output/"

// integer to string conversion:
#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

class FileOperator
{
	private:
		DualQuaternion read_dual_quaternion_coefficients(std::string file_name, bool angle_conversion);
	public:
		FileOperator(void);
		~FileOperator(void);
		void append_to_output_file(std::string text, std::string file_name);
		void delete_file_content(std::string file_name);
		void save_timeseries_to_file(MultiManipRobot* multi_manip_robot, RealNumMatrix timeseries, char timeseries_type);
		SystemParameters read_system_parameters(std::string file_name);
		ManipulatorParameters read_manipulator_parameters(ShortInteger manipulator_number);
		MultiManipRobotParameters read_multi_manip_robot_parameters(void);
		RealNumMatrix read_base_pose_derivative(ShortInteger derivative_number);
		RealNumMatrix read_joint_vector(MultiManipRobot* multi_manip_robot, ShortInteger manipulator_number);
		RealNumMatrix read_joint_vector(MultiManipRobot* multi_manip_robot);
		RealNumMatrix read_joint_velocities(MultiManipRobot* multi_manip_robot, ShortInteger manipulator_number);
		RealNumMatrix read_joint_velocities(MultiManipRobot* multi_manip_robot);
		DualQuaternion read_initial_momentum(void);
		DualQuatMatrix read_end_effectors_velocities(MultiManipRobot* multi_manip_robot);
		DualQuaternion read_end_effector_velocity(UnsignShortInteger manipulator_number);
		RealNumMatrix read_redundant_joints_velocities(MultiManipRobot* multi_manip_robot);
};

#endif
