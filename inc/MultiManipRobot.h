#ifndef __MULTIMANIPROBOT_H_INCLUDED__
#define __MULTIMANIPROBOT_H_INCLUDED__

#define PROGRAM_NAME "ACSIOM IK-1"

#include "Base.h"
#include "Manipulator.h"
#include "MultiManipRobotParameters.h"
#include "DualQuatMatrix.h"

class MultiManipRobot : public Mechanism
{
	private: 
		UnsignShortInteger num_man;
		Manipulator* man;
		Base base;
		DualQuatMatrix inertia_dependent_on_manip(UnsignShortInteger manipulator_number, RealNumMatrix* base_pose, RealNumMatrix* joint_vector);
	protected:
		UnsignShortInteger get_number_of_degrees_of_freedom_of_all_manipulators(void);
		DualQuatMatrix extract_redundant_degrees_of_freedom(DualQuatMatrix* jacobian_matrix);
		RealNumMatrix extract_redundant_degrees_of_freedom(RealNumMatrix* joint_velocities_vector);
		RealNumMatrix get_vector_of_single_manip_from_off_ang_vec(RealNumMatrix* full_vector, UnsignShortInteger manipulator_number);
		RealNumMatrix get_vector_of_single_manip_from_joint_var(RealNumMatrix* full_vector, UnsignShortInteger manipulator_number);
	public:
		RealNumMatrix extend_joint_variables_vector_to_off_ang_vec(RealNumMatrix* initial_full_offset_angle_vector, RealNumMatrix* full_joint_variables_vector);
		RealNumMatrix get_joint_variables_vector_from_off_ang_vec(RealNumMatrix* full_offset_angle_vector);
		MultiManipRobot(MultiManipRobotParameters* multi_manip_robot_parameters);
		MultiManipRobot(const MultiManipRobot& multi_manip_robot_object);
		MultiManipRobot(void);	
		~MultiManipRobot(void);	
		void operator=(const MultiManipRobot& right_side);
		Manipulator* get_manipulator(ShortInteger manipulator_number);
		Base* get_base(void);
		UnsignShortInteger get_number_of_manipulators(void);
		DualQuaternion pose(FrameSymbol* frame_of_expression, FrameSymbol* expressed_frame, DualQuaternion* frame_of_expression_to_master, DualQuaternion* expressed_frame_to_master, RealNumMatrix* base_pose, RealNumMatrix* joint_vector);
		DualQuaternion derivative_wrt_translation(DualQuaternion* master_jacobian_element, DualQuaternion* transform_to_expression_frame);
		DualQuaternion derivative_wrt_rotation(DualQuaternion* master_jacobian_element, Quaternion* rotation_axis, Quaternion* frame_position, DualQuaternion* transform_to_expression_frame);
		DualQuatMatrix jacobian_dep_base(FrameSymbol* frame_of_interest, FrameSymbol* frame_of_expression, DualQuaternion* frame_of_interest_to_master, DualQuaternion* frame_of_expression_to_master, RealNumMatrix* base_pose, RealNumMatrix* joint_vector);
		DualQuatMatrix jacobian_dep_manip(FrameSymbol* frame_of_interest, FrameSymbol* frame_of_expression, DualQuaternion* frame_of_interest_to_master, DualQuaternion* frame_of_expression_to_master, RealNumMatrix* base_pose, RealNumMatrix* joint_vector);
		DualQuatMatrix inertia_dependent_on_base(RealNumMatrix* base_pose, RealNumMatrix* joint_vector);
		DualQuatMatrix inertia_dependent_on_manip(RealNumMatrix* base_pose, RealNumMatrix* joint_vector);
		DualQuatMatrix jacobian_dep_base_2(FrameSymbol* frame_of_expression, DualQuaternion* frame_of_expression_to_master, RealNumMatrix* base_pose, RealNumMatrix* joint_vector);
		DualQuatMatrix jacobian_dep_manip_2(FrameSymbol* frame_of_expression, DualQuaternion* frame_of_expression_to_master, RealNumMatrix* base_pose, RealNumMatrix* joint_vector);
};

#endif
