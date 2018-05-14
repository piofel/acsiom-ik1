#ifndef __MANIPULATOR_H_INCLUDED__
#define __MANIPULATOR_H_INCLUDED__

#include "Mechanism.h"
#include "Link.h"
#include "EndEffector.h"
#include "ManipulatorParameters.h"
#include "DualQuatMatrix.h"

class Manipulator : public Mechanism
{
	private: 
		UnsignShortInteger num_dof;
		UnsignShortInteger manip_num;
		Link* links;
		EndEffector end_eff;
	public:
		Manipulator(ManipulatorParameters* manipulator_parameters, ShortInteger manipulator_number);
		Manipulator(const Manipulator& manipulator_object);
		Manipulator(void);	
		~Manipulator(void);	
		void operator=(const Manipulator& right_side);
		DualQuaternion joint_pose(RealNumMatrix* joint_vector, ShortInteger reference_joint_number, ShortInteger joint_number);
		Link* get_link(UnsignShortInteger link_number);
		EndEffector* get_end_effector(void);
		UnsignShortInteger get_number_of_degrees_of_freedom(void);
		UnsignShortInteger get_manipulator_number(void);
};

#endif

