#ifndef __RIGIDBODY_H_INCLUDED__
#define __RIGIDBODY_H_INCLUDED__ 

#include "TypeDefinitions.h"
#include "DualQuaternion.h"
#include "SystemParameters.h"
#include "SpatialTransforms.h"
#include "ConstantFrame.h"
#include "DualMatrix.h"

class RigidBody
{
	protected:
		RealNumber mass;
		ConstantFrame center_of_mass_frame;
		RealNumber mass_moments_of_inertia[3];
	public:
		RigidBody(SystemParameters* parameters, ShortInteger master_frame_number, ShortInteger master_manipulator_number);
		RigidBody(void);
		RealNumber get_mass(void);
		ConstantFrame* get_center_of_mass_frame(void);
		RealNumber* get_mass_moments_of_inertia(void);
		DualMatrix get_dual_inertia_matrix(void);
};

#endif

