#ifndef __SPATIALTRANSFORMS_H_INCLUDED__
#define __SPATIALTRANSFORMS_H_INCLUDED__

#include "DualQuaternion.h"
#include "Axes.h"
#include "RealNumMatrix.h"
#include <math.h>

class SpatialTransforms
{
	private:
	public:
		RealNumber rad_to_deg(RealNumber angle);
		RealNumber deg_to_rad(RealNumber angle);
		Quaternion dual_quaternion_to_translation_quaternion(DualQuaternion* pose);
		RealNumMatrix dual_quaternion_to_translation_vector(DualQuaternion* pose);
		RealNumMatrix quaternion_to_rotation_matrix(Quaternion* rotation_quaternion);
		RealNumMatrix dual_quaternion_to_homogeneous_transform(DualQuaternion* pose);
		Quaternion angle_axis_to_quaternion(RealNumber angle_rad, RealNumMatrix* axis);
		Quaternion distance_axis_to_quaternion(RealNumber distance, RealNumMatrix* axis, Quaternion* rotation_quaternion);
		Quaternion roll_pitch_yaw_to_quaternion(RealNumber roll, RealNumber pitch, RealNumber yaw);
		DualQuaternion angle_axis_to_dual_quaternion(RealNumber angle_rad, RealNumMatrix* axis);
		DualQuaternion distance_axis_to_dual_quaternion(RealNumber distance, RealNumMatrix* axis, Quaternion* rotation_quaternion);
		DualQuaternion translation_roll_pitch_yaw_to_dual_quaternion(RealNumMatrix* translation, RealNumMatrix* rotation);
		DualQuaternion translation_roll_pitch_yaw_to_dual_quaternion(RealNumMatrix* pose);
		DualQuaternion screw_transform(RealNumMatrix* axis, RealNumber distance, RealNumber angle_rad);
};

#endif
