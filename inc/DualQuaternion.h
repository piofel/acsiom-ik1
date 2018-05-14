#ifndef __DUALQUATERNION_H_INCLUDED__
#define __DUALQUATERNION_H_INCLUDED__

#include "Quaternion.h"

class DualQuaternion
{
	private:
		Quaternion qr, qd;
	public:
		DualQuaternion(const Quaternion& real_part, const Quaternion& dual_part);
		DualQuaternion(void);
		DualQuaternion operator+(const DualQuaternion& addend_right);
		DualQuaternion operator-(const DualQuaternion& subtrahend);
		DualQuaternion operator*(const DualQuaternion& factor_right);
		DualQuaternion conjugate_s(void);
		DualQuaternion conjugate_d(void);
		void multiply_by_scalar(RealNumber scalar);  
		DualQuaternion multiply_by_scalar_r(RealNumber scalar);  
		Quaternion get_real_part(void);
		Quaternion get_dual_part(void);
		void set_to_zero(void);
		void set_to_identity(void);
		DualQuaternion transform_rho(DualQuaternion* transforming_dual_quaternion);
		DualQuaternion norm_squared(void);
		DualQuaternion reciprocal(void);
		RealNumMatrix column_6(void);
};

#endif
