#ifndef __QUATERNION_H_INCLUDED__
#define __QUATERNION_H_INCLUDED__

#include <math.h>
#include "RealNumMatrix.h"

class Quaternion
{
	private:
		RealNumber q[4];
	public:
		Quaternion(RealNumber coeff_0, RealNumber coeff_1, RealNumber coeff_2, RealNumber coeff_3);
		Quaternion(RealNumber coeff[]);
		Quaternion(RealNumber scalar_part, RealNumMatrix* vector_part);
		Quaternion(const Quaternion& quaternion_object);
		Quaternion(void);
		Quaternion operator+(const Quaternion& addend_right);
		Quaternion operator-(const Quaternion& subtrahend);
		Quaternion operator*(const Quaternion& factor_right);
		Quaternion cross(Quaternion* factor_right);
		void multiply_by_scalar(RealNumber scalar);  
		Quaternion conjugate(void);
		RealNumber* get_coefficients(void);
		RealNumber get_coefficient(UnsignShortInteger coefficient_number);
		RealNumber* get_vector_part(void);
		RealNumMatrix get_vector_part_m(void);
		RealNumber norm(void);
		RealNumber norm_squared(void);
		Quaternion reciprocal(void);
		bool check_purity(void);
};

#endif

