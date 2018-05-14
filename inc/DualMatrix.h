#ifndef __DUALMATRIX_H_INCLUDED__
#define __DUALMATRIX_H_INCLUDED__

#include "RealNumMatrix.h"
#include "DualQuaternion.h"

class DualMatrix
{
	private:
		bool diff_oper;
		RealNumMatrix rp, dp;
	public:
		DualMatrix(const RealNumMatrix& real_part, const RealNumMatrix& dual_part);
		DualMatrix(void);
		void operator=(const DualMatrix& right_side);
		DualQuaternion multiply_by_dual_quaternion(DualQuaternion* dual_quaternion);
		void set_differential_operator_in_real_part(void);
};

#endif
