#include "DualMatrix.h"

DualMatrix::DualMatrix(const RealNumMatrix& real_part, const RealNumMatrix& dual_part)
{
	rp = real_part;
	dp = dual_part;
	diff_oper = false;
}

DualMatrix::DualMatrix(void)
{
	diff_oper = false;
}

void DualMatrix::operator=(const DualMatrix& right_side)
{
	rp = right_side.rp;
	dp = right_side.dp;
	diff_oper = right_side.diff_oper;
}

DualQuaternion DualMatrix::multiply_by_dual_quaternion(DualQuaternion* dual_quaternion)
{
	Quaternion hr, hd, rr, rd;
	hr = dual_quaternion->get_real_part();
	hd = dual_quaternion->get_dual_part();
	if(diff_oper == false)
	{
		rr = rp.multiply_by_quaternion(&hr);
		rd = rp.multiply_by_quaternion(&hd) + dp.multiply_by_quaternion(&hr);
	}
	else
	{
		rr = rp.multiply_by_quaternion(&hd);
		rd = dp.multiply_by_quaternion(&hr);
	}
	DualQuaternion result(rr,rd);
	return result;
}

void DualMatrix::set_differential_operator_in_real_part(void)
{
	diff_oper = true;
}
