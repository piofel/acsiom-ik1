#include "DualQuaternion.h"

DualQuaternion::DualQuaternion(const Quaternion& real_part, const Quaternion& dual_part)
{
	qr = real_part;
	qd = dual_part;
}

DualQuaternion::DualQuaternion(void)
{
	qr = Quaternion(1,0,0,0);
	qd = Quaternion(0,0,0,0);
}

DualQuaternion DualQuaternion::operator+(const DualQuaternion& addend_right)
{
	DualQuaternion result;
	result.qr = qr + addend_right.qr;
	result.qd = qd + addend_right.qd;
	return result;
}

DualQuaternion DualQuaternion::operator-(const DualQuaternion& subtrahend)
{
	DualQuaternion result;
	result.qr = qr - subtrahend.qr;
	result.qd = qd - subtrahend.qd;
	return result;
}

DualQuaternion DualQuaternion::operator*(const DualQuaternion& factor_right)
{
	Quaternion rr = factor_right.qr;
	Quaternion rd = factor_right.qd;
	Quaternion qr_result, qd_result;
	qr_result = qr * rr;
	qd_result = qr * rd + qd * rr;
	DualQuaternion result(qr_result, qd_result);
	return result;
}

DualQuaternion DualQuaternion::conjugate_s(void)
{
	Quaternion qr_result, qd_result;
	qr_result = qr.conjugate();
	qd_result = qd.conjugate();
	DualQuaternion result(qr_result, qd_result);
	return result;
}

DualQuaternion DualQuaternion::conjugate_d(void)
{
	Quaternion qr_result, qd_result;
	qr_result = qr.conjugate();
	qd_result = qd.conjugate();
	qd_result.multiply_by_scalar(-1.0);
	DualQuaternion result(qr_result, qd_result);
	return result;
}

void DualQuaternion::multiply_by_scalar(RealNumber scalar)
{
	qr.multiply_by_scalar(scalar);
	qd.multiply_by_scalar(scalar);
}

DualQuaternion DualQuaternion::multiply_by_scalar_r(RealNumber scalar)
{
	Quaternion r, d;
	r = qr;
	d = qd;
	r.multiply_by_scalar(scalar);
	d.multiply_by_scalar(scalar);
	DualQuaternion result(r,d);
	return result;
}

Quaternion DualQuaternion::get_real_part(void)
{
	return qr;
}

Quaternion DualQuaternion::get_dual_part(void)
{
	return qd;
}

void DualQuaternion::set_to_zero(void)
{
	qr = Quaternion(0,0,0,0);
	qd = Quaternion(0,0,0,0);
}

void DualQuaternion::set_to_identity(void)
{
	qr = Quaternion(1,0,0,0);
	qd = Quaternion(0,0,0,0);
}

DualQuaternion DualQuaternion::transform_rho(DualQuaternion* transforming_dual_quaternion)
{
	Quaternion r, d, rot;
	Errors er;
	if(qr.check_purity())
	{
		er.display_error(16, "DualQuaternion::transform_rho");
	}
	if(qd.check_purity())
	{
		er.display_error(17, "DualQuaternion::transform_rho");
	}
	rot = transforming_dual_quaternion->get_real_part();
	r = rot * qr * rot.conjugate();
	d = rot * qd * rot.conjugate();
	DualQuaternion result(r,d);
	return result;
}

DualQuaternion DualQuaternion::norm_squared(void)
{
	DualQuaternion result;
	result = DualQuaternion(qr,qd);
	result = result * result.conjugate_s();
	return result;
}

DualQuaternion DualQuaternion::reciprocal(void)
{
	Quaternion rp, dp, rec;
	rec = qr.reciprocal();
	rp = rec;
	dp = rec * qd * rec;
	dp.multiply_by_scalar(-1.0);
	DualQuaternion result(rp,dp);
	return result;
}

RealNumMatrix DualQuaternion::column_6(void)
{
	UnsignShortInteger i;
	RealNumber e;
	RealNumMatrix result(6,1);
	for(i=1;i<=3;i++)
	{
		e = qr.get_coefficient(i);
		result.set_element(i,1,e);
		e = qd.get_coefficient(i);
		result.set_element(i+3,1,e);
	}
	return result;
}
