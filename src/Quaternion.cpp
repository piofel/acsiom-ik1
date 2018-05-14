#include "Quaternion.h"

Quaternion::Quaternion(RealNumber coeff_0, RealNumber coeff_1, RealNumber coeff_2, RealNumber coeff_3)
{
	q[0] = coeff_0;
	q[1] = coeff_1;
	q[2] = coeff_2;
	q[3] = coeff_3;
}

Quaternion::Quaternion(RealNumber coeff[])
{
	UnsignShortInteger i;
	for(i=0; i<4; i++)
	{
		q[i] = coeff[i];
	}
}

Quaternion::Quaternion(RealNumber scalar_part, RealNumMatrix* vector_part)
{
	UnsignShortInteger i;
	q[0] = scalar_part;
	for(i=1; i<4; i++)
	{
		q[i] = vector_part->get_element(i,1);
	}
}

Quaternion::Quaternion(void)
{
	UnsignShortInteger i;
	for(i=0; i<4; i++)
	{
		q[i] = 0;
	}
}

Quaternion::Quaternion(const Quaternion& quaternion_object)
{
	UnsignShortInteger i;
	for(i=0; i<4; i++)
	{
		q[i] = quaternion_object.q[i];
	}
}

Quaternion Quaternion::operator+(const Quaternion& addend_right)
{
	RealNumber res[4];
	UnsignShortInteger i;
	for(i=0; i<4; i++)
	{
		res[i] = q[i] + addend_right.q[i];
	}
	Quaternion res_quat(res);
	return res_quat;
}

Quaternion Quaternion::operator-(const Quaternion& subtrahend)
{
	RealNumber res[4];
	UnsignShortInteger i;
	for(i=0; i<4; i++)
	{
		res[i] = q[i] - subtrahend.q[i];
	}
	Quaternion res_quat(res);
	return res_quat;
}

Quaternion Quaternion::operator*(const Quaternion& factor_right)
{

	RealNumber r[4], res[4];
	UnsignShortInteger i;
	for(i=0; i<4; i++)
	{
		r[i] = factor_right.q[i];
	}
	res[0] = q[0]*r[0] - q[1]*r[1] - q[2]*r[2] - q[3]*r[3];
	res[1] = q[1]*r[0] + q[0]*r[1] - q[3]*r[2] + q[2]*r[3];
	res[2] = q[2]*r[0] + q[3]*r[1] + q[0]*r[2] - q[1]*r[3];
	res[3] = q[3]*r[0] - q[2]*r[1] + q[1]*r[2] + q[0]*r[3];
	Quaternion res_quat(res);
	return res_quat;
}

Quaternion Quaternion::cross(Quaternion* factor_right)
{
	UnsignShortInteger i;
	RealNumber g0, h0;
	RealNumMatrix vg(3,1);
	RealNumMatrix vh(3,1);
	RealNumMatrix r(3,1);
	g0 = this->get_coefficients()[0];	
	h0 = factor_right->get_coefficients()[0];	
	for(i=1; i<4; i++)
	{
		vg.set_element(i,1,this->get_coefficients()[i]);
		vh.set_element(i,1,factor_right->get_coefficients()[i]);
	}
	r = vg.multiply_by_scalar(h0) + vh.multiply_by_scalar(g0) + vg.cross(&vh);
	Quaternion result(0, r.get_element(1,1), r.get_element(2,1), r.get_element(3,1));
	return result;
}

void Quaternion::multiply_by_scalar(RealNumber scalar)
{
	UnsignShortInteger i;
	for(i=0; i<4; i++)
	{
		q[i] *= scalar;
	}
}

Quaternion Quaternion::conjugate(void)
{
	Quaternion con(q[0], -q[1], -q[2], -q[3]);
	return con;
}

RealNumber* Quaternion::get_coefficients(void)
{
	return q;

}

RealNumber Quaternion::get_coefficient(UnsignShortInteger coefficient_number)
{
	Errors er;
	RealNumber result;
	if(coefficient_number>=0 && coefficient_number<=3)
	{
		result = q[coefficient_number];
	}
	else
	{
		er.display_error(21, "Quaternion::get_coefficient");
		result = 0.0;
	}
	return result;
}

RealNumber* Quaternion::get_vector_part(void)
{
	RealNumber* v = &q[1];
	return v;
}

RealNumMatrix Quaternion::get_vector_part_m(void)
{
	UnsignShortInteger i;
	RealNumMatrix v(3,1);
	for(i=1; i<4; i++)
	{
		v.set_element(i,1,q[i]);
	}
	return v;
}

RealNumber Quaternion::norm(void)
{
	RealNumber result = norm_squared();
	result = sqrt(result);
	return result;
}

RealNumber Quaternion::norm_squared(void)
{
	RealNumber result;
	Quaternion qt(q);
	qt = qt * qt.conjugate();
	result = qt.get_coefficients()[0];
	return result;
}

Quaternion Quaternion::reciprocal(void)
{
	RealNumber ns;
	Quaternion result(q);
	ns = result.norm_squared();
	result = result.conjugate();
	result.multiply_by_scalar(1/ns);
	return result;
}

bool Quaternion::check_purity(void)
{
	bool result;
	if(q[0]>ZERO_DETECTION_TOLERANCE || q[0]<-ZERO_DETECTION_TOLERANCE)
	{
		result = true;
	}
	else
	{
		result = false;
	}
	return result;
}
