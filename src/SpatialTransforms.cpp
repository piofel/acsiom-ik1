#include "SpatialTransforms.h"

RealNumber SpatialTransforms::rad_to_deg(RealNumber angle)
{
	return angle * 180 / M_PI;
}
RealNumber SpatialTransforms::deg_to_rad(RealNumber angle)
{
	return angle * M_PI / 180;
}

Quaternion SpatialTransforms::dual_quaternion_to_translation_quaternion(DualQuaternion* pose)
{
	Quaternion r, q, t;
	r = pose->get_real_part();
	t = pose->get_dual_part();
	q = t * r.conjugate();
	q.multiply_by_scalar(2.0);
	return q;
}

RealNumMatrix SpatialTransforms::dual_quaternion_to_translation_vector(DualQuaternion* pose)
{
	ShortInteger i;
	Quaternion q;
	RealNumMatrix p(3,1);
	q = dual_quaternion_to_translation_quaternion(pose);
	for(i=1; i<4; i++)
	{
		p.set_element(i,1,q.get_vector_part()[i-1]);
	}
	return p;
}

RealNumMatrix SpatialTransforms::dual_quaternion_to_homogeneous_transform(DualQuaternion* pose)
{
	ShortInteger i, j;
	RealNumMatrix ht(4,4);
	Quaternion r = pose->get_real_part();
	RealNumMatrix p(dual_quaternion_to_translation_vector(pose));
	RealNumMatrix rm(quaternion_to_rotation_matrix(&r));
	for(i=1; i<4; i++)
	{
		for(j=1; j<4; j++)
		{
			ht.set_element(i,j,rm.get_element(i,j));
		}
		ht.set_element(4,i,0);
		ht.set_element(i,4,p.get_element(i,1));
	} 	
	ht.set_element(4,4,1);
	return ht;
}

RealNumMatrix SpatialTransforms::quaternion_to_rotation_matrix(Quaternion* rotation_quaternion)
{
	ShortInteger i, j;
	RealNumMatrix rm(3,3);
	RealNumber e[3][3];
	RealNumber* q = rotation_quaternion->get_coefficients();
	e[0][0] = 1 - 2 * pow(q[2],2) - 2 * pow(q[3],2);
	e[0][1] = 2 * (q[1]*q[2] - q[3]*q[0]);
	e[0][2] = 2 * (q[1]*q[3] + q[2]*q[0]);
	e[1][0] = 2 * (q[1]*q[2] + q[3]*q[0]);
	e[1][1] = 1 - 2 * pow(q[1],2) - 2 * pow(q[3],2);
	e[1][2] = 2 * (q[2]*q[3] - q[1]*q[0]);
	e[2][0] = 2 * (q[1]*q[3] - q[2]*q[0]);
	e[2][1] = 2 * (q[2]*q[3] + q[1]*q[0]);
	e[2][2] = 1 - 2 * pow(q[1],2) - 2 * pow(q[2],2);
	for(i=0; i<3; i++)
	{
		for(j=0; j<3; j++)
		{
			rm.set_element(i+1, j+1, e[i][j]);
		}
	}
	return rm;
}

Quaternion SpatialTransforms::angle_axis_to_quaternion(RealNumber angle_rad, RealNumMatrix* axis)
{
	// axis must be an unit vector
	RealNumber q0 = cos(angle_rad/2);
	RealNumber q13[3];
	ShortInteger i;
	for(i=0; i<3; i++)
	{
		q13[i] = axis->get_element(i+1,1) * sin(angle_rad/2);
	}
	Quaternion q(q0, q13[0], q13[1], q13[2]);
	return q;
}

Quaternion SpatialTransforms::distance_axis_to_quaternion(RealNumber distance, RealNumMatrix* axis, Quaternion* rotation_quaternion)
{
	// axis must be an unit vector
	RealNumber p[3];
	UnsignShortInteger i;
	for(i=0; i<3; i++)
	{
		p[i] = axis->get_element(i+1,1) * distance;
	}
	Quaternion t(0,p[0],p[1],p[2]);
	t = t * *rotation_quaternion;
	t.multiply_by_scalar(0.5);
	return t;
}

Quaternion SpatialTransforms::roll_pitch_yaw_to_quaternion(RealNumber roll, RealNumber pitch, RealNumber yaw)
{
	Axes a;
	Quaternion r1 = angle_axis_to_quaternion(roll, a.get_axis(1)); 
	Quaternion r2 = angle_axis_to_quaternion(pitch, a.get_axis(2)); 
	Quaternion r3 = angle_axis_to_quaternion(yaw, a.get_axis(3)); 
	Quaternion r = r3 * r2 * r1;
	return r;
}

DualQuaternion SpatialTransforms::angle_axis_to_dual_quaternion(RealNumber angle_rad, RealNumMatrix* axis)
{
	// axis must be an unit vector
	Quaternion r;
	Quaternion t(0,0,0,0);
        r = angle_axis_to_quaternion(angle_rad,axis);
	DualQuaternion dq(r,t);
	return dq;
}

DualQuaternion SpatialTransforms::distance_axis_to_dual_quaternion(RealNumber distance, RealNumMatrix* axis, Quaternion* rotation_quaternion)
{
	// axis must be an unit vector
	Quaternion t;
	t = distance_axis_to_quaternion(distance, axis, rotation_quaternion);
	DualQuaternion dq(*rotation_quaternion,t);
	return dq;
}

DualQuaternion SpatialTransforms::translation_roll_pitch_yaw_to_dual_quaternion(RealNumMatrix* translation, RealNumMatrix* rotation)
{
	RealNumber roll,pitch,yaw,p1,p2,p3;
	p1 = translation->get_element(1,1);
	p2 = translation->get_element(2,1);
	p3 = translation->get_element(3,1);
	roll = rotation->get_element(1,1);
	pitch = rotation->get_element(2,1);
	yaw = rotation->get_element(3,1);
	Quaternion r = roll_pitch_yaw_to_quaternion(roll,pitch,yaw);
	Quaternion p_org(0.0,p1,p2,p3);
	Quaternion t = p_org * r;
	t.multiply_by_scalar(0.5);
	DualQuaternion dq(r,t);
	return dq;
}

DualQuaternion SpatialTransforms::translation_roll_pitch_yaw_to_dual_quaternion(RealNumMatrix* pose)
{
	ShortInteger i;
	RealNumMatrix t(3,1);
	RealNumMatrix r(3,1);
	for(i=1;i<4;i++)
	{
		t.set_element(i,1,pose->get_element(i,1));
		r.set_element(i,1,pose->get_element(i+3,1));
	}
	return translation_roll_pitch_yaw_to_dual_quaternion(&t,&r);
}

DualQuaternion SpatialTransforms::screw_transform(RealNumMatrix* axis, RealNumber distance, RealNumber angle_rad)
{
	// axis must be an unit vector
	Quaternion r, t;
       	r = angle_axis_to_quaternion(angle_rad,axis);
	t = distance_axis_to_quaternion(distance,axis,&r);
	DualQuaternion dq(r,t);
	return dq;
}
