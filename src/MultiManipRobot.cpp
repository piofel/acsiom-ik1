#include "MultiManipRobot.h"

MultiManipRobot::MultiManipRobot(MultiManipRobotParameters* multi_manip_robot_parameters)
{
	ShortInteger i;
	ManipulatorParameters mp;
	SystemParameters bp;
	num_man = multi_manip_robot_parameters->get_number_of_manipulators();
	man = new Manipulator[num_man];
	for(i=0; i<num_man; i++)
	{
		mp = multi_manip_robot_parameters->get_manipulators_parameters()[i];
		man[i] = Manipulator(&mp, i+1);
	}	
	bp = multi_manip_robot_parameters->get_base_satellite_parameters()[0];
	base = Base(&bp); 
}

MultiManipRobot::MultiManipRobot(const MultiManipRobot& multi_manip_robot_object)
{
	ShortInteger i;
	num_man = multi_manip_robot_object.num_man;
	base = multi_manip_robot_object.base;
	man = new Manipulator[num_man];
	for(i=0; i<num_man; i++)
	{
		man[i] = Manipulator(multi_manip_robot_object.man[i]);
	}
}

MultiManipRobot::MultiManipRobot(void)
{
	man = new Manipulator[1];
}

MultiManipRobot::~MultiManipRobot(void)
{
	delete [] man;
}

void MultiManipRobot::operator=(const MultiManipRobot& right_side)
{
	ShortInteger i;
	num_man = right_side.num_man;
	base = right_side.base;
	delete [] man;
	man = new Manipulator[num_man];
	for(i=0; i<num_man; i++)
	{
		man[i] = Manipulator(right_side.man[i]);
	}
}

Manipulator* MultiManipRobot::get_manipulator(ShortInteger manipulator_number)
{
	Manipulator* m;
	Errors er;
	if(manipulator_number>0 && manipulator_number<=num_man)
	{
		m = man - 1 + manipulator_number;
	}
	else
	{
		er.display_error(3, "MultiManipRobot::get_manipulator");	
		m = man; 
	}
	return m;
}

Base* MultiManipRobot::get_base(void)
{
	return &base;
}

UnsignShortInteger MultiManipRobot::get_number_of_manipulators(void)
{
	return num_man;
}

RealNumMatrix MultiManipRobot::get_vector_of_single_manip_from_off_ang_vec(RealNumMatrix* full_vector, UnsignShortInteger manipulator_number)
{
	// This method gets a vector of joint offsets and angles of a single manipulator
	// from vector of offsets and angles of all manipulators
	ShortInteger i, ndof, begin_index;
	RealNumMatrix result;
	begin_index = 1;
	for(i=1; i<manipulator_number; i++)
	{
		ndof = get_manipulator(i)->get_number_of_degrees_of_freedom();
		begin_index = begin_index + 2*ndof;	
	}
	ndof = get_manipulator(manipulator_number)->get_number_of_degrees_of_freedom();
	result = RealNumMatrix(2*ndof,1);
	for(i=0; i<2*ndof; i++)
	{
		result.set_element(i+1,1,full_vector->get_element(begin_index+i,1));	
	}
	return result;	
}

RealNumMatrix MultiManipRobot::get_vector_of_single_manip_from_joint_var(RealNumMatrix* full_vector, UnsignShortInteger manipulator_number)
{
	// This method gets a vector of joint variable parameters of a single manipulator
	// from vector of joint variables of all manipulators 
	ShortInteger i, ndof, begin_index;
	RealNumMatrix result;
	begin_index = 1;
	for(i=1; i<manipulator_number; i++)
	{
		ndof = get_manipulator(i)->get_number_of_degrees_of_freedom();
		begin_index = begin_index + ndof;	
	}
	ndof = get_manipulator(manipulator_number)->get_number_of_degrees_of_freedom();
	result = RealNumMatrix(ndof,1);
	for(i=1; i<=ndof; i++)
	{
		result.set_element(i,1,full_vector->get_element(begin_index+i-1,1));	
	}
	return result;	
}

DualQuaternion MultiManipRobot::pose(FrameSymbol* frame_of_expression, FrameSymbol* expressed_frame, DualQuaternion* frame_of_expression_to_master, DualQuaternion* expressed_frame_to_master, RealNumMatrix* base_pose, RealNumMatrix* joint_vector)
{
	ShortInteger za, zb, ca, cb, ndof;
	Manipulator* m;
	DualQuaternion mtm, result;
	RealNumMatrix joint_vector_sm;
	SpatialTransforms st;
	za = frame_of_expression->get_master_frame_number();
	zb = expressed_frame->get_master_frame_number();
	ca = frame_of_expression->get_master_manipulator_number();
	cb = expressed_frame->get_master_manipulator_number();
	if(za == zb && ca == cb)
	{
		result = frame_of_expression_to_master->conjugate_s() * *expressed_frame_to_master;
	}
	else
	{
		if(ca>0 && ca == cb)
		{
			m = get_manipulator(ca);
			joint_vector_sm = get_vector_of_single_manip_from_off_ang_vec(joint_vector, ca);
			mtm = m->joint_pose(&joint_vector_sm, za, zb);
			result = frame_of_expression_to_master->conjugate_s() * mtm * *expressed_frame_to_master;
		}
		else
		{
			if(ca == -1)
			{
				mtm = st.translation_roll_pitch_yaw_to_dual_quaternion(base_pose);			
				result = frame_of_expression_to_master->conjugate_s() * mtm;
			}
			if(ca == 0)
			{
				result = frame_of_expression_to_master->conjugate_s();
			}
			if(ca > 0)
			{
				m = get_manipulator(ca);
				joint_vector_sm = get_vector_of_single_manip_from_off_ang_vec(joint_vector, ca);
				mtm = m->joint_pose(&joint_vector_sm, za, 0);
				result = frame_of_expression_to_master->conjugate_s() * mtm;
			}
			if(cb == -1)
			{
				mtm = st.translation_roll_pitch_yaw_to_dual_quaternion(base_pose);			
				result = result * mtm.conjugate_s() * *expressed_frame_to_master;
			}
			if(cb == 0)
			{
				result = result * *expressed_frame_to_master;
			}
			if(cb > 0)
			{
				m = get_manipulator(cb);
				joint_vector_sm = get_vector_of_single_manip_from_off_ang_vec(joint_vector, cb);
				mtm = m->joint_pose(&joint_vector_sm, 0, zb);
				result = result * mtm * *expressed_frame_to_master;
			}
		}
	}
	return result;
}

DualQuaternion MultiManipRobot::derivative_wrt_translation(DualQuaternion* master_jacobian_element, DualQuaternion* transform_to_expression_frame)
{
	return master_jacobian_element->transform_rho(transform_to_expression_frame);
}

DualQuaternion MultiManipRobot::derivative_wrt_rotation(DualQuaternion* master_jacobian_element, Quaternion* rotation_axis, Quaternion* frame_position, DualQuaternion* transform_to_expression_frame)
{
	Quaternion dp;
	DualQuaternion result;
	Quaternion zero(0,0,0,0);
	dp = rotation_axis->cross(frame_position);
	result = DualQuaternion(zero, dp);
	result = result + *master_jacobian_element;
	result = result.transform_rho(transform_to_expression_frame);
	return result;
}

DualQuatMatrix MultiManipRobot::jacobian_dep_base(FrameSymbol* frame_of_interest, FrameSymbol* frame_of_expression, DualQuaternion* frame_of_interest_to_master, DualQuaternion* frame_of_expression_to_master, RealNumMatrix* base_pose, RealNumMatrix* joint_vector)
{
	ShortInteger c;
	FrameSymbol* base_symb;
	Quaternion axis, frame_position;
	DualQuaternion identity, frame_to_base, base_to_expression_frame, deriv; 
	SpatialTransforms st;
	DualQuatMatrix jacobian(1,6);
	Quaternion zero(0,0,0,0);
	identity.set_to_identity();
	base_symb = base.get_main_frame_symbol();
	frame_to_base = pose(base_symb, frame_of_interest, &identity, frame_of_interest_to_master, base_pose, joint_vector); 
	base_to_expression_frame = pose(frame_of_expression, base_symb, frame_of_expression_to_master, &identity, base_pose, joint_vector);
	frame_position = st.dual_quaternion_to_translation_quaternion(&frame_to_base);
	for(c=1; c<=3; c++)
	{
		axis = base.get_axis_of_rotation_as_quaternion_in_base_frame(c, base_pose); 
		deriv = DualQuaternion(zero,axis);
		deriv = derivative_wrt_translation(&deriv, &base_to_expression_frame);
		jacobian.set_element(1,c,&deriv);
		deriv = DualQuaternion(axis,zero);
		deriv = derivative_wrt_rotation(&deriv, &axis, &frame_position, &base_to_expression_frame);
		jacobian.set_element(1,c+3,&deriv);
	}
	return jacobian;
}

DualQuatMatrix MultiManipRobot::jacobian_dep_manip(FrameSymbol* frame_of_interest, FrameSymbol* frame_of_expression, DualQuaternion* frame_of_interest_to_master, DualQuaternion* frame_of_expression_to_master, RealNumMatrix* base_pose, RealNumMatrix* joint_vector)
{
	UnsignShortInteger zb, cb, l;
	Quaternion position_of_frame_master_in_joint_frame, frame_position_in_master_frame, axis, joint_to_master_orient, axis_in_master;
	DualQuaternion identity, interest_frame_master_to_expression_frame, joint_to_master, deriv, master_to_joint;
	FrameSymbol master_frame, joint_frame;
	SpatialTransforms st;
	Quaternion zero(0,0,0,0);
	identity.set_to_identity();
	zb = frame_of_interest->get_master_frame_number();
	cb = frame_of_interest->get_master_manipulator_number();
	DualQuatMatrix jacobian(1,zb);
	master_frame = FrameSymbol(zb,cb);
	frame_position_in_master_frame = st.dual_quaternion_to_translation_quaternion(frame_of_interest_to_master);
	interest_frame_master_to_expression_frame = pose(frame_of_expression, &master_frame, frame_of_expression_to_master, &identity, base_pose, joint_vector);
	for(l=1; l<=zb; l++)
	{
		joint_frame = FrameSymbol(l,cb);
		joint_to_master = pose(&master_frame, &joint_frame, &identity, &identity, base_pose, joint_vector);
		axis = Quaternion(0,0,0,1);
		if(get_manipulator(cb)->get_link(l)->is_joint_prismatic())
		{
			deriv = DualQuaternion(zero,axis);
			deriv = derivative_wrt_translation(&deriv, &joint_to_master); 
			deriv = derivative_wrt_translation(&deriv, &interest_frame_master_to_expression_frame); 
		}
		if(get_manipulator(cb)->get_link(l)->is_joint_revolute())
		{
			joint_to_master_orient = joint_to_master.get_real_part();	
			axis_in_master = joint_to_master_orient * axis * joint_to_master_orient.conjugate(); 
			master_to_joint = joint_to_master.conjugate_s();
			position_of_frame_master_in_joint_frame = st.dual_quaternion_to_translation_quaternion(&master_to_joint);
			deriv = DualQuaternion(axis, zero);
			deriv = derivative_wrt_rotation(&deriv, &axis, &position_of_frame_master_in_joint_frame, &joint_to_master);
			deriv = derivative_wrt_rotation(&deriv, &axis_in_master, &frame_position_in_master_frame, &interest_frame_master_to_expression_frame);
		}
		jacobian.set_element(1,l,&deriv);
	}
	return jacobian;
}

DualQuatMatrix MultiManipRobot::inertia_dependent_on_base(RealNumMatrix* base_pose, RealNumMatrix* joint_vector)
{
	UnsignShortInteger c, w, i, j, nj;
	Link* link;
	EndEffector* end_eff;
	ConstantFrame* end_eff_com_frame;
	FrameSymbol* end_eff_com_symbol;
	ConstantFrame* link_com_frame;
	FrameSymbol* link_com_symbol;
	ConstantFrame* base_com_frame;
	DualQuaternion end_eff_com_to_inertial, end_eff_com_to_wrist, base_com_to_main, base_com_to_inertial, identity, dq, link_com_to_joint, link_com_to_inertial;
	DualMatrix n0, nb;
	Manipulator* m;
	DualQuatMatrix base_com_jacobian, link_com_jacobian, end_eff_com_jacobian;
	DualQuatMatrix n(1,6);
	identity.set_to_identity();
	base_com_frame = base.get_center_of_mass_frame();
	n.set_to_zero();
	FrameSymbol inertial_frame_symbol(-1,-1);
	FrameSymbol* base_com_symbol = base_com_frame->get_frame_symbol();
	n0 = base.get_dual_inertia_matrix();
	base_com_to_main = base_com_frame->get_dual_quaternion_representation();
	base_com_jacobian = jacobian_dep_base(base_com_symbol, base_com_symbol, &base_com_to_main, &base_com_to_main, base_pose, joint_vector);
	base_com_to_inertial = pose(&inertial_frame_symbol, base_com_symbol, &identity, &base_com_to_main, base_pose, joint_vector);
	for(c=1; c<=6; c++)
	{
		dq = base_com_jacobian.get_element(1,c);
		dq = n0.multiply_by_dual_quaternion(&dq);
		dq = dq.transform_rho(&base_com_to_inertial);
		n.increment_element(1,c,&dq);
		for(j=1; j<=num_man; j++)
		{
			m = get_manipulator(j);
			nj = m->get_number_of_degrees_of_freedom();
			end_eff = m->get_end_effector();
			end_eff_com_frame = end_eff->get_center_of_mass_frame();
			end_eff_com_symbol = end_eff_com_frame->get_frame_symbol();
			nb = end_eff->get_dual_inertia_matrix();
			end_eff_com_to_wrist = end_eff_com_frame->get_dual_quaternion_representation();
			end_eff_com_jacobian = jacobian_dep_base(end_eff_com_symbol, end_eff_com_symbol, &end_eff_com_to_wrist, &end_eff_com_to_wrist, base_pose, joint_vector);   
			end_eff_com_to_inertial = pose(&inertial_frame_symbol, end_eff_com_symbol, &identity, &end_eff_com_to_wrist, base_pose, joint_vector);
			dq = end_eff_com_jacobian.get_element(1,c);
			dq = nb.multiply_by_dual_quaternion(&dq);
			dq = dq.transform_rho(&end_eff_com_to_inertial);
			n.increment_element(1,c,&dq);
			for(i=1; i<=nj; i++)
			{
				link = m->get_link(i);
				link_com_frame = link->get_center_of_mass_frame();
				link_com_symbol = link_com_frame->get_frame_symbol();
				nb = link->get_dual_inertia_matrix();
				link_com_to_joint = link_com_frame->get_dual_quaternion_representation(); 
				link_com_jacobian = jacobian_dep_base(link_com_symbol, link_com_symbol, &link_com_to_joint, &link_com_to_joint, base_pose, joint_vector);  
				link_com_to_inertial = pose(&inertial_frame_symbol, link_com_symbol, &identity, &link_com_to_joint, base_pose, joint_vector);
				dq = link_com_jacobian.get_element(1,c);
				dq = nb.multiply_by_dual_quaternion(&dq);
				dq = dq.transform_rho(&link_com_to_inertial);
				n.increment_element(1,c,&dq);
			}
		}
	}
	return n;
}

DualQuatMatrix MultiManipRobot::inertia_dependent_on_manip(UnsignShortInteger manipulator_number, RealNumMatrix* base_pose, RealNumMatrix* joint_vector)
{
	UnsignShortInteger i, l, nj;
	DualMatrix nb, ne;
	ConstantFrame* end_eff_com_frame;
	ConstantFrame* link_com_frame;
	FrameSymbol* end_eff_com_symbol;
       	FrameSymbol* link_com_symbol;
	DualQuatMatrix link_com_jacobian, end_eff_com_jacobian, nmj;
	Link* link;
	EndEffector* end_eff;
	Manipulator* m;
	DualQuaternion identity, dq, end_eff_com_to_inertial, end_eff_com_to_wrist, link_com_to_joint, link_com_to_inertial;
	identity.set_to_identity();
	FrameSymbol inertial_frame_symbol(-1,-1);
	m = get_manipulator(manipulator_number);
	nj = m->get_number_of_degrees_of_freedom();
	nmj = DualQuatMatrix(1,nj);
	nmj.set_to_zero();
	end_eff = m->get_end_effector();
	end_eff_com_frame = end_eff->get_center_of_mass_frame();
	end_eff_com_symbol = end_eff_com_frame->get_frame_symbol();
	ne = end_eff->get_dual_inertia_matrix();
	end_eff_com_to_wrist = end_eff_com_frame->get_dual_quaternion_representation();
	end_eff_com_jacobian = jacobian_dep_manip(end_eff_com_symbol, end_eff_com_symbol, &end_eff_com_to_wrist, &end_eff_com_to_wrist, base_pose, joint_vector);   
	end_eff_com_to_inertial = pose(&inertial_frame_symbol, end_eff_com_symbol, &identity, &end_eff_com_to_wrist, base_pose, joint_vector);
	for(l=1; l<=nj; l++)
	{
		dq = end_eff_com_jacobian.get_element(1,l);
		dq = ne.multiply_by_dual_quaternion(&dq);
		dq = dq.transform_rho(&end_eff_com_to_inertial);
		nmj.increment_element(1,l,&dq);
		for(i=l; i<=nj; i++)
		{
			link = m->get_link(i);
			link_com_frame = link->get_center_of_mass_frame();
			link_com_symbol = link_com_frame->get_frame_symbol();
			nb = link->get_dual_inertia_matrix();
			link_com_to_joint = link_com_frame->get_dual_quaternion_representation(); 
			link_com_jacobian = jacobian_dep_manip(link_com_symbol, link_com_symbol, &link_com_to_joint, &link_com_to_joint, base_pose, joint_vector);  
			link_com_to_inertial = pose(&inertial_frame_symbol, link_com_symbol, &identity, &link_com_to_joint, base_pose, joint_vector);
			dq = link_com_jacobian.get_element(1,l);
			dq = nb.multiply_by_dual_quaternion(&dq);
			dq = dq.transform_rho(&link_com_to_inertial);
			nmj.increment_element(1,l,&dq);
		}
	}
	return nmj;
}

DualQuatMatrix MultiManipRobot::inertia_dependent_on_manip(RealNumMatrix* base_pose, RealNumMatrix* joint_vector)
{
	UnsignShortInteger l, n, w, j, nj;
	DualQuaternion dq;
	DualQuatMatrix nmj;
	w=0;
	n=get_number_of_degrees_of_freedom_of_all_manipulators();
	DualQuatMatrix nm(1,n);
	nm.set_to_zero();
	for(j=1; j<=num_man; j++)
	{
		nmj = inertia_dependent_on_manip(j, base_pose, joint_vector);
		nj = get_manipulator(j)->get_number_of_degrees_of_freedom();
		for(l=w+1; l<=w+nj; l++)
		{
			dq = nmj.get_element(1,l-w);
			nm.set_element(1,l,&dq);
		}
		w = w + nj;
	}
	return nm;
}

DualQuatMatrix MultiManipRobot::jacobian_dep_base_2(FrameSymbol* frame_of_expression, DualQuaternion* frame_of_expression_to_master, RealNumMatrix* base_pose, RealNumMatrix* joint_vector)
{
	UnsignShortInteger i, j;
	DualQuatMatrix jsm;
	ConstantFrame* end_eff_main_frame;
	FrameSymbol* end_eff_symbol;
	DualQuaternion e, end_eff_to_wrist;
	DualQuatMatrix result(num_man,6);
	for(i=1; i<=num_man; i++)
	{
		end_eff_main_frame = get_manipulator(i)->get_end_effector()->get_main_frame();
		end_eff_symbol = end_eff_main_frame->get_frame_symbol();
		end_eff_to_wrist = end_eff_main_frame->get_dual_quaternion_representation();
		jsm = jacobian_dep_base(end_eff_symbol, frame_of_expression, &end_eff_to_wrist, frame_of_expression_to_master, base_pose, joint_vector);
		for(j=1; j<=6; j++)
		{
			e = jsm.get_element(1,j);
			result.set_element(i,j,&e);
		}	
	}
	return result;
}

DualQuatMatrix MultiManipRobot::jacobian_dep_manip_2(FrameSymbol* frame_of_expression, DualQuaternion* frame_of_expression_to_master, RealNumMatrix* base_pose, RealNumMatrix* joint_vector)
{
	UnsignShortInteger i, l, nj, n, w;
	Manipulator* m;
	EndEffector* end_eff;
	ConstantFrame* end_eff_main_frame;
	FrameSymbol* end_eff_symbol;
	DualQuaternion e, end_eff_to_wrist;
	DualQuatMatrix jsm;
	w=0;
	n=get_number_of_degrees_of_freedom_of_all_manipulators();
	DualQuatMatrix result(num_man,n);
	result.set_to_zero();
	for(i=1; i<=num_man; i++)
	{
		m = get_manipulator(i);
		end_eff_main_frame = m->get_end_effector()->get_main_frame();
		end_eff_symbol = end_eff_main_frame->get_frame_symbol();
		end_eff_to_wrist = end_eff_main_frame->get_dual_quaternion_representation();
		jsm = jacobian_dep_manip(end_eff_symbol, frame_of_expression, &end_eff_to_wrist, frame_of_expression_to_master, base_pose, joint_vector);
		nj = m->get_number_of_degrees_of_freedom();
		for(l=w+1; l<=w+nj; l++)
		{
			e = jsm.get_element(1,l-w);
			result.set_element(i,l,&e);
		}
		w = w + nj;
	}
	return result;
}

DualQuatMatrix MultiManipRobot::extract_redundant_degrees_of_freedom(DualQuatMatrix* jacobian_matrix)
{
	bool fdoff;
	UnsignShortInteger i, j, nj, w, n;
	Manipulator* m;
	DualQuatMatrix column;
	DualQuatMatrix redundant_jacobian(num_man,1);
	redundant_jacobian.set_to_zero();
	fdoff = true;
	n = get_number_of_degrees_of_freedom_of_all_manipulators();
	w = 0;
	for(j=0; j<num_man; j++)
	{
		m = get_manipulator(num_man-j);
		nj = m->get_number_of_degrees_of_freedom();	
		for(i=0; i<nj; i++)
		{
			if(m->get_link(nj-i)->is_joint_redundant())
			{
				if(fdoff)
				{
					redundant_jacobian = jacobian_matrix->extract_column(n-w-i);
					fdoff=false;
				}
				else
				{
					column = jacobian_matrix->extract_column(n-w-i);
					redundant_jacobian.append_column(&column, false);
				}
			}
		}
		w = w + nj;
	}
	return redundant_jacobian;
}

RealNumMatrix MultiManipRobot::extract_redundant_degrees_of_freedom(RealNumMatrix* joint_velocities_vector)
{
	bool fdoff;
	UnsignShortInteger i, j, nj, w, n;
	Manipulator* m;
	RealNumMatrix redundant_velocities_vector, row;
	redundant_velocities_vector.set_to_zero();
	fdoff = true;
	n = get_number_of_degrees_of_freedom_of_all_manipulators();
	w = 0;
	for(j=0; j<num_man; j++)
	{
		m = get_manipulator(num_man-j);
		nj = m->get_number_of_degrees_of_freedom();	
		for(i=0; i<nj; i++)
		{
			if(m->get_link(nj-i)->is_joint_redundant())
			{
				if(fdoff)
				{
					redundant_velocities_vector = joint_velocities_vector->extract_row(n-w-i);
					fdoff = false;
				}
				else
				{
					row = joint_velocities_vector->extract_row(n-w-i);
					redundant_velocities_vector.append_row(&row, false);
				}
			}
		}
		w = w + nj;
	}
	return redundant_velocities_vector;
}

UnsignShortInteger MultiManipRobot::get_number_of_degrees_of_freedom_of_all_manipulators(void)
{
	UnsignShortInteger j, n, nj;
	n = 0;
	for(j=1; j<=num_man; j++)
	{
		nj=get_manipulator(j)->get_number_of_degrees_of_freedom();
		n = n + nj;
	}
	return n;
}

RealNumMatrix MultiManipRobot::extend_joint_variables_vector_to_off_ang_vec(RealNumMatrix* initial_full_offset_angle_vector, RealNumMatrix* full_joint_variables_vector)
{
	// This method extends a vector of joint variable parameters to a vector of both joint offsets and angles
	// Vectors contain parameters of the all manipulators 
	UnsignShortInteger i, n, j, ndof;
	RealNumber element;
	RealNumMatrix joint_variables_sm, off_ang_vec_sm, result_sm;
	Manipulator* man_ptr;
	Link* lnk_ptr;
	n = get_number_of_degrees_of_freedom_of_all_manipulators();
	RealNumMatrix result(2*n,1);
	for(j=1;j<=num_man;j++)
	{
		man_ptr = get_manipulator(j);
		ndof = man_ptr->get_number_of_degrees_of_freedom();
		joint_variables_sm = get_vector_of_single_manip_from_joint_var(full_joint_variables_vector, j);
		off_ang_vec_sm = get_vector_of_single_manip_from_off_ang_vec(initial_full_offset_angle_vector, j);
		result_sm = RealNumMatrix(2*ndof,1);
		for(i=1;i<=ndof;i++)
		{
			lnk_ptr = man_ptr->get_link(i);
			element = joint_variables_sm.get_element(i,1);
			if(lnk_ptr->is_joint_prismatic())
			{
				result_sm.set_element(i,1,element);
				element = off_ang_vec_sm.get_element(ndof+i,1);	
				result_sm.set_element(ndof+i,1,element);
			}
			else
			{
				result_sm.set_element(ndof+i,1,element);
				element = off_ang_vec_sm.get_element(i,1);	
				result_sm.set_element(i,1,element);
			}
		}
		if(j==1)
		{
			result = result_sm;
		}
		else
		{
			result.append_vector(&result_sm);
		}
	}
	return result;
}

RealNumMatrix MultiManipRobot::get_joint_variables_vector_from_off_ang_vec(RealNumMatrix* full_offset_angle_vector)
{
	// This method gets a vector of joint variable parameters from a vector of both joint offsets and angles
	// Vectors contain parameters of the all manipulators 
	UnsignShortInteger i, j, ndof;
	RealNumber element;
	Manipulator* man_ptr;
	Link* lnk_ptr;
	RealNumMatrix result, joint_variables_sm, off_ang_vec_sm, result_sm;
	for(j=1;j<=num_man;j++)
	{
		man_ptr = get_manipulator(j);
		ndof = man_ptr->get_number_of_degrees_of_freedom();
		off_ang_vec_sm = get_vector_of_single_manip_from_off_ang_vec(full_offset_angle_vector, j);
		result_sm = RealNumMatrix(ndof,1);
		for(i=1;i<=ndof;i++)
		{
			lnk_ptr = man_ptr->get_link(i);
			if(lnk_ptr->is_joint_prismatic())
			{
				element = off_ang_vec_sm.get_element(i,1);
			}
			else
			{
				element = off_ang_vec_sm.get_element(ndof+i,1);
			}
			result_sm.set_element(i,1,element);
		}
		if(j==1)
		{
			result = result_sm;
		}
		else
		{
			result.append_vector(&result_sm);
		}
	}
	return result;
}
