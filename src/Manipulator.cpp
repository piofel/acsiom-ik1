#include "Manipulator.h"

using namespace std;

Manipulator::Manipulator(ManipulatorParameters* manipulator_parameters, ShortInteger manipulator_number)
{
	ShortInteger i;
	SystemParameters lp, ep;
	manip_num = manipulator_number;
	num_dof = manipulator_parameters->get_number_of_degrees_of_freedom();
	links = new Link[num_dof];
	for(i=0; i<num_dof; i++)
	{
		lp = manipulator_parameters->get_links_parameters()[i];
		links[i] = Link(&lp, i+1, manipulator_number);
	}
	ep = manipulator_parameters->get_end_effector_parameters()[0];
	end_eff = EndEffector(&ep, num_dof, manipulator_number);	
}

Manipulator::Manipulator(const Manipulator& manipulator_object)
{
	ShortInteger i;
	manip_num = manipulator_object.manip_num;
	num_dof = manipulator_object.num_dof;
	end_eff = manipulator_object.end_eff;
	links = new Link[num_dof];
	for(i=0; i<num_dof; i++)
	{
		links[i] = manipulator_object.links[i];
	}
}

Manipulator::Manipulator(void)
{
	links = new Link[1];
}

Manipulator::~Manipulator(void)
{
	delete [] links;
}

void Manipulator::operator=(const Manipulator& right_side)
{
	ShortInteger i;
	manip_num = right_side.manip_num;
	num_dof = right_side.num_dof;
	end_eff = right_side.end_eff;
	delete [] links;
	links = new Link[num_dof];
	for(i=0; i<num_dof; i++)
	{
		links[i] = right_side.links[i];
	}
}

DualQuaternion Manipulator::joint_pose(RealNumMatrix* joint_vector_sm, ShortInteger reference_joint_number, ShortInteger joint_number)
{
	ShortInteger k, l, m, noe;
	RealNumber offset, angle;
	DualQuaternion tmppose;
	Quaternion one(1,0,0,0);
	Quaternion zero(0,0,0,0);
	DualQuaternion result(one, zero);
	if(joint_number != reference_joint_number)
	{
		tmppose = result;
		if(reference_joint_number>joint_number)
		{
			m = reference_joint_number;
			l = joint_number;
		}
		else
		{
			l = reference_joint_number;
			m = joint_number;
		}
		for(k=l+1; k<=m; k++)
		{
			noe = joint_vector_sm->get_number_of_rows();
			offset = joint_vector_sm->get_element(k,1);
			angle = joint_vector_sm->get_element(noe/2+k,1);
			tmppose = tmppose * links[k-1].calc_joint_frame(offset, angle);
		}
		if(reference_joint_number>joint_number)
		{
			result = tmppose.conjugate_s();
		}
		else
		{
			result = tmppose;
		}
	}
	return result;
}

Link* Manipulator::get_link(UnsignShortInteger link_number)
{
	Link* l;
	Errors er;
	l = links;
	if(link_number>0 && link_number<=num_dof)
	{
		l = links - 1 + link_number; 
	}
	else
	{
		er.display_error(11);	
	}
	return l;
}

EndEffector* Manipulator::get_end_effector(void)
{
	return &end_eff;
}

UnsignShortInteger Manipulator::get_number_of_degrees_of_freedom(void)
{
	return num_dof;
}

UnsignShortInteger Manipulator::get_manipulator_number(void)
{
	return manip_num;
}
