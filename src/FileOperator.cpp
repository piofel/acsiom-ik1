#include "FileOperator.h"

using namespace std;

FileOperator::FileOperator(void)
{
}

FileOperator::~FileOperator(void)
{
}

void FileOperator::append_to_output_file(std::string text, std::string file_name)
{
	const char* file_name_c;
	Errors er;
	string fp;
	ofstream out_stream;
	fp = string(OUTPUT_FILE_PATH);
	file_name = fp + file_name;
	file_name_c = file_name.c_str();
	out_stream.open(file_name_c, ios::app);
	if(out_stream.fail())
	{
		er.display_error(0, "FileOperator::append_to_output_file");
	}
	else
	{
		out_stream << text;
		out_stream.close();
	}
}

void FileOperator::delete_file_content(std::string file_name)
{
	const char* file_name_c;
	Errors er;
	string fp;
	ofstream out_stream;
	fp = string(OUTPUT_FILE_PATH);
	file_name = fp + file_name;
	file_name_c = file_name.c_str();
	out_stream.open(file_name_c, ios::trunc);
	if(out_stream.fail())
	{
		er.display_error(0, "FileOperator::delete_file_content");
	}
	else
	{
		out_stream.close();
	}
}

void FileOperator::save_timeseries_to_file(MultiManipRobot* multi_manip_robot, RealNumMatrix timeseries, char timeseries_type)
{
	Errors er;
	UnsignShortInteger nm, j, i;
	string units_header, file_name, row;
	Link* l;
	nm = multi_manip_robot->get_number_of_manipulators();
	units_header = "#\ts\t"; 
	switch(timeseries_type)
	{
		case 'b':
			file_name = "base_pose.txt";
			for(i=1;i<=3;i++)
			{
				units_header += "m\t ";
			}
			for(i=1;i<=3;i++)
			{
				units_header += "deg\t ";
				timeseries.multiply_column_by_scalar(4+i,180/M_PI);
			}
			break;
		case 'j':
			file_name = "joint_variables.txt";
			for(j=0;j<nm;j++)
			{
				for(i=1;i<=6;i++)
				{
					l = multi_manip_robot->get_manipulator(j+1)->get_link(i);
					if(l->is_joint_prismatic())
					{
						units_header += "m\t";
					}
					else
					{
						units_header += "deg\t ";
						timeseries.multiply_column_by_scalar(1+i+6*j,180/M_PI);
					}
				}
			}
			break;
		case 'c':
			file_name = "joint_variables_rates.txt";
			for(j=0;j<nm;j++)
			{
				for(i=1;i<=6;i++)
				{
					l = multi_manip_robot->get_manipulator(j+1)->get_link(i);
					if(l->is_joint_prismatic())
					{
						units_header += "m/s\t";
					}
					else
					{
						units_header += "deg/s\t ";
						timeseries.multiply_column_by_scalar(1+i+6*j,180/M_PI);
					}
				}
			}
			break;
		case 'v':
			file_name = "end_effectors_velocities.txt";
			for(j=0;j<nm;j++)
			{
				for(i=1;i<=3;i++)
				{
					timeseries.multiply_column_by_scalar(1+i+6*j,180/M_PI);
					units_header += "deg/s\t ";
				}
				for(i=1;i<=3;i++)
				{
					units_header += "m/s\t";
				}
			}
			break;
		case 'p':
			file_name = "end_effectors_poses.txt";
			for(j=0;j<nm;j++)
			{
				for(i=1;i<=3;i++)
				{
					timeseries.multiply_column_by_scalar(1+i+6*j,180/M_PI);
					units_header += "deg\t ";
				}
				for(i=1;i<=3;i++)
				{
					units_header += "m\t ";
				}
			}
			break;
		case 'e':
			file_name = "end_eff_velocities_error.txt";
			for(j=0;j<nm;j++)
			{
				for(i=1;i<=3;i++)
				{
					timeseries.multiply_column_by_scalar(1+i+6*j,180/M_PI);
					units_header += "deg/s\t ";
				}
				for(i=1;i<=3;i++)
				{
					units_header += "m/s\t";
				}
			}
			break;
		case 'n':
			units_header += "condition\t";
			file_name = "jacobian_condition.txt";
			break;
		default:
			er.display_error(8, "FileOperator::save_timeseries_to_file");
	}
	units_header += "\n";
	delete_file_content(file_name);
	append_to_output_file(units_header, file_name);
	for(i=1;i<=timeseries.get_number_of_rows();i++)
	{
		row = "";
		for(j=1;j<=timeseries.get_number_of_columns();j++)
		{
			row += SSTR(timeseries.get_element(i,j));
			row += "\t";
		}
		row += "\n";
		append_to_output_file(row, file_name);
	}
}

SystemParameters FileOperator::read_system_parameters(std::string file_name)
{
	const char* file_name_c;
	UnsignShortInteger pos_of_number_beginning, int_params_begin_line, float_params_begin_line;
	UnsignShortInteger input_file_line_number = 0;
	UnsignShortInteger int_parameter_number = 0;
	UnsignShortInteger float_parameter_number = 0;
	Integer* int_parameters;
	RealNumber* float_parameters;
	ifstream in_stream;
	SystemParameters parameters;
	string line, number_str, fp;
	Errors er;
	fp = string(INPUT_FILE_PATH);
	file_name = fp + file_name;
	file_name_c = file_name.c_str();
	in_stream.open(file_name_c);
	if(in_stream.fail())
	{
		er.display_error(0, "FileOperator::read_system_parameters");
	}
	else
	{
		while(!in_stream.eof())
		{		
			getline(in_stream, line);
			input_file_line_number++;
			if(line.find("Integer parameters") != string::npos)
			{
				int_params_begin_line = input_file_line_number;
			}
			if(line.find("Floating point parameters") != string::npos)
			{
				float_params_begin_line = input_file_line_number;
			}
		}
		in_stream.close();
		int_parameters = new Integer[float_params_begin_line - int_params_begin_line - 2];
		float_parameters = new RealNumber[input_file_line_number - float_params_begin_line - 2];
		input_file_line_number = 0;
		in_stream.open(file_name_c);
		while(!in_stream.eof())
		{		
			getline(in_stream, line);
			input_file_line_number++;
			pos_of_number_beginning = line.find_first_of("+-0123456789.", 0);
			if(pos_of_number_beginning <= line.length()) // checking if record contains a number
			{
				number_str = line.substr(pos_of_number_beginning);
				if(input_file_line_number > int_params_begin_line && input_file_line_number < float_params_begin_line)
				{
					int_parameter_number++;
					int_parameters[int_parameter_number-1] = atoi(number_str.c_str());
				}
				if(input_file_line_number > float_params_begin_line)	
				{
					float_parameter_number++;
					float_parameters[float_parameter_number-1] = atof(number_str.c_str());
				}		
			}
		}	
		in_stream.close();
		parameters = SystemParameters(int_parameter_number, float_parameter_number, int_parameters, float_parameters);
		delete [] int_parameters;
		delete [] float_parameters;
	}
	return parameters;
} 

ManipulatorParameters FileOperator::read_manipulator_parameters(ShortInteger manipulator_number)
{
	Errors er;
	ShortInteger num_dof, i;
	string file;
	SystemParameters lp, ep, smp;
	ManipulatorParameters params;
	file = string("man");
	file += SSTR(manipulator_number);
	file += string(".txt");
	smp = read_system_parameters(file);
	num_dof = smp.get_int_parameter(1);
	if(num_dof>0)
	{
		SystemParameters* links_parameters;
		SystemParameters* end_eff_parameters;
		links_parameters = new SystemParameters[num_dof];
		for(i=1; i<=num_dof; i++)
		{
			file = string("link");
			file += SSTR(i);
			file += SSTR(manipulator_number);
			file += string(".txt");
			links_parameters[i-1] = read_system_parameters(file);
		}
		end_eff_parameters = new SystemParameters;
		file = string("ee");
		file += SSTR(manipulator_number);
		file += string(".txt");
		end_eff_parameters[0] = read_system_parameters(file);
		params = ManipulatorParameters(num_dof, links_parameters, end_eff_parameters);
		delete [] links_parameters;
		delete end_eff_parameters;
	}
	else
	{
		er.display_error(25, "FileOperator::read_manipulator_parameters");
	}
	return params;
}

MultiManipRobotParameters FileOperator::read_multi_manip_robot_parameters(void)
{
	UnsignShortInteger num_man, i;
	SystemParameters srp;
	SystemParameters* base_parameters;
	ManipulatorParameters* manip_parameters;
	srp = read_system_parameters("robot.txt");
	num_man = srp.get_int_parameter(1);
	manip_parameters = new ManipulatorParameters[num_man];
	for(i=0; i<num_man; i++)
	{
		manip_parameters[i] = read_manipulator_parameters(i+1); 
	}	
	base_parameters = new SystemParameters;
	base_parameters[0] = read_system_parameters("base.txt");
	MultiManipRobotParameters params(num_man, manip_parameters, base_parameters);
	delete [] manip_parameters;
	delete base_parameters;
	return params;
}

RealNumMatrix FileOperator::read_base_pose_derivative(ShortInteger derivative_number)
{
	ShortInteger i, u;
	RealNumber p;
	SystemParameters bp;
	SpatialTransforms st;
	Errors er;
	string file;
	RealNumMatrix bpd(6,1); 
	switch(derivative_number)
	{
		case 0:
			file = string("base_pose.txt");
			break;
		case 1:
			file = string("base_pose_rate.txt");
			break;
		default:
			file = string("base_pose.txt");
			er.display_error(2);
	}
	bp = read_system_parameters(file);
	for(i=0; i<3; i++)
	{
		p = bp.get_float_parameter(i+1);
		bpd.set_element(i+1,1,p);
		u = bp.get_int_parameter(i+1);
		p = bp.get_float_parameter(i+4);
		if(u == 1)
		{
			p = st.deg_to_rad(p);
		}
		bpd.set_element(i+4,1,p);
	}
	return bpd;
}

RealNumMatrix FileOperator::read_joint_vector(MultiManipRobot* multi_manip_robot, ShortInteger manipulator_number)
{
	ShortInteger i, n, u, np;
	RealNumber p;
	string file;
	SpatialTransforms st;
	Errors er;
	Manipulator* m;
	SystemParameters jv;
	file = string("joint_vector_");
	file += SSTR(manipulator_number);
	file += string(".txt");
	jv = read_system_parameters(file);
	m = multi_manip_robot->get_manipulator(manipulator_number);
	n = m->get_number_of_degrees_of_freedom();
	RealNumMatrix jvd(2*n,1);
	np = (jv.get_number_of_float_parameters() - 1) / 2;
	for(i=0; i<n; i++)
	{
		p = jv.get_float_parameter(i+1);
		jvd.set_element(i+1,1,p);
		u = jv.get_int_parameter(i+1);
		p = jv.get_float_parameter(np+i+1);
		if(u == 1)
		{
			p = st.deg_to_rad(p);
		}
		jvd.set_element(n+i+1,1,p);
	}
	return jvd;
}

RealNumMatrix FileOperator::read_joint_vector(MultiManipRobot* multi_manip_robot)
{
	UnsignShortInteger i, nm;
	RealNumMatrix joint_vector, joint_vector_sing_man;
	nm = multi_manip_robot->get_number_of_manipulators();
	joint_vector = read_joint_vector(multi_manip_robot, 1);
	for(i=2; i<=nm; i++)
	{
		joint_vector_sing_man = read_joint_vector(multi_manip_robot, i);
		joint_vector.append_vector(&joint_vector_sing_man);	
	}
	return joint_vector;
}

DualQuaternion FileOperator::read_dual_quaternion_coefficients(std::string file_name, bool angle_conversion)
{
	UnsignShortInteger i, np, u;
	SpatialTransforms st;
	RealNumber pr[4], pd[4];
	Errors er;
	DualQuaternion result;
	SystemParameters sp; 
	Quaternion r, d;
	sp = read_system_parameters(file_name);
	np = sp.get_number_of_float_parameters();
	switch(np)
	{
		case 6:
			for(i=0;i<3;i++)
			{
				pr[i] = sp.get_float_parameter(i);
				pd[i] = sp.get_float_parameter(i+3);
				if(angle_conversion)
				{
					u = sp.get_int_parameter(i+1);  
					if(u==1)
					{
						pr[i] = st.deg_to_rad(pr[i]);
					}
				}
			}
			r = Quaternion(0,pr[0],pr[1],pr[2]);
			d = Quaternion(0,pd[0],pd[1],pd[2]);
			break;
		case 8:
			for(i=0;i<4;i++)
			{
				pr[i] = sp.get_float_parameter(i);
				pd[i] = sp.get_float_parameter(i+4);
				if(angle_conversion)
				{
					u = sp.get_int_parameter(i+1);  
					if(u==1)
					{
						pr[i] = st.deg_to_rad(pr[i]);
					}
				}
			}
			r = Quaternion(pr);
			d = Quaternion(pd);
			break;
		default:
			er.display_error(22, "FileOperator::read_dual_quaternion_coefficients");
	}
	result = DualQuaternion(r,d);
	return result;
}

DualQuaternion FileOperator::read_initial_momentum(void)
{
	DualQuaternion result;
	result = read_dual_quaternion_coefficients("initial_momentum.txt", false);
	return result;
}

RealNumMatrix FileOperator::read_joint_velocities(MultiManipRobot* multi_manip_robot, ShortInteger manipulator_number)
{
	ShortInteger u;
	UnsignShortInteger i, n, np;
	RealNumber p;
	string file;
	SpatialTransforms st;
	Errors er;
	Manipulator* m;
	SystemParameters jv;
	file = string("joint_vector_rate_");
	file += SSTR(manipulator_number);
	file += string(".txt");
	jv = read_system_parameters(file);
	m = multi_manip_robot->get_manipulator(manipulator_number);
	n = m->get_number_of_degrees_of_freedom();
	RealNumMatrix result(n,1);
	np = (jv.get_number_of_float_parameters() - 1) / 2;
	for(i=1; i<=n; i++)
	{
		if(m->get_link(i)->is_joint_prismatic())
		{
			p = jv.get_float_parameter(i);
		}
		if(m->get_link(i)->is_joint_revolute())
		{
			u = jv.get_int_parameter(i);
			p = jv.get_float_parameter(np+i);
			if(u == 1)
			{
				p = st.deg_to_rad(p);
			}
		}
		result.set_element(i,1,p);
	}
	return result;
}

RealNumMatrix FileOperator::read_joint_velocities(MultiManipRobot* multi_manip_robot)
{
	UnsignShortInteger i, nm;
	RealNumMatrix joint_velocities, joint_velocities_sing_man;
	nm = multi_manip_robot->get_number_of_manipulators();
	joint_velocities = read_joint_velocities(multi_manip_robot, 1);
	for(i=2; i<=nm; i++)
	{
		joint_velocities_sing_man = read_joint_velocities(multi_manip_robot, i);
		joint_velocities.append_vector(&joint_velocities_sing_man);	
	}
	return joint_velocities;
}

DualQuaternion FileOperator::read_end_effector_velocity(UnsignShortInteger manipulator_number)
{
	DualQuaternion e;
	string file;
	file = string("end_eff_velocity_");
	file += SSTR(manipulator_number);
	file += string(".txt");
	e = read_dual_quaternion_coefficients(file, true);
	return e;
}

DualQuatMatrix FileOperator::read_end_effectors_velocities(MultiManipRobot* multi_manip_robot)
{
	UnsignShortInteger i, nm;
	DualQuaternion e;
	nm = multi_manip_robot->get_number_of_manipulators();
	DualQuatMatrix result(nm,1);
	for(i=1; i<=nm; i++)
	{
		e = read_end_effector_velocity(i);
		result.set_element(i,1,&e);	
	}
	return result;
}

RealNumMatrix FileOperator::read_redundant_joints_velocities(MultiManipRobot* multi_manip_robot)
{
	bool first_red_manip;
	UnsignShortInteger j, np, i, u, nm;
	SpatialTransforms st;
	RealNumMatrix joint_velocities_sing_man;
	RealNumber p;
	SystemParameters sp;
	string file;
	RealNumMatrix joint_velocities(1,1);
	joint_velocities.set_to_zero();
	first_red_manip = true;
	nm = multi_manip_robot->get_number_of_manipulators();
	for(j=1; j<=nm; j++)
	{
		file = string("redund_dof_veloc_");
		file += SSTR(j);
		file += string(".txt");
		sp = read_system_parameters(file);
		np = sp.get_number_of_float_parameters();
		if(np!=0)
		{
			joint_velocities_sing_man = RealNumMatrix(np,1);
			for(i=0;i<np;i++)
			{
				p = sp.get_float_parameter(i);
				u = sp.get_int_parameter(i+1);
				if(u==1)
				{
					p = st.deg_to_rad(p);
				}
				joint_velocities_sing_man.set_element(i+1,1,p);
			}
			if(first_red_manip)
			{
				first_red_manip = false;
				joint_velocities = joint_velocities_sing_man;
			}
			else
			{
				joint_velocities.append_vector(&joint_velocities_sing_man);	
			}
		}
	}
	return joint_velocities;
}
