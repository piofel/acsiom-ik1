#include "RealNumMatrix.h"

using Eigen::MatrixXd;

RealNumMatrix::RealNumMatrix(UnsignShortInteger number_of_rows, UnsignShortInteger number_of_columns) : Matrix(number_of_rows, number_of_columns)
{
	elements = new RealNumber[nrow * ncol];
}

RealNumMatrix::RealNumMatrix(void) : Matrix()
{
	elements = new RealNumber[1];
	elements[0] = 0;
}

RealNumMatrix::RealNumMatrix(const RealNumMatrix& matrix_object) : Matrix(matrix_object)
{
	UnsignShortInteger i;
	elements = new RealNumber[nrow * ncol];
	for(i = 0; i < nrow * ncol; i++)
	{
		elements[i] = matrix_object.elements[i];
	}
}

RealNumMatrix::RealNumMatrix(Eigen::MatrixXd matrixxd)
{
	UnsignShortInteger i,j;
	RealNumber e;
	index_checking = DEFAULT_INDEX_CHECKING;
	nrow = matrixxd.rows();
	ncol = matrixxd.cols();
	elements = new RealNumber[nrow * ncol];
	for(i=0;i<nrow;i++)
	{
		for(j=0;j<ncol;j++)
		{
			e = matrixxd(i,j);
			set_element(i+1,j+1,e);
		}
	}
}

RealNumMatrix::~RealNumMatrix(void)
{
	delete [] elements;
}

void RealNumMatrix::operator=(const RealNumMatrix& right_side)
{
	Errors er;
	if(right_side.nrow > 0 && right_side.ncol > 0)
	{
		UnsignShortInteger i;
		nrow = right_side.nrow;
		ncol = right_side.ncol;
		delete [] elements;
		elements = new RealNumber[nrow * ncol];
		for(i = 0; i < nrow * ncol; i++)
		{
			elements[i] = right_side.elements[i];
		}
	}
	else
	{
		er.display_error(10, "RealNumMatrix::operator=");
	}
}

RealNumMatrix RealNumMatrix::operator+(const RealNumMatrix& addend_right)
{
	Errors er;
	RealNumMatrix result(1,1);
	if(nrow==addend_right.nrow && ncol==addend_right.ncol)
	{
		UnsignShortInteger i;
		result = RealNumMatrix(nrow,ncol);
		for(i=0; i<nrow*ncol; i++)
		{
			result.elements[i] = this->elements[i] + addend_right.elements[i];
		}
	}
	else
	{
		er.display_error(10,"RealNumMatrix::operator+");
	}
	return result;
}

RealNumMatrix RealNumMatrix::operator-(const RealNumMatrix& subtrahend)
{
	Errors er;
	RealNumMatrix result(1,1);
	if(nrow==subtrahend.nrow && ncol==subtrahend.ncol)
	{
		UnsignShortInteger i;
		result = RealNumMatrix(nrow,ncol);
		for(i=0; i<nrow*ncol; i++)
		{
			result.elements[i] = this->elements[i] - subtrahend.elements[i];
		}
	}
	else
	{
		er.display_error(10,"RealNumMatrix::operator-");
	}
	return result;
}

RealNumMatrix RealNumMatrix::cross(RealNumMatrix* factor_right)
{
	UnsignShortInteger i;
	RealNumber s[3], u[3], v[3];
	Errors er;
	RealNumMatrix result(3,1);
	if(nrow!=3 || ncol!=1 || factor_right->nrow!=3 || factor_right->ncol!=1)
	{
		er.display_error(10,"RealNumMatrix::cross");
	}
	else
	{
		for(i=0; i<3; i++)
		{
			u[i] = this->get_element(i+1,1);
			v[i] = factor_right->get_element(i+1,1);
		}
		s[0] = u[1] * v[2] - u[2] * v[1];
		s[1] = u[2] * v[0] - u[0] * v[2];
		s[2] = u[0] * v[1] - u[1] * v[0];
		for(i=0; i<3; i++)
		{
			result.set_element(i+1, 1, s[i]);
		}
	}
	return result;
}

RealNumMatrix RealNumMatrix::dot(RealNumMatrix* factor_right)
{
	UnsignShortInteger i, j, k;
	RealNumber e;
	Errors er;
	RealNumMatrix result(nrow,factor_right->ncol);
	if(ncol==factor_right->nrow)
	{
		for(i=1; i<=result.nrow; i++)
		{
			for(j=1; j<=result.ncol; j++)
			{
				e = 0;
				for(k=1; k<=ncol; k++)
				{
					e = e + this->get_element(i,k) * factor_right->get_element(k,j);  
				}
				result.set_element(i,j,e);
			}
		}
	}
	else
	{
		er.display_error(10,"RealNumMatrix::dot");
	}
	return result;
}

RealNumMatrix RealNumMatrix::multiply_by_scalar(RealNumber scalar)
{
	UnsignShortInteger i, nr, nc;
	nr = this->get_number_of_rows();
	nc = this->get_number_of_columns();
	RealNumMatrix result(nr,nc);
	for(i=0; i<nr*nc; i++)
	{
		result.elements[i] = this->elements[i] * scalar;
	}
	return result;
}

Quaternion RealNumMatrix::multiply_by_quaternion(Quaternion* quaternion)
{
	Errors er;
	Quaternion result;
	if(nrow==4 && ncol==4)
	{
		UnsignShortInteger i,j;
		RealNumMatrix s(1,1);
		RealNumMatrix sr(1,1);
		RealNumMatrix v(3,1);
		RealNumMatrix vr(3,1);
		RealNumMatrix g11(1,1);
		RealNumMatrix g12(1,3);
		RealNumMatrix g21(3,1);
		RealNumMatrix g22(3,3);
		s.set_element(1,1,quaternion->get_coefficients()[0]);
		v = quaternion->get_vector_part_m();
		g11.set_element(1,1,this->get_element(1,1));
		for(i=1; i<=3; i++)
		{
			g12.set_element(1,i,this->get_element(1,i+1));	
			g21.set_element(i,1,this->get_element(i+1,1));
			for(j=1; j<=3; j++)
			{
				g22.set_element(i,j,this->get_element(i+1,j+1));
			}
		}
		sr = g11.dot(&s) + g12.dot(&v);
		vr = g21.dot(&s) + g22.dot(&v);
		result = Quaternion(sr.get_element(1,1),&vr);
	}
	else
	{
		er.display_error(10,"RealNumMatrix::multiply_by_quaternion");
	}
	return result;
}	

void RealNumMatrix::set_element(UnsignShortInteger row, UnsignShortInteger column, RealNumber element)
{
	Errors er;
	if(index_checking)
	{
		if(row>0 && row<=nrow && column>0 && column<=ncol)
		{
			elements[(row - 1) * ncol + column - 1] =  element;
		}
		else
		{	
			display_index_error(row, column);
			er.display_error_location("RealNumMatrix::set_element");
		}
	}
	else
	{
			elements[(row - 1) * ncol + column - 1] =  element;
	}
}

RealNumber RealNumMatrix::get_element(UnsignShortInteger row, UnsignShortInteger column)
{
	Errors er;
	RealNumber el;
	if(index_checking)
	{
		if(row>0 && row<=nrow && column>0 && column<=ncol)
		{
			el = elements[(row-1) * ncol + column - 1];
		}
		else
		{
			el = 0;
			display_index_error(row, column);
			er.display_error_location("RealNumMatrix::get_element");
		}
	}
	else
	{
		el = elements[(row-1) * ncol + column - 1];
	}
	return el;
}

void RealNumMatrix::append_vector(RealNumMatrix* vector)
{
	UnsignShortInteger i, k;
	RealNumber* new_elements;
	Errors er;
	if(ncol>1)
	{
		er.display_error(7);
	}
	else
	{
		new_elements = new RealNumber[nrow+vector->nrow]; 
		for(i=0; i<nrow; i++)
		{
			new_elements[i] = elements[i];	
		}
		k = 0;
		for(i=nrow; i<nrow+vector->nrow; i++)
		{
			new_elements[i] = vector->elements[k];
			k++;
		}
	}
	delete [] elements;
	elements = new_elements;	
	nrow = nrow+vector->nrow;
}

void RealNumMatrix::set_to_zero(void)
{
	UnsignShortInteger i;
	for(i=0;i<nrow*ncol;i++)
	{
		elements[i] = 0;
	}
}	

RealNumMatrix RealNumMatrix::transpose(void)
{
	UnsignShortInteger i, j;
	RealNumber e;
	RealNumMatrix result(ncol,nrow);
	for(i=1; i<=nrow ; i++)
	{
		for(j=1; j<=ncol; j++)
		{
			e = get_element(i,j);
			result.set_element(j,i,e);
		}
	}
	return result;
}	

RealNumMatrix RealNumMatrix::submatrix_rc(UnsignShortInteger row, UnsignShortInteger column)
{
	Errors er;
	RealNumMatrix result;
	if(nrow>1 && ncol>1)
	{
		UnsignShortInteger i, j, ro, co;
		RealNumber e;
		result = RealNumMatrix(nrow-1,ncol-1);
		ro = 0;
		for(i=1;i<nrow;i++)
		{
			co = 0;
			for(j=1;j<ncol;j++)
			{
				if(i==row)
				{
					ro = 1;
				}
				if(j==column)
				{
					co = 1;
				}
				e = get_element(i+ro,j+co);
				result.set_element(i,j,e);
			}
		}
	}
	else
	{
		er.display_error(10, "RealNumMatrix::submatrix_rc");
	}
	return result;
}

RealNumber RealNumMatrix::first_minor(UnsignShortInteger row, UnsignShortInteger column)
{
	RealNumber result;
	RealNumMatrix sm;
	sm = submatrix_rc(row,column);
	result = sm.determinant();
	return result;
}	

RealNumber RealNumMatrix::cofactor(UnsignShortInteger row, UnsignShortInteger column)
{
	UnsignShortInteger rem;
	RealNumber m, result;
	m = first_minor(row,column);
	rem = (row+column) % 2;
	if(rem==1)
	{
		result = -m;
	}
	else
	{
		result = m;
	}
	return result;
}	

RealNumber RealNumMatrix::determinant(void)
{
	UnsignShortInteger j;
	RealNumber result, e, c;
	Errors er;
	result = 0;
	if(nrow==ncol)
	{
		if(nrow==1)
		{
			
			result = get_element(1,1);
		}
		if(nrow>1)
		{
			for(j=1;j<=ncol;j++)
			{
				e = get_element(1,j);
				c = cofactor(1,j);
				result = result + e*c;
			}
		}
	}
	else
	{
		er.display_error(18);
	}
	return result;
}

RealNumMatrix RealNumMatrix::adjugate(void)
{
	UnsignShortInteger i, j;
	RealNumber e;
	RealNumMatrix result(nrow,ncol);
	for(i=1; i<=nrow ; i++)
	{
		for(j=1; j<=ncol; j++)
		{
			e = cofactor(i,j);
			result.set_element(i,j,e);
		}
	}
	result = result.transpose();
	return result;
}


RealNumMatrix RealNumMatrix::inverse(void)
{
	RealNumber det;
	RealNumMatrix result;
	Errors er;
	det = determinant();
	if(det<ZERO_TOLERANCE_SINGULARITY_CHECK && det>-ZERO_TOLERANCE_SINGULARITY_CHECK)
	{
		er.display_error(19, "RealNumMatrix::inverse");
	}
	else
	{
		UnsignShortInteger i, j;
		RealNumber e;
		result = adjugate();
		for(i=1; i<=nrow ; i++)
		{
			for(j=1; j<=ncol; j++)
			{
				e = result.get_element(i,j);
				e = e/det;
				result.set_element(i,j,e);
			}
		}
	}
	return result;
}

void RealNumMatrix::remove_row(UnsignShortInteger row_number)
{
	Errors er;
	if(nrow>1)
	{
		UnsignShortInteger i, j, ro;
		RealNumber e;
		RealNumMatrix new_matrix(nrow-1,ncol);
		ro = 0;
		for(i=1;i<nrow;i++)
		{
			for(j=1;j<=ncol;j++)
			{
				if(i==row_number)
				{
					ro = 1;
				}
				e = get_element(i+ro,j);
				new_matrix.set_element(i,j,e);
			}
		}
		nrow = nrow-1;
		substitute_elements(&new_matrix);
	}
	else
	{
		er.display_error(5, "RealNumMatrix::remove_row");
	}
}	

RealNumMatrix RealNumMatrix::extract_row(UnsignShortInteger row_number)
{
	RealNumMatrix row(1,ncol);
	RealNumber e;
	UnsignShortInteger i;
	for(i=1;i<=ncol;i++)
	{
		e = get_element(row_number,i);
		row.set_element(1,i,e);
	}
	remove_row(row_number);
	return row;
}

void RealNumMatrix::append_row(RealNumMatrix* row, bool side)
{
	Errors er;
	if(ncol==row->ncol)
	{
		UnsignShortInteger i, j;
		RealNumber e;
		RealNumMatrix new_matrix(nrow+1,ncol);
		if(side)
		{
			// at bottom
			for(j=1;j<=ncol;j++)
			{
				for(i=1;i<=nrow;i++)
				{
					e = get_element(i,j);
					new_matrix.set_element(i,j,e);
				}
				e = row->get_element(1,j);
				new_matrix.set_element(nrow+1,j,e);
			}
		}
		else
		{
			// at top
			for(j=1;j<=ncol;j++)
			{
				e = row->get_element(1,j);
				new_matrix.set_element(1,j,e);
				for(i=1;i<=nrow;i++)
				{
					e = get_element(i,j);
					new_matrix.set_element(i+1,j,e);
				}
			}
		}
		nrow = nrow+1;
		substitute_elements(&new_matrix);
	}
	else
	{
			er.display_error(6, "RealNumMatrix::append_row");
	}
}	

void RealNumMatrix::substitute_elements(RealNumMatrix* new_matrix)
{
	UnsignShortInteger i;
	delete [] elements;
	elements = new RealNumber[nrow * ncol];
	for(i=0;i<nrow*ncol;i++)
	{
		elements[i] = new_matrix->elements[i];
	}
}

void RealNumMatrix::insert_row(RealNumMatrix* row, UnsignShortInteger row_number)
{
	Errors er;
	if(ncol==row->ncol)
	{
		UnsignShortInteger i,j;
		RealNumber e;
		RealNumMatrix new_matrix(nrow+1,ncol);
		for(i=1;i<row_number;i++)
		{
			for(j=1;j<=ncol;j++)
			{
				e = get_element(i,j);
				new_matrix.set_element(i,j,e);
			}
		}
		for(j=1;j<=ncol;j++)
		{
			e = row->get_element(1,j);
			new_matrix.set_element(row_number,j,e);
		}
		for(i=row_number; i<=nrow; i++)
		{
			for(j=1;j<=ncol;j++)
			{
				e = get_element(i,j);
				new_matrix.set_element(i+1,j,e);
			}
		}
		nrow = nrow + 1;
		substitute_elements(&new_matrix);
	}
	else
	{
			er.display_error(6, "RealNumMatrix::insert_row");
	}
}

RealNumMatrix RealNumMatrix::solve_linear_equation(RealNumMatrix* matrix)
{
	MatrixXd a, b, x;
	a = convert_to_matrixxd();
	b = matrix->convert_to_matrixxd();
	x = a.colPivHouseholderQr().solve(b);
	RealNumMatrix result(x);
	return result;
}

MatrixXd RealNumMatrix::convert_to_matrixxd(void)
{
	UnsignShortInteger i,j;
	MatrixXd m(nrow,ncol);
	RealNumber e;
	for(i=0;i<nrow;i++)
	{
		for(j=0;j<ncol;j++)
		{
			e = get_element(i+1,j+1);
			m(i,j)=e;
		}
	}
	return m;
}

void RealNumMatrix::multiply_column_by_scalar(UnsignShortInteger column_number, RealNumber scalar)
{
	Errors er;
	if(column_number>0 && column_number<=ncol)
	{
		UnsignShortInteger i;
		RealNumber e;
		for(i=1;i<=nrow;i++)
		{
			e = get_element(i,column_number);
			e = e * scalar;
			set_element(i,column_number,e);
		}
	}
	else
	{
		er.display_error(14, "RealNumMatrix::multiply_column_by_scalar");
	}
}

RealNumber RealNumMatrix::condition_number(void)
{
	RealNumber cond;
	MatrixXd mxd;
	mxd = convert_to_matrixxd();
	Eigen::JacobiSVD<MatrixXd> svd(mxd);
	cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
	return cond;
}
