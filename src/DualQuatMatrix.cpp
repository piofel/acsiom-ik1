#include "DualQuatMatrix.h"

DualQuatMatrix::DualQuatMatrix(UnsignShortInteger number_of_rows, UnsignShortInteger number_of_columns) : Matrix(number_of_rows, number_of_columns)
{
	elements = new DualQuaternion[nrow * ncol];
}

DualQuatMatrix::DualQuatMatrix(void) : Matrix()
{
	elements = new DualQuaternion[1];
}

DualQuatMatrix::DualQuatMatrix(const DualQuatMatrix& matrix_object) : Matrix(matrix_object)
{
	ShortInteger i;
	elements = new DualQuaternion[nrow * ncol];
	for(i = 0; i < nrow * ncol; i++)
	{
		elements[i] = matrix_object.elements[i];
	}
}

DualQuatMatrix::~DualQuatMatrix(void)
{
	delete [] elements;
}

void DualQuatMatrix::operator=(const DualQuatMatrix& right_side)
{
	Errors er;
	if(right_side.nrow > 0 && right_side.ncol > 0)
	{
		ShortInteger i;
		nrow = right_side.nrow;
		ncol = right_side.ncol;
		delete [] elements;
		elements = new DualQuaternion[nrow * ncol];
		for(i = 0; i < nrow * ncol; i++)
		{
			elements[i] = right_side.elements[i];
		}
	}
	else
	{
		er.display_error(10, "DualNumMatrix::operator=");
	}
}

DualQuatMatrix DualQuatMatrix::operator+(const DualQuatMatrix& addend_right)
{
	Errors er;
	DualQuatMatrix result;
	if(nrow==addend_right.nrow && ncol==addend_right.ncol)
	{
		UnsignShortInteger i;
		result = DualQuatMatrix(nrow,ncol);
		for(i=0; i<nrow*ncol; i++)
		{
			result.elements[i] = this->elements[i] + addend_right.elements[i];
		}
	}
	else
	{
		er.display_error(10,"DualQuatMatrix::operator+");
		result = DualQuatMatrix(1,1);
	}
	return result;
}

DualQuatMatrix DualQuatMatrix::operator-(const DualQuatMatrix& subtrahend)
{
	Errors er;
	DualQuatMatrix result;
	if(nrow==subtrahend.nrow && ncol==subtrahend.ncol)
	{
		UnsignShortInteger i;
		result = DualQuatMatrix(nrow,ncol);
		for(i=0; i<nrow*ncol; i++)
		{
			result.elements[i] = this->elements[i] - subtrahend.elements[i];
		}
	}
	else
	{
		er.display_error(10,"DualQuatMatrix::operator-");
		result = DualQuatMatrix(1,1);
	}
	return result;
}

DualQuatMatrix DualQuatMatrix::dot(DualQuatMatrix* factor_right)
{
	UnsignShortInteger i, j, k;
	DualQuaternion e;
	Errors er;
	DualQuatMatrix result(nrow,factor_right->ncol);
	if(ncol==factor_right->nrow)
	{
		for(i=1; i<=result.nrow; i++)
		{
			for(j=1; j<=result.ncol; j++)
			{
				e.set_to_zero();
				for(k=1; k<=ncol; k++)
				{
					e = e + this->get_element(i,k) * factor_right->get_element(k,j);  
				}
				result.set_element(i,j,&e);
			}
		}
	}
	else
	{
		er.display_error(10,"DualQuatMatrix::dot");
	}
	return result;
}

void DualQuatMatrix::set_element(UnsignShortInteger row, UnsignShortInteger column, DualQuaternion* element)
{
	Errors er;
	if(index_checking)
	{
		if(row>0 && row<=nrow && column>0 && column<=ncol)
		{
			elements[(row - 1) * ncol + column - 1] =  *element;
		}
		else
		{
			display_index_error(row, column);
			er.display_error_location("DualQuatMatrix::set_element");
		}
	}
	else
	{
		elements[(row - 1) * ncol + column - 1] =  *element;
	}
}

DualQuaternion DualQuatMatrix::get_element(UnsignShortInteger row, UnsignShortInteger column)
{
	Errors er;
	DualQuaternion el;
	if(index_checking)
	{
		if(row>0 && row<=nrow && column>0 && column<=ncol)
		{
			el = elements[(row-1) * ncol + column - 1];
		}
		else
		{
			el.set_to_zero();
			display_index_error(row, column);
			er.display_error_location("DualQuatMatrix::get_element");
		}
	}
	else
	{
		el = elements[(row-1) * ncol + column - 1];
	}
	return el;
}

void DualQuatMatrix::increment_element(UnsignShortInteger row, UnsignShortInteger column, DualQuaternion* increment_value)
{
	Errors er;
	DualQuaternion el;
	if(index_checking)
	{
		if(row>0 && row<=nrow && column>0 && column<=ncol)
		{
			el = get_element(row,column);
			el = el + *increment_value;
			set_element(row,column,&el);
		}
		else
		{
			display_index_error(row, column);
			er.display_error_location("DualQuatMatrix::increment_element");
		}
	}
	else
	{
		el = get_element(row,column);
		el = el + *increment_value;
		set_element(row,column,&el);
	}
}

DualQuatMatrix DualQuatMatrix::multiply_by_real_num_matrix(RealNumMatrix* real_matrix)
{
	UnsignShortInteger i, j, k;
	DualQuaternion e, d;
	Errors er;
	DualQuatMatrix result(nrow,real_matrix->get_number_of_columns());
	if(ncol==real_matrix->get_number_of_rows())
	{
		for(i=1; i<=result.nrow; i++)
		{
			for(j=1; j<=result.ncol; j++)
			{
				e.set_to_zero();
				for(k=1; k<=ncol; k++)
				{
					d = this->get_element(i,k);
					d.multiply_by_scalar(real_matrix->get_element(k,j));
					e = e + d;  
				}
				result.set_element(i,j,&e);
			}
		}
	}
	else
	{
		er.display_error(10,"DualQuatMatrix::multiply_by_real_num_matrix");
	}	
	return result;
}

DualQuatMatrix DualQuatMatrix::transpose(void)
{
	UnsignShortInteger i, j;
	DualQuaternion e;
	DualQuatMatrix result(ncol,nrow);
	for(i=1; i<=nrow ; i++)
	{
		for(j=1; j<=ncol; j++)
		{
			e = get_element(i,j);
			result.set_element(j,i,&e);
		}
	}
	return result;
}

void DualQuatMatrix::set_to_zero(void)
{
	UnsignShortInteger i;
	DualQuaternion e;
	e.set_to_zero();
	for(i=0;i<nrow*ncol;i++)
	{
		elements[i] = e;
	}
}

RealNumMatrix DualQuatMatrix::real_matrix_6(void)
{
	UnsignShortInteger i, j, r;
	Errors er;
	DualQuaternion e;
	RealNumMatrix result(nrow*6,ncol);
	RealNumMatrix col;
	RealNumber el;
	for(r=1; r<=nrow; r++)
	{
		for(j=1; j<=ncol; j++)
		{
			e = get_element(r,j);
			col = e.column_6();
			for(i=1; i<=6; i++)
			{
				el = col.get_element(i,1);
				result.set_element((r-1)*6+i,j,el);
			}
		}
	}
	return result;
}

void DualQuatMatrix::remove_column(UnsignShortInteger column_number)
{
	Errors er;
	if(ncol>1)
	{
		UnsignShortInteger i, j, co;
		DualQuaternion e;
		DualQuatMatrix new_matrix(nrow,ncol-1);
		for(i=1;i<=nrow;i++)
		{
			co = 0;
			for(j=1;j<ncol;j++)
			{
				if(j==column_number)
				{
					co = 1;
				}
				e = get_element(i,j+co);
				new_matrix.set_element(i,j,&e);
			}
		}
		ncol = ncol-1;
		delete [] elements;
		elements = new DualQuaternion[nrow * ncol];
		for(i=0;i<nrow*ncol;i++)
		{
			elements[i] = new_matrix.elements[i];
		}
	}
	else
	{
		er.display_error(6, "DualQuatMatrix::remove_column");
	}
}

DualQuatMatrix DualQuatMatrix::extract_column(UnsignShortInteger column_number)
{
	DualQuatMatrix column(nrow,1);
	DualQuaternion e;
	UnsignShortInteger i;
	for(i=1;i<=nrow;i++)
	{
		e = get_element(i,column_number);
		column.set_element(i,1,&e);
	}
	remove_column(column_number);
	return column;
}

void DualQuatMatrix::append_column(DualQuatMatrix* column, bool side)
{
	Errors er;
	if(nrow==column->nrow)
	{
		UnsignShortInteger i, j;
		DualQuaternion e;
		DualQuatMatrix new_matrix(nrow,ncol+1);
		if(side)
		{
			for(i=1;i<=nrow;i++)
			{
				for(j=1;j<=ncol;j++)
				{
					e = get_element(i,j);
					new_matrix.set_element(i,j,&e);
				}
				e = column->get_element(i,1);
				new_matrix.set_element(i,ncol+1,&e);
			}
		}
		else
		{
			for(i=1;i<=nrow;i++)
			{
				e = column->get_element(i,1);
				new_matrix.set_element(i,1,&e);
				for(j=1;j<=ncol;j++)
				{
					e = get_element(i,j);
					new_matrix.set_element(i,j+1,&e);
				}
			}
		}
		ncol = ncol+1;
		delete [] elements;
		elements = new DualQuaternion[nrow * ncol];
		for(i=0;i<nrow*ncol;i++)
		{
			elements[i] = new_matrix.elements[i];
		}
	}
	else
	{
		er.display_error(5, "DualQuatMatrix::append_column");
	}
}
