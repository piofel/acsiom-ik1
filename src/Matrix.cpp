#include "Matrix.h"

Matrix::Matrix(ShortInteger number_of_rows, ShortInteger number_of_columns)
{
	Errors er;
	if(number_of_rows > 0)
	{
		nrow = number_of_rows;
	}
	else
	{
		er.display_error(5, "Matrix::Matrix");
		nrow = 1;
	}
	if(number_of_columns > 0)
	{
		ncol = number_of_columns;
	}
	else
	{
		er.display_error(6, "Matrix::Matrix");
		ncol = 1;
	}
	index_checking = DEFAULT_INDEX_CHECKING;
}

Matrix::Matrix(void)
{
	nrow = 1;
	ncol = 1;
	index_checking = DEFAULT_INDEX_CHECKING;
}

Matrix::Matrix(const Matrix& matrix_object)
{
	nrow = matrix_object.nrow;
	ncol = matrix_object.ncol;
	index_checking = matrix_object.index_checking;
}

UnsignShortInteger Matrix::get_number_of_rows(void)
{
	return nrow;
}

UnsignShortInteger Matrix::get_number_of_columns(void)
{
	return ncol;
}

void Matrix::display_index_error(UnsignShortInteger row, UnsignShortInteger column)
{
	Errors er;
	if(row<1 || row>nrow)
	{
		er.display_error(13);
	}
	if(column<1 || column>ncol)
	{
		er.display_error(14);
	}
}

void Matrix::set_index_checking(void)
{
	index_checking = true;
}

bool Matrix::is_the_same_size_as(Matrix* the_other_matrix)
{
	bool result;
	if(nrow==the_other_matrix->nrow && ncol==the_other_matrix->ncol)
	{
		result = true;
	}
	else
	{
		result = false;
	}
	return result;
}
