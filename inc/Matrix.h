#ifndef __MATRIX_H_INCLUDED__
#define __MATRIX_H_INCLUDED__

#include "TypeDefinitions.h"
#include "Errors.h"
#include "Messages.h"


#define DEFAULT_INDEX_CHECKING true

class Matrix
{
	protected:
		bool index_checking;
		UnsignShortInteger nrow, ncol;
		void display_index_error(UnsignShortInteger row, UnsignShortInteger column);
	public:
		Matrix(ShortInteger number_of_rows, ShortInteger number_of_columns);
		Matrix(void);
		Matrix(const Matrix& matrix_object);
		UnsignShortInteger get_number_of_rows(void);
		UnsignShortInteger get_number_of_columns(void);
		void set_index_checking(void);
		bool is_the_same_size_as(Matrix* the_other_matrix);
};

#endif

