#ifndef __DUALQUATMATRIX_H_INCLUDED__
#define __DUALQUATMATRIX_H_INCLUDED__

#include "Matrix.h"
#include "DualQuaternion.h"

class DualQuatMatrix : public Matrix
{
	private:
		DualQuaternion* elements;
	public:
		DualQuatMatrix(UnsignShortInteger number_of_rows, UnsignShortInteger number_of_columns);
		DualQuatMatrix(void);
		DualQuatMatrix(const DualQuatMatrix& matrix_object);
		~DualQuatMatrix(void);
		void operator=(const DualQuatMatrix& right_side);
		DualQuatMatrix operator+(const DualQuatMatrix& addend_right);
		DualQuatMatrix operator-(const DualQuatMatrix& subtrahend);
		DualQuatMatrix dot(DualQuatMatrix* factor_right);
		void set_element(UnsignShortInteger row, UnsignShortInteger column, DualQuaternion* element);
		void increment_element(UnsignShortInteger row, UnsignShortInteger column, DualQuaternion* increment_value);
		DualQuaternion get_element(UnsignShortInteger row, UnsignShortInteger column);
		DualQuatMatrix multiply_by_real_num_matrix(RealNumMatrix* real_matrix);
		DualQuatMatrix transpose(void); 
		void set_to_zero(void);
		RealNumMatrix real_matrix_6(void);
		void remove_column(UnsignShortInteger column_number);
		DualQuatMatrix extract_column(UnsignShortInteger column_number);
		void append_column(DualQuatMatrix* column, bool side);
};

#endif

