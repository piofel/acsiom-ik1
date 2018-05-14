#ifndef __REALNUMMATRIX_H_INCLUDED__
#define __REALNUMMATRIX_H_INCLUDED__

class RealNumMatrix;
class Quaternion;

#include <../Eigen/Dense>
#include "Matrix.h"
#include "Quaternion.h"

class RealNumMatrix : public Matrix
{
	private:
		RealNumber* elements;
		void substitute_elements(RealNumMatrix* new_matrix);
	public:
		RealNumMatrix(UnsignShortInteger number_of_rows, UnsignShortInteger number_of_columns);
		RealNumMatrix(void);
		RealNumMatrix(const RealNumMatrix& matrix_object);
		RealNumMatrix(Eigen::MatrixXd matrixxd);
		~RealNumMatrix(void);
		void operator=(const RealNumMatrix& right_side);
		RealNumMatrix operator+(const RealNumMatrix& addend_right);
		RealNumMatrix operator-(const RealNumMatrix& subtrahend);
		RealNumMatrix cross(RealNumMatrix* factor_right);
		RealNumMatrix dot(RealNumMatrix* factor_right);
		RealNumMatrix multiply_by_scalar(RealNumber scalar);  
		Quaternion multiply_by_quaternion(Quaternion* quaternion);  
		void set_element(UnsignShortInteger row, UnsignShortInteger column, RealNumber element);
		RealNumber get_element(UnsignShortInteger row, UnsignShortInteger column);
		void append_vector(RealNumMatrix* vector); 
		void set_to_zero(void);
		RealNumMatrix transpose(void); 
		RealNumMatrix submatrix_rc(UnsignShortInteger row, UnsignShortInteger column); 
		RealNumber first_minor(UnsignShortInteger row, UnsignShortInteger column); 
		RealNumber cofactor(UnsignShortInteger row, UnsignShortInteger column); 
		RealNumber determinant(void);
		RealNumMatrix adjugate(void); 
		RealNumMatrix inverse(void); 
		void remove_row(UnsignShortInteger row_number); 
		RealNumMatrix extract_row(UnsignShortInteger row_number);
		void append_row(RealNumMatrix* row, bool side); 
		void insert_row(RealNumMatrix* row, UnsignShortInteger row_number); 
		RealNumMatrix solve_linear_equation(RealNumMatrix* matrix);
		Eigen::MatrixXd convert_to_matrixxd(void);
		void multiply_column_by_scalar(UnsignShortInteger column_number, RealNumber scalar);
		RealNumber condition_number(void);
};

#endif

