#include "Errors.h"

using namespace std;

Errors::Errors(void)
{
	errors[0] = "file opening failed";
	errors[1] = "unknown frame";
	errors[2] = "incorrect derivative number";
	errors[3] = "incorrect manipulator number";
	errors[4] = "incorrect joint number";
	errors[5] = "incorrect number of rows";
	errors[6] = "incorrect number of colums";
	errors[7] = "operands must be vectors";
	errors[8] = "option does not exist";
	errors[9] = "test error message";
	errors[10] = "incorrect matrix dimensions";
	errors[11] = "incorrect link number";
	errors[12] = "incorrect dependency type";
	errors[13] = "incorrect row number";
	errors[14] = "incorrect column number";
	errors[15] = "incorrect robot type";
	errors[16] = "the real part is not a pure quaternion";
	errors[17] = "the dual part is not a pure quaternion";
	errors[18] = "not a square matrix";
	errors[19] = "matrix is singular";
	errors[20] = "method not implemented properly";
	errors[21] = "incorrect coefficient number";
	errors[22] = "incorrect number of parameters";
	errors[23] = "incorrect joint type";
	errors[24] = "incorrect parameter number";
	errors[25] = "incorrect number of degrees of freedom";
	errors[26] = "incorrect number of redundant joint velocities";
}

void Errors::display_error(UnsignShortInteger error_number)
{
	cout << "Acsiom system error no. " << error_number << ": " << errors[error_number] << "." << endl;
}

void Errors::display_error(UnsignShortInteger error_number, std::string error_location)
{
	display_error(error_number);
	display_error_location(error_location);
}

void Errors::display_error_location(std::string error_location)
{
	cout << "\tError location: " << error_location << endl;
}
