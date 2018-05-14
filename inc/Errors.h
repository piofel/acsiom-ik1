#ifndef __ERRORS_H_INCLUDED__
#define __ERRORS_H_INCLUDED__

#include <iostream>
#include "TypeDefinitions.h"

#define ERROR_BASE_SIZE 30

class Errors
{
	private:
		std::string errors[ERROR_BASE_SIZE];
	public:
		Errors(void);
		void display_error(UnsignShortInteger error_number);
		void display_error(UnsignShortInteger error_number, std::string error_location);
		void display_error_location(std::string error_location);
};

#endif
