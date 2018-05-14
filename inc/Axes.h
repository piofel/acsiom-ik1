#ifndef __AXES_H_INCLUDED__
#define __AXES_H_INCLUDED__

#include "TypeDefinitions.h"
#include "RealNumMatrix.h"
#include "Quaternion.h"

class Axes
{
	private:
		RealNumMatrix* a;
		Quaternion* aq;
	public:
		Axes(void);
		~Axes(void);
		RealNumMatrix* get_axis(ShortInteger axis_number);
		Quaternion* get_axis_in_quaternion_form(ShortInteger axis_number);
};

#endif
