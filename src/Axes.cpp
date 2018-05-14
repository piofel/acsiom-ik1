#include "Axes.h"

Axes::Axes(void)
{
	ShortInteger i, j;
	a = new RealNumMatrix[3];
	aq = new Quaternion;
	for(i=0; i<3; i++)
	{
		a[i] = RealNumMatrix(3,1);
		for(j=1; j<=3; j++)
		{
			a[i].set_element(j,1,0);
		}
		a[i].set_element(i+1,1,1);
	}
}

Axes::~Axes(void)
{
	delete [] a;
	delete aq;
}

RealNumMatrix* Axes::get_axis(ShortInteger axis_number)
{
	return a + axis_number - 1;
}

Quaternion* Axes::get_axis_in_quaternion_form(ShortInteger axis_number)
{
	RealNumMatrix* av;
	av = get_axis(axis_number);
	Quaternion axis(0, av->get_element(1,1), av->get_element(2,1), av->get_element(3,1));
	*aq = axis;
	return aq;
}
