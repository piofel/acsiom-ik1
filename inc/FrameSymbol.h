#ifndef __FRAMESYMBOL_H_INCLUDED__ 
#define __FRAMESYMBOL_H_INCLUDED__ 

#include "TypeDefinitions.h"

class FrameSymbol
{
	protected:
		char symbol[3];
		ShortInteger mast_frame;
		ShortInteger mast_man;
	public:
		FrameSymbol(ShortInteger master_frame_number, ShortInteger master_manipulator_number);
		FrameSymbol(ShortInteger master_frame_number, ShortInteger master_manipulator_number, char symbol_part_0, char symbol_part_1, char symbol_part_2);
		FrameSymbol(void);
		void operator=(const FrameSymbol& right_side);
		ShortInteger get_master_frame_number(void);
		ShortInteger get_master_manipulator_number(void);
		char get_symbol_part(ShortInteger part_number);
};

#endif
