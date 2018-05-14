#include "FrameSymbol.h"

FrameSymbol::FrameSymbol(ShortInteger master_frame_number, ShortInteger master_manipulator_number)
{
	ShortInteger i;
	mast_frame = master_frame_number;
	mast_man = master_manipulator_number;
	for(i=0;i<3;i++)
	{
		symbol[i] = 'x';
	}
}

FrameSymbol::FrameSymbol(ShortInteger master_frame_number, ShortInteger master_manipulator_number, char symbol_part_0, char symbol_part_1, char symbol_part_2)
{
	mast_frame = master_frame_number;
	mast_man = master_manipulator_number;
	symbol[0] = symbol_part_0;
	symbol[1] = symbol_part_1;
	symbol[2] = symbol_part_2;
}

FrameSymbol::FrameSymbol(void)
{
	ShortInteger i;
	mast_frame = -9;
	mast_man = -9;
	for(i=0;i<3;i++)
	{
		symbol[i] = '?';
	}
}

void FrameSymbol::operator=(const FrameSymbol& right_side)
{
	ShortInteger i;
	mast_frame = right_side.mast_frame;
	mast_man = right_side.mast_man;
	for(i=0;i<3;i++)
	{
		symbol[i] = right_side.symbol[i];
	}
}

ShortInteger FrameSymbol::get_master_frame_number(void)
{
	return mast_frame;
}

ShortInteger FrameSymbol::get_master_manipulator_number(void)
{
	return mast_man;
}

char FrameSymbol::get_symbol_part(ShortInteger part_number)
{
	return symbol[part_number];
}
