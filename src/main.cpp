#include "UserInterface.h"

int main(int argc, char* argv[])
{
	UserInterface ui(argc,argv);
	ui.display_main_menu();
	return 0;
}
