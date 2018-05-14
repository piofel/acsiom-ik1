#ifndef __VISUALIZATION_H_INCLUDED__
#define __VISUALIZATION_H_INCLUDED__

#include <GL/glew.h> /* Use glew.h instead of gl.h to get all the GL prototypes declared */
#include <GL/freeglut.h> /* Using the GLUT library for the base windowing setup */
#include "Simulator.h"

class Visualization
{
	private:
		void setup(void);
		static void display_multi_manip_robot_subfunc(void);
		static void display_base(void);
	public:
		Visualization(int argc, char* argv[]);
		void display_multi_manip_robot(void);
};

#endif
