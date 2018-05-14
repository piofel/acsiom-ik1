#include "Visualization.h"

Visualization::Visualization(int argc, char* argv[])
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
       	glutInitWindowSize(800,600);
}

void Visualization::setup(void)
{
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
}

void Visualization::display_base(void)
{
	UnsignShortInteger nm;
	//nm = robot->get_number_of_manipulators();
	glColor3f(0.5f, 0.0f, 0.3f);
	float a = .8;
	glBegin(GL_QUADS);
		glVertex3f(-0.25f, 0.25f, .5f); // vertex 1
		glVertex2f(-0.5f, -0.25f); // vertex 2
		glVertex2f(a, -0.25f); // vertex 3
		glVertex2f(0.25f, 0.25f); // vertex 4
	glEnd();
}

void Visualization::display_multi_manip_robot_subfunc(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	Simulator s;
	display_base();
	glutSwapBuffers();
}

void Visualization::display_multi_manip_robot(void)
{
       	glutCreateWindow(PROGRAM_NAME);
	setup();
       	glutDisplayFunc(display_multi_manip_robot_subfunc);
       	glutMainLoop();
}
