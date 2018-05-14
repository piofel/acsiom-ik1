acsiom: main.o UserInterface.o SpatialTransforms.o Quaternion.o DualQuaternion.o RigidBody.o Link.o FileOperator.o SystemParameters.o Base.o RealNumMatrix.o ConstantFrame.o EndEffector.o Mechanism.o Manipulator.o Axes.o MultiManipRobot.o ManipulatorParameters.o MultiManipRobotParameters.o Simulator.o Matrix.o DualQuatMatrix.o Errors.o FrameSymbol.o DualMatrix.o FreeFloatingRobot.o FreeFlyingRobot.o Messages.o Integrator.o Visualization.o
	g++ -o acsiom main.o UserInterface.o SpatialTransforms.o Quaternion.o DualQuaternion.o RigidBody.o Link.o FileOperator.o SystemParameters.o Base.o RealNumMatrix.o ConstantFrame.o EndEffector.o Mechanism.o Manipulator.o Axes.o MultiManipRobot.o ManipulatorParameters.o MultiManipRobotParameters.o Simulator.o Matrix.o DualQuatMatrix.o Errors.o FrameSymbol.o DualMatrix.o FreeFloatingRobot.o FreeFlyingRobot.o Messages.o Integrator.o Visualization.o -lglut -lGLEW -lGL

.PHONY: all clean

clean:
	rm -f *.o acsiom

Visualization.o: ../src/Visualization.cpp ../inc/Visualization.h
	g++ -I ../inc  -c ../src/Visualization.cpp 

Simulator.o: ../src/Simulator.cpp ../inc/Simulator.h
	g++ -I ../inc  -c ../src/Simulator.cpp

MultiManipRobot.o: ../src/MultiManipRobot.cpp ../inc/MultiManipRobot.h
	g++ -I ../inc  -c ../src/MultiManipRobot.cpp

main.o: ../src/main.cpp ../inc/UserInterface.h
	g++ -I ../inc  -c ../src/main.cpp

UserInterface.o: ../src/UserInterface.cpp ../inc/UserInterface.h
	g++ -I ../inc  -c ../src/UserInterface.cpp

SpatialTransforms.o: ../src/SpatialTransforms.cpp ../inc/SpatialTransforms.h
	g++ -I ../inc  -c ../src/SpatialTransforms.cpp

Quaternion.o: ../src/Quaternion.cpp ../inc/Quaternion.h
	g++ -I ../inc  -c ../src/Quaternion.cpp

DualQuaternion.o: ../src/DualQuaternion.cpp ../inc/DualQuaternion.h
	g++ -I ../inc  -c ../src/DualQuaternion.cpp

RigidBody.o: ../src/RigidBody.cpp ../inc/RigidBody.h
	g++ -I ../inc  -c ../src/RigidBody.cpp

Errors.o: ../src/Errors.cpp ../inc/Errors.h
	g++ -I ../inc  -c ../src/Errors.cpp

FrameSymbol.o: ../src/FrameSymbol.cpp ../inc/FrameSymbol.h
	g++ -I ../inc  -c ../src/FrameSymbol.cpp

DualMatrix.o: ../src/DualMatrix.cpp ../inc/DualMatrix.h
	g++ -I ../inc  -c ../src/DualMatrix.cpp

FreeFloatingRobot.o: ../src/FreeFloatingRobot.cpp ../inc/FreeFloatingRobot.h
	g++ -I ../inc  -c ../src/FreeFloatingRobot.cpp

FreeFlyingRobot.o: ../src/FreeFlyingRobot.cpp ../inc/FreeFlyingRobot.h
	g++ -I ../inc  -c ../src/FreeFlyingRobot.cpp

Messages.o: ../src/Messages.cpp ../inc/Messages.h
	g++ -I ../inc  -c ../src/Messages.cpp

Integrator.o: ../src/Integrator.cpp ../inc/Integrator.h 
	g++ -I ../inc  -c ../src/Integrator.cpp

Link.o: ../src/Link.cpp ../inc/Link.h
	g++ -I ../inc  -c ../src/Link.cpp

FileOperator.o: ../src/FileOperator.cpp ../inc/FileOperator.h
	g++ -I ../inc  -c ../src/FileOperator.cpp

SystemParameters.o: ../src/SystemParameters.cpp ../inc/SystemParameters.h
	g++ -I ../inc  -c ../src/SystemParameters.cpp

Base.o: ../src/Base.cpp ../inc/Base.h
	g++ -I ../inc  -c ../src/Base.cpp

Matrix.o: ../src/Matrix.cpp ../inc/Matrix.h
	g++ -I ../inc  -c ../src/Matrix.cpp

RealNumMatrix.o: ../src/RealNumMatrix.cpp ../inc/RealNumMatrix.h
	g++ -I ../inc  -c ../src/RealNumMatrix.cpp

DualQuatMatrix.o: ../src/DualQuatMatrix.cpp ../inc/DualQuatMatrix.h
	g++ -I ../inc  -c ../src/DualQuatMatrix.cpp

ConstantFrame.o: ../src/ConstantFrame.cpp ../inc/ConstantFrame.h
	g++ -I ../inc  -c ../src/ConstantFrame.cpp

EndEffector.o: ../src/EndEffector.cpp ../inc/EndEffector.h
	g++ -I ../inc  -c ../src/EndEffector.cpp

Mechanism.o: ../src/Mechanism.cpp ../inc/Mechanism.h
	g++ -I ../inc  -c ../src/Mechanism.cpp

Manipulator.o: ../src/Manipulator.cpp ../inc/Manipulator.h
	g++ -I ../inc  -c ../src/Manipulator.cpp

Axes.o: ../src/Axes.cpp ../inc/Axes.h
	g++ -I ../inc  -c ../src/Axes.cpp

ManipulatorParameters.o: ../src/ManipulatorParameters.cpp
	g++ -I ../inc  -c ../src/ManipulatorParameters.cpp

MultiManipRobotParameters.o: ../src/MultiManipRobotParameters.cpp
	g++ -I ../inc  -c ../src/MultiManipRobotParameters.cpp
