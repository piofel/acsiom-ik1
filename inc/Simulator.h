#ifndef __SIMULATOR_H_INCLUDED__
#define __SIMULATOR_H_INCLUDED__

#define DEFAULT_TIME_DIFFERENTIAL 0.1
#define DEFAULT_SIMULATION_START_TIME 0.0
#define DEFAULT_SIMULATION_STOP_TIME 0.2

#include "FreeFloatingRobot.h"
#include "FreeFlyingRobot.h"
#include "FileOperator.h"
#include "Integrator.h"

class Simulator
{
	private:
	       	RealNumber simulation_start_time; 
		RealNumber simulation_stop_time;
		RealNumber time_differential;
		FreeFloatingRobot* float_robot;
		FreeFlyingRobot* fly_robot;
		DualQuaternion get_pose_of_frame_to_its_master_frame(FrameSymbol* frame, char robot_type);
		RealNumMatrix convert_to_timeseries_row(RealNumMatrix* vector, RealNumber time);
	public:
		Simulator(void);
		~Simulator(void);
		void reload_parameters(void);
		FreeFlyingRobot* get_free_flying_robot(void);
		FreeFloatingRobot* get_free_floating_robot(void);
		DualQuaternion free_flyer_pose_simulation(FrameSymbol* frame_of_expression, FrameSymbol* expressed_frame);
		DualQuaternion free_flyer_velocity_simulation(FrameSymbol* frame_of_interest, FrameSymbol* frame_of_expression);
		DualQuatMatrix free_floater_velocity_simulation(void);
		RealNumMatrix free_floater_inverse_velocity_kinematics_simulation(void);
		void free_floater_trajectory_simulation(char output_type);
};

#endif
