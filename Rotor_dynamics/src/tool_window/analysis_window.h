#pragma once
#include <iostream>
#include "../ImGui/imgui.h"
#include "../ImGui/implot.h"
#include "../ImGui/imgui_impl_glfw.h"
#include "../ImGui/imgui_impl_opengl3.h"
#include "../geometry_store/geom_parameters.h"

class analysis_window
{
public:
	bool is_show_window = false;
	bool execute_accl_analysis = false; // Main solver run event flag
	bool execute_open = false; // Solver window execute opening event flag
	bool execute_close = false; // Closing of solution window event flag

	// analysis results
	bool accl_analysis_complete = false;
	bool show_model = true; // show undeformed model 
	bool show_contour = false; // Show the acceleration contour
	bool show_vector = true; // Show the vector

	// Time period and time interval
	double ramp_up_period = 12.0; // Ramp up period sec
	double uniform_period = 20.0; // Uniform period
	double ramp_down_period = 16.0; // Ramp down period
	double time_interval = 0.1; // time interval
	double total_time_period = 48.0; // Total time period

	// RPM values 
	std::vector<double> rpm_values;
	int selected_curvepath_option = 0;

	// Maximum & minimum values
	std::vector<double> contour_maxvalue;
	std::vector<double> contour_minvalue;

	// Angular acceleration and time value list
	double chart_time_max_val = 0.0;
	double chart_angular_accl_min_val = 0.0;
	double chart_angular_accl_max_val = 0.0;
	std::vector<double> time_value_list;
	std::vector<double> angular_acceleration;

	// Animation control
	bool animate_play = true;
	bool animate_pause = false;
	double deformation_scale_max = 10.0;
	double animation_speed = 1.0;

	// Time step control
	double time_interval_atrun = 0.0; // Value of time interval used in the pulse response 
	int time_step_count = 0;
	int time_step = 0;


	analysis_window();
	~analysis_window();
	void init();
	void render_window();
	void set_maxmin(const std::vector<double>& contour_maxvalue, const std::vector<double>& contour_minvalue);
	void set_acceleration_values(const std::vector<double>& angular_acceleration);

private:
	Stopwatch stopwatch;

	std::vector<double> generateTrapezoidalProfile(double rampUpPeriod, double uniformPeriod, double rampDownPeriod, double timeInterval);
	std::vector<double> generateSmoothTrapezoidalProfile(double rampUpPeriod,double uniformPeriod, double rampDownPeriod, double timeInterval);
	double sigmoid(double x);
};