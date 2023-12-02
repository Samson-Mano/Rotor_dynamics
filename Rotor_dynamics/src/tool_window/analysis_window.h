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
	bool show_contour = true; // Show the acceleration contour
	bool show_vector = true; // Show the vector

	// RPM values 
	std::vector<double> rpm_values;
	int selected_curvepath_option = 0;


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
	void set_maxmin(const double& contour_maxvalue, const double& contour_minvalue);

private:
	Stopwatch stopwatch;

	double contour_maxvalue = 100.0;
	double contour_minvalue = 0.0;

	std::vector<double> generateTrapezoidalProfile(double rampUpPeriod, double uniformPeriod, double rampDownPeriod, double timeInterval);
	std::vector<double> generateSmoothTrapezoidalProfile(double rampUpPeriod,double uniformPeriod, double rampDownPeriod, double timeInterval);
	double sigmoid(double x);
};