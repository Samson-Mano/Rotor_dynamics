#include "analysis_window.h"

analysis_window::analysis_window()
{
	// Empty constructor
}

analysis_window::~analysis_window()
{
	// Empty destructor
}

void analysis_window::init()
{
	is_show_window = false; // Solver window open event flag
	execute_accl_analysis = false; // Main solver run event flag
	execute_open = false; // Solver window execute opening event flag
	execute_close = false; // Closing of solution window event flag

	accl_analysis_complete = false;
}

void analysis_window::render_window()
{
	if (is_show_window == false)
		return;

	ImGui::Begin("Circular Disk Acceleration Solver");
	//_________________________________________________________________________________________

	
	
	// Input text box
	ImGui::InputDouble("Ramp up period", &ramp_up_period, 0, 0, "%.3f");
	ImGui::InputDouble("Uniform period", &uniform_period, 0, 0, "%.3f");
	ImGui::InputDouble("Ramp down period", &ramp_down_period, 0, 0, "%.3f");
	ImGui::InputDouble("Time interval", &time_interval, 0, 0, "%.5f");


	// Options for the path (linear curve or smooth curve)
	if (ImGui::RadioButton("Linear curve", selected_curvepath_option == 0))
	{
		selected_curvepath_option = 0;
	}
	ImGui::SameLine();
	if (ImGui::RadioButton("Smooth curve", selected_curvepath_option == 1))
	{
		selected_curvepath_option = 1;
	}


	//_________________________________________________________________________________________
	// Add a Analysis button
	if (ImGui::Button("Analysis"))
	{
		// check the values
		if (ramp_up_period > 0.0 && uniform_period > 0.0 && ramp_down_period > 0.0 && time_interval > 0.0)
		{
			// Total time period
			this->tota_time_period = ramp_up_period + uniform_period + ramp_down_period;

			if (selected_curvepath_option == 0)
			{
				// Linear rampup and Linear ramp down
				this->rpm_values = generateTrapezoidalProfile(ramp_up_period,
					uniform_period,
					ramp_down_period,
					time_interval);

			}
			else
			{
				// Smooth rampup and Smooth ramp down
				this->rpm_values = generateSmoothTrapezoidalProfile(ramp_up_period,
					uniform_period,
					ramp_down_period,
					time_interval);
			}

			execute_accl_analysis = true;
		}
	}

	ImGui::Spacing();
	ImGui::Spacing();

	
	// Add check boxes to show the UnDeformed model
	ImGui::Checkbox("Show Model", &show_model);
	ImGui::Checkbox("Show Contour", &show_contour);
	ImGui::Checkbox("Show Vector", &show_vector);


	ImGui::Spacing();
	ImGui::Spacing();
	//_________________________________________________________________________________________

		// Close button
	if (ImGui::Button("Close"))
	{
		execute_close = true;
		execute_open = false;
		accl_analysis_complete = false;
		is_show_window = false; // set the flag to close the window
	}

	if (accl_analysis_complete == true)
	{
		// Contour Bar
		float minValue = contour_minvalue;
		float maxValue = contour_maxvalue;
		int numLevels = 5;

		// Show contour bars in the ImGui window
		ImGui::Text("Contour Bar");

		ImPlot::CreateContext();

		static int cmap = ImPlotColormap_Jet;

		ImPlot::ColormapScale("##Scale", minValue, maxValue, { 100,320.0f }, "%g \xC2\xB0 C", 0, cmap);
		ImPlot::DestroyContext();
	}

	//__________________________________________________________________________________________________


	// Input box to give input via text
	static bool defscale_input_mode = false;
	static char defscale_str[16] = ""; // buffer to store input deformation scale string
	static double defscale_input = 0; // buffer to store input deformation scale value

	// Button to switch to input mode
	if (!defscale_input_mode)
	{
		if (ImGui::Button("Deformation Scale"))
		{
			defscale_input_mode = true;
			snprintf(defscale_str, 16, "%.1f", deformation_scale_max); // set the buffer to current deformation scale value
		}
	}
	else // input mode
	{
		// Text box to input value
		ImGui::SetNextItemWidth(60.0f);
		if (ImGui::InputText("##Deformation Scale", defscale_str, IM_ARRAYSIZE(defscale_str), ImGuiInputTextFlags_CharsDecimal))
		{
			// convert the input string to int
			defscale_input = atoi(defscale_str);
			// set the load value to input value
			deformation_scale_max = defscale_input;
		}

		// Button to switch back to slider mode
		ImGui::SameLine();
		if (ImGui::Button("OK"))
		{
			defscale_input_mode = false;
		}
	}

	// Text for load value
	ImGui::SameLine();
	ImGui::Text(" %.1f", deformation_scale_max);

	// Slider for Deflection scale
	float deformation_scale_flt = static_cast<float>(deformation_scale_max);

	ImGui::Text("Deformation Scale");
	ImGui::SameLine();
	ImGui::SliderFloat(".", &deformation_scale_flt, 0.0f, 100.0f, "%.1f");
	deformation_scale_max = deformation_scale_flt;

	////Set the deformation scale
	//normailzed_defomation_scale = 1.0f;
	//deformation_scale = deformation_scale_max;

	ImGui::Spacing();

	//_________________________________________________________________________________________

	if (ImGui::CollapsingHeader("Animate",ImGuiTreeNodeFlags_DefaultOpen))
	{
		// Animate the solution
		// Start a horizontal layout
		ImGui::BeginGroup();

		// Play button active

		if (animate_pause == true)
		{
			// Add the Play button
			if (ImGui::Button("Play"))
			{
				// Handle Play button click
				animate_play = !animate_play;
				animate_pause = false;
			}
		}

		if (animate_play == true)
		{
			// Add the Pause button
			if (ImGui::Button("Pause"))
			{
				// Handle Pause button click
				animate_pause = !animate_pause;
				animate_play = false;
			}
		}

		// Add some spacing between buttons
		ImGui::SameLine();

		// Add the Stop button
		if (ImGui::Button("Stop"))
		{
			// Handle Stop button click
			animate_play = false;
			animate_pause = true;
		}

		// Animation speed control
		// Input box to give input via text
		static bool animation_speed_input_mode = false;
		static char animation_speed_str[16] = ""; // buffer to store input deformation scale string
		static float animation_speed_input = 0; // buffer to store input deformation scale value

		// Button to switch to input mode
		if (!animation_speed_input_mode)
		{
			if (ImGui::Button("Animation Speed"))
			{
				animation_speed_input_mode = true;
				snprintf(animation_speed_str, 16, "%.3f", animation_speed); // set the buffer to current deformation scale value
			}
		}
		else // input mode
		{
			// Text box to input value
			ImGui::SetNextItemWidth(60.0f);
			if (ImGui::InputText("##Animation Speed", animation_speed_str, IM_ARRAYSIZE(animation_speed_str), ImGuiInputTextFlags_CharsDecimal))
			{
				// convert the input string to int
				animation_speed_input = static_cast<float>(atof(animation_speed_str));
				// set the load value to input value
				animation_speed = animation_speed_input;
			}

			// Button to switch back to slider mode
			ImGui::SameLine();
			if (ImGui::Button("OK"))
			{
				animation_speed_input_mode = false;
			}
		}

		// Text for Animation speed value
		ImGui::SameLine();
		ImGui::Text(" %.3f", animation_speed);

		// Display the time step and time value
		ImGui::Text("Time = %.3f secs",
			time_interval_atrun * time_step);

		//ImGui::Text("Time value = %.3f secs",
		//	stopwatch.current_elapsed());

		// Display the frame rate
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
			1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

		// End the horizontal layout
		ImGui::EndGroup();
	}

	ImGui::Spacing();



	ImGui::End();


	// Cycle through the pulse response time step
	if (accl_analysis_complete == true)
	{

		if (animate_play == true)
		{
			// Stop watch
			if ((stopwatch.current_elapsed() * animation_speed) > time_interval_atrun)
			{
				stopwatch.reset_time(); // reset the time
				time_step++; // increment the time step

				// Adjust the time step such that it didnot exceed the time_step_total
				if (time_step >= time_step_count)
				{
					time_step = 0;
				}
			}
		}
		else if (animate_pause == true)
		{
			// Pause the animation
		}
		else
		{
			// Stop the animation (show the end of animation)
			time_step = time_step_count - 1;
		}
	}

}

void analysis_window::set_maxmin(const double& contour_maxvalue, const double& contour_minvalue)
{
	// Set the contour maximum and minimum
	this->contour_maxvalue = contour_maxvalue;
	this->contour_minvalue = contour_minvalue;
}


std::vector<double> analysis_window::generateTrapezoidalProfile(double rampUpPeriod, 
	double uniformPeriod, double rampDownPeriod, double timeInterval)
{
	// Calculate the total time
	double totalTime = rampUpPeriod + uniformPeriod + rampDownPeriod;

	// Calculate the number of points in each segment
	int numRampUpPoints = static_cast<int>(rampUpPeriod / timeInterval);
	int numUniformPoints = static_cast<int>(uniformPeriod / timeInterval);
	int numRampDownPoints = static_cast<int>(rampDownPeriod / timeInterval);

	// Generate trapezoidal profile
	std::vector<double> trapezoidalProfile;

	// Ramp-up
	for (int i = 0; i < numRampUpPoints; ++i)
	{
		double value = i * (1.0 / numRampUpPoints);
		trapezoidalProfile.push_back(value);
	}

	// Uniform
	for (int i = 0; i < numUniformPoints; ++i)
	{
		trapezoidalProfile.push_back(1.0);
	}

	// Ramp-down
	for (int i = 0; i < numRampDownPoints; ++i)
	{
		double value = 1.0 - i * (1.0 / numRampDownPoints);
		trapezoidalProfile.push_back(value);
	}

	return trapezoidalProfile;
}

double analysis_window::sigmoid(double x)
{
	return 1.0 / (1.0 + exp(-x));
}

std::vector<double> analysis_window::generateSmoothTrapezoidalProfile(double rampUpPeriod,
	double uniformPeriod, double rampDownPeriod, double timeInterval)
{
	double totalTime = rampUpPeriod + uniformPeriod + rampDownPeriod;

	int numRampUpPoints = static_cast<int>(rampUpPeriod / timeInterval);
	int numUniformPoints = static_cast<int>(uniformPeriod / timeInterval);
	int numRampDownPoints = static_cast<int>(rampDownPeriod / timeInterval);

	std::vector<double> smoothTrapezoidalProfile;

	// Smooth Ramp-up
	for (int i = 0; i < numRampUpPoints; ++i)
	{
		double t = i * (1.0 / numRampUpPoints);
		double value = sigmoid(6.0 * (t - 0.5));  // Adjust the parameter 6.0 for desired slope at the transition point
		smoothTrapezoidalProfile.push_back(value);
	}

	// Uniform
	for (int i = 0; i < numUniformPoints; ++i)
	{
		smoothTrapezoidalProfile.push_back(1.0);
	}

	// Smooth Ramp-down
	for (int i = 0; i < numRampDownPoints; ++i)
	{
		double t = i * (1.0 / numRampDownPoints);
		double value = 1.0 - sigmoid(6.0 * (t - 0.5));  // Adjust the parameter 6.0 for desired slope at the transition point
		smoothTrapezoidalProfile.push_back(value);
	}

	return smoothTrapezoidalProfile;
}
