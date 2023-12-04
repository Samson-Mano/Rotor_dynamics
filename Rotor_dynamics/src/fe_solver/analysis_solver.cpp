#include "analysis_solver.h"

analysis_solver::analysis_solver()
{
	// Empty Constructor
}

analysis_solver::~analysis_solver()
{
	// Empty Destructor
}

void analysis_solver::accl_analysis_start(nodes_list_store& model_nodes,
	const std::vector<glm::vec2>& model_vector_nodes,
	const elementline_list_store& model_edgeelements,
	const elementtri_list_store& model_trielements,
	const elementquad_list_store& model_quadelements,
	const std::vector<double>& rpm_values,
	const double& total_time,
	const double& time_interval,
	quadcontour_list_store& model_contourresults,
	nodevector_list_store& model_vectorresults,
	bool& is_accl_analysis_complete)
{
	// Main Solver Call
	is_accl_analysis_complete = false;

	// Check the model
	// Node check
	if (model_nodes.node_count == 0)
	{
		// No nodes
		return;
	}

	// Element check
	if ((model_trielements.elementtri_count + model_quadelements.elementquad_count) == 0)
	{
		// No Elements
		return;
	}

	//____________________________________________
	stopwatch.start();
	std::stringstream stopwatch_elapsed_str;
	stopwatch_elapsed_str << std::fixed << std::setprecision(6);

	std::cout << "Acceleration analysis started" << std::endl;

	// Calculate angular velocity and angular acceleration
	angular_velocity.clear();
	angular_acceleration.clear();

	for (int i = 0; i < static_cast<int>(rpm_values.size() - 1); i++)
	{
		double delta_omega = rpm_values[i + 1] - rpm_values[i];
		double delta_t = time_interval;
		double alpha = delta_omega / delta_t;
		angular_acceleration.push_back(alpha);

		// Add to angular velocity
		angular_velocity.push_back(rpm_values[i]);
	}

	// Add one more to account for delta
	angular_acceleration.push_back(0.0);
	angular_velocity.push_back(0.0);

	// Pad the angular acceleration values to account for the delayed time effect 
	// Disk radius = 1
	int pad_size = static_cast<int>(1.0 / (wave_velocity*time_interval));

	for (int i = 0; i < pad_size; i++)
	{
		// Pad with zeroes
		angular_acceleration.push_back(0.0);
		angular_velocity.push_back(0.0);
	}


	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Angular acceleration calculation completed at = " << stopwatch_elapsed_str.str() << std::endl;

	//______________________________________________

	// Find the acceleration values to all the nodes
	glm::vec2 nd_origin = glm::vec2(0); // zero
	std::unordered_map<int, std::vector <double>> nd_scalar_results;
	int accl_data_count = static_cast<int>(angular_acceleration.size()); // Store the RPM Data size
	std::vector<double> accl_max_vals;
	std::vector<double> accl_min_vals;

	// set the max values
	for (int i = 0; i < accl_data_count; i++)
	{
		accl_max_vals.push_back(-DBL_MAX);
		accl_min_vals.push_back(DBL_MAX);
	}

	// Find the scalar contour values
	for (const auto& nd_m : model_nodes.nodeMap)
	{
		// Get the element quad
		int nd_id = nd_m.first; // Node id
		node_store nd = nd_m.second; 
		double eccentricity_r = geom_parameters::get_line_length(nd_origin, nd.node_pt);

		// Acceleration scalar values for particular node at all RPMs
		std::vector<double> accl_scalar_values;

		// Temporary variable to store the scalar acceleration result
		double temp_accl_val = 0.0;

		// Go through the RPM
		int i = 0;
		for (const auto& alpha : angular_acceleration)
		{
			// Delayed time
			double delayed_time = (i * time_interval) - (eccentricity_r / wave_velocity);
			
			// Find the index of acceleration at delayed time
			int delayed_index = static_cast<int>(std::round(delayed_time / time_interval));

			// Find the Scalar acceleration values at delayed index
			temp_accl_val = 0.0;

			if (delayed_index >= 0 && 
				delayed_index <= static_cast<int>(rpm_values.size() - 1))
			{
				float scalar_tangent_acceleration = eccentricity_r * angular_acceleration[delayed_index];
				float scalar_radial_acceleration = -1.0 * eccentricity_r * std::pow(angular_velocity[delayed_index], 2);

				// Check whether Delayed index is not out of range (it is not)
				temp_accl_val = scalar_tangent_acceleration + scalar_radial_acceleration;
			}
			
			// Add to the list
			accl_scalar_values.push_back(temp_accl_val);

			// Set the maximum at i
			if (accl_max_vals[i] <  temp_accl_val)
			{
				accl_max_vals[i] = temp_accl_val;
			}

			// Set the minimum at i
			if (accl_min_vals[i] > temp_accl_val)
			{
				accl_min_vals[i] = temp_accl_val;
			}

			i++;
		}

		// Add to the temp node result map
		nd_scalar_results[nd_id] = accl_scalar_values;
	}

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Scalar acceleration values calculation completed at = " << stopwatch_elapsed_str.str() << std::endl;

	//_________________________________________________________________________________________________________________
	// Find the vector linear acceleration data
	std::unordered_map<int, std::vector <glm::vec2>> nd_vector_results;
	std::vector<double> accl_vec_max_vals;
	std::vector<double> accl_vec_min_vals;

	// set the max values
	for (int i = 0; i < accl_data_count; i++)
	{
		accl_vec_max_vals.push_back(-DBL_MAX);
		accl_vec_min_vals.push_back(DBL_MAX);
	}

	int nd_vector_id = 0;
	for (const auto& nd_pt : model_vector_nodes)
	{
		double eccentricity_r = geom_parameters::get_line_length(nd_origin, nd_pt);

		// Acceleration vector values for particular node at all RPMs
		std::vector<glm::vec2> accl_vector_values;

		// Temporary variable to store the vector acceleration result
		glm::vec2 temp_accl_vector = glm::vec2(0);

		// Unit radial vector (towards center)
		glm::vec2 unit_radial_vector = glm::normalize(nd_origin - nd_pt);

		// Unit tangential vector(counter clock wise rotation)
		glm::vec2 unit_tangential_vector = glm::rotate(glm::mat4(1.0f),
			glm::radians(90.0f),
			glm::vec3(0.0f, 0.0f, 1.0f)) * glm::vec4(unit_radial_vector, 0.0f, 1.0f);


		// Go through the RPM
		int i = 0;
		for (const auto& alpha : angular_acceleration)
		{
			// Delayed time
			double delayed_time = (i * time_interval) - (eccentricity_r / wave_velocity);

			// Find the index of acceleration at delayed time
			int delayed_index = static_cast<int>(std::round(delayed_time / time_interval));

			// Find the Vector acceleration values at delayed index
			temp_accl_vector = glm::vec2(0);

			if (delayed_index >= 0 &&
				delayed_index <= static_cast<int>(rpm_values.size() - 1))
			{
				// Check whether Delayed index is not out of range (it is not)
				float scalar_tangent_acceleration = eccentricity_r * angular_acceleration[delayed_index];
				float scalar_radial_acceleration = -1.0*eccentricity_r * std::pow(angular_velocity[delayed_index], 2);

				temp_accl_vector = (scalar_tangent_acceleration * unit_tangential_vector)+
					 (scalar_radial_acceleration * unit_radial_vector);
			}

			// Add to the list
			accl_vector_values.push_back(temp_accl_vector);

			// Set the maximum at i
			if (accl_vec_max_vals[i] < glm::length(temp_accl_vector))
			{
				accl_vec_max_vals[i] = glm::length(temp_accl_vector);
			}

			// Set the minimum at i
			if (accl_vec_min_vals[i] > glm::length(temp_accl_vector))
			{
				accl_vec_min_vals[i] = glm::length(temp_accl_vector);
			}

			i++;
		}

		// Add to the temp node result map
		nd_vector_results[nd_vector_id] = accl_vector_values;

		// Increment node vector id
		nd_vector_id++;
	}



	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Vector values calculation completed at = " << stopwatch_elapsed_str.str() << std::endl;

	//_________________________________________________________________________________________________________________

	// Set the maximum
	this->accl_vec_max_vals = accl_vec_max_vals;
	this->accl_vec_min_vals = accl_vec_min_vals;


	// Add to the scalar list (Quad elements)
	model_contourresults.clear_data();

	for (const auto& quad_m : model_quadelements.elementquadMap)
	{
		// Extract element quad
		elementquad_store quad = quad_m.second;
		int quad_ndid1 = quad.nd1->node_id; // node id 1
		int quad_ndid2 = quad.nd2->node_id; // node id 2
		int quad_ndid3 = quad.nd3->node_id; // node id 3
		int quad_ndid4 = quad.nd4->node_id; // node id 4


		// Add to the contour
		model_contourresults.add_quadcontour(quad.quad_id, 
			&model_nodes.nodeMap[quad_ndid1], &model_nodes.nodeMap[quad_ndid2], 
			&model_nodes.nodeMap[quad_ndid3], &model_nodes.nodeMap[quad_ndid4],
			nd_scalar_results[quad_ndid1], nd_scalar_results[quad_ndid2],
			nd_scalar_results[quad_ndid3], nd_scalar_results[quad_ndid4],
			accl_max_vals, accl_min_vals);

	}


	// Add to the vector list
	model_vectorresults.clear_data();

	for (const auto& vector_nd_m : nd_vector_results)
	{
		int vector_id = vector_nd_m.first;
		glm::vec2 vector_loc = model_vector_nodes[vector_id];
		std::vector<glm::vec2> vector_datas = vector_nd_m.second;

		// Add to the vector
		model_vectorresults.add_vector(vector_id, vector_loc, vector_datas, 
			accl_vec_max_vals, accl_vec_min_vals);

	}

	// Set the analysis setting
	this->time_interval = time_interval;
	this->time_step_count = static_cast<int>(angular_acceleration.size());
	this->total_simulation_time = static_cast<int>(angular_acceleration.size()) * time_interval;

	is_accl_analysis_complete = true;

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Results mapping complete at = " << stopwatch_elapsed_str.str() << std::endl;

	//____________________________________________________________________________________________________________________
	stopwatch.stop();



}

