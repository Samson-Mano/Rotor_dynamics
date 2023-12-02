#include "analysis_solver.h"

analysis_solver::analysis_solver()
{
	// Empty Constructor
}

analysis_solver::~analysis_solver()
{
	// Empty Destructor
}

void analysis_solver::accl_analysis_start(const nodes_list_store& model_nodes,
	const std::vector<glm::vec2>& model_vector_nodes,
	const elementline_list_store& model_edgeelements,
	const elementtri_list_store& model_trielements,
	const elementquad_list_store& model_quadelements,
	const std::vector<double> rpm_values,
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

	// Find the acceleration values to all the nodes
	glm::vec2 nd_origin = glm::vec2(0); // zero
	std::unordered_map<int, std::vector <double>> nd_results;
	int rpm_data_count = static_cast<int>(rpm_values.size()); // Store the RPM Data size
	std::vector<double> accl_max_vals;
	std::vector<double> accl_min_vals;

	// set the max values
	for (int i = 0; i < rpm_data_count; i++)
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

		// Acceleration at all RPMs
		std::vector<double> accl_values;

		// Temporary variable to store the results
		double temp_accl_val = 0.0;

		// Go through the RPM
		int i = 0;
		for (const auto& rpm : rpm_values)
		{
			// Find the Scalar acceleration values
			temp_accl_val = 0.0;

			// Add to the list
			accl_values.push_back(temp_accl_val);

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
		nd_results[nd_id] = accl_values;
	}

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Scalar values calculation completed at = " << stopwatch_elapsed_str.str() << std::endl;

	//_________________________________________________________________________________________________________________




	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Vector values calculation completed at = " << stopwatch_elapsed_str.str() << std::endl;

	//_________________________________________________________________________________________________________________



	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Results mapping complete at = " << stopwatch_elapsed_str.str() << std::endl;

	//____________________________________________________________________________________________________________________
	stopwatch.stop();



}

