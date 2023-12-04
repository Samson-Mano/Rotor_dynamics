#pragma once
#include <iostream>
#include <fstream>
#include <unordered_map>

// FE Objects
#include "../geometry_store/fe_objects/nodes_list_store.h"
#include "../geometry_store/fe_objects/elementline_list_store.h"
#include "../geometry_store/fe_objects/elementtri_list_store.h"
#include "../geometry_store/fe_objects/elementquad_list_store.h"

// FE Result Objects Heat analysis
#include "../geometry_store/analysis_result_objects/quadcontour_list_store.h";
#include "../geometry_store/analysis_result_objects/nodevector_list_store.h";

// Stop watch
#include "../events_handler/Stopwatch_events.h"

#include "../geometry_store/geom_parameters.h"

#pragma warning(push)
#pragma warning (disable : 26451)
#pragma warning (disable : 26495)
#pragma warning (disable : 6255)
#pragma warning (disable : 6294)
#pragma warning (disable : 6993)
#pragma warning (disable : 4067)
#pragma warning (disable : 26813)
#pragma warning (disable : 26454)

// Optimization for Eigen Library
// 1) OpenMP (Yes (/openmp)
//	 Solution Explorer->Configuration Properties -> C/C++ -> Language -> Open MP Support
// 2) For -march=native, choose "AVX2" or the latest supported instruction set.
//   Solution Explorer->Configuration Properties -> C/C++ -> Code Generation -> Enable Enhanced Instruction Set 

#include <Eigen/Dense>
#include <Eigen/Sparse>
// Define the sparse matrix type for the reduced global stiffness matrix
typedef Eigen::SparseMatrix<double> SparseMatrix;
#pragma warning(pop)


class analysis_solver
{
public:
	const double m_pi = 3.14159265358979323846;
	const double epsilon = 0.000001;
	const double wave_velocity = 0.1; // wave velocity
	bool print_matrix = false;
	Stopwatch_events stopwatch;

	// Maximum and minimum values
	std::vector<double> accl_vec_max_vals;
	std::vector<double> accl_vec_min_vals;

	// Store the angular velocity and angular acceleration
	std::vector<double> angular_velocity;
	std::vector<double> angular_acceleration;

	int time_step_count = 0;
	double time_interval = 0.0;
	double total_simulation_time = 0.0;

	analysis_solver();
	~analysis_solver();
	void accl_analysis_start(nodes_list_store& model_nodes,
							 const std::vector<glm::vec2>& model_vector_nodes,
							 const elementline_list_store& model_edgeelements,
							 const elementtri_list_store& model_trielements,
							 const elementquad_list_store& model_quadelements,
							 const std::vector<double>& rpm_values,
							 const double& total_time,
							 const double& time_interval,
							 quadcontour_list_store& model_contourresults,
							 nodevector_list_store& model_vectorresults,
							 bool& is_accl_analysis_complete);


private:

};