#pragma once
#include "geom_parameters.h"

// File system
#include <fstream>
#include <sstream>
#include <iomanip>

// Window includes
#include "../tool_window/analysis_window.h"
#include "../tool_window/options_window.h"

// Solver
#include "../fe_solver/analysis_solver.h"

// FE Objects
#include "fe_objects/nodes_list_store.h"
#include "fe_objects/elementline_list_store.h"
#include "fe_objects/elementtri_list_store.h"
#include "fe_objects/elementquad_list_store.h"

// Geometry Objects
#include "geometry_objects/dynamic_selrectangle_store.h"

// FE Result Objects Heat analysis
#include "analysis_result_objects/quadcontour_list_store.h";
#include "analysis_result_objects/nodevector_list_store.h"

class geom_store
{
public: 
	const double m_pi = 3.14159265358979323846;
	bool is_geometry_set = false;

	// Main Variable to strore the geometry parameters
	geom_parameters geom_param;

	geom_store();
	~geom_store();

	void init(analysis_window* sol_window, options_window* op_window);
	void fini();

	// Reading and writing the geometry file
	void read_rawdata(std::ifstream& input_file, std::ifstream& node_vector_file);

	// Functions to control the drawing area
	void update_WindowDimension(const int& window_width, const int& window_height);
	void update_model_matrix();
	void update_model_zoomfit();
	void update_model_pan(glm::vec2& transl);
	void update_model_zoom(double& z_scale);
	void update_model_transperency(bool is_transparent);

	// Function to paint the selection rectangle
	void update_selection_rectangle(const glm::vec2& o_pt, const glm::vec2& c_pt,
		const bool& is_paint, const bool& is_select, const bool& is_rightbutton);

	// Functions to paint the geometry and results
	void paint_geometry();
private:
	dynamic_selrectangle_store selection_rectangle;

	// Geometry objects
	nodes_list_store model_nodes;
	std::vector<glm::vec2> model_vector_nodes;
	elementline_list_store model_edgeelements;
	elementtri_list_store model_trielements;
	elementquad_list_store model_quadelements;

	// Acceleration analysis result 
	quadcontour_list_store model_contourresults;
	nodevector_list_store model_vectorresults;

	// Analysis
	bool is_accl_analysis_complete = false;

	// Window pointers
	analysis_window* sol_window = nullptr;
	options_window* op_window = nullptr;

	void paint_model(); // Paint the model
	void paint_model_results(); // Paint the results
};

