#include "geom_store.h"

geom_store::geom_store()
{
	// Empty Constructor
}

geom_store::~geom_store()
{
	// Empty Destructor
}

void geom_store::init(analysis_window* sol_window, options_window* op_window)
{
	// Initialize
	// Initialize the geometry parameters
	geom_param.init();

	// Intialize the selection rectangle
	selection_rectangle.init(&geom_param);

	is_geometry_set = false;
	is_accl_analysis_complete = false;

	// Add the window pointers
	this->sol_window = sol_window; // Solver window
	this->op_window = op_window; // Option window
}

void geom_store::fini()
{
	// Deinitialize
	is_geometry_set = false;
}

void geom_store::read_rawdata(std::ifstream& input_file, std::ifstream& node_vector_file)
{
	// Create stopwatch
	Stopwatch_events stopwatch;
	stopwatch.start();
	std::stringstream stopwatch_elapsed_str;
	stopwatch_elapsed_str << std::fixed << std::setprecision(6);

	std::cout << "Reading of input started" << std::endl;

	// Reinitialize the model geometry
	is_geometry_set = false;
	is_accl_analysis_complete = false;

	// Initialize the model items
	this->model_nodes.init(&geom_param);
	this->model_vector_nodes.clear();
	this->model_edgeelements.init(&geom_param);
	this->model_trielements.init(&geom_param);
	this->model_quadelements.init(&geom_param);

	// Initialize the result store
	this->model_contourresults.init(&geom_param);
	this->model_vectorresults.init(&geom_param);

	//Node Point list (To find the geometric center)
	std::vector<glm::vec2> node_pts_list;

	if (input_file.is_open())
	{
		// Read node data
		while (!input_file.eof())
		{
			char type;
			input_file.read(&type, sizeof(char));
			switch (type)
			{
			case 'n':
			{
				int nd_id;
				glm::vec2 nd_pt;
				input_file.read(reinterpret_cast<char*>(&nd_id), sizeof(nd_id));
				input_file.read(reinterpret_cast<char*>(&nd_pt), sizeof(nd_pt));


				node_pts_list.push_back(nd_pt);

				// Add the nodes
				this->model_nodes.add_node(nd_id, nd_pt);
				break;
			}
			case 'q':
			{
				int quad_id, quad_ndid1, quad_ndid2, quad_ndid3, quad_ndid4;
				input_file.read(reinterpret_cast<char*>(&quad_id), sizeof(quad_id));
				input_file.read(reinterpret_cast<char*>(&quad_ndid1), sizeof(quad_ndid1));
				input_file.read(reinterpret_cast<char*>(&quad_ndid2), sizeof(quad_ndid2));
				input_file.read(reinterpret_cast<char*>(&quad_ndid3), sizeof(quad_ndid3));
				input_file.read(reinterpret_cast<char*>(&quad_ndid4), sizeof(quad_ndid4));


				// Add the Quadrilateral Elements
				this->model_quadelements.add_elementquadrilateral(quad_id, &model_nodes.nodeMap[quad_ndid1],
					&model_nodes.nodeMap[quad_ndid2],
					&model_nodes.nodeMap[quad_ndid3],
					&model_nodes.nodeMap[quad_ndid4]);
				break;
			}
			case 'e':
			{
				int edge_id, edge_startid, edge_endid;
				input_file.read(reinterpret_cast<char*>(&edge_id), sizeof(edge_id));
				input_file.read(reinterpret_cast<char*>(&edge_startid), sizeof(edge_startid));
				input_file.read(reinterpret_cast<char*>(&edge_endid), sizeof(edge_endid));

				// Add to the edges
				this->model_edgeelements.add_elementline(edge_id, &model_nodes.nodeMap[edge_startid],
					&model_nodes.nodeMap[edge_endid]);

				break;
			}
			}
		}

		// Close the file
		input_file.close();
	}
	//____________________________________________________________________________________

	if (node_vector_file.is_open())
	{
		// Read node data
		while (!node_vector_file.eof())
		{
			int nd_id;
			glm::vec2 nd_pt;

			node_vector_file.read(reinterpret_cast<char*>(&nd_id), sizeof(nd_id));
			node_vector_file.read(reinterpret_cast<char*>(&nd_pt), sizeof(nd_pt));

			// Add to the model vector nodes
			this->model_vector_nodes.push_back(nd_pt);
		}

		// Close the file
		node_vector_file.close();
	}
	//____________________________________________________________________________________


	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Input reading completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	// Geometry is loaded
	is_geometry_set = true;

	// Set the boundary of the geometry
	std::pair<glm::vec2, glm::vec2> result = geom_parameters::findMinMaxXY(node_pts_list);
	this->geom_param.min_b = result.first;
	this->geom_param.max_b = result.second;
	this->geom_param.geom_bound = geom_param.max_b - geom_param.min_b;

	// Set the center of the geometry
	this->geom_param.center = geom_parameters::findGeometricCenter(node_pts_list);

	// Set the geometry
	update_model_matrix();
	update_model_zoomfit();

	// Set the geometry buffers
	this->model_nodes.set_buffer();
	this->model_edgeelements.set_buffer();
	this->model_trielements.set_buffer();
	this->model_quadelements.set_buffer();

	// Set the result object buffers
	this->model_contourresults.set_buffer();
	this->model_vectorresults.set_buffer();

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Model read completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
}


void geom_store::update_WindowDimension(const int& window_width, const int& window_height)
{
	// Update the window dimension
	this->geom_param.window_width = window_width;
	this->geom_param.window_height = window_height;

	if (is_geometry_set == true)
	{
		// Update the model matrix
		update_model_matrix();
		// !! Zoom to fit operation during window resize is handled in mouse event class !!
	}
}


void geom_store::update_model_matrix()
{
	// Set the model matrix for the model shader
	// Find the scale of the model (with 0.9 being the maximum used)
	int max_dim = geom_param.window_width > geom_param.window_height ? geom_param.window_width : geom_param.window_height;

	double normalized_screen_width = 1.8f * (static_cast<double>(geom_param.window_width) / static_cast<double>(max_dim));
	double normalized_screen_height = 1.8f * (static_cast<double>(geom_param.window_height) / static_cast<double>(max_dim));


	geom_param.geom_scale = std::min(normalized_screen_width / geom_param.geom_bound.x,
		normalized_screen_height / geom_param.geom_bound.y);

	// Translation
	glm::vec3 geom_translation = glm::vec3(-1.0f * (geom_param.max_b.x + geom_param.min_b.x) * 0.5f * geom_param.geom_scale,
		-1.0f * (geom_param.max_b.y + geom_param.min_b.y) * 0.5f * geom_param.geom_scale,
		0.0f);

	glm::mat4 g_transl = glm::translate(glm::mat4(1.0f), geom_translation);

	geom_param.modelMatrix = g_transl * glm::scale(glm::mat4(1.0f), glm::vec3(static_cast<float>(geom_param.geom_scale)));

	// Update the model matrix
	model_nodes.update_geometry_matrices(true, false, false, false, false);
	model_edgeelements.update_geometry_matrices(true, false, false, false, false);
	model_trielements.update_geometry_matrices(true, false, false, false, false);
	model_quadelements.update_geometry_matrices(true, false, false, false, false);

	// Update the modal analysis result matrix
	model_contourresults.update_geometry_matrices(true, false, false, false, false);
	model_vectorresults.update_geometry_matrices(true, false, false, false, false);
}

void geom_store::update_model_zoomfit()
{
	if (is_geometry_set == false)
		return;

	// Set the pan translation matrix
	geom_param.panTranslation = glm::mat4(1.0f);

	// Set the zoom scale
	geom_param.zoom_scale = 1.0f;

	// Update the zoom scale and pan translation
	model_nodes.update_geometry_matrices(false, true, true, false, false);
	model_edgeelements.update_geometry_matrices(false, true, true, false, false);
	model_trielements.update_geometry_matrices(false, true, true, false, false);
	model_quadelements.update_geometry_matrices(false, true, true, false, false);

	// Update the modal analysis result matrix
	model_contourresults.update_geometry_matrices(false, true, true, false, false);
	model_vectorresults.update_geometry_matrices(false, true, true, false, false);

}

void geom_store::update_model_pan(glm::vec2& transl)
{
	if (is_geometry_set == false)
		return;

	// Pan the geometry
	geom_param.panTranslation = glm::mat4(1.0f);

	geom_param.panTranslation[0][3] = -1.0f * transl.x;
	geom_param.panTranslation[1][3] = transl.y;

	// Update the pan translation
	model_nodes.update_geometry_matrices(false, true, false, false, false);
	model_edgeelements.update_geometry_matrices(false, true, false, false, false);
	model_trielements.update_geometry_matrices(false, true, false, false, false);
	model_quadelements.update_geometry_matrices(false, true, false, false, false);

	// Update the modal analysis result matrix
	model_contourresults.update_geometry_matrices(false, true, false, false, false);
	model_vectorresults.update_geometry_matrices(false, true, false, false, false);
}

void geom_store::update_model_zoom(double& z_scale)
{
	if (is_geometry_set == false)
		return;

	// Zoom the geometry
	geom_param.zoom_scale = z_scale;

	// Update the Zoom
	model_nodes.update_geometry_matrices(false, false, true, false, false);
	model_edgeelements.update_geometry_matrices(false, false, true, false, false);
	model_trielements.update_geometry_matrices(false, false, true, false, false);
	model_quadelements.update_geometry_matrices(false, false, true, false, false);

	// Update the modal analysis result matrix
	model_contourresults.update_geometry_matrices(false, false, true, false, false);
	model_vectorresults.update_geometry_matrices(false, false, true, false, false);
}

void geom_store::update_model_transperency(bool is_transparent)
{
	if (is_geometry_set == false)
		return;

	if (is_transparent == true)
	{
		// Set the transparency value
		geom_param.geom_transparency = 0.2f;
	}
	else
	{
		// remove transparency
		geom_param.geom_transparency = 1.0f;
	}

	// Update the model transparency
	model_nodes.update_geometry_matrices(false, false, false, true, false);
	model_edgeelements.update_geometry_matrices(false, false, false, true, false);
	model_trielements.update_geometry_matrices(false, false, false, true, false);
	model_quadelements.update_geometry_matrices(false, false, false, true, false);
}

void geom_store::update_selection_rectangle(const glm::vec2& o_pt, const glm::vec2& c_pt,
	const bool& is_paint, const bool& is_select, const bool& is_rightbutton)
{
	// Draw the selection rectangle
	selection_rectangle.update_selection_rectangle(o_pt, c_pt, is_paint);

	// Selection commence (mouse button release)
	if (is_paint == false && is_select == true)
	{



	}
}


void geom_store::paint_geometry()
{
	if (is_geometry_set == false)
		return;

	// Clean the back buffer and assign the new color to it
	glClear(GL_COLOR_BUFFER_BIT);

	// Paint the model
	paint_model();

	// Paint the results
	paint_model_results();

}

void geom_store::paint_model()
{
	if (sol_window->is_show_window == true && is_accl_analysis_complete == true && sol_window->show_model == false)
	{
		// Analysis complete and user turned off model view
		return;
	}

	//______________________________________________
	// Paint the model
	if (op_window->is_show_modelelements == true)
	{
		if (op_window->is_show_shrunkmesh == false)
		{
			// Show the model mesh tri elements
			model_trielements.paint_elementtriangles();

			// Show the model mesh quad elements
			model_quadelements.paint_elementquadrilaterals();
		}
		else
		{
			// Show shrunk triangle mesh
			model_trielements.paint_elementtriangles_shrunk();

			// Show shrunk quadrilateral mesh
			model_quadelements.paint_elementquadrilaterals_shrunk();
		}
	}

	if (op_window->is_show_modeledges == true)
	{
		// Show the model edges
		model_edgeelements.paint_elementlines();
	}

	if (op_window->is_show_modelnodes == true)
	{
		// Show the model nodes
		model_nodes.paint_model_nodes();
	}
}

void geom_store::paint_model_results()
{
	// Paint the results
	// Check closing sequence for Heat analysis window
	if (sol_window->execute_close == true)
	{
		// Execute the close sequence
		if (is_accl_analysis_complete == true)
		{
			// sol_window->heat_analysis_complete = false;
			// sol_window->set_maxmin(model_contourresults.contour_max_vals, model_contourresults.contour_min_vals);
			// Heat analysis is complete
			update_model_transperency(false);
		}

		sol_window->execute_close = false;
	}

	// Check whether the Heat analysis solver window is open or not
	if (sol_window->is_show_window == false)
	{
		return;
	}

	// Paint the Heat analysis result
	if (is_accl_analysis_complete == true)
	{
		// Update the deflection scale
		geom_param.normalized_defl_scale = 1.0f;
		geom_param.defl_scale = sol_window->deformation_scale_max;

		// Update the deflection scale
		model_contourresults.update_geometry_matrices(false, false, false, false, true);
		model_vectorresults.update_geometry_matrices(false, false, false, false, true);
		// ______________________________________________________________________________________
		
		// Paint the Contour triangles
		if (sol_window->show_contour == true)
		{
			model_contourresults.paint_quadcontour(sol_window->time_step);
		}

		//// Paint the Contour lines
		//glLineWidth(3.2f);
		////		model_contourresults.paint_tricontour_lines();
		//glLineWidth(1.2f);

		// Paint the contour vector
		if (sol_window->show_vector == true)
		{
			model_vectorresults.paint_vectors(sol_window->time_step);
		}
	}

	if (sol_window->execute_open == true)
	{
		// Execute the open sequence
		if (is_accl_analysis_complete == true)
		{
			sol_window->accl_analysis_complete = true;
			//		sol_window->set_maxmin(model_contourresults.contour_max_vals, model_contourresults.contour_min_vals);
					// Heat analysis is complete
			update_model_transperency(true);
		}

		sol_window->execute_open = false;
	}

	if (sol_window->execute_accl_analysis == true)
	{
		// Execute Acceleartion Analysis
		analysis_solver accl_solver;
		accl_solver.accl_analysis_start(model_nodes,
			model_vector_nodes,
			model_edgeelements,
			model_trielements,
			model_quadelements,
			sol_window->rpm_values,
			sol_window->tota_time_period,
			sol_window->time_interval,
			model_contourresults,
			model_vectorresults,
			is_accl_analysis_complete);

		// Check whether the heat analysis is complete or not
		if (is_accl_analysis_complete == true)
		{
			sol_window->accl_analysis_complete = true;
			//sol_window->set_maxmin(model_contourresults.contour_max_vals, model_contourresults.contour_min_vals);
			
			// Set the analysis result time results
			sol_window->time_interval_atrun = accl_solver.time_interval;
			sol_window->time_step_count = accl_solver.time_step_count;

			// Reset the buffers for result contour
			model_contourresults.set_buffer();
			model_vectorresults.set_buffer();

			// Accl analysis is complete (Transperency change)
			update_model_transperency(true);
		}

		sol_window->execute_accl_analysis = false;
	}
}
