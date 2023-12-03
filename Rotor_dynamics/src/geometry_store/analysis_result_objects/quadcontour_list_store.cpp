#include "quadcontour_list_store.h"

quadcontour_list_store::quadcontour_list_store()
{
	// Empty Constructor
}

quadcontour_list_store::~quadcontour_list_store()
{
	// Empty Destructor
}

void quadcontour_list_store::init(geom_parameters* geom_param_ptr)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;

	// Set the geometry parameters for the two triangles (forming quadrilateral) contours
	tri_contours123.init(geom_param_ptr);
	tri_contours134.init(geom_param_ptr);

	// Clear the Quadrilateral contour data
	quadcontour_count = 0;
	quadcontourMap.clear();

}

void quadcontour_list_store::add_quadcontour(int& quad_id, node_store* nd1, node_store* nd2, node_store* nd3, node_store* nd4, 
	std::vector<double>& nd1_values, std::vector<double>& nd2_values, std::vector<double>& nd3_values, std::vector<double>& nd4_values, 
	std::vector<double>& contour_timestep_max_mag, std::vector<double>& contour_timestep_min_mag)
{
	// ratio with maximum and minimum = (nd_values - min)/ (max - min)
	std::vector<double> nd1_mag_ratio;
	std::vector<double> nd2_mag_ratio;
	std::vector<double> nd3_mag_ratio;
	std::vector<double> nd4_mag_ratio;

	// Nodal colors at each time step
	std::vector<glm::vec3> nd1_colors;
	std::vector<glm::vec3> nd2_colors;
	std::vector<glm::vec3> nd3_colors;
	std::vector<glm::vec3> nd4_colors;

	// Time step count
	int t_step_count = static_cast<int>(contour_timestep_max_mag.size());

	for (int i = 0; i < t_step_count; i++)
	{
		double temp_nd1_mag = (nd1_values[i] - contour_timestep_min_mag[i]) / (contour_timestep_max_mag[i] - contour_timestep_min_mag[i]);
		double temp_nd2_mag = (nd2_values[i] - contour_timestep_min_mag[i]) / (contour_timestep_max_mag[i] - contour_timestep_min_mag[i]);
		double temp_nd3_mag = (nd3_values[i] - contour_timestep_min_mag[i]) / (contour_timestep_max_mag[i] - contour_timestep_min_mag[i]);
		double temp_nd4_mag = (nd4_values[i] - contour_timestep_min_mag[i]) / (contour_timestep_max_mag[i] - contour_timestep_min_mag[i]);

		// Add to the mag_ratio list
		nd1_mag_ratio.push_back(temp_nd1_mag);
		nd2_mag_ratio.push_back(temp_nd2_mag);
		nd3_mag_ratio.push_back(temp_nd3_mag);
		nd4_mag_ratio.push_back(temp_nd4_mag);

		// Add to the color list
		nd1_colors.push_back(geom_parameters::getHeatMapColor(1.0f - temp_nd1_mag));
		nd2_colors.push_back(geom_parameters::getHeatMapColor(1.0f - temp_nd2_mag));
		nd3_colors.push_back(geom_parameters::getHeatMapColor(1.0f - temp_nd3_mag));
		nd4_colors.push_back(geom_parameters::getHeatMapColor(1.0f - temp_nd4_mag));
	}

	quadcontour_data temp_quadcontour;
	temp_quadcontour.quad_id = quad_id;
	temp_quadcontour.nd1 = nd1;
	temp_quadcontour.nd2 = nd2;
	temp_quadcontour.nd3 = nd3;
	temp_quadcontour.nd4 = nd4;

	// Add the node values data
	temp_quadcontour.nd1_values = nd1_values;
	temp_quadcontour.nd2_values = nd2_values;
	temp_quadcontour.nd3_values = nd3_values;
	temp_quadcontour.nd4_values = nd4_values;

	// Add to the quad_contour mag data
	temp_quadcontour.nd1_mag_ratio = nd1_mag_ratio;
	temp_quadcontour.nd2_mag_ratio = nd2_mag_ratio;
	temp_quadcontour.nd3_mag_ratio = nd3_mag_ratio;
	temp_quadcontour.nd4_mag_ratio = nd4_mag_ratio;

	// Add to the quad_contour color data
	temp_quadcontour.nd1_colors = nd1_colors;
	temp_quadcontour.nd2_colors = nd2_colors;
	temp_quadcontour.nd3_colors = nd3_colors;
	temp_quadcontour.nd4_colors = nd4_colors;

	//_____________________________________________________________________

	// Insert to the quadcontourMap
	quadcontourMap.insert({ quad_id, temp_quadcontour });
	quadcontour_count++;

}

void quadcontour_list_store::clear_data()
{
	// Clear all the data
	tri_contours123.clear_dyntricontours();
	tri_contours134.clear_dyntricontours();

	// Clear the quadrilateral contour data
	quadcontour_count = 0;
	quadcontourMap.clear();
}

void quadcontour_list_store::set_buffer()
{
	// Clear all the contours
	tri_contours123.clear_dyntricontours();
	tri_contours134.clear_dyntricontours();

	//_____________________ Add the Dynamic Tri Contours (from quadrilateral data)
	for (auto& dynquad_m : quadcontourMap)
	{
		quadcontour_data dynquad = dynquad_m.second;

		// Get the node pt
		glm::vec2 quad_pt1 = dynquad.nd1->node_pt; // Pt1
		glm::vec2 quad_pt2 = dynquad.nd2->node_pt; // Pt2
		glm::vec2 quad_pt3 = dynquad.nd3->node_pt; // Pt3
		glm::vec2 quad_pt4 = dynquad.nd4->node_pt; // Pt4


		// Add to the contour list (Triangle 123)
		int tri123_id = tri_contours123.dyn_tri_count;

		tri_contours123.add_dyntricontour(tri123_id, quad_pt1, quad_pt2, quad_pt3,
			dynquad.nd1_mag_ratio, dynquad.nd2_mag_ratio, dynquad.nd3_mag_ratio,
			dynquad.nd1_colors, dynquad.nd2_colors, dynquad.nd3_colors);

		// Add to the contour list (Triangle 134)
		int tri134_id = tri_contours134.dyn_tri_count;

		tri_contours134.add_dyntricontour(tri134_id, quad_pt1, quad_pt3, quad_pt4,
			dynquad.nd1_mag_ratio, dynquad.nd3_mag_ratio, dynquad.nd4_mag_ratio,
			dynquad.nd1_colors, dynquad.nd3_colors, dynquad.nd4_colors);
	}

	// Set the buffer (Only the index buffer is set because its a dynamic paint)
	tri_contours123.set_buffer();
	tri_contours134.set_buffer();

}

void quadcontour_list_store::paint_quadcontour(const int& dyn_index)
{
	// Paint the contour
	tri_contours123.paint_dyntricontour(dyn_index);
	tri_contours134.paint_dyntricontour(dyn_index);
}

void quadcontour_list_store::update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation, bool set_zoomtranslation, bool set_transparency, bool set_deflscale)
{
	// Update the geometry matrices 
	tri_contours123.update_opengl_uniforms(set_modelmatrix, set_pantranslation, set_zoomtranslation, set_transparency, set_deflscale);
	tri_contours134.update_opengl_uniforms(set_modelmatrix, set_pantranslation, set_zoomtranslation, set_transparency, set_deflscale);
}
