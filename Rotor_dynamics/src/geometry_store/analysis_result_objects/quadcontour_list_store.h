#pragma once
#include "../fe_objects/nodes_list_store.h"
#include "../geometry_objects/dynamic_tricontour_list_store.h"

struct  quadcontour_data
{
	int quad_id = 0; // ID of the Quadrilateral element
	node_store* nd1 = nullptr; // node 1
	node_store* nd2 = nullptr; // node 2
	node_store* nd3 = nullptr; // node 3
	node_store* nd4 = nullptr; // node 4

	// Node values at each time step 
	std::vector<double> nd1_values;
	std::vector<double> nd2_values;
	std::vector<double> nd3_values;
	std::vector<double> nd4_values;

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
};


class quadcontour_list_store
{
public:
	int quadcontour_count = 0;
	std::unordered_map<int, quadcontour_data> quadcontourMap;

	quadcontour_list_store();
	~quadcontour_list_store();
	void init(geom_parameters* geom_param_ptr);
	void add_quadcontour(int& quad_id, node_store* nd1, node_store* nd2, node_store* nd3, node_store* nd4,
		std::vector<double>& nd1_values, std::vector<double>& nd2_values, std::vector<double>& nd3_values, std::vector<double>& nd4_values,
		std::vector<double>& contour_timestep_max_mag, std::vector<double>& contour_timestep_min_mag);

	void clear_data();
	void set_buffer();
	void paint_quadcontour(const int& dyn_index);
	void update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation, bool set_zoomtranslation, bool set_transparency, bool set_deflscale);

private:
	geom_parameters* geom_param_ptr = nullptr;
	dynamic_tricontour_list_store tri_contours123;
	dynamic_tricontour_list_store tri_contours134;

};
