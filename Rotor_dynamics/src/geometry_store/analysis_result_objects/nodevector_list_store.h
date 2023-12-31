#pragma once
#include "../fe_objects/nodes_list_store.h"
#include "../geometry_objects/dynamic_vector_list_store.h"

struct vector_data
{
	int vector_id = 0; // Vector id
	glm::vec2 vector_loc = glm::vec2(0); // Vector location
	std::vector<glm::vec2> vector_values; // Vector magnitude and direction (list)
	std::vector<glm::vec2> vector_values_scaled; // Vector magnitude and direction (list) scaled to 1
	std::vector<double> vec_ratio;
	// std::vector<double> vector_timestep_max_mag; // Maximum magnitude at time step
	// std::vector<double> vector_timestep_min_mag; // Minimum magnitude at time step
	std::vector<glm::vec3> vector_colors; // Vector colors
};

class nodevector_list_store
{
public:
	int vector_count = 0;
	std::unordered_map<int, vector_data> vectorMap;

	nodevector_list_store();
	~nodevector_list_store();
	void init(geom_parameters* geom_param_ptr);
	void add_vector(int& vector_id, glm::vec2& vector_loc, std::vector<glm::vec2>& vector_values,
		std::vector<double>& vector_timestep_max_mag, std::vector<double>& vector_timestep_min_mag);
	void delete_vector(int& vector_id);
	void clear_data();
	void set_buffer();
	void paint_vectors(const int& dyn_index);
	void update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation, bool set_zoomtranslation, bool set_transparency, bool set_deflscale);

private:
	geom_parameters* geom_param_ptr = nullptr;
	dynamic_vector_list_store vector_lines;

};
