#pragma once
#include "../geometry_buffers/gBuffers.h"
#include "../geom_parameters.h"

struct dynamic_tricontour_store
{
	// store the individual point
	int tri_id = 0;
	glm::vec2 tri_coord1 = glm::vec2(0); // Dynamic triangle pt1
	glm::vec2 tri_coord2 = glm::vec2(0); // Dynamic triangle pt2
	glm::vec2 tri_coord3 = glm::vec2(0); // Dynamic triangle pt3

	// Displacement
	std::vector<double> tri_displ1; // Dynamic Displacement list of pt1
	std::vector<double> tri_displ2;// Dynamic Displacement list of pt2
	std::vector<double> tri_displ3;// Dynamic Displacement list of pt3

	// Color
	std::vector<glm::vec3> tri_pt1_color; // Dynamic Color list of pt1
	std::vector<glm::vec3> tri_pt2_color; // Dynamic Color list of pt2
	std::vector<glm::vec3> tri_pt3_color; // Dynamic Color list of pt3
};



class dynamic_tricontour_list_store
{
public:
	const int contour_level_count = 10; // Number of contour levels
	geom_parameters* geom_param_ptr = nullptr;
	unsigned int dyn_tri_count = 0;
	std::vector<dynamic_tricontour_store> dyn_triMap;

	dynamic_tricontour_list_store();
	~dynamic_tricontour_list_store();
	void init(geom_parameters* geom_param_ptr);
	void add_dyntricontour(int& tri_id, glm::vec2 tri_coord1, glm::vec2 tri_coord2, glm::vec2 tri_coord3,
		std::vector<double>& tri_displ1, std::vector<double>& tri_displ2, std::vector<double>& tri_displ3,
		std::vector<glm::vec3>& tri_pt1_color, std::vector<glm::vec3>& tri_pt2_color, std::vector<glm::vec3>& tri_pt3_color);
	void set_buffer();
	void paint_dyntricontour(const int& dyn_index);
	void update_buffer(const int& dyn_index);
	void clear_dyntricontours();
	void update_opengl_uniforms(bool set_modelmatrix, bool set_pantranslation, bool set_zoomtranslation, bool set_transparency, bool set_deflscale);
private:
	gBuffers dyn_contour_buffer;
	Shader dyn_contour_shader;

	void get_contour_vertex_buffer(dynamic_tricontour_store& dyntri, const int& dyn_index,
		float* dyn_contour_vertices, unsigned int& dyn_contour_v_index);

	void get_contour_index_buffer(unsigned int* dyn_contour_vertex_indices, unsigned int& dyn_contour_i_index);
};
