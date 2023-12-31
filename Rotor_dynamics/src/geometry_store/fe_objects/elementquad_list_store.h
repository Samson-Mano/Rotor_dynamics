#pragma once
#include "elementline_list_store.h"
#include "../geometry_objects/tri_list_store.h"


struct elementquad_store
{
	int quad_id = 0; // ID of the triangle element
	node_store* nd1 = nullptr; // node 1
	node_store* nd2 = nullptr; // node 2
	node_store* nd3 = nullptr; // node 3
	node_store* nd4 = nullptr; // node 4
	int material_id = 0;
};


class elementquad_list_store
{
public:
	unsigned int elementquad_count = 0;
	std::unordered_map<int, elementquad_store> elementquadMap; // Create an unordered_map to store Quadrilaterals with ID as key

	elementquad_list_store();
	~elementquad_list_store();
	void init(geom_parameters* geom_param_ptr);
	void add_elementquadrilateral(int& quad_id, node_store* nd1, node_store* nd2, node_store* nd3, node_store* nd4);
	void add_selection_quadrilaterals(const std::vector<int>& selected_element_ids);
	void update_material(const std::vector<int> selected_element_quad, const int& material_id);
	void execute_delete_material(const int& del_material_id);

	void set_buffer();
	void paint_elementquadrilaterals();
	void paint_selected_elementquadrilaterals();
	void paint_tri_material_id();
	void paint_elementquadrilaterals_shrunk();

	std::vector<int> is_quad_selected(const glm::vec2& corner_pt1, const glm::vec2& corner_pt2);

	void update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation, bool set_zoomtranslation, bool set_transparency, bool set_deflscale);
private:
	geom_parameters* geom_param_ptr = nullptr;
	label_list_store element_materialid;
	//   4______3     4____3      3
	//   |      |     |   /     / |
	//   |      |     | /     /   | 
	//   1______2     1      1____2      

	tri_list_store element_tris123; // Tri 123
	tri_list_store element_tris134; // Tri 134

	tri_list_store element_tris123_shrunk; // Tri shrunk 123
	tri_list_store element_tris134_shrunk; // Tri shrunk 134

	tri_list_store selected_element_tris123_shrunk;
	tri_list_store selected_element_tris134_shrunk;

	//Update material Id
	void update_material_id_labels();
};
