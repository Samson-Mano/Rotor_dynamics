#include "analysis_solver.h"

analysis_solver::analysis_solver()
{
	// Empty Constructor
}

analysis_solver::~analysis_solver()
{
	// Empty Destructor
}

void analysis_solver::heat_analysis_start(nodes_list_store& model_nodes,
	const std::vector<glm::vec2>& model_vector_nodes,
	elementline_list_store& model_edgeelements,
	const elementtri_list_store& model_trielements,
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
	if (model_trielements.elementtri_count == 0)
	{
		// No Elements
		return;
	}

	// Create a file to keep track of matrices
	std::ofstream output_file;
	output_file.open("heat_analysis_results.txt");

	//____________________________________________
	Eigen::initParallel();  // Initialize Eigen's thread pool

	stopwatch.start();
	std::stringstream stopwatch_elapsed_str;
	stopwatch_elapsed_str << std::fixed << std::setprecision(6);

	std::cout << "Steady state heat analysis started" << std::endl;

	// Create a node ID map (to create a nodes as ordered and numbered from 0,1,2...n)
	int i = 0;
	for (auto& elm_m : model_trielements.elementtriMap)
	{
		// get the element
		elementtri_store elm = elm_m.second;

		// Id 1
		if (nodeid_map.find(elm.nd1->node_id) == nodeid_map.end())
		{
			// Node ID does not exist add to the list
			nodeid_map[elm.nd1->node_id] = i;
			i++;
		}

		// Id 2
		if (nodeid_map.find(elm.nd2->node_id) == nodeid_map.end())
		{
			// Node ID does not exist add to the list
			nodeid_map[elm.nd2->node_id] = i;
			i++;
		}

		// Id 3
		if (nodeid_map.find(elm.nd3->node_id) == nodeid_map.end())
		{
			// Node ID does not exist add to the list
			nodeid_map[elm.nd3->node_id] = i;
			i++;
		}
	}

	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Node maping completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;


	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Results mapping complete at = " << stopwatch_elapsed_str.str() << std::endl;

	//____________________________________________________________________________________________________________________
	stopwatch.stop();

	output_file.close();
}

