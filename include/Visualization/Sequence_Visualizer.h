#ifndef SEQUENCE_VISUALIZATION_H
#define SEQUENCE_VISUALIZATION_H

#include "Windows_Linux.h"
#include "Alternative_Graph.h"
#include "Greedy_Heuristic_Utils.h"
#include <fstream>

//#define PLOT_INFEASIBLE_CASES

class Sequence_Visualization
{
	private:
		void plot_enabling_cons_alt_graph(std::string strFilePath, const Alternative_Graph &alt_graph);
		void plot_coll_cons_alt_graph(std::string strFilePath, const Alternative_Graph &alt_graph);
		void plot_visted_states(std::string strFilePath, const Alternative_Graph &alt_graph, const std::unordered_map<State, int, StateHasher>& map_visited_states);

	public:
		void plot_alternative_graph(std::string strFolderPath, const Alternative_Graph &alt_graph, const std::unordered_map<State, int, StateHasher>& map_visited_states);
		Sequence_Visualization();
};

#endif