#pragma once
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
		void plot_robot_hole_sequence(std::string strFolderPath, size_t uiRobot, std::vector<std::pair<double, double>> vec_coord, size_t sampling);
		Sequence_Visualization();
};

#endif