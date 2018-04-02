#pragma once
#ifndef TOPOLOGICAL_SORTING_UTILS_H
#define TOPOLOGICAL_SORTING_UTILS_H

#include <list>
#include <unordered_map>
#include <unordered_set>
#include <Alternative_Graph.h>
#include "Kosaraju_Algo.h"

class Topological_Sorting_Utils
{
	private:
		void clear_prev_info();
		void construct_in_out_graphs(const Alternative_Graph &alt_graph);
		bool Check_Pos_Loop_Remove_1comp(const Alternative_Graph &alt_graph, const size_t c_uiNumRobots);
		void construct_out_graph(const Alternative_Graph &alt_graph);
		void construct_in_graph();

		void Topological_sort_out_graph();
		void Topological_Dfs(int iVtx, std::unordered_map<int, std::string> &map_seen);

	protected:
		std::list<std::unordered_set<size_t>> m_list_Super_Comp; // contains components with more than 1 vertex
		std::unordered_map<int, std::unordered_map<int, size_t>> m_out_graph;   // <vtx, <out_vtx, arc cost>>
		std::unordered_map<int, std::unordered_map<int, size_t>> m_in_graph;	//  <vtx, <in_vtx, arc cost>> 
		std::list<int> m_list_order;

		bool construct_graph_populate_order(const Alternative_Graph &alt_graph, const size_t c_uiNumRobots);
};

#endif