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
		void Remove_1comp();
		void construct_out_graph(const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &out_graph, const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &in_graph);
		void construct_in_graph();
		void Topological_Dfs(int iVtx, std::unordered_map<int, std::string> &map_seen);

	protected:
		std::list<std::unordered_set<size_t>> m_list_Super_Comp; // contains components with more than 1 vertex
		std::unordered_map<int, std::unordered_map<int, size_t>> m_out_graph;   // <vtx, <out_vtx, arc cost>>
		std::unordered_map<int, std::unordered_map<int, size_t>> m_in_graph;	//  <vtx, <in_vtx, arc cost>> 
		std::list<int> m_list_order;

		void clear_prev_info();
		void construct_in_out_graphs(const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &out_graph, const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &in_graph);	
		void Topological_sort_out_graph();
		size_t get_any_vertex_from_scc(int iVtx);
		int get_vtx_containing_rob_vtx(size_t uiVtx);
		void construct_graph_populate_order(const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &out_graph, const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &in_graph);

	public:
		inline const std::list<int>& get_top_list() const { return m_list_order; };
		inline const std::unordered_map<int, std::unordered_map<int, size_t>>& get_out_graph() const { return m_out_graph; };
		inline const std::unordered_map<int, std::unordered_map<int, size_t>>& get_in_graph() const { return m_in_graph; };
		inline const std::list<std::unordered_set<size_t>>& get_super_comp() const { return m_list_Super_Comp; };		
};

#endif