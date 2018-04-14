#pragma once
#ifndef DAG_DISTANCE_UTIL_H
#define DAG_DISTANCE_UTIL_H

#include "Topological_Sorting_Utils.h"

class Topological_Sorting_Utils_Dist :public Topological_Sorting_Utils
{
	protected:
		int m_iLastVtx; // last vtx on the critical path of m_out_graph
		std::unordered_map<int, size_t> m_map_from_costs;
		std::unordered_map<int, size_t> m_map_to_costs;
		std::unordered_map<int, size_t> m_map_vtx_slack;
		std::list<int> m_list_critical_path;
		size_t m_uiMakeSpan;

		void clear_prev_dist_info();
		inline void clear_prev_slack_info() { m_map_vtx_slack.clear(); };
		size_t compute_shortest_from_costs(std::unordered_map<size_t, size_t> &start_times);
		size_t Compute_FROM_costs_each_Vertex(std::unordered_map<size_t, size_t> &start_times);
		size_t Compute_GO_costs_each_Vertex();	
		void compute_critical_path();
		void compute_vertex_slack();
		
	public:
		size_t construct_graph_populate_order_with_dist(const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &out_graph, const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &in_graph, std::unordered_map<size_t, size_t> &start_times);
		void compute_vertex_slack(std::unordered_map<size_t, size_t> &map_vtx_slacks);
		inline size_t get_makespan() { return m_uiMakeSpan; };
};

#endif
