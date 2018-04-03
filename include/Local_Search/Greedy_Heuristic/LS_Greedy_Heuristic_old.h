#pragma once
#ifndef LS_GREEDY_HEURISTIC_OLD_H
#define LS_GREEDY_HEURISTIC_OLD_H

#include "Greedy_Heuristic_old.h"

class LS_Greedy_Heuristic_Old : public Greedy_Heuristic_old
{
	private:
		std::unordered_map<size_t, size_t> m_map_rob_start_vtx; // <start vtx , time> //we are not storing the corresponding robot
		std::unordered_set<size_t> m_set_skip_enabling; // vertices that are already enabled
		
		size_t getTime(size_t uiVert);
		void clear_prev_info_buffers();
		void perform_initializations(const std::vector<std::list<size_t>> &rob_seq, const std::vector<size_t>& vec_start_times, const std::unordered_set<size_t> &set_enabled_verts);
		void populate_enabled_verts(const std::unordered_set<size_t> &set_enabled_vertices);
		void populate_rob_start_times(const std::vector<std::list<size_t>> &rob_seq, const std::vector<size_t>& vec_start_times);
		int check_if_enabling_feasible(const State& state) override;

	public:
		int compute_greedy_sol(const std::vector<std::list<size_t>> &rob_seq, const std::vector<size_t>& vec_start_times, const std::unordered_set<size_t> &set_enabled_verts, std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch);
		LS_Greedy_Heuristic_Old(const size_t uiRobotNum, const Layout_LS &graph, Power_Set &power);
};

#endif 

