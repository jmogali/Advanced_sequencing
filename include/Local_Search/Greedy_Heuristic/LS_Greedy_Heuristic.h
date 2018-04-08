#pragma once
#ifndef LS_GREEDY_HEURISTIC_H
#define LS_GREEDY_HEURISTIC_H

#include "Greedy_Heuristic.h"

class LS_Greedy_Heuristic : public Greedy_Heuristic
{
	private:
		std::unordered_map<size_t, size_t> m_map_rob_start_vtx_time; // <start vtx , time> //we are not storing the corresponding robot
		std::unordered_set<size_t> m_set_skip_enabling; // vertices that are already enabled
		
		size_t getTime(size_t uiVert);
		void clear_prev_info_buffers();
		bool perform_initializations(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::list<size_t>> &new_rob_seq, const size_t c_uiUpperBound, const std::vector<size_t>& vec_start_times, const std::unordered_set<size_t> &set_enabled_verts);
		void populate_enabled_verts(const std::unordered_set<size_t> &set_enabled_vertices);
		void populate_rob_start_times(const std::vector<std::list<size_t>> &rob_seq, const std::vector<size_t>& vec_start_times);
		void get_verts_not_self_enabled(size_t uiRobot, const std::vector<std::list<size_t>> &rob_seq, std::list<size_t>& list_dep_vert) override;

	public:
		int compute_greedy_sol(const std::vector<std::list<size_t>> &rob_seq, const std::vector<size_t>& vec_start_times, const std::unordered_set<size_t> &set_enabled_verts, std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, std::string strPlotFolder, const size_t c_uiUpperBound = std::numeric_limits<size_t>::max());
		LS_Greedy_Heuristic(const size_t uiRobotNum, const Layout_LS &graph, Power_Set &power);
};

#endif 

