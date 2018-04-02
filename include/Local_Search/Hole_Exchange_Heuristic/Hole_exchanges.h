#pragma once
#ifndef HOLE_EXCHANGE_H
#define HOLE_EXCAHNGE_H

#include <unordered_map>
#include <list>
#include <vector>
#include "Greedy_Heuristic_Utils.h"
#include "Topological_Sorting_Utils\Topological_Sorting_Utils.h"

struct State_pos
{
	std::vector<size_t> m_vec_rob_vtx;
	State_pos(size_t uiNumRobots) { m_vec_rob_vtx.resize(uiNumRobots); };
};

class Hole_Exchange:public Topological_Sorting_Utils
{
	private:
		const size_t m_uiNumRobots;
		std::vector<State_pos> m_vec_state_path;
		void construct_state_transition_path(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch);
		void clear_prev_info();

	public:
		Hole_Exchange(size_t uiNumRobots);
		void perform_heuristic_moves(const std::vector<std::list<size_t>> &rob_seq, const Alternative_Graph &alt_graph, const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch);
};

#endif
