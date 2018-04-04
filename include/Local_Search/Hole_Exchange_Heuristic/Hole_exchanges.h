#pragma once
#ifndef HOLE_EXCHANGE_H
#define HOLE_EXCAHNGE_H

#include <unordered_map>
#include <list>
#include <vector>
#include "Greedy_Heuristic_Utils.h"
#include "Topological_Sorting_Utils.h"
#include "LS_Greedy_Heuristic_old.h"
#include "Local_Search_Constants.h"

struct State_vtx
{
	std::vector<size_t> m_vec_rob_vtx;
	State_vtx(size_t uiNumRobots) { m_vec_rob_vtx.resize(uiNumRobots); };
};

class Hole_Exchange:public Topological_Sorting_Utils
{
	private:
		const size_t m_uiNumRobots;
		LS_Greedy_Heuristic_Old ls_heur;
		std::vector<State_vtx> m_vec_state_path;
		const Layout_LS &m_graph;
		
		void clear_prev_info();
		
		//heuristic operations
		bool check_if_retraction_feasible(const size_t c_uiVtx, const size_t c_uiRobot, const std::vector<State_vtx> &vec_state_path, const std::vector<std::list<size_t>> &inp_seq, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch);
		bool check_if_insertion_feasible(const size_t c_uiVtx, const size_t c_uiRobot, const std::pair<size_t, size_t> pr_hole_pair,const std::vector<State_vtx> &vec_state_path, const std::vector<std::list<size_t>> &inp_seq, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch);
		void check_ifsub_seq_construction_correct(std::vector<std::list<size_t>> &rob_sub_seq, const size_t c_uiLeft, const size_t c_uiRight, const std::vector<State_vtx> &vec_state_path);


		//heuristic utilities
		void construct_state_transition_path(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch);
		void construct_rob_sub_sequences(std::vector<std::list<size_t>> &rob_sub_seq, const size_t c_uiLeft, const size_t c_uiRight, const std::vector<std::list<size_t>> &rob_seq, const std::vector<State_vtx> &vec_state_path
										, std::vector<std::tuple<std::list<size_t>::const_iterator, std::list<size_t>::const_iterator, size_t>> &vec_start_end_itr_start_pos, std::unordered_set<size_t> &set_comp_verts);
		void construct_rob_sub_sequences_with_iterators(std::vector<std::list<size_t>> &rob_sub_seq, const size_t c_uiLeft, const size_t c_uiRight, const std::vector<State_vtx> &vec_state_path
			, std::vector<std::tuple<std::list<size_t>::const_iterator, std::list<size_t>::const_iterator, size_t>> &vec_start_end_itr_start_pos, std::unordered_set<size_t> &set_comp_verts);
		void compute_enabled_holes_for_rob_sub_seq(const std::vector<std::list<size_t>> &rob_sub_seq, const std::unordered_set<size_t> &set_comp_verts, std::unordered_set<size_t> set_enabled_holes);

	public:
		Hole_Exchange(size_t uiNumRobots, const Layout_LS &graph, Power_Set &power);
		void perform_heuristic_moves(const std::vector<std::list<size_t>> &rob_seq, const Alternative_Graph &alt_graph, const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch);
};

#endif
