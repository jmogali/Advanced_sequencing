#pragma once
#ifndef HOLE_EXCHANGE_H
#define HOLE_EXCAHNGE_H

#include <unordered_map>
#include <list>
#include <vector>
#include "Greedy_Heuristic_Utils.h"
#include "LS_Greedy_Heuristic.h"
#include "Local_Search_Constants.h"
#include "Topological_Sorting_Utils_Dist.h"
#include "Enabling_Graph.h"

struct State_vtx_time
{
	std::vector<size_t> m_vec_rob_vtx;
	size_t m_uiTime;
	State_vtx_time(size_t uiNumRobots) { m_vec_rob_vtx.resize(uiNumRobots); };
};

class Hole_Exchange
{
	private:
		const size_t m_uiNumRobots;
		std::vector<std::list<size_t>> m_rob_seq;
		LS_Greedy_Heuristic m_ls_heur;
		std::vector<State_vtx_time> m_vec_state_path;
		std::vector<std::vector<Vertex_Schedule>> m_vec_full_rob_sch;
		const Layout_LS &m_graph;	
		const Enabling_Graph &m_en_graph;
		std::unordered_map<size_t, std::unordered_map<size_t, size_t>> m_alt_out_graph;   // <vtx, <out_vtx, arc cost>>
		std::unordered_map<size_t, std::unordered_map<size_t, size_t>> m_alt_in_graph;	//  <vtx, <in_vtx, arc cost>> 
		std::unordered_map<size_t, size_t> m_map_start_times; //<vtx, start time>

		void clear_prev_info();
		void populate_rob_graphs(const Alternative_Graph &alt_graph);

		//picking hole function
		void get_cand_vertex_critical_path(size_t uiChoice, std::list<size_t> &critical_path, std::list<std::tuple<size_t, size_t, size_t>> &list_best_cand); //<vtx, min time, max time>
		std::pair<size_t, size_t> compute_enabler_flexibility(const size_t uiVtx, const size_t c_uiMakeSpan);
		size_t compute_min_time(const size_t uiVtx);
		size_t compute_max_time(const size_t uiVtx, const size_t c_uiMakeSpan);

		//heuristic operations
		bool check_if_retraction_feasible(const size_t c_uiVtx, const size_t c_uiRobot, const std::vector<State_vtx_time> &vec_state_path, const std::vector<std::list<size_t>> &inp_seq, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch);
		bool check_if_insertion_feasible(const size_t c_uiVtx, const size_t c_uiRobot, const std::pair<size_t, size_t> pr_hole_pair,const std::vector<State_vtx_time> &vec_state_path, const std::vector<std::list<size_t>> &inp_seq, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch);
		void check_ifsub_seq_construction_correct(std::vector<std::list<size_t>> &rob_sub_seq, const size_t c_uiLeft, const size_t c_uiRight, const std::vector<State_vtx_time> &vec_state_path);
		
		//heuristic utilities
		void construct_state_transition_path(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch);
		void construct_rob_sub_sequences(std::vector<std::list<size_t>> &rob_sub_seq, const size_t c_uiLeft, const size_t c_uiRight, const std::vector<std::list<size_t>> &rob_seq, const std::vector<State_vtx_time> &vec_state_path
										, std::vector<std::tuple<std::list<size_t>::const_iterator, std::list<size_t>::const_iterator, size_t>> &vec_start_end_itr_start_pos, std::unordered_set<size_t> &set_comp_verts);
		void construct_rob_sub_sequences_with_iterators(std::vector<std::list<size_t>> &rob_sub_seq, const size_t c_uiLeft, const size_t c_uiRight, const std::vector<State_vtx_time> &vec_state_path
			, std::vector<std::tuple<std::list<size_t>::const_iterator, std::list<size_t>::const_iterator, size_t>> &vec_start_end_itr_start_pos, std::unordered_set<size_t> &set_comp_verts);
		void compute_enabled_holes_for_rob_sub_seq(const std::vector<std::list<size_t>> &rob_sub_seq, const std::unordered_set<size_t> &set_comp_verts, std::unordered_set<size_t> set_enabled_holes);
		
		void perform_swap_operation();

	public:
		Hole_Exchange(size_t uiNumRobots, const Layout_LS &graph, Power_Set &power, const Enabling_Graph &en_graph);
		void perform_heuristic_moves(const std::vector<std::list<size_t>> &rob_seq, const Alternative_Graph &alt_graph, const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch);
};

// this function computes start times using longest path for each vertex, start time is 0 if incoming edges are 0.
// this function can tolerate 0 length cycles but does not have detection mechanisms for positive length cycles.
void compute_start_times(const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &out_graph, const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &in_graph, std::unordered_map<size_t , size_t> &start_times);   // <vtx, <out_vtx, arc cost>>
void compute_critical_path(const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &in_graph, std::unordered_map<size_t, size_t> &start_times, std::list<size_t> &critical_path);   // <vtx, <out_vtx, arc cost>>
#endif
