#pragma once
#ifndef HOLE_EXCHANGE_H
#define HOLE_EXCAHNGE_H

#include <unordered_map>
#include <list>
#include <vector>
#include "Greedy_Heuristic_Utils.h"
#include "LS_Greedy_Heuristic_Old.h"
#include "Local_Search_Constants.h"
#include "Topological_Sorting_Utils_Dist.h"
#include "Enabling_Graph.h"

struct State_vtx_time
{
	std::vector<size_t> m_vec_rob_vtx;
	size_t m_uiTime;
	State_vtx_time(size_t uiNumRobots) { m_vec_rob_vtx.resize(uiNumRobots); };
};

struct Cand_for_insertion
{
		R_Ind m_uiRobot;
		N_Ind m_uiHD1, m_uiHD2;
		int m_iVal;	
	
		Cand_for_insertion(size_t uiHD1, size_t uiHD2, size_t uiRobot, int iVal);

		bool operator< (const Cand_for_insertion &val) const
		{
			// the less m_iVal the better
			if (m_iVal < val.m_iVal) return true;
			else if (m_iVal > val.m_iVal) return false;

			//to resolve ties
			if (m_uiHD1.getInd() < val.m_uiHD1.getInd()) return true;
			else if (m_uiHD1.getInd() > val.m_uiHD1.getInd()) return false;

			if (m_uiHD2.getInd() < val.m_uiHD2.getInd()) return true;
			else if (m_uiHD2.getInd() > val.m_uiHD2.getInd()) return false;

			if (m_uiRobot.getInd() < val.m_uiRobot.getInd()) return true;
			else if (m_uiRobot.getInd() > val.m_uiRobot.getInd()) return false;

			return false;
		}
};

struct Cand_for_picking
{
	N_Ind m_uiHole;
	size_t m_uiMinTime, m_uiMaxTime;
	size_t m_uiDist;
	size_t m_uiPredWait; //predecessor wait time (include predecessor IV and its predecessor hole)
	Cand_for_picking(size_t c_uiHole) :m_uiHole(c_uiHole) {};
};

bool sort_by_distance_wait(const Cand_for_picking& lhs, const Cand_for_picking &rhs);
bool sort_by_flexbility_wait(const Cand_for_picking& lhs, const Cand_for_picking &rhs);

class Hole_Exchange
{
	private:
		const size_t m_uiNumRobots;
		std::vector<std::list<size_t>> m_rob_seq;
		std::vector<std::list<size_t>> m_rob_seq_best;
		//LS_Greedy_Heuristic m_ls_heur;
		LS_Greedy_Heuristic_Old m_ls_heur_old;
		std::vector<State_vtx_time> m_vec_state_path;
		std::vector<std::vector<Vertex_Schedule>> m_vec_full_rob_sch;
		const Layout_LS &m_graph;	
		const Enabling_Graph &m_en_graph;
		std::unordered_map<size_t, std::unordered_map<size_t, size_t>> m_alt_out_graph;   // <vtx, <out_vtx, arc cost>>
		std::unordered_map<size_t, std::unordered_map<size_t, size_t>> m_alt_in_graph;	//  <vtx, <in_vtx, arc cost>> 
		std::unordered_map<size_t, size_t> m_map_start_times; //<vtx, start time>
		std::unordered_map<size_t, size_t> m_map_completion_times; //<vtx, end time>
		std::unordered_map<size_t, size_t> m_hole_rob_owner; //<vtx, robot>
		Topological_Sorting_Utils_Dist m_top_order_dist;
		std::unordered_map<size_t, size_t> m_map_vertex_slack;
		size_t m_uiTargetMakeSpan , m_uiBestMakeSpan;
		std::set<size_t> m_set_taboo_holes;

		void clear_prev_info();
		void populate_robot_owners(const std::vector<std::list<size_t>> &rob_seq);
		void populate_rob_graphs(const Alternative_Graph &alt_graph);

		//picking hole function
		void compute_critical_path(std::list<size_t> &critical_path);
		void get_cand_vertex_critical_path(size_t uiChoice, std::list<size_t> &critical_path, std::list<Cand_for_picking> &list_best_cand); //<vtx, min time, max time>
		void get_cand_for_insertion(const size_t c_uiHole, const size_t c_uiMinTime, const size_t c_uiMaxTime, std::list<Cand_for_insertion> &list_cand_insertion, const std::pair<size_t, size_t> &taboo_hole_pair);
		std::tuple<bool, size_t, size_t> compute_enabler_flexibility(const size_t uiVtx, const size_t c_uiMakeSpan);
		size_t compute_predecessor_wait_time(const size_t c_uiHole);
		size_t compute_hole_dist_from_to_neigh_holes(const size_t c_uiHole);
		size_t compute_min_time(const size_t uiVtx);
		std::pair<bool , size_t> compute_max_time(const size_t uiVtx, const size_t c_uiMakeSpan);

		//heuristic operations
		bool check_if_retraction_feasible(const size_t c_uiVtx, const size_t c_uiRobot, std::vector<std::list<size_t>> &rob_sub_seq);
		bool check_if_insertion_feasible(const size_t c_uiVtx, const size_t c_uiRobot, const std::pair<size_t, size_t> pr_hole_pair, std::vector<std::list<size_t>> &rob_sub_seq);
		void check_ifsub_seq_construction_correct(std::vector<std::list<size_t>> &rob_sub_seq, const size_t c_uiLeft, const size_t c_uiRight, const std::vector<State_vtx_time> &vec_state_path);
		bool check_if_candidate_improving(const size_t c_uiHole, const size_t c_uiMinTime, const size_t c_uiMaxTime);
		void compute_vertex_slack();
		int compute_desirability_of_insertion(const size_t c_uiHD1, const size_t c_uiHD2, const size_t c_uiRobot, const size_t c_uiHole);

		//heuristic graph and sequence updates
		bool update_sequence_graphs_for_removal(const size_t c_uiHole, const size_t c_uiRobot, const std::vector<std::list<size_t>> &rob_sub_seq, std::pair<size_t, size_t> &taboo_hole_pair);
		bool update_sequence_graphs_for_insertion(const size_t c_uiHole, const std::pair<size_t, size_t> pr_hole_pair, const size_t c_uiRobot, const std::vector<std::list<size_t>> &rob_sub_seq);
		std::tuple<size_t, size_t, bool> remove_hole_update_rob_sequence(const size_t c_uiHole, const size_t c_uiRobot);
		bool insert_hole_update_rob_sequence(const size_t c_uiHole, const std::pair<size_t, size_t> pr_hole_pair, const size_t c_uiRobot);
		void update_out_in_graphs_for_removal(const size_t c_uiHole, const size_t c_uiRobot, const std::vector<std::list<size_t>> &rob_sub_seq);
		void update_out_in_graphs_for_insertion(const size_t c_uiHole, const std::pair<size_t, size_t> pr_hole_pair, const size_t c_uiRobot, const std::vector<std::list<size_t>> &rob_sub_seq);
		void perform_out_in_graph_modifications(const std::set<size_t> &set_rob_sub_seq_vts, const std::vector<std::list<size_t>> &rob_sub_seq);
		void gather_arcs_betw_rob_sub_seq_start_vtcs(std::vector<arc> &vec_arcs, const std::vector<std::list<size_t>> &rob_sub_seq);
		void perform_del_vtx_arcs_in_update(const std::set<size_t> &set_rob_sub_seq_vts);
		void perform_patch_rob_sub_seq_graph(int iOption, const std::vector<std::list<size_t>> &rob_sub_seq, const std::vector<arc> &vec_arcs);
		void populate_removed_hole_prev_next_iv(const size_t c_uiHole, const size_t c_uiRobot, std::set<size_t> &set_vts);
		void populate_IV_insertion_betw_holes(const std::pair<size_t, size_t> pr_hole_pair, const size_t c_uiRobot, std::set<size_t> &set_vts);
		bool make_solution_feasible(const std::vector<std::list<size_t>> &rob_sub_seq, const size_t c_uiHole = std::numeric_limits<size_t>::max());
		std::pair<bool, bool> resolve_collisions_unenabled_vts_dynamically(const std::set<size_t> &set_vts_before_sub_seq, const std::set<size_t> &set_vts_sub_seq, const std::set<size_t> &set_vts_after_sub_seq, const std::unordered_map<size_t, size_t> &map_old_start_times, const std::unordered_map<size_t, size_t> &map_old_completion_times, const size_t c_uiHole);
		bool check_and_resolve_collision(const std::vector<std::list<size_t>::iterator>& vec_rob_vtx_itr, const std::set<size_t> &set_vts_before_sub_seq, const std::set<size_t> &set_vts_sub_seq, const std::set<size_t> &set_vts_after_sub_seq, const std::unordered_map<size_t, size_t> &map_old_start_times);
		bool check_if_vtx1_prec_vtx2_sequence_partition(const size_t c_uiVtx1, const size_t c_uiVtx2, const std::set<size_t> &set_vts_before_sub_seq, const std::set<size_t> &set_vts_sub_seq, const std::set<size_t> &set_vts_after_sub_seq);
		std::pair<bool, bool> check_and_resolve_enablers(const std::vector<std::list<size_t>::iterator>& vec_rob_vtx_itr, const std::unordered_map<size_t, size_t> &map_old_completion_times, const size_t c_uiHole);
		bool examine_super_comps_for_infeasibility();

		//heuristic utilities
		void compute_start_completion_times_from_schedule();
		void compute_completion_times_from_start_times();
		void construct_vertex_schedule();
		void construct_state_transition_path();
		void construct_state_transition_path(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch);
		void construct_rob_sub_sequences(std::vector<std::list<size_t>> &rob_sub_seq, const size_t c_uiLeft, const size_t c_uiRight, const std::vector<std::list<size_t>> &rob_seq, const std::vector<State_vtx_time> &vec_state_path,
										 std::vector<std::tuple<std::list<size_t>::const_iterator, std::list<size_t>::const_iterator, size_t>> &vec_start_end_itr_start_pos, std::unordered_set<size_t> &set_comp_HD);
		void construct_rob_sub_sequences_with_iterators(std::vector<std::list<size_t>> &rob_sub_seq, const size_t c_uiLeft, const size_t c_uiRight, const std::vector<State_vtx_time> &vec_state_path,
										 std::vector<std::tuple<std::list<size_t>::const_iterator, std::list<size_t>::const_iterator, size_t>> &vec_start_end_itr_start_pos, std::unordered_set<size_t> &set_comp_HD);
		void compute_enabled_holes_for_rob_sub_seq(const std::vector<std::list<size_t>> &rob_sub_seq, const std::unordered_set<size_t> &set_comp_HD, std::unordered_set<size_t> &set_enabled_holes);
		void copy_to_temp_buffers(std::vector<std::list<size_t>> &rob_seq, std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &out_graph, std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &in_graph, std::vector<State_vtx_time> &vec_state_path, std::vector<std::vector<Vertex_Schedule>> &vec_full_rob_sch, std::unordered_map<size_t, size_t> &map_start_times, std::unordered_map<size_t, size_t> &map_completion_times);
		void copy_from_temp_buffers(std::vector<std::list<size_t>> &rob_seq, std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &out_graph, std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &in_graph, std::vector<State_vtx_time> &vec_state_path, std::vector<std::vector<Vertex_Schedule>> &vec_full_rob_sch, std::unordered_map<size_t, size_t> &map_start_times, std::unordered_map<size_t, size_t> &map_completion_times);
		void gather_vertices_before_sub_seq_after(std::set<size_t> &set_vts_before_sub_seq, std::set<size_t> &set_vts_sub_seq, std::set<size_t> &set_vts_after_sub_seq, const std::vector<std::list<size_t>> &rob_sub_seq);
		void remove_robo_hole_owner(const size_t c_uiHole);
		void assign_robo_hole_owner(const size_t c_uiHole, const size_t c_uiRobot);
		void add_edge_to_out_in_graphs(size_t uiTail, size_t uiHead, size_t uiCost);
		void modify_arc_cost(size_t uiTail, size_t uiHead, size_t uiNewCost);
		size_t find_vtx_owner(size_t uiVtx);
		
		bool perform_swap_operation();

	public:
		Hole_Exchange(size_t uiNumRobots, const Layout_LS &graph, Power_Set &power, const Enabling_Graph &en_graph);
		bool perform_heuristic_moves(const std::vector<std::list<size_t>> &rob_seq, const Alternative_Graph &alt_graph, const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, size_t uiTargetMakeSpan);
		inline size_t get_best_makespan_found() { return m_uiBestMakeSpan; };
		inline const std::vector<std::list<size_t>>& get_best_robot_sequence() { return (true == m_rob_seq_best.empty()) ? m_rob_seq : m_rob_seq_best; };
};

std::tuple<size_t, size_t, bool> remove_INP_HOLE_in_rob_sub_seq(size_t c_uiHole, const size_t c_uiRobot, std::vector<std::list<size_t>> &rob_sub_seq, const Layout_LS &graph);
bool insert_INP_HOLE_in_rob_seq(size_t c_uiHole, const size_t c_uiRobot, const std::pair<size_t, size_t> pr_hole_pair, std::vector<std::list<size_t>> &rob_sub_seq, const Layout_LS &graph);

#endif
