#pragma once
#ifndef GREEDY_HEURISTIC_H
#define GREEDY_HEURISTIC_H

#include "Greedy_Heuristic_Utils.h"
#include "Powerset.h"
#include "Alternative_Graph.h"
#include "Collision_Filtering.h"

class Greedy_Heuristic
{
	private:
		const size_t m_uiNumRobots;
		const Layout_LS &m_graph;
		Alternative_Graph m_alt_graph;
		Power_Set &m_power;
		std::vector<std::unordered_map<N_Ind, size_t, IndHasher>> m_set_prev_HD_states;     //records the depth just before which the vertex was completed 
		std::unordered_set<size_t> m_set_to_do_verts;							//indictaes which vertices are not yet completed
		std::vector<std::unordered_map<N_Ind, size_t, IndHasher>> m_set_prev_all_states;   //records the depth info regarding the first time a vertex is encountered
		std::vector<std::unordered_map<size_t, std::pair<std::pair<arc, std::string>, std::pair<arc, std::string>>>> m_vec_map_new_sel_alt_arcs;
		std::vector<NoConstraint_EFT> m_vec_nc_eft;
		std::unordered_map<State, int, StateHasher> m_map_states_feas;                       // -1 is infeasible, 0 if feasible
		std::unordered_map<N_Ind, bool, IndHasher> m_map_self_enabling;
		std::vector<std::unordered_map<N_Ind, ST_Time, IndHasher>> m_rob_hole_times;
		Collision_Filtering m_coll_filter;
		
		bool perform_initializations(const std::vector<std::list<size_t>> &rob_seq);
		void allocate_buffers(const std::vector<std::list<size_t>> &rob_seq);
		void initialize_to_do_verts(const std::vector<std::list<size_t>> &rob_seq);
		void clear_prev_info_buffers();
		void compute_NC_makespan(const std::vector<std::list<size_t>> &rob_seq);
		
		bool construct_Alt_Graph_STN(const std::vector<std::list<size_t>> &rob_seq);
		bool construct_Alt_Graph(const std::vector<std::list<size_t>> &rob_seq);
		
		void populate_root_node_info(State &root, const std::vector<std::list<size_t>> &rob_seq);
		int compute_DFS(const State& state, size_t uiDepth, size_t uiStartTime);
		bool wasStatePreviouslySeen(const State &state);
		int check_if_final_state(const State& state);
		
		int make_selection_pos_and_check_if_feasible(size_t uiDepth, const State& state);
		int check_if_enabling_feasible(const State& state);
		bool check_if_self_enabling(size_t uiRobot, const State& state);
		bool check_if_other_enabling(size_t uiRobot, const State& state);
		int check_if_coll_feasible(const State& state);

		void compute_succ_nodes(const size_t uiCurrTime, const State& state, std::vector<std::pair<Comparison_Object, State>> &vec_children);
		void compute_B_Q(const State& state, std::unordered_set<size_t> &B_Q);
		const std::vector<std::vector<size_t>>& get_child_states_iter_inc(const State& state);
		void get_updatable_robots(const State& state, std::vector<size_t> &set_robots);

		void safe_initialize(size_t uiStartTime, size_t uiDepth, const State& state);
		void safe_backtrack(size_t uiDepth, const State& state);

		bool make_state_positional_sel(size_t uiDepth, const State& state);
		void allocate_positional_arc_buffer(size_t uiDepth);
		void deallocate_positional_arc_buffer(size_t uiDepth);

		void update_vis_unvis_states_schedule(size_t uiCurrTime, size_t uiDepth, const State& state);
		void update_visited_all_states_schedule(size_t uiRobot, size_t uiCurrTime, size_t uiDepth, const State& state);
		//void update_visited_HD_states(size_t uiRobot, size_t uiDepth, const State& state);
		
		void remove_vis_unvis_state_and_schedule(size_t uiDepth, const State& state);
		void remove_visited_all_states_schedule(size_t uiRobot, size_t uiDepth, const State& state);
		//void remove_visited_HD_states(size_t uiRobot, size_t uiDepth, const State& state);
		
		std::pair<size_t, size_t> compute_exp_Mkspn_delay(const size_t uiCurrTime, const State& state);
		void vectorize_schedule(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch);

		void print_state(size_t uiDepth, size_t uiTime, const State &state);
		
	public:
		Greedy_Heuristic(const size_t uiRobotNum, const Layout_LS &graph, Power_Set &power);
		int compute_greedy_sol(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, std::string strFolder);
};

#endif

