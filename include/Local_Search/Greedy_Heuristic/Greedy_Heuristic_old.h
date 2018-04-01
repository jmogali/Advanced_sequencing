#pragma once
#ifndef GREEDY_HEURISTIC_OLD_H
#define GREEDY_HEURISTIC_OLD_H

#include "Greedy_Heuristic_Utils.h"
#include "Powerset.h"

class Greedy_Heuristic_old
{
	protected:
		const size_t m_uiNumRobots;
		const Layout_LS &m_graph;
		Power_Set &m_power;
		std::vector<std::unordered_map<N_Ind, size_t, IndHasher>> m_set_prev_HD_states;     //records the depth just before which the vertex was completed 
		std::vector<std::unordered_map<N_Ind, size_t, IndHasher>> m_set_prev_all_states;   //records the depth info regarding the first time a vertex is encountered
		std::vector<NoConstraint_EFT> m_vec_nc_eft;
		std::unordered_map<State, int, StateHasher> m_map_states_feas;                       // -1 is infeasible, 0 if feasible
		std::unordered_map<N_Ind, bool, IndHasher> m_map_self_enabling;
		std::vector<std::unordered_map<N_Ind, ST_Time, IndHasher>> m_rob_hole_times;
		
		void perform_initializations(const std::vector<std::list<size_t>> &rob_seq);
		void clear_prev_info_buffers();
		void allocate_interval_buffer(const std::vector<std::list<size_t>> &rob_seq);
		void compute_NC_makespan(const std::vector<std::list<size_t>> &rob_seq);
		
		void populate_root_node_info(State &root, const std::vector<std::list<size_t>> &rob_seq);
		int compute_DFS(const State& state, size_t uiDepth, size_t uiStartTime);
		int check_if_final_state(const State& state);
		
		int check_if_feasible(const State& state);
		virtual int check_if_enabling_feasible(const State& state);
		bool check_if_self_enabling(size_t uiRobot, const State& state);
		bool check_if_other_enabling(size_t uiRobot, const State& state);
		int check_if_coll_feasible(const State& state);

		void compute_succ_nodes(const size_t uiCurrTime, const State& state, std::vector<std::tuple<int, int, int, size_t, State>> &vec_children);
		const std::vector<std::vector<size_t>>& get_child_states_iter_inc(const State& state);
		void get_updatable_robots(const State& state, std::vector<size_t> &set_robots);

		void update_visited_states_schedule(size_t uiCurrTime, size_t uiDepth, const State& state);
		void update_visited_all_states_schedule(size_t uiRobot, size_t uiCurrTime, size_t uiDepth, const State& state);
		//void update_visited_HD_states(size_t uiRobot, size_t uiDepth, const State& state);
		
		void remove_visited_state_and_schedule(size_t uiDepth, const State& state);
		void remove_visited_all_states_schedule(size_t uiRobot, size_t uiDepth, const State& state);
		//void remove_visited_HD_states(size_t uiRobot, size_t uiDepth, const State& state);
		
		std::pair<int, int> compute_greedy_heursitic1(const size_t uiTime, const State& state);
		void vectorize_schedule(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch);

		void print_state(size_t uiDepth, size_t uiTime, const State &state);
		size_t getTime(size_t uiVert);
	public:
		Greedy_Heuristic_old(const size_t uiRobotNum, const Layout_LS &graph, Power_Set &power);
		int compute_greedy_sol(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch);
};

#endif

