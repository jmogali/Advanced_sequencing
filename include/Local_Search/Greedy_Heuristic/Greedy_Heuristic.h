#pragma once
#ifndef GREEDY_HEURISTIC_H
#define GREEDY_HEURISTIC_H

#include "Greedy_Heuristic_Utils.h"
#include "Powerset.h"
#include "Alternative_Graph.h"
#include "Collision_Filtering.h"

class Greedy_Heuristic
{
	protected:
		const size_t m_uiNumRobots;
		const Layout_LS &m_graph;
		Alternative_Graph m_alt_graph;
		Power_Set &m_power;
		std::unordered_set<size_t> m_set_to_do_verts;							//indicates which vertices are not yet completed
		std::vector<std::unordered_map<N_Ind, size_t, IndHasher>> m_set_prev_all_states;   //records the depth info regarding the first time a vertex is encountered
		std::vector<std::unordered_map<size_t, std::pair<std::pair<arc, std::string>, std::pair<arc, std::string>>>> m_vec_map_new_sel_alt_arcs;
		std::vector<NoConstraint_EFT> m_vec_nc_eft;
		std::unordered_map<State, int, StateHasher> m_map_states_feas;                       // -1 is infeasible, 0 if feasible
		std::unordered_map<N_Ind, bool, IndHasher> m_map_self_enabling;
		std::vector<std::unordered_map<N_Ind, ST_Time, IndHasher>> m_rob_hole_times;
		Collision_Filtering m_coll_filter;
		std::unordered_map<size_t, std::unordered_map<size_t, std::pair<size_t, size_t>>> m_map_enabler_pos_vert; // uiRobot, <position, vertex>
		bool m_bWait , m_bVectorizeSchedule;
		std::vector<std::pair<size_t, size_t>> m_vec_rob_first_last_vtx;
		std::vector<std::list<size_t>> m_rob_seq;
		bool m_bComplete_Graph;

#ifdef ENABLE_FULL_CHECKING		
		std::vector<std::unordered_map<N_Ind, size_t, IndHasher>> m_set_prev_HD_states;     //records the depth just before which the vertex was completed 
#endif

		//Heuristic utilities
		bool perform_initializations(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::list<size_t>> &new_rob_seq, const size_t c_uiUpperBound);
		void allocate_buffers(const std::vector<std::list<size_t>> &rob_seq);
		void initialize_to_do_verts(const std::vector<std::list<size_t>> &rob_seq);
		void clear_prev_info_buffers();
		void compute_NC_makespan(const std::vector<std::list<size_t>> &rob_seq);
		void set_first_last_vertices(const std::vector<std::list<size_t>> &rob_seq);
		
		//Alt_graph construction
		bool construct_Alt_Graph_STN(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::list<size_t>> &new_rob_seq, const size_t c_uiUpperBound);
		bool construct_Alt_Graph(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::list<size_t>> &new_rob_seq, const size_t c_uiUpperBound);
		
		//Alt graph construction + Search intersection utilities
		void construct_prec_graph_for_each_operation(const std::vector<std::list<size_t>> &rob_seq);
		virtual void get_verts_not_self_enabled(size_t uiRobot, const std::vector<std::list<size_t>> &rob_seq, std::list<size_t>& list_dep_vert);
		bool add_prec_arcs_for_dep_vert_of_job(size_t uiGivenRobot, const std::vector<std::list<size_t>> &rob_seq, std::list<arc> &list_prec_arcs_betw_jobs, std::list<size_t> &list_dep_vert);
		bool add_enabling_cons(const std::vector<std::list<size_t>> &rob_seq, std::list<arc> &list_prec_arcs_betw_jobs, std::vector<std::list<size_t>> &vec_dep_vert);
		bool check_if_new_precedences_can_be_added(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::list<size_t>> &vec_dep_vert);
		bool add_coll_cons_bet_pair_jobs(size_t uiRobot1, size_t uiRobot2, const std::vector<std::list<size_t>> &rob_seq, const std::vector<std::vector<size_t>> &vec_cost_from_source, const std::vector<std::vector<size_t>> &vec_cost_to_go, const size_t c_uiUpperBound);
		bool get_coll_cons_bet_pair_jobs(size_t uiRobot1, size_t uiRobot2, const std::vector<std::list<size_t>> &rob_seq, std::unordered_set<Coll_Pair, CollHasher> &set_coll);
		bool add_coll_cons(const std::vector<std::list<size_t>> &rob_seq, const std::vector<std::vector<size_t>> &vec_cost_from_source, const std::vector<std::vector<size_t>> &vec_cost_to_go, const size_t c_uiUpperBound);
		bool get_coll_cons(const std::vector<std::list<size_t>> &rob_seq, std::unordered_set<Coll_Pair, CollHasher> &set_coll);
		bool add_imp_prec_coll_con(const std::vector<std::list<size_t>> &rob_seq, const arc &inp_arc, std::list<arc> &list_prec_arcs_betw_jobs);
		bool add_imp_con_for_given_prec_arc_forward_dir(const std::vector<std::list<size_t>> &rob_seq, const arc &inp_arc, std::list<arc> &list_prec_arcs_betw_jobs);
		bool add_imp_con_for_given_prec_arc_reverse_dir(const std::vector<std::list<size_t>> &rob_seq, const arc &inp_arc, std::list<arc> &list_prec_arcs_betw_jobs);
		bool add_imp_con_for_given_pred_arc(const std::vector<std::list<size_t>> &rob_seq, const arc &inp_arc, std::list<arc> &list_prec_arcs_betw_jobs);
		bool add_impl_cons_rem_prev_cons(const std::vector<std::list<size_t>> &rob_seq, std::list<arc> &list_prec_arcs_betw_jobs);
		std::pair<bool, bool> add_scc_check_coll_feasible(std::list<arc> &list_prec_arcs_betw_jobs);
		bool add_enabling_fix_coll_cons(const std::vector<std::list<size_t>> &rob_seq);
		
		//DFS functions
		void populate_root_node_info(State &root, const std::vector<std::list<size_t>> &rob_seq);
		int compute_DFS(const State& state, size_t uiDepth, size_t uiStartTime);
		bool wasStatePreviouslySeen(const State &state);
		int check_if_final_state(const State& state);
		void safe_initialize(size_t uiStartTime, size_t uiDepth, const State& state);
		void safe_backtrack(size_t uiDepth, const State& state);
		void update_vis_unvis_states_schedule(size_t uiCurrTime, size_t uiDepth, const State& state);
		void update_visited_all_states_schedule(size_t uiRobot, size_t uiCurrTime, size_t uiDepth, const State& state);
		void remove_vis_unvis_state_and_schedule(size_t uiDepth, const State& state);
		void remove_visited_all_states_schedule(size_t uiRobot, size_t uiDepth, const State& state);
		bool check_if_backwards_coll_state(const State& state);
		bool check_if_coll_backwards_vtx(size_t uiVtx, size_t uiGivenRobot, const std::vector<size_t>& vec_rob_vertpos);
		std::pair<size_t, size_t> compute_exp_Mkspn_delay(const size_t uiCurrTime, const State& state);
		virtual size_t getTime(size_t uiVert);
		size_t getTime(N_Ind Ind);

		//Feasibility checks
		int make_selection_pos_and_check_if_feasible(size_t uiDepth, const State& state);
		int check_if_enabling_feasible(const State& state);
		int check_if_coll_feasible(const State& state);
		
		//Alt graph traversal
		void compute_succ_nodes(const size_t uiCurrTime, const State& state, std::vector<std::pair<Comparison_Object, State>> &vec_children);
		void compute_B_Q(const State& state, std::unordered_set<size_t> &B_Q);
		const std::vector<std::vector<size_t>>& get_child_states_iter_inc(const State& state);
		void get_updatable_robots(const State& state, std::vector<size_t> &set_robots);
		bool make_state_positional_sel(size_t uiDepth, const State& state);
		void allocate_positional_arc_buffer(size_t uiDepth);
		void deallocate_positional_arc_buffer(size_t uiDepth);
		
		//some additional useful utilities	
		void vectorize_schedule(const std::vector<std::list<size_t>> &new_rob_seq, std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const std::vector<std::list<size_t>> &rob_seq);
		void print_state(size_t uiDepth, size_t uiTime, const State &state);	
		bool sanity_check_schedule(const std::vector<std::list<size_t>> &rob_seq, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch);
				
		//Filling the pruned collisions arcs and enabling arcs. This is used for swap heuristic.
		void fill_back_pruned_collision_arcs();
		void fill_back_pruned_collision_arcs(size_t uiRobot1, size_t uiRobot2);
		void insert_missing_enabling_arcs();
		void insert_missing_enabling_arcs(const size_t c_uiGivenRobot);

		//dummy functions, this makes sense only for LS_Greedy_Heur
		virtual bool isVtxPreEnabled(size_t uiVtx);
		virtual bool isEnablerHolePresent(size_t uiEnablerVtx);
		
	public:
		Greedy_Heuristic(const size_t uiRobotNum, const Layout_LS &graph, Power_Set &power);
		int compute_greedy_sol(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, std::string strPlotFolder, const size_t c_uiUpperBound = std::numeric_limits<size_t>::max());
		inline bool doRobotsWait() const { assert(true == m_bVectorizeSchedule); return m_bWait; };
		const Alternative_Graph& get_complete_alt_graph(int iOptions); //0 - none, 1 - only enabling, 2 - only collision, 3 - both
};

#endif

