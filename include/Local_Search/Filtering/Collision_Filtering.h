#pragma once
#ifndef COLLISION_FILTERING_H
#define COLLISION_FILTERING_H

#include "Alternative_Graph.h"
#include "Coll_Topological_Sorting_Utils.h"

class Collision_Filtering : public Coll_Topological_Sorting_Utils
{
	private:
		std::unordered_map<size_t, std::unordered_map<size_t, std::pair<size_t, size_t>>> m_map_bounds; //<N_Ind , <R_Ind , <N_Ind, N_Ind>>>
		std::unordered_map<size_t, std::vector<size_t>> m_map_lower_bounds;  // N_ind, vector<N_Ind> (vertex, vector containing min position for other robot)
								
		void clear_prev_info();
		
		void Compute_lower_bounds(const Alternative_Graph &alt_graph, const std::vector<std::list<size_t>> &rob_seq);
		void compute_lower_bound_for_component(const Alternative_Graph &alt_graph, int iVtx, const size_t c_uiNumRobots);
		void Initialize_lower_bounds_map(const std::vector<std::list<size_t>> &rob_seq);

		size_t Compute_FROM_costs_each_Vertex(const Alternative_Graph &alt_graph, std::vector<std::vector<size_t>> &vec_cost_from_source);
		size_t Compute_GO_costs_each_Vertex(const Alternative_Graph &alt_graph, std::vector<std::vector<size_t>> &vec_cost_to_go);

		void Compute_bounds(const Alternative_Graph &alt_graph, const std::vector<std::list<size_t>> &rob_seq);
		bool check_bounds_validity(const std::vector<std::list<size_t>> &rob_seq);
		bool check_bounds_validity(size_t uiRobot, const std::vector<std::list<size_t>> &rob_seq);

		void Compute_lower_bounds(size_t uiRobot1, size_t uiRobot2, const std::list<int> &rob_list_order, const std::vector<std::list<size_t>> &rob_seq, const Alternative_Graph &alt_graph);
		void initialize_lower_bounds(size_t uiRobot1, size_t uiRobot2, const std::vector<std::list<size_t>> &rob_seq);
		std::pair<bool, size_t> get_LB_from_pred(int iVtx, size_t uiRobot, size_t uiOtherRobot, const Alternative_Graph &alt_graph);
		void insert_LB_non_r1_r2_comp_vtx(int iVtx, size_t uiVtx, size_t uiRobot, size_t uiOtherRobot, const Alternative_Graph &alt_graph);

		void Compute_upper_bounds(size_t uiRobot1, size_t uiRobot2, const std::list<int> &rob_list_order, const std::vector<std::list<size_t>> &rob_seq, const Alternative_Graph &alt_graph);
		void initialize_upper_bounds(size_t uiRobot1, size_t uiRobot2, const std::vector<std::list<size_t>> &rob_seq);
		void insert_UB_non_r1_r2_comp_vtx(int iVtx, size_t uiVtx, size_t uiRobot, size_t uiOtherRobot, const Alternative_Graph &alt_graph, std::unordered_map<size_t, size_t> &set_marked_nodes);
		std::pair<std::pair<bool, size_t>, std::pair<bool, size_t>> get_UB_from_succ(int iVtx, size_t uiRobot, size_t uiOtherRobot, const Alternative_Graph &alt_graph, const std::unordered_map<size_t, size_t> &set_marked_nodes);

		void compute_robot_pair_list_order(size_t uiRobot1, size_t uiRobot2, std::list<int> &rob_pair_list_order, const Alternative_Graph &alt_graph);

		void Initialize_bounds_map(std::unordered_map<size_t, std::unordered_map<size_t, std::pair<size_t, size_t>>> &map_bounds, const std::vector<std::list<size_t>> &rob_seq);

		std::pair<bool, size_t> containsVertex(size_t uiRobot, size_t uiComp, const Alternative_Graph &alt_graph);
		std::pair<bool, size_t> containsVertex(size_t uiRobot, const Alternative_Graph &alt_graph, const std::unordered_set<size_t> &comp);
				
	public:
		Collision_Filtering() {};
		bool Check_Feasibility_Compute_Bounds_For_Each_Vertex(const std::vector<std::list<size_t>> &rob_seq, const Alternative_Graph &alt_graph);
		size_t Compute_costs_for_each_Vertex(const std::vector<std::list<size_t>> &rob_seq, const Alternative_Graph &alt_graph, std::vector<std::vector<size_t>> &vec_cost_from_source, std::vector<std::vector<size_t>> &vec_cost_to_go);
		std::pair<size_t, size_t> get_bounds(size_t uiVtx, size_t uiOtherRobot);
		size_t get_lower_bound_pos(size_t uiVtx, size_t uiOtherRobot) const;
		inline const std::list<std::unordered_set<size_t>>& get_scc() const { return m_list_Super_Comp; };
		bool add_scc_comps(Alternative_Graph &alt_graph, std::list<arc> &list_prec_arcs_betw_jobs) const;		
};
#endif

