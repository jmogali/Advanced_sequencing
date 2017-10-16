#ifndef COLLISION_FILTERING_H
#define COLLISION_FILTERING_H

#include "Alternative_Graph.h"
#include "Kosaraju_Algo.h"

class Collision_Filtering
{
	private:
		std::list<std::unordered_set<size_t>> m_list_Comp;
		std::unordered_map<int, std::unordered_set<int>> m_out_graph;
		std::list<int> m_list_order;
		std::unordered_map<size_t, std::unordered_map<size_t, std::pair<size_t, size_t>>> m_map_bounds; //<N_Ind , <R_Ind , <N_Ind, N_Ind>>>
		std::unordered_map<int, std::unordered_set<int>> m_in_graph;
		
		void clear_prev_info();

		void construct_in_out_graphs(const Alternative_Graph &alt_graph);
		void copy_to_out_graph(const std::unordered_map<size_t, std::unordered_map<size_t, size_t>>& inp_graph);
		bool Check_Pos_Loop_Remove_1comp(const Alternative_Graph &alt_graph);
		void replace_strong_conn_comp_out_graph(const Alternative_Graph &alt_graph);
		void construct_out_graph(const Alternative_Graph &alt_graph);
		void construct_in_graph();

		void Topological_sort_out_graph();
		void Topological_Dfs(int iVtx, std::unordered_map<int, std::string> &map_seen);

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
};
#endif

