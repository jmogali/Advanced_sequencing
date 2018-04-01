#pragma once
#ifndef LOCAL_SEARCH_H
#define LOCAL_SEARCH_H
#include "Local_Search_Constants.h"
#include "Node_Partitions.h"
#include "Greedy_Heuristic.h"
#include "Greedy_Heuristic_old.h"
#include "LS_Greedy_Heuristic_old.h"
#include "Enabling_Graph.h"
#include <random>

class Local_Search
{
	private:
		std::random_device m_rd;     // only used once to initialise (seed) engine
		std::mt19937 m_rng;
		const Node_Partitions &m_node_data;
		const Layout_LS &m_graph;
		const double m_dWeight_Factor;

		// Sequence generation section
		void allocate_holes_to_robots_common_with_bias(std::vector<std::unordered_set<size_t>> &vec_com_hole_par, std::string strBias);
		void generate_constructive_sequence_VBSS(std::vector<std::list<size_t>> &rob_seq);
		void gen_seq_VBSS_march_for_robot(size_t uiRobot, std::unordered_set<size_t> &set_holes, std::list<size_t> &hole_seq, const Enabling_Graph &en_graph);
		
		// validation section
		bool check_validity_of_sequence(const std::vector<std::list<size_t>> &rob_seq);
		bool check_validity_of_schedule(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch);
		
		//local search section
		void generate_new_sequence(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, const bool c_bWait, std::vector<std::list<size_t>> &rob_seq, bool bSuccess);

		//local search operators section
		//random operators section
		std::tuple<bool, size_t, size_t> inter_rand_oper(std::vector<std::list<size_t>> &rob_seq, std::string strType);
		bool string_exchange(size_t uiRobot1, size_t uiRobot2, std::vector<std::list<size_t>> &rob_seq);
		bool string_relocation(size_t uiRobot1, size_t uiRobot2, std::vector<std::list<size_t>> &rob_seq);		
		std::pair<bool, size_t> intra_rand_oper(std::vector<std::list<size_t>> &rob_seq, std::string strType);
		bool swap_intra_sequence(size_t uiRobot , std::vector<std::list<size_t>> &rob_seq);
		bool string_cross_intra_sequence(size_t uiRobot, std::vector<std::list<size_t>> &rob_seq, bool bDist);
		
		//wait based operators
		std::tuple<bool, size_t, size_t> wait_based_oper(std::vector<std::list<size_t>> &rob_seq, const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, std::string strType);
		std::tuple<bool, size_t, size_t> wait_based_swap_for_robot(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, std::vector<std::list<size_t>> &rob_seq, size_t uiRobot, std::string strType);
		std::pair < std::string , size_t > wait_based_move_inter_sequence(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, std::vector<std::list<size_t>> &rob_seq, size_t uiRobot, size_t uiWaitHoleInd, size_t uiWaitHolePos, size_t uiTime, std::string strWaitMove);
		bool wait_based_move_intra_sequence(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, std::vector<std::list<size_t>> &rob_seq, size_t uiRobot, size_t uiWaitHoleInd, size_t uiWaitHolePos, std::string strWaitMove);

		//auxillary section
		void convert_hole_seq_to_full_seq(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::list<size_t>> &full_rob_seq);
		size_t get_bottleneck_robot(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch); // returns robot having maximum makespan
		void get_Wait_Holes_For_Robot(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, size_t uiRobot, std::vector<std::pair<size_t, size_t>> &vec_wait_ind_pos);

		// scheduling section
		int perform_greedy_scheduling(Greedy_Heuristic &heur, std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, std::string strPlotFolder);
		int perform_greedy_scheduling_old(Greedy_Heuristic_old &heur_old, std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &full_rob_sch);
		int perform_greedy_scheduling(LS_Greedy_Heuristic_Old &heur_old, const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &full_rob_sch);

	public:
		Local_Search(const Node_Partitions &node_data, const Layout_LS &graph, const double dWeightFactor);
		void perform_local_search(std::string strFolderPath, std::string strDataDumpFolder);
		void perform_VBSS_search(std::string strFolderPath);
};

bool string_Exchange(std::list<size_t> &r1, const std::pair<size_t, size_t> &pr1, size_t uiRobot1, std::list<size_t> &r2, const std::pair<size_t, size_t> &pr2, size_t uiRobot2, const Layout_LS& graph);
bool string_relocate(std::list<size_t> &r1, const std::pair<size_t, size_t> &pr1, size_t uiRobot1, std::list<size_t> &r2, size_t uiPos2, size_t uiRobot2, const Layout_LS& graph);
bool swap_Intra_sequence(size_t uiPos1, size_t uiLen1, size_t uiPos2, size_t uiLen2, std::list<size_t> &seq, size_t uiRobot, const Layout_LS &graph);

struct sort_by_max_second_val
{
	//<position, wait value>
	inline bool operator() (const std::pair<size_t, size_t>& pr1, const std::pair<size_t, size_t>& pr2)
	{
		if (pr1.second > pr2.second) return true;
		if (pr1.second < pr2.second) return false;

		if (pr1.first < pr2.first) return true;
		return false;
	}
};



#endif
