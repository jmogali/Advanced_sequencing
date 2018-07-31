#pragma once
#ifndef LOCAL_SEARCH_H
#define LOCAL_SEARCH_H
#include "Local_Search_Constants.h"
#include "Node_Partitions.h"
#include "Greedy_Heuristic.h"
#include "Greedy_Heuristic_old.h"
#include "Enabling_Graph.h"
#include "Hole_exchanges.h"
#include <random>

class Local_Search
{
	private:
		std::random_device m_rd;     // only used once to initialise (seed) engine
		std::mt19937 m_rng;
		const Node_Partitions &m_node_data;
		const Layout_LS &m_graph;
		const double m_dWeight_Factor;
		const Enabling_Graph m_en_graph;
		
		// Sequence generation section
		void allocate_holes_to_robots_common_with_bias(std::vector<std::unordered_set<size_t>> &vec_com_hole_par, std::string strBias);
		void generate_constructive_sequence_VBSS(std::vector<std::list<size_t>> &rob_seq);
		void gen_seq_VBSS_march_for_robot(std::vector<std::unordered_set<size_t>> &vec_com_hole_par, std::vector<std::list<size_t>> &rob_seq);
		bool add_new_enabled_holes(const std::vector<size_t> &vec_rob_curr_node, std::unordered_set<size_t> &set_curr_enabled_holes, const std::unordered_set<size_t> &set_completed_verts);

		//utility
		void populate_new_sequence(const std::vector<std::list<size_t>> &rob_sequence_with_IV, std::vector<std::list<size_t>> &rob_sequence_without_IV);
		void free_VLNS_buffers();
		void print_state_transition_path(std::string strFilePath, const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch);
		void print_best_solution_progress(std::string strFilePath, const std::vector<std::pair<size_t, double>> &vec_impr_sol);

		// validation section
		bool check_validity_of_sequence(const std::vector<std::list<size_t>> &rob_seq);
		bool check_validity_of_schedule(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch);
		
		//local search section
		void generate_new_sequence_rand_moves(std::vector<std::list<size_t>> &rob_seq);
		bool gen_seq_hole_exchange(Hole_Exchange &hole_exchange, Greedy_Heuristic &heur, const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch,std::vector<std::list<size_t>> &rob_seq, size_t &c_uiTargetMakeSpan);
		void gen_seq_TSP(std::string strTSPFolder, Greedy_Heuristic &heur, const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, std::vector<std::list<size_t>> &rob_seq, size_t &uiTargetMakeSpan, const size_t c_uiKVal);

		//local search operators section
		//random operators section
		std::tuple<bool, size_t, size_t> inter_rand_oper(std::vector<std::list<size_t>> &rob_seq, std::string strType);
		bool string_exchange(size_t uiRobot1, size_t uiRobot2, std::vector<std::list<size_t>> &rob_seq);
		bool string_relocation(size_t uiRobot1, size_t uiRobot2, std::vector<std::list<size_t>> &rob_seq);		
		std::pair<bool, size_t> intra_rand_oper(std::vector<std::list<size_t>> &rob_seq, std::string strType);
		bool swap_intra_sequence(size_t uiRobot , std::vector<std::list<size_t>> &rob_seq);
		bool Two_opt_intra_sequence(const size_t c_uiRobot, std::vector<std::list<size_t>> &rob_seq);
		
		//auxillary section
		void convert_hole_seq_to_full_seq(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::list<size_t>> &full_rob_seq);
		
		// scheduling section
		int perform_greedy_scheduling(Greedy_Heuristic &heur, std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, std::string strPlotFolder, const size_t c_uiUpperBound = std::numeric_limits<size_t>::max());
		int perform_greedy_scheduling_old(Greedy_Heuristic_old &heur_old, std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &full_rob_sch);	

	public:
		Local_Search(const Node_Partitions &node_data, const Layout_LS &graph, const double dWeightFactor);
		void perform_local_search(std::string strFolderPath, std::string strDataDumpFolder, std::string strTSPFolder, size_t ui_KVal, size_t uiSimulNum);
		void perform_local_search_improved(std::string strFolderPath, std::string strDataDumpFolder, std::string strTSPFolder, size_t ui_KVal, size_t uiSimulNum);
		void perform_VBSS_search(std::string strFolderPath);
		inline const Enabling_Graph& get_Enabling_graph() { return m_en_graph; };
};

bool string_Exchange(std::list<size_t> &r1, const std::pair<size_t, size_t> &pr1, size_t uiRobot1, std::list<size_t> &r2, const std::pair<size_t, size_t> &pr2, size_t uiRobot2, std::mt19937 &rng, const Layout_LS& graph);
bool string_relocate(std::list<size_t> &r1, const std::pair<size_t, size_t> &pr1, size_t uiRobot1, std::list<size_t> &r2, size_t uiPos2, size_t uiRobot2, const Layout_LS& graph);
bool swap_Intra_sequence(size_t uiPos1, size_t uiLen1, size_t uiPos2, size_t uiLen2, std::list<size_t> &seq, size_t uiRobot, const Layout_LS &graph);
size_t generate_rand_ind_from_cdf(const size_t uiNormFactor, std::mt19937 &rng, const std::vector<size_t> &vec_CDF);

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
