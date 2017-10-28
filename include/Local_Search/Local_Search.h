#pragma once
#ifndef LOCAL_SEARCH_H
#define LOCAL_SEARCH_H
#include "Local_Search_Constants.h"
#include "Node_Partitions.h"
#include "Greedy_Heuristic.h"
#include "Greedy_Heuristic_old.h"
#include "Enabling_Graph.h"
#include <random>

class Local_Search
{
	private:
		std::random_device m_rd;     // only used once to initialise (seed) engine
		std::mt19937 m_rng;
		const Node_Partitions &m_node_data;
		const Layout_LS &m_graph;

		// Sequence generation section
		void allocate_holes_to_robots_common_with_bias(std::vector<std::unordered_set<size_t>> &vec_com_hole_par, std::string strBias);
		void generate_constructive_sequence_VBSS(std::vector<std::list<size_t>> &rob_seq);
		void gen_seq_VBSS_march_for_robot(size_t uiRobot, std::unordered_set<size_t> &set_holes, std::list<size_t> &hole_seq, const Enabling_Graph &en_graph);
		
		// validation section
		bool check_validity_of_sequence(const std::vector<std::list<size_t>> &rob_seq);
		bool check_validity_of_schedule(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch);
				
		//local search operators section
		//random operators section
		std::tuple<bool, size_t, size_t> inter_rand_oper(std::vector<std::list<size_t>> &rob_seq, std::string strType);
		bool string_exchange(size_t uiRobot1, size_t uiRobot2, std::vector<std::list<size_t>> &rob_seq);
		bool string_relocation(size_t uiRobot1, size_t uiRobot2, std::vector<std::list<size_t>> &rob_seq);		
		std::pair<bool, size_t> intra_rand_oper(std::vector<std::list<size_t>> &rob_seq, std::string strType);
		bool swap_intra_sequence(size_t uiRobot , std::vector<std::list<size_t>> &rob_seq);
		bool string_cross_intra_sequence(size_t uiRobot, std::vector<std::list<size_t>> &rob_seq, bool bDist);
		
		//auxillary section
		void convert_hole_seq_to_full_seq(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::list<size_t>> &full_rob_seq);
		
		// scheduling section
		int perform_greedy_scheduling(Greedy_Heuristic &heur, const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, std::string strFolderPath);
		int perform_greedy_scheduling_old(Greedy_Heuristic_old &heur_old, const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &full_rob_sch);

	public:
		Local_Search(const Node_Partitions &node_data, const Layout_LS &graph);
		void perform_local_search(std::string strFolderPath);
};

#endif
