#pragma once
#ifndef LOCAL_SEARCH_H
#define LOCAL_SEARCH_H
#include "Local_Search_Constants.h"
#include "Node_Partitions.h"
#include "Greedy_Heuristic.h"
#include "Greedy_Heuristic_old.h"
#include <random>

class Local_Search
{
	private:
		std::random_device m_rd;     // only used once to initialise (seed) engine
		std::mt19937 m_rng;
		const Node_Partitions &m_node_data;
		const Layout_LS &m_graph;

		void generate_rand_allocation_of_nodes_to_robots(std::vector<std::vector<size_t>> &vec_com_hole_par);
		void generate_nearest_neigh_alloc_to_robots(std::vector<std::vector<size_t>> &vec_com_hole_par);
		void generate_rand_sequence(std::vector<std::list<size_t>> &rob_seq);
		void generate_constructive_sequence(std::vector<std::list<size_t>> &rob_seq);
		
		bool check_validity_of_sequence(const std::vector<std::list<size_t>> &rob_seq);
		bool check_validity_of_schedule(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch);
				
		std::tuple<bool, size_t, size_t> inter_rand_oper(std::vector<std::list<size_t>> &rob_seq, std::string strType);
		bool string_exchange(size_t uiRobot1, size_t uiRobot2, std::vector<std::list<size_t>> &rob_seq);
		bool string_relocation(size_t uiRobot1, size_t uiRobot2, std::vector<std::list<size_t>> &rob_seq);		
		std::pair<bool, size_t> intra_rand_oper(std::vector<std::list<size_t>> &rob_seq, std::string strType);
		bool swap_intra_sequence(size_t uiRobot , std::vector<std::list<size_t>> &rob_seq);
		bool string_cross_intra_sequence(size_t uiRobot, std::vector<std::list<size_t>> &rob_seq, bool bDist);
		void convert_hole_seq_to_full_seq(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::list<size_t>> &full_rob_seq);
		int perform_greedy_scheduling(Greedy_Heuristic &heur, const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, std::string strFolderPath);
		int perform_greedy_scheduling_old(Greedy_Heuristic_old &heur_old, const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &full_rob_sch);

		void remove_holes_that_are_not_assigned_to_robots_from_enabler_seq(size_t c_uiNumRobots, std::vector<std::vector<size_t>> &vec_com_hole_par, std::vector<std::list<size_t>> &vec_rob_enab_seq);
		void add_holes_not_reachable_by_enabler_sequence(size_t c_uiNumRobots, std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<size_t>> &vec_com_hole_par, std::vector<std::list<size_t>> &vec_rob_enab_seq);
		void assign_hole_seq_to_robots_from_enabling_seq(size_t c_uiNumRobots, std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<size_t>> &vec_com_hole_par, std::vector<std::list<size_t>> &vec_rob_enab_seq);

	public:
		Local_Search(const Node_Partitions &node_data, const Layout_LS &graph);
		void perform_local_search(std::string strFolderPath);
};

#endif
