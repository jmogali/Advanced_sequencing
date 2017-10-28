#pragma once
#ifndef ENABLING_GRAPH_H
#define ENABLING_GRAPH_H

#include <vector>
#include <unordered_set>
#include "Layout_LS.h"
#include <random>

class Enabling_Node
{
	private:
		//std::unordered_set<size_t> m_map_reachable_robots; //robot indices
		std::vector<size_t> m_vec_neigh_node_ind;
		inline void add_neigh(size_t neigh) { m_vec_neigh_node_ind.push_back(neigh); };
		//inline void add_robot(size_t uiRobot) { m_map_reachable_robots.emplace(uiRobot); };

	public:
		Enabling_Node();
		//bool isNodeReachable(size_t uiRobot) const;
		inline const std::vector<size_t>& get_neighs() const { return m_vec_neigh_node_ind;};
		friend class Enabling_Graph;
};

class Enabling_Graph
{
	private:
		std::unordered_map<size_t, Enabling_Node> m_set_nodes;
		inline void add_neigh(size_t uiIndex , size_t neigh) { return m_set_nodes[uiIndex].add_neigh(neigh); };
		//inline void add_robot(size_t uiIndex, size_t uiRobot) { return m_set_nodes[uiIndex].add_robot(uiRobot); };
		//void traverse_graph(size_t uiStart, std::list<size_t> &seq_enab, std::vector<std::string> &vec_visit_status, const Layout_LS &graph, std::mt19937 &rng);

	public:
		inline const unordered_map<size_t, Enabling_Node>& get_Node_vec() const {return m_set_nodes;};
		//void compute_rand_biased_enabled_seq_from_start_vtx(size_t uiStart, std::list<size_t> &vec_enab, const Layout_LS &graph, std::mt19937 &rng);
		Enabling_Graph(const Layout_Graph &graph);
};

#endif
