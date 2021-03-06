#pragma once
#ifndef NODE_PARTITION_H
#define NODE_PARTITION_H

#include "Layout_LS.h"
#include "Local_Search_Utils.h"

class Node_Partitions
{
	private:
		const size_t m_uiNumRobots;
		std::vector<std::pair<size_t, size_t>> m_rob_depo;
		std::vector<Node_Set> m_vec_local_nodes;
		std::unordered_map<size_t, Local_Robots> m_map_common_nodes;
		std::unordered_map<Or_Pair, Node_Set , Or_Pair_Hasher> m_map_pair_rob_com_nodes;
		void populate_containers(const Layout_LS &graph);

	public:
		inline bool doContain_common_nodes(size_t uiRobot1, size_t uiRobot2) const { return m_map_pair_rob_com_nodes.find(Or_Pair(uiRobot1, uiRobot2)) == m_map_pair_rob_com_nodes.end() ? false : true; };
		inline const std::unordered_set<size_t>& get_common_nodes(size_t uiRobot1, size_t uiRobot2) const { return m_map_pair_rob_com_nodes.at(Or_Pair(uiRobot1, uiRobot2)).set; };
		bool isLocalVertex(size_t uiInd) const;  // is local the hole can be accessed by only one robot
		inline const Local_Robots& get_common_robots_for_Ind(size_t uiInd) const { return m_map_common_nodes.at(uiInd); };
		Node_Partitions(const Layout_LS &graph);
		friend class Random_LS_Oper;
		friend class Local_Search;
};

#endif
