#pragma once
#ifndef LAYOUT_GRAPH_H
#define LAYOUT_GRAPH_H

#include "Layout.h"

class Layout_Graph : public Layout
{
	protected:
		std::unordered_map<N_Ind, std::tuple<std::string, size_t, Coordinates>, IndHasher> m_map_node_info;
		std::vector<std::unordered_map<N_Ind, IV_Hole_Pair, IndHasher>> m_map_IV_hole_pair;	// vector for robots, N_Ind-: IV vertex, pair-: hole1, hole2
		void append_node_info();
		void append_depot_info();
		void append_hole_info();
		void append_iv_info();
		inline const std::unordered_set<Coll_Pair, CollHasher>& getHoleCollMap() const { return m_conf_map; };
		
	public:
		inline const std::unordered_map<N_Ind, Node_Desc, IndHasher>& getHolesMap() const { return m_map_V_H; };
		inline const std::unordered_map<R_Ind, Depo_Desc, IndHasher>& getDepotMap() const { return m_vec_V_D; };
		inline const std::vector<V_I_V>& get_IV_Vec() const { return m_edge_inter_vert; };
		inline const std::vector<N_EN>& get_Enablers() const { return m_vec_set_enablers; };
		inline const std::tuple<std::string, size_t, Coordinates> getNodeInfo(N_Ind Ind) const { return m_map_node_info.at(Ind); };
		Layout_Graph(size_t uiNumRobots, size_t uiNumHoles);
		void finish_construction();
		bool areColliding(Coll_Pair coll_pair) const;
		friend class Data_Parser;
		friend class Data_Generator;
};

#endif
