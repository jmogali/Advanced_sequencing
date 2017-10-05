#pragma once
#ifndef LAYOUT_H
#define LAYOUT_H

//Contains the pure graphical representation of the problem
#include "Data_Constants.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include "Layout_utils.h"

class Layout
{
	private:		
		
	protected:
		size_t m_uiNumRobots;
		size_t m_uiNumHoles;
		std::unordered_map<N_Ind , Node_Desc, IndHasher> m_map_V_H;
		std::unordered_map<R_Ind, Depo_Desc, IndHasher> m_vec_V_D;
		std::vector<V_V> m_in_edge;
		std::vector<V_I_V> m_edge_inter_vert;
		std::vector<N_EN> m_vec_set_enablers;
		std::unordered_set<Coll_Pair, CollHasher> m_conf_map;
		
	public:
		Layout(size_t uiNumRobots , size_t uiNumHoles);
		size_t inline get_num_robots() const { return m_uiNumRobots; };
		size_t inline get_num_holes() const { return m_uiNumHoles; };
};

#endif
