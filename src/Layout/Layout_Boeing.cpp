#include "Layout_Boeing.h"
#include <assert.h>

Layout_Boeing::Layout_Boeing(size_t uiNumRobots , size_t uiNumHoles) : Layout_Graph(uiNumRobots , uiNumHoles)
{ }

void Layout_Boeing::add_depot(size_t uiRobot, size_t uiTime, size_t uiFromDepotInd , size_t uiToDepotInd, const Coordinates &loc)
{
	auto it_insert = m_vec_V_D.emplace(uiRobot, Depo_Desc(uiTime, uiFromDepotInd, uiToDepotInd , loc));
	assert(true == it_insert.second);
}

void Layout_Boeing::add_hole(size_t uiHoleInd, size_t uiTime, const Coordinates &loc)
{
	auto it_insert = m_map_V_H.emplace(uiHoleInd, Node_Desc(uiTime , loc));
	assert(true == it_insert.second);
}

void Layout_Boeing::add_edge(size_t uiRobot, size_t uiSvtx, size_t uiDvtx, size_t uiVal)
{
	m_in_edge[uiRobot].map[uiDvtx].map[uiSvtx] = uiVal;	
}

void Layout_Boeing::add_iv(size_t uiRobot, size_t uiSvtx, size_t uiDvtx, size_t uiIndex, size_t uiVal)
{
	m_edge_inter_vert[uiRobot].map[uiSvtx].map[uiDvtx].vec.push_back(IV_Ind(uiIndex, uiVal));	
}

void Layout_Boeing::add_conflict(N_Ind s1, R_Ind r1, N_Ind s2, R_Ind r2)
{
	auto it_insert = m_conf_map.emplace(Coll_Pair(s1, r1, s2, r2));
	assert(true == it_insert.second);
}

void Layout_Boeing::add_enabler(N_Ind src, N_Ind neigh)
{
	auto it_insert = m_vec_set_enablers[src.getInd()].set.emplace(neigh);
	assert(true == it_insert.second);
}