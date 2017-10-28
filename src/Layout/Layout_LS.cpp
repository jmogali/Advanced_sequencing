#include "Layout_LS.h"

Layout_LS::Layout_LS(size_t uiNumRobots, size_t uiNumHoles): Layout_Graph(uiNumRobots, uiNumHoles)
{}

size_t Layout_LS::getTime(N_Ind Ind) const
{
	return std::get<1>(m_map_node_info.at(Ind));
}

std::string Layout_LS::getType(N_Ind Ind) const
{
	return std::get<0>(m_map_node_info.at(Ind));
}

Coordinates Layout_LS::getLoc(N_Ind Ind) const
{
	assert("IV" != getType(Ind));
	return std::get<2>(m_map_node_info.at(Ind));
}

size_t Layout_LS::getEdgeDist(R_Ind Robot_Ind, N_Ind Ind1, N_Ind Ind2) const
{
	return m_in_edge.at(Robot_Ind.getInd()).map.at(Ind2).map.at(Ind1);
}

bool Layout_LS::doesEdgeExist(R_Ind Robot_Ind, N_Ind Ind1, N_Ind Ind2) const
{
	size_t uiRobot = Robot_Ind.getInd();
	auto it = m_in_edge.at(uiRobot).map.find(Ind2);
	if (it != m_in_edge.at(uiRobot).map.end())
	{
		if (it->second.map.find(Ind1) == it->second.map.end())
		{
			return false;
		}
	}
	else
	{
		return false;
	}

	return true;
}

void Layout_LS::get_nearest_robots_for_hole(N_Ind Ind, std::unordered_set<size_t> &set_near_robots) const
{
	assert("H" == getType(Ind));
	auto loc = getLoc(Ind);
	double dDist, dMinDist = std::numeric_limits<double>::max();
	
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		dDist = loc.getDist_XY(m_vec_V_D.at(uiRobot).getLoc());
		if (dMinDist > dDist)
		{
			set_near_robots.clear();
			set_near_robots.emplace(uiRobot);
			dMinDist = dDist;
		}
		else if (dMinDist == dDist)
		{
			set_near_robots.emplace(uiRobot);
		}
	}	
}


