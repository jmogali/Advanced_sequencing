#include "Layout_Graph.h"

Layout_Graph::Layout_Graph(size_t uiNumRobots, size_t uiNumHoles):Layout(uiNumRobots, uiNumHoles)
{
	m_in_edge.resize(uiNumRobots);
	m_edge_inter_vert.resize(uiNumRobots);
	m_vec_set_enablers.resize((2 * uiNumRobots) + uiNumHoles);
	m_map_IV_hole_pair.resize(uiNumRobots);
}

void Layout_Graph::finish_construction()
{
	append_node_info();	
}

void Layout_Graph::append_node_info()
{
	append_depot_info();
	append_hole_info();
	append_iv_info();
}

void Layout_Graph::append_depot_info()
{
	for (auto it = m_vec_V_D.begin(); it != m_vec_V_D.end(); it++)
	{
		auto it_insert = m_map_node_info.emplace(it->second.getFromInd(), std::make_tuple("D",it->second.getTime(), it->second.getLoc()));
		assert(true == it_insert.second);

		it_insert = m_map_node_info.emplace(it->second.getToInd(), std::make_tuple("D", it->second.getTime(), it->second.getLoc()));
		assert(true == it_insert.second);
	}
}

void Layout_Graph::append_hole_info()
{
	for (auto it = m_map_V_H.begin(); it != m_map_V_H.end(); it++)
	{
		auto it_insert = m_map_node_info.emplace(it->first, std::make_tuple("H" , it->second.getTime(), it->second.getLoc()));
		assert(true == it_insert.second);
	}
}

void Layout_Graph::append_iv_info()
{
	for (auto it_r = m_edge_inter_vert.begin(); it_r != m_edge_inter_vert.end(); it_r++)
	{
		for (auto it_1 = it_r->map.begin(); it_1 != it_r->map.end(); it_1++)
		{
			for (auto it_2 = it_1->second.map.begin(); it_2 != it_1->second.map.end(); it_2++)
			{
				for (auto it_iv = it_2->second.vec.begin(); it_iv != it_2->second.vec.end(); it_iv++)
				{
					auto it_insert = m_map_node_info.emplace(it_iv->getInd(), std::make_tuple("IV", it_iv->getTime(), Coordinates(-1, -1, -1))); //dummy initialization for location
					assert(true == it_insert.second);
				}
			}
		}
	}
}

bool Layout_Graph::areColliding(Coll_Pair coll_pair) const
{
	auto pr1 = coll_pair.getPair1();
	auto pr2 = coll_pair.getPair2();

	std::string strType1 = std::get<0>(m_map_node_info.at(pr1.getInd1()));
	size_t uiInd1 = pr1.getInd1();
	size_t uiRobot1 = pr1.getInd2();

	std::string strType2 = std::get<0>(m_map_node_info.at(pr2.getInd1()));
	size_t uiInd2 = pr2.getInd1();
	size_t uiRobot2 = pr2.getInd2();

	if (("IV" != strType1) && ("IV" != strType2)) return m_conf_map.find(coll_pair) != m_conf_map.end() ? true:false;
	else if (("IV" == strType1) && ("IV" != strType2))
	{
		auto pr = m_map_IV_hole_pair[uiRobot1].at(uiInd1);	//hole pair
		if (m_conf_map.find(Coll_Pair(pr.getHole1(), uiRobot1, uiInd2, uiRobot2)) != m_conf_map.end()) return true;
		else if (m_conf_map.find(Coll_Pair(pr.getHole2(), uiRobot1, uiInd2, uiRobot2)) != m_conf_map.end()) return true;
		else return false;
	}
	else if (("IV" != strType1) && ("IV" == strType2))
	{
		auto pr = m_map_IV_hole_pair[uiRobot2].at(uiInd2);	//hole pair
		if (m_conf_map.find(Coll_Pair(uiInd1, uiRobot1, pr.getHole1(), uiRobot2)) != m_conf_map.end()) return true;
		else if (m_conf_map.find(Coll_Pair(uiInd1, uiRobot1, pr.getHole2(), uiRobot2)) != m_conf_map.end()) return true;
		else return false;
	}
	return false;
}