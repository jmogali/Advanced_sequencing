#include "Node_Partitions.h"

Node_Partitions::Node_Partitions(const Layout_LS &graph) : m_uiNumRobots(graph.get_num_robots())
{
	m_vec_local_nodes.resize(graph.get_num_robots());
	populate_containers(graph);
}

void Node_Partitions::populate_containers(const Layout_LS &graph)
{
	const auto &vec_rob_iv = graph.get_IV_Vec();
	const auto &map_depo = graph.getDepotMap();

	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		for (auto it = vec_rob_iv[uiRobot].map.begin(); it != vec_rob_iv[uiRobot].map.end(); it++)
		{	
			if (it->first.getInd() == map_depo.at(uiRobot).getFromInd()) continue;
			m_map_common_nodes[it->first.getInd()].set.emplace(uiRobot);
		}
		m_rob_depo.push_back(std::make_pair(map_depo.at(uiRobot).getFromInd(), map_depo.at(uiRobot).getToInd()));
	}

	for (auto it = m_map_common_nodes.begin(); it != m_map_common_nodes.end(); )
	{
		if (1 == it->second.set.size())
		{
			size_t uiRobot = *(it->second.set.begin());
			m_vec_local_nodes[uiRobot].set.emplace(it->first);
			it = m_map_common_nodes.erase(it);
		}
		else
			it++;		
	}

	for (auto it = m_map_common_nodes.begin(); it != m_map_common_nodes.end(); it++)
	{
		for (auto it1 = it->second.set.begin(); it1 != it->second.set.end(); it1++)
		{
			auto it2 = it1;
			std::advance(it2, 1);
			
			for (; it2 != it->second.set.end(); it2++)
			{
				m_map_pair_rob_com_nodes[Or_Pair(*it1, *it2)].set.emplace(it->first);
			}
		}
	}
}





