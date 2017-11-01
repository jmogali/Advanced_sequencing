#include "Local_Search.h"

bool Local_Search::check_validity_of_schedule(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch)
{
	return true;
}


bool Local_Search::check_validity_of_sequence(const std::vector<std::list<size_t>> &rob_seq)
{
	size_t uiNumHoles = 0;
	std::unordered_set<size_t> set_seen_verts;
	for (size_t uiRobot = 0; uiRobot < m_node_data.m_uiNumRobots; uiRobot++)
	{
		auto it2 = rob_seq[uiRobot].begin();
		if (*it2 != m_node_data.m_rob_depo.at(uiRobot).first)
		{
			cout << "SEQUENCE_ERROR: Starting depot wrong for Robot: " << uiRobot << endl;
			return false;
		}
		it2++;
		for (auto it1 = rob_seq[uiRobot].begin(); it1 != rob_seq[uiRobot].end(); it1++)
		{
#ifdef WINDOWS			
			assert(set_seen_verts.end() == set_seen_verts.find(*it1));
#else
			if (set_seen_verts.end() != set_seen_verts.find(*it1))
			{
				cout << "Duplicate vertices \n";
				return false;
			}
#endif
			set_seen_verts.emplace(*it1);
			if (it2 == rob_seq[uiRobot].end())
			{
				if (*it1 != m_node_data.m_rob_depo.at(uiRobot).second)
				{
					cout << "SEQUENCE_ERROR: Ending depot wrong for Robot: " << uiRobot << endl;
					return false;
				}
				break;
			}
			if (false == m_graph.doesEdgeExist(uiRobot, *it1, *it2))
				return false;
			it2++;
		}
		uiNumHoles += rob_seq[uiRobot].size();
	}

	assert(set_seen_verts.size() == m_graph.get_num_holes() + (2 * m_graph.get_num_robots()));

	if (uiNumHoles != m_graph.get_num_holes() + (2 * m_graph.get_num_robots()))
	{
		cout << "SEQUENCE_ERROR: Holes mismatch \n";
		return false;
	}
	return true;
}