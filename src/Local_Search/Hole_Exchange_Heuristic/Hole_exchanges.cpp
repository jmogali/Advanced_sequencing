#include "Hole_exchanges.h"

Hole_Exchange::Hole_Exchange(size_t uiNumRobots) : m_uiNumRobots(uiNumRobots)
{}

void Hole_Exchange::clear_prev_info()
{
	m_vec_state_path.clear();
}

void Hole_Exchange::construct_state_transition_path(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch)
{
	std::vector<size_t> vec_rob_index;
	vec_rob_index.resize(m_uiNumRobots, 0);
	size_t uiNextTime;
	bool bEnd = false;

	while (!bEnd)
	{
		m_vec_state_path.emplace_back(State_pos(m_uiNumRobots));
		State_pos &newState = m_vec_state_path[m_vec_state_path.size() - 1];
		uiNextTime = std::numeric_limits<size_t>::max();

		for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
		{
			newState.m_vec_rob_vtx[uiRobot] = full_rob_sch[uiRobot][vec_rob_index[uiRobot]].m_uiInd;
			if (vec_rob_index[uiRobot] == full_rob_sch[uiRobot].size()-1) continue;
			uiNextTime = std::min(uiNextTime , full_rob_sch[uiRobot][vec_rob_index[uiRobot]].m_uiEnd);
		}

		bEnd = true;
		for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
		{
			if (vec_rob_index[uiRobot] == full_rob_sch[uiRobot].size() - 1) continue;
			else bEnd = false;

			if (full_rob_sch[uiRobot][vec_rob_index[uiRobot]].m_uiEnd == uiNextTime)
			{
				vec_rob_index[uiRobot] = vec_rob_index[uiRobot] + 1;
#ifdef WINDOWS
				assert(uiNextTime == full_rob_sch[uiRobot][vec_rob_index[uiRobot]].m_uiStart);
#else
				if (uiNextTime != full_rob_sch[uiRobot][vec_rob_index[uiRobot]].m_uiStart)
				{
					cout << "Computation of state transitions is incorrect \n";
					exit(-1);
				}
#endif
			}
		}		
	}
}

void Hole_Exchange::perform_heuristic_moves(const std::vector<std::list<size_t>> &rob_seq, const Alternative_Graph &alt_graph, const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch)
{
	assert(rob_seq.size() == m_uiNumRobots);
	clear_prev_info();
	bool bFeasible = construct_graph_populate_order(alt_graph, m_uiNumRobots);
	assert(true == bFeasible); // we will assume that we are always given a feasible sequence as input
	construct_state_transition_path(full_rob_sch);

}