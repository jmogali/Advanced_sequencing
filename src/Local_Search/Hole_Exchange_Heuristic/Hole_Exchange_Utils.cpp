#include "Hole_exchanges.h"

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
			if (vec_rob_index[uiRobot] == full_rob_sch[uiRobot].size() - 1) continue;
			uiNextTime = std::min(uiNextTime, full_rob_sch[uiRobot][vec_rob_index[uiRobot]].m_uiEnd);
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

void Hole_Exchange::construct_rob_sub_sequences(std::vector<std::list<size_t>> &rob_sub_seq, const size_t c_uiLeft, const size_t c_uiRight, const std::vector<std::list<size_t>> &rob_seq, const std::vector<State_pos> &vec_state_path
												, std::vector<std::pair<std::list<size_t>::const_iterator, std::list<size_t>::const_iterator>> &vec_start_end_itr, std::unordered_set<size_t> &set_comp_verts)
{
	size_t uiStartHole, uiEndHole;
	rob_sub_seq.clear();
	rob_sub_seq.resize(m_uiNumRobots);
	assert(0 == set_comp_verts.size());

	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		uiStartHole = vec_state_path[c_uiLeft].m_vec_rob_vtx[uiRobot];
		uiEndHole = vec_state_path[c_uiRight].m_vec_rob_vtx[uiRobot];
		bool bAdd = false;

		for (auto it = rob_seq[uiRobot].cbegin(); it != rob_seq[uiRobot].cend(); it++)
		{
			if (*it == uiStartHole)
			{
				bAdd = true;
				vec_start_end_itr.emplace_back(std::make_pair(it, rob_seq[uiRobot].end()));
			}

			if (bAdd) rob_sub_seq[uiRobot].emplace_back(*it);
			else
			{
				if("IV" != m_graph.getType(*it)) set_comp_verts.emplace(*it); // we are not storing "IV" here
			}

			if (*it == uiEndHole)
			{
				vec_start_end_itr[uiRobot].second = it;
				break;
			}
		}
	}
}

void Hole_Exchange::construct_rob_sub_sequences_with_iterators(std::vector<std::list<size_t>> &rob_sub_seq, const size_t c_uiLeft, const size_t c_uiRight, const std::vector<std::list<size_t>> &rob_seq, const std::vector<State_pos> &vec_state_path
	, std::vector<std::pair<std::list<size_t>::const_iterator, std::list<size_t>::const_iterator>> &vec_start_end_itr, std::unordered_set<size_t> &set_comp_verts)

{
	size_t uiStartHole, uiEndHole, uiErase;
	rob_sub_seq.clear();
	rob_sub_seq.resize(m_uiNumRobots);
	
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		uiStartHole = vec_state_path[c_uiLeft].m_vec_rob_vtx[uiRobot];
		uiEndHole = vec_state_path[c_uiRight].m_vec_rob_vtx[uiRobot];
		
		while (uiStartHole != *vec_start_end_itr[uiRobot].first)
		{
			uiErase = set_comp_verts.erase(*vec_start_end_itr[uiRobot].first);
#ifdef WINDOWS
			assert(1 == uiErase);
#else
			if (1 != uiErase)
			{
				cout << "Mistake in updation of completed vertices\n";
				exit(-1);
			}
#endif
			vec_start_end_itr[uiRobot].first--;
		}

		while (uiEndHole != *vec_start_end_itr[uiRobot].second)
		{
			vec_start_end_itr[uiRobot].second++;
		}

		auto it = vec_start_end_itr[uiRobot].first;
		while(1)
		{
			rob_sub_seq[uiRobot].emplace_back(*it);
			if (it == vec_start_end_itr[uiRobot].second) break;
			it++;
		}
	}
}