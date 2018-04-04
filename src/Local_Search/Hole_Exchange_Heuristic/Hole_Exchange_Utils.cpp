#include "Hole_exchanges.h"

void Hole_Exchange::construct_state_transition_path(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch)
{
	std::vector<size_t> vec_rob_index;
	vec_rob_index.resize(m_uiNumRobots, 0);
	size_t uiNextTime;
	bool bEnd = false;

	while (!bEnd)
	{
		m_vec_state_path.emplace_back(State_vtx(m_uiNumRobots));
		State_vtx &newState = m_vec_state_path[m_vec_state_path.size() - 1];
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

void Hole_Exchange::construct_rob_sub_sequences(std::vector<std::list<size_t>> &rob_sub_seq, const size_t c_uiLeft, const size_t c_uiRight, const std::vector<std::list<size_t>> &rob_seq, const std::vector<State_vtx> &vec_state_path
												, std::vector<std::tuple<std::list<size_t>::const_iterator, std::list<size_t>::const_iterator, size_t>> &vec_start_end_itr_start_pos, std::unordered_set<size_t> &set_comp_verts)
{
	size_t uiStartVtx, uiEndVtx;
	rob_sub_seq.clear();
	rob_sub_seq.resize(m_uiNumRobots);
	assert(0 == set_comp_verts.size());

	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		uiStartVtx = vec_state_path[c_uiLeft].m_vec_rob_vtx[uiRobot];
		uiEndVtx = vec_state_path[c_uiRight].m_vec_rob_vtx[uiRobot];
		bool bAdd = false;
		size_t uiStartPos = 0;

		for (auto it = rob_seq[uiRobot].cbegin(); it != rob_seq[uiRobot].cend(); it++, uiStartPos++)
		{
			if (*it == uiStartVtx)
			{
				bAdd = true;
				vec_start_end_itr_start_pos.emplace_back(std::make_tuple(it, rob_seq[uiRobot].end(), uiStartPos));
			}

			if (bAdd) rob_sub_seq[uiRobot].emplace_back(*it);
			else
			{
				if("IV" != m_graph.getType(*it)) set_comp_verts.emplace(*it); // we are not storing "IV" here
			}

			if (*it == uiEndVtx)
			{
				std::get<1>(vec_start_end_itr_start_pos[uiRobot]) = it;
				break;
			}
		}
	}
}

void Hole_Exchange::construct_rob_sub_sequences_with_iterators(std::vector<std::list<size_t>> &rob_sub_seq, const size_t c_uiLeft, const size_t c_uiRight, const std::vector<State_vtx> &vec_state_path
	, std::vector<std::tuple<std::list<size_t>::const_iterator, std::list<size_t>::const_iterator, size_t>> &vec_start_end_itr_start_pos, std::unordered_set<size_t> &set_comp_verts)

{
	size_t uiStartVtx, uiEndVtx;
	rob_sub_seq.clear();
	rob_sub_seq.resize(m_uiNumRobots);
	
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		uiStartVtx = vec_state_path[c_uiLeft].m_vec_rob_vtx[uiRobot];
		uiEndVtx = vec_state_path[c_uiRight].m_vec_rob_vtx[uiRobot];
		
		while (1)
		{
			set_comp_verts.erase(*(std::get<0>(vec_start_end_itr_start_pos[uiRobot])));
			
			if (*rob_sub_seq[uiRobot].begin() != *(std::get<0>(vec_start_end_itr_start_pos[uiRobot])))
			{
				rob_sub_seq[uiRobot].emplace_front(*(std::get<0>(vec_start_end_itr_start_pos[uiRobot])));
			}

			if (uiStartVtx == *(std::get<0>(vec_start_end_itr_start_pos[uiRobot]))) break;
			std::get<0>(vec_start_end_itr_start_pos[uiRobot])--;
			std::get<2>(vec_start_end_itr_start_pos[uiRobot])--;
		}

		while (uiEndVtx != *(std::get<1>(vec_start_end_itr_start_pos[uiRobot])))
		{
			std::get<1>(vec_start_end_itr_start_pos[uiRobot])++;
			rob_sub_seq[uiRobot].emplace_back(*(std::get<1>(vec_start_end_itr_start_pos[uiRobot])));
		}		
	}
}

void Hole_Exchange::compute_enabled_holes_for_rob_sub_seq(const std::vector<std::list<size_t>> &rob_sub_seq, const std::unordered_set<size_t> &set_comp_verts, std::unordered_set<size_t> set_enabled_holes)
{
	set_enabled_holes.clear();
	const auto &vec_Enablers = m_graph.get_Enablers();

	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		for (auto it = rob_sub_seq[uiRobot].begin(); it != rob_sub_seq[uiRobot].end(); it++)
		{
			if ("H" != m_graph.getType(*it)) continue;
			bool bEnabled = false;

			for (auto it_enablers = vec_Enablers.at(*it).set.cbegin(); it_enablers != vec_Enablers.at(*it).set.cend(); it_enablers++)
			{
				if (set_comp_verts.end() != set_comp_verts.find(it_enablers->getInd()))
				{
					bEnabled = true;
					break;
				}
			}

			if (bEnabled) set_enabled_holes.emplace(*it);
		}
	}
}