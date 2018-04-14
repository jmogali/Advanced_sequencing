#include "Hole_exchanges.h"

void Hole_Exchange::compute_start_completion_times_from_schedule()
{
	m_map_start_times.clear();
	m_map_completion_times.clear();

	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		for (auto it = m_vec_full_rob_sch[uiRobot].begin(); it != m_vec_full_rob_sch[uiRobot].end(); it++)
		{
			m_map_start_times.emplace(it->m_uiInd, it->m_uiStart);
			m_map_completion_times.emplace(it->m_uiInd, it->m_uiEnd);
		}
	}
}

void Hole_Exchange::compute_completion_times()
{
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		auto it_next = m_rob_seq[uiRobot].begin();
		for (auto it = m_rob_seq[uiRobot].begin(); it != m_rob_seq[uiRobot].end(); it++ , it_next++)
		{
			//note for last vertex we assume that the processing time is 0, even if that is not the case
			if (it_next == m_rob_seq[uiRobot].end()) 
			{
				m_map_completion_times.emplace(*it, m_map_start_times.at(*it));
				break;
			}

			m_map_completion_times.emplace(*it, m_map_start_times.at(*it_next));			
		}
	}
}

//assumes that m_rob_seq has been updated
void Hole_Exchange::construct_vertex_schedule()
{
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++) m_vec_full_rob_sch[uiRobot].clear();
	size_t uiStart, uiEnd, uiWait;
	
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		auto it1 = m_rob_seq[uiRobot].begin();
		auto it2 = it1;
		it2++;
		uiStart = m_map_start_times.at(*it1);
		for (; it2 != m_rob_seq[uiRobot].end(); it2++)
		{
			uiEnd = m_map_start_times.at(*it2);
			assert(uiEnd == m_map_completion_times.at(*it1));
			uiWait = uiEnd - m_graph.getTime(*it1) - uiStart;
			assert((int)uiEnd - (int)m_graph.getTime(*it1) - (int)uiStart >= 0);
			m_vec_full_rob_sch[uiRobot].emplace_back(*it1, uiStart, uiEnd, uiWait);
			uiStart = uiEnd;
			it1++;
		}
		m_vec_full_rob_sch[uiRobot].emplace_back(*it1, uiStart, uiStart, 0);
		assert(uiStart  == m_map_completion_times.at(*it1));
	}
}

void Hole_Exchange::construct_state_transition_path()
{
	m_vec_state_path.clear();
	construct_state_transition_path(m_vec_full_rob_sch);
}

void Hole_Exchange::construct_state_transition_path(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch)
{
	assert(0 == m_vec_state_path.size());
	std::vector<size_t> vec_rob_pos_index;
	vec_rob_pos_index.resize(m_uiNumRobots, 0);
	size_t uiNextTime, uiCurrTime = std::numeric_limits<size_t>::min();
	bool bEnd = false;

	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		uiCurrTime = std::max(uiCurrTime , full_rob_sch[uiRobot][0].m_uiStart);
	}

	while (1)
	{
		m_vec_state_path.emplace_back(State_vtx_time(m_uiNumRobots));
		State_vtx_time &newState = m_vec_state_path[m_vec_state_path.size() - 1];
		uiNextTime = std::numeric_limits<size_t>::max();

		bEnd = true;
		for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
		{
			newState.m_vec_rob_vtx[uiRobot] = full_rob_sch[uiRobot][vec_rob_pos_index[uiRobot]].m_uiInd;

			if (vec_rob_pos_index[uiRobot] == full_rob_sch[uiRobot].size() - 1) continue;
			else bEnd = false;
			
			uiNextTime = std::min(uiNextTime, full_rob_sch[uiRobot][vec_rob_pos_index[uiRobot]].m_uiEnd);
		}
		newState.m_uiTime = uiCurrTime;
		if (bEnd) break;

		for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
		{
			if (vec_rob_pos_index[uiRobot] == full_rob_sch[uiRobot].size() - 1) continue;

			if (full_rob_sch[uiRobot][vec_rob_pos_index[uiRobot]].m_uiEnd == uiNextTime)
			{
				vec_rob_pos_index[uiRobot] = vec_rob_pos_index[uiRobot] + 1;
#ifdef WINDOWS
				assert(uiNextTime == full_rob_sch[uiRobot][vec_rob_pos_index[uiRobot]].m_uiStart);
#else
				if (uiNextTime != full_rob_sch[uiRobot][vec_rob_pos_index[uiRobot]].m_uiStart)
				{
					cout << "Computation of state transitions is incorrect \n";
					exit(-1);
				}
#endif
			}
		}
		uiCurrTime = uiNextTime;
	}
}

void Hole_Exchange::construct_rob_sub_sequences(std::vector<std::list<size_t>> &rob_sub_seq, const size_t c_uiLeft, const size_t c_uiRight, const std::vector<std::list<size_t>> &rob_seq, const std::vector<State_vtx_time> &vec_state_path,
												std::vector<std::tuple<std::list<size_t>::const_iterator, std::list<size_t>::const_iterator, size_t>> &vec_start_end_itr_start_pos, std::unordered_set<size_t> &set_comp_HD)
{
	size_t uiStartVtx, uiEndVtx;
	assert(0 == set_comp_HD.size());
	
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++) assert(0 == rob_sub_seq[uiRobot].size());

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
				if("IV" != m_graph.getType(*it)) set_comp_HD.emplace(*it); // we are not storing "IV" here
			}

			if (*it == uiEndVtx)
			{
				std::get<1>(vec_start_end_itr_start_pos[uiRobot]) = it;
				break;
			}
		}
	}
}

void Hole_Exchange::construct_rob_sub_sequences_with_iterators(std::vector<std::list<size_t>> &rob_sub_seq, const size_t c_uiLeft, const size_t c_uiRight, const std::vector<State_vtx_time> &vec_state_path,
	std::vector<std::tuple<std::list<size_t>::const_iterator, std::list<size_t>::const_iterator, size_t>> &vec_start_end_itr_start_pos, std::unordered_set<size_t> &set_comp_HD)
{
	size_t uiStartVtx, uiEndVtx;
	
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		uiStartVtx = vec_state_path[c_uiLeft].m_vec_rob_vtx[uiRobot];
		uiEndVtx = vec_state_path[c_uiRight].m_vec_rob_vtx[uiRobot];
		
		while (1)
		{
			set_comp_HD.erase(*(std::get<0>(vec_start_end_itr_start_pos[uiRobot])));

			if (uiStartVtx == *(std::get<0>(vec_start_end_itr_start_pos[uiRobot])))
			{
				if (*rob_sub_seq[uiRobot].begin() != uiStartVtx) rob_sub_seq[uiRobot].emplace_front(uiStartVtx);
				break;
			}
			else rob_sub_seq[uiRobot].emplace_front(*(std::get<0>(vec_start_end_itr_start_pos[uiRobot])));

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

// in this function, remember that we are always iterating over all vertices in rob_sub_seq to compute which ones need to be enabled.
// the logic has to remain that way despite the computational inefficiency
void Hole_Exchange::compute_enabled_holes_for_rob_sub_seq(const std::vector<std::list<size_t>> &rob_sub_seq, const std::unordered_set<size_t> &set_comp_HD, std::unordered_set<size_t> &set_enabled_holes)
{
	const auto &vec_Enablers = m_graph.get_Enablers();

	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		for (auto it = rob_sub_seq[uiRobot].begin(); it != rob_sub_seq[uiRobot].end(); it++)
		{
			if ("H" != m_graph.getType(*it)) continue;

			//we also need to add start holes as enabled
			if (rob_sub_seq[uiRobot].begin() == it)
			{
				set_enabled_holes.emplace(*it);
				continue;
			}

			bool bEnabled = false;

			for (auto it_enablers = vec_Enablers.at(*it).set.cbegin(); it_enablers != vec_Enablers.at(*it).set.cend(); it_enablers++)
			{
				if (set_comp_HD.end() != set_comp_HD.find(it_enablers->getInd()))
				{
					bEnabled = true;
					break;
				}
			}

			if (bEnabled) set_enabled_holes.emplace(*it);
		}
	}
}

// some effeciency through timing variables
bool check_if_vtx_already_exists_on_critical_path(size_t uiPred, size_t uiPredTime, const std::list<size_t> &critical_path, const std::list<size_t> &critical_path_times)
{	
	auto it_cpathtime = critical_path_times.begin();
	for (auto it_cpath = critical_path.begin(); it_cpath != critical_path.end(); it_cpath++, it_cpathtime++)
	{
		if (*it_cpathtime > uiPredTime) break;
		if (*it_cpath == uiPred) return true;		
	}		
	return false;
}


void add_vtx_to_critical_path_time(size_t uiVtx, size_t uiTime, std::list<size_t> &critical_path, std::list<size_t> &critical_path_times)
{
	critical_path.push_front(uiVtx);
	critical_path_times.push_front(uiTime);
}

void Hole_Exchange::compute_critical_path(std::list<size_t> &critical_path)
{
	assert(0 == critical_path.size());
	std::list<size_t> critical_path_times;
	size_t uiVtx, uiPred, c_uiMakespan = std::numeric_limits<size_t>::min(), uiPredTime, uiCurrTime;

	//compute last vertex
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		size_t uiTime = m_map_completion_times.at(*m_rob_seq[uiRobot].rbegin());
		if (c_uiMakespan < uiTime)
		{
			uiVtx = *m_rob_seq[uiRobot].rbegin();
			c_uiMakespan = uiTime;					
		}
	}
	add_vtx_to_critical_path_time(uiVtx, m_map_start_times.at(uiVtx), critical_path, critical_path_times);
	
	while (1)
	{
		//note uiVtx would have already been added in the critical path and time
		if (0 == m_alt_in_graph.at(uiVtx).size())
		{
			assert(0 == m_map_start_times.at(uiVtx));
			break;
		}

		uiCurrTime = m_map_start_times.at(uiVtx);

		std::vector<size_t> vec_pred_0_vtx; //stores 0 tight predecessors i.e. uiPredTime = uiCurrTime
		bool bTightPredecessor = false;

		for (auto it_pred = m_alt_in_graph.at(uiVtx).begin(); it_pred != m_alt_in_graph.at(uiVtx).end(); it_pred++)
		{
			uiPred = it_pred->first;
			uiPredTime = m_map_start_times.at(uiPred);
			if (uiPredTime + it_pred->second == uiCurrTime)
			{
				//select predecessor, if there exists a tight predecessor that is not same start time, then choose that
				if (uiPredTime < uiCurrTime) 
				{
					add_vtx_to_critical_path_time(uiPred, uiPredTime, critical_path, critical_path_times);
					uiVtx = uiPred;
					bTightPredecessor = true;
					break;
				}
				else vec_pred_0_vtx.push_back(uiPred); // uiCurrTime == uiPredTime cases
			}
		}

		// here we are looking 0 predecessor, i.e. uiPredTime == uiCurrTime
		if (false == bTightPredecessor)
		{		
			bool bFound;
			for (size_t uiCount = 0; uiCount < vec_pred_0_vtx.size(); uiCount++)
			{
				bFound = check_if_vtx_already_exists_on_critical_path(vec_pred_0_vtx[uiCount], uiCurrTime, critical_path, critical_path_times);

				if (false == bFound)
				{
					add_vtx_to_critical_path_time(vec_pred_0_vtx[uiCount], uiCurrTime, critical_path, critical_path_times);
					uiVtx = vec_pred_0_vtx[uiCount];
					break;
				}
			}
#ifdef WINDOWS
			assert(false == bFound); // we are expecting to find atleast predecessor
#else
			if (false != bFound)
			{
				cout << "we are expecting to find atleast predecessor \n";
				exit(-1);
			}
#endif
		}		
	}
}

void Hole_Exchange::gather_vertices_before_sub_seq_after(std::set<size_t> &set_vts_before_sub_seq, std::set<size_t> &set_vts_sub_seq, std::set<size_t> &set_vts_after_sub_seq, const std::vector<std::list<size_t>> &rob_sub_seq)
{
	size_t uiVert;
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		bool bBefore = true, bSub_Seq = false, bAfter = false;
		for (auto it = m_rob_seq[uiRobot].begin(); it != m_rob_seq[uiRobot].end(); it++)
		{
			uiVert = *it;
			if (uiVert == *rob_sub_seq[uiRobot].begin())
			{
				bSub_Seq = true;
				bBefore = false;
			}
			
			if (true == bBefore) set_vts_before_sub_seq.emplace(uiVert);
			else if (true == bSub_Seq) set_vts_sub_seq.emplace(uiVert);
			else set_vts_after_sub_seq.emplace(uiVert);

			if (uiVert == *rob_sub_seq[uiRobot].rbegin())
			{
				bSub_Seq = false;
				bAfter = true;				
			}
		}	
	}	
}

void Hole_Exchange::remove_robo_hole_owner(const size_t c_uiHole)
{
	size_t uiErase = m_hole_rob_owner.erase(c_uiHole);
	assert(1 == uiErase);
}