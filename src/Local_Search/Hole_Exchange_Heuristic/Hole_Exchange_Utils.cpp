#include "Hole_exchanges.h"

void Hole_Exchange::construct_state_transition_path(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch)
{
	std::vector<size_t> vec_rob_index;
	vec_rob_index.resize(m_uiNumRobots, 0);
	size_t uiNextTime, uiCurrTime = std::numeric_limits<size_t>::min();
	bool bEnd = false;

	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		uiCurrTime = std::max(uiCurrTime , full_rob_sch[uiRobot][0].m_uiStart);
	}

	while (!bEnd)
	{
		m_vec_state_path.emplace_back(State_vtx_time(m_uiNumRobots));
		State_vtx_time &newState = m_vec_state_path[m_vec_state_path.size() - 1];
		uiNextTime = std::numeric_limits<size_t>::max();

		for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
		{
			newState.m_vec_rob_vtx[uiRobot] = full_rob_sch[uiRobot][vec_rob_index[uiRobot]].m_uiInd;
			if (vec_rob_index[uiRobot] == full_rob_sch[uiRobot].size() - 1) continue;
			uiNextTime = std::min(uiNextTime, full_rob_sch[uiRobot][vec_rob_index[uiRobot]].m_uiEnd);
		}
		newState.m_uiTime = uiCurrTime;

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
		uiCurrTime = uiNextTime;
	}
}

void Hole_Exchange::construct_rob_sub_sequences(std::vector<std::list<size_t>> &rob_sub_seq, const size_t c_uiLeft, const size_t c_uiRight, const std::vector<std::list<size_t>> &rob_seq, const std::vector<State_vtx_time> &vec_state_path
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

void Hole_Exchange::construct_rob_sub_sequences_with_iterators(std::vector<std::list<size_t>> &rob_sub_seq, const size_t c_uiLeft, const size_t c_uiRight, const std::vector<State_vtx_time> &vec_state_path
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

void update_queue(const size_t c_uiVtx, const size_t c_uiCost, std::list<size_t> &queue_open_list, std::unordered_map<size_t, size_t> &start_times)
{
	auto it_find = start_times.find(c_uiVtx);

	if (start_times.end() == it_find)
	{
		queue_open_list.push_back(c_uiVtx);
		start_times.emplace(c_uiVtx, c_uiCost);
	}
	else
	{
		if (it_find->second < c_uiCost)
		{
			it_find->second = c_uiCost;
			auto it_list = std::find(queue_open_list.begin(), queue_open_list.end(), c_uiVtx);
			if (queue_open_list.end() == it_list) queue_open_list.push_back(c_uiVtx);
		}
	}	
}

void compute_start_times(const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &out_graph, const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &in_graph, std::unordered_map<size_t, size_t> &start_times)   // <vtx, <out_vtx, arc cost>>
{
	assert(0 == start_times.size());
	size_t uiVtx;
	std::list<size_t> queue_open_list; // <vert, start time>

	//gather all 0 start vertices and set their start times to 0
	for (auto it = in_graph.begin(); it != in_graph.end(); it++)
	{
		if (0 == it->second.size())
		{
			start_times.emplace(it->first, 0);
			for (auto it_neigh = out_graph.at(it->first).begin(); it_neigh != out_graph.at(it->first).end(); it_neigh++)
			{
				update_queue(it_neigh->first, it_neigh->second, queue_open_list, start_times);
			}
		}
	}

	while (false == queue_open_list.empty())
	{
		uiVtx = queue_open_list.front();
		
		for (auto it_neigh = out_graph.at(uiVtx).begin(); it_neigh != out_graph.at(uiVtx).end(); it_neigh++)
		{
			update_queue(it_neigh->first, it_neigh->second + start_times[uiVtx], queue_open_list, start_times);
		}
		
		queue_open_list.pop_front();
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

void compute_critical_path(const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &in_graph, std::unordered_map<size_t, size_t> &start_times, std::list<size_t> &critical_path)
{
	assert(0 == critical_path.size());
	std::list<size_t> critical_path_times;
	size_t uiVtx, uiPred, c_uiMakespan = std::numeric_limits<size_t>::min(), uiPredTime, uiCurrTime;

	//compute last vertex
	for (auto it = start_times.begin(); it != start_times.end(); it++)
	{
		if (c_uiMakespan < it->second)
		{
			uiVtx = it->first;
			c_uiMakespan = it->second;					
		}
	}
	add_vtx_to_critical_path_time(uiVtx, c_uiMakespan, critical_path, critical_path_times);
	
	while (1)
	{
		if (0 == in_graph.at(uiVtx).size())
		{
			assert(0 == start_times.at(uiVtx));
			break;
		}

		uiCurrTime = start_times.at(uiVtx);

		std::vector<size_t> vec_pred_0_vtx; //stores 0 tight predecessors i.e. uiPredTime = uiCurrTime
		bool bTightPredecessor = false;

		for (auto it_pred = in_graph.at(uiVtx).begin(); it_pred != in_graph.at(uiVtx).end(); it_pred++)
		{
			uiPred = it_pred->first;
			uiPredTime = start_times.at(uiPred);
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