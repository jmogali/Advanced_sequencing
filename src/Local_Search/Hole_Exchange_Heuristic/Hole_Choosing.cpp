#include "Hole_exchanges.h"

bool isWorthInserting(const std::tuple<size_t, size_t, size_t> &curr_choice, const std::tuple<size_t, size_t, size_t> &new_choice)
{
	int iSlackCurr = (int)std::get<2>(curr_choice) - (int)std::get<1>(curr_choice);
	int iNewSlack = (int)std::get<2>(new_choice) - (int)std::get<1>(new_choice);

	if (iSlackCurr < iNewSlack) return true;
	return false;
}
																	                                           //<vtx, min time, max time>
void Hole_Exchange::get_cand_vertex_critical_path(size_t uiChoice, std::list<size_t> &critical_path, std::list<std::tuple<size_t, size_t, size_t>> &list_best_cand)
{
	critical_path.clear();
	list_best_cand.clear();

	size_t uiVtx;
	std::tuple<size_t, size_t, size_t> val;
	const size_t c_uiMakeSpan = m_map_completion_times.at(*critical_path.rbegin());
	
	for (auto it_vtx = critical_path.begin(); it_vtx != critical_path.end(); it_vtx++)
	{
		uiVtx = *it_vtx;
		if ("H" != m_graph.getType(uiVtx)) continue;
		
		if (1 == uiChoice)
		{
			auto res = compute_enabler_flexibility(uiVtx, c_uiMakeSpan);
			std::get<0>(val) = uiVtx;
			std::get<1>(val) = res.first;
			std::get<2>(val) = res.second;
		}

		if (0 == list_best_cand.size()) list_best_cand.emplace_back(val);
		else
		{
			bool bInsert = false;
			for (auto it_cand = list_best_cand.begin(); it_cand != list_best_cand.end(); it_cand++)
			{
				bInsert = isWorthInserting(*it_cand, val);
				if (true == bInsert)
				{
					list_best_cand.insert(it_cand, val);
					break;
				}
			}
			if ( (false == bInsert) && (list_best_cand.size() < c_uiMaxCriticalPathCandidates)) list_best_cand.emplace_back(val);			
			if (list_best_cand.size() > c_uiMaxCriticalPathCandidates) list_best_cand.pop_back();
		}
	}	
}

// computes the earliest time c_uiVtx is enabled
size_t Hole_Exchange::compute_min_time(const size_t c_uiVtx)
{
	size_t uiMinTime = std::numeric_limits<size_t>::max();
	size_t uiEnabler;
	const auto& vec_enablers = m_graph.get_Enablers();

	for (auto it_enabler = vec_enablers.at(c_uiVtx).set.begin(); it_enabler != vec_enablers.at(c_uiVtx).set.end(); it_enabler++)
	{
		uiEnabler = it_enabler->getInd();
		uiMinTime = std::min(uiMinTime, m_map_start_times.at(uiEnabler) + m_graph.getTime(uiEnabler));
	}
	return uiMinTime;
}

//computes max time based on vertices enabled by c_uiVtx
size_t Hole_Exchange::compute_max_time(const size_t c_uiVtx, const size_t c_uiMakeSpan)
{
	size_t uiMaxTime = c_uiMakeSpan;
	size_t uiEnabler;
	size_t uiEnabledVtx, uiEnabledTime;
		
	const auto& vec_enablers = m_graph.get_Enablers();
	const auto& map_enabled = m_en_graph.get_Node_vec();
	const auto& vtx_vec_enabled = map_enabled.at(c_uiVtx).get_neighs();

	//*it_enabled are the vertices enabled by c_uiVtx
	for (auto it_enabled = vtx_vec_enabled.begin(); it_enabled != vtx_vec_enabled.end(); it_enabled++)
	{
		size_t uiEnablingVts = 0;
		uiEnabledVtx = *it_enabled;
		size_t uiEnabledVtxStartTime = m_map_start_times.at(uiEnabledVtx);

		//need to check if uiEnabledVtx is actually enabled by uiVtx
		for (auto it_enabler = vec_enablers.at(uiEnabledVtx).set.begin(); it_enabler != vec_enablers.at(uiEnabledVtx).set.end(); it_enabler++)
		{
			uiEnabler = it_enabler->getInd();
			uiEnabledTime = m_map_start_times.at(uiEnabler) + m_graph.getTime(uiEnabler);

			if (uiEnabledVtxStartTime >= uiEnabledTime) uiEnablingVts++;
		}

		if (1 == uiEnablingVts)
		{
			if (uiEnabledVtxStartTime > m_map_start_times.at(c_uiVtx) + m_graph.getTime(c_uiVtx))
			{
				uiMaxTime = std::min(uiMaxTime, uiEnabledVtxStartTime - m_graph.getTime(c_uiVtx));
			}
		}
	}
	return uiMaxTime;
}

std::pair<size_t, size_t> Hole_Exchange::compute_enabler_flexibility(const size_t c_uiVtx, const size_t c_uiMakeSpan)
{
	size_t uiMinTime = compute_min_time(c_uiVtx);
	size_t uiMaxTime = compute_max_time(c_uiVtx, c_uiMakeSpan);
	
	return std::make_pair(uiMinTime, uiMaxTime);
}