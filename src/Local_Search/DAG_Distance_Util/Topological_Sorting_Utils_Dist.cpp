#include "Topological_Sorting_Utils_Dist.h"

void Topological_Sorting_Utils_Dist::clear_prev_dist_info()
{
	m_iLastVtx = std::numeric_limits<int>::max();
	m_map_from_costs.clear();
	m_map_to_costs.clear();
	m_list_critical_path.clear();
}

size_t Topological_Sorting_Utils_Dist::Compute_FROM_costs_each_Vertex(std::unordered_map<size_t, size_t> &start_times)
{
	int iVtx, iPrev;
	size_t uiCost;
	size_t uiMakeSpan = std::numeric_limits<size_t>::min();
	assert(0 == m_map_from_costs.size());

	for (auto it_curr = m_list_order.begin(); it_curr != m_list_order.end(); it_curr++)
	{
		iVtx = *it_curr;
		uiCost = std::numeric_limits<size_t>::min();

		if (0 == m_in_graph.at(iVtx).size())
		{
			m_map_from_costs.emplace(iVtx, 0);
			continue;
		}

		for (auto it_prev = m_in_graph.at(iVtx).begin(); it_prev != m_in_graph.at(iVtx).end(); it_prev++)
		{
			iPrev = it_prev->first;
			assert(m_map_from_costs.end() != m_map_from_costs.find(iPrev));
			uiCost = std::max(uiCost, m_map_from_costs[iPrev] + it_prev->second);
		}
		m_map_from_costs.emplace(iVtx, uiCost);

		//fills the start_time costs as well
		if (iVtx < 0)
		{
			auto it_comp = m_list_Super_Comp.begin();
			size_t uiComp = (size_t)(-1 * iVtx) - 1;
			std::advance(it_comp, uiComp);

			for (auto it_vtx = it_comp->begin(); it_vtx != it_comp->end(); it_vtx++) start_times.emplace(*it_vtx, uiCost);
		}
		else start_times.emplace((size_t) iVtx, uiCost);

		if (uiMakeSpan < uiCost)
		{
			uiMakeSpan = uiCost;
			m_iLastVtx = iVtx;
		}
	}
	return uiMakeSpan;
}

size_t Topological_Sorting_Utils_Dist::Compute_GO_costs_each_Vertex()
{
	int iVtx, iNext;
	size_t uiCost;
	size_t uiMakeSpan = std::numeric_limits<size_t>::min();

	assert(0 == m_map_to_costs.size());

	for (auto it_curr = m_list_order.rbegin(); it_curr != m_list_order.rend(); it_curr++)
	{
		iVtx = *it_curr;
		uiCost = std::numeric_limits<size_t>::min();

		if (0 == m_out_graph.at(iVtx).size())
		{
			m_map_to_costs.emplace(iVtx, 0);
			continue;
		}

		for (auto it_next = m_out_graph.at(iVtx).begin(); it_next != m_out_graph.at(iVtx).end(); it_next++)
		{
			iNext = it_next->first;
			assert(m_map_to_costs.end() != m_map_to_costs.find(iNext));
			uiCost = std::max(uiCost, m_map_to_costs[iNext] + it_next->second);
		}
		m_map_to_costs.emplace(iVtx, uiCost);
		uiMakeSpan = std::max(uiMakeSpan, uiCost);
	}
	return uiMakeSpan;
}

void Topological_Sorting_Utils_Dist::compute_critical_path()
{
	int iCurrCriticalVtx = m_iLastVtx;
	m_list_critical_path.push_front(iCurrCriticalVtx);
	bool bFound;

	while (1)
	{
		bFound = false;
		for (auto it_prev = m_in_graph.at(iCurrCriticalVtx).begin(); it_prev != m_in_graph.at(iCurrCriticalVtx).end(); it_prev++)
		{
			if (m_map_from_costs.at(it_prev->first) + it_prev->second == m_map_from_costs.at(iCurrCriticalVtx))
			{
				iCurrCriticalVtx = it_prev->first;
				m_list_critical_path.push_front(iCurrCriticalVtx);
				bFound = true;
				break;
			}
		}
		
		if (0 == m_in_graph.at(iCurrCriticalVtx).size()) break;
		assert(true == bFound);		
	}
}

size_t Topological_Sorting_Utils_Dist::construct_graph_populate_order_with_dist(const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &out_graph, const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &in_graph, std::unordered_map<size_t, size_t> &start_times)
{
	assert(0 == start_times.size());
	construct_graph_populate_order(out_graph, in_graph);
	return compute_shortest_from_costs(start_times);
}

size_t Topological_Sorting_Utils_Dist::compute_shortest_from_costs(std::unordered_map<size_t, size_t> &start_times)
{
	clear_prev_dist_info();
	return Compute_FROM_costs_each_Vertex(start_times);
}