#include "Topological_Sorting_Utils_Dist.h"

void Topological_Sorting_Utils_Dist::clear_prev_dist_info()
{
	m_iLastVtx = std::numeric_limits<int>::max();
	m_map_from_costs.clear();
	m_map_to_costs.clear();
	m_list_critical_path.clear();
}

size_t Topological_Sorting_Utils_Dist::Compute_FROM_costs_each_Vertex()
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

void Topological_Sorting_Utils_Dist::construct_graph_populate_order_with_dist(const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &out_graph, const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &in_graph)
{
	construct_graph_populate_order(out_graph, in_graph);
	compute_shortest_path_costs();
	compute_critical_path();
}

void Topological_Sorting_Utils_Dist::compute_shortest_path_costs()
{
	clear_prev_dist_info();
	Compute_FROM_costs_each_Vertex();
	Compute_GO_costs_each_Vertex();
}