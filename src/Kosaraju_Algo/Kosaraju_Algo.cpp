#include "Kosaraju_Algo.h"

Kosaraju_Algo::Kosaraju_Algo()
{}

void Kosaraju_Algo::perform_dfs(const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &graph)
{
	for (auto it = graph.begin(); it != graph.end(); it++)
	{
		if (false == m_dfs_data.at(it->first))
		{
			dfs(it->first, graph);
		}
	}
}

void Kosaraju_Algo::dfs(size_t uiVtx , const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &graph, std::unordered_set<size_t> *pComp)
{
	m_dfs_data.at(uiVtx) = true;
	if (pComp != NULL) pComp->emplace(uiVtx);
		
	for (auto it = graph.at(uiVtx).begin(); it != graph.at(uiVtx).end(); it++)
	{
		if (false == m_dfs_data.at(it->first))
		{
			dfs(it->first, graph, pComp);
		}
	}
	m_stack_comp_vts.push(uiVtx);
}

void Kosaraju_Algo::reverse_graph(std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &rev_graph , const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &graph)
{
	for (auto it = graph.begin(); it != graph.end(); it++)
	{
		rev_graph.emplace(it->first, std::unordered_map<size_t, size_t>());
	}

	for (auto it1 = graph.begin(); it1 != graph.end(); it1++)
	{
		for (auto it2 = it1->second.begin(); it2 != it1->second.end(); it2++)
		{
			rev_graph.at(it2->first).emplace(it1->first, it2->second);
		}
	}
}

void Kosaraju_Algo::get_components(const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &rev_graph, std::list<std::unordered_set<size_t>> &listComp)
{
	size_t uiVtx;
	
	while (false == m_stack_comp_vts.empty())
	{
		uiVtx = m_stack_comp_vts.top();
		m_stack_comp_vts.pop();

		if (false == m_dfs_data.at(uiVtx))
		{
			listComp.emplace_back(std::unordered_set<size_t>());
			dfs(uiVtx , rev_graph, &(*listComp.rbegin()));
		}		
	}
}

void Kosaraju_Algo::compute_maximal_components(const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &graph, std::list<std::unordered_set<size_t>> &listComp)
{
	initialize_DFS_container(graph);
	perform_dfs(graph);

	std::unordered_map<size_t, std::unordered_map<size_t, size_t>> rev_graph;
	reverse_graph(rev_graph, graph);

	initialize_DFS_container(rev_graph);
	get_components(rev_graph, listComp);
}

void Kosaraju_Algo::compute_maximal_components(const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &graph, const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &rev_graph, std::list<std::unordered_set<size_t>> &listComp)
{
	initialize_DFS_container(graph);
	perform_dfs(graph);

	initialize_DFS_container(rev_graph);
	get_components(rev_graph, listComp);
}

void Kosaraju_Algo::initialize_DFS_container(const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &graph)
{
	m_dfs_data.clear();

	for (auto it = graph.begin(); it != graph.end(); it++)
	{
		m_dfs_data.emplace(it->first, false);
	}
}