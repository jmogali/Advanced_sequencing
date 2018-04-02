#include "Topological_Sorting_Utils.h"

void Topological_Sorting_Utils::clear_prev_info()
{
	m_out_graph.clear();
	m_in_graph.clear();
	m_list_order.clear();
	m_list_Super_Comp.clear();
}

// m_out_graph(m_in_graph) graphs are different from those in alt_graph in the sense that some vertices correspond to strongly connected components
void Topological_Sorting_Utils::construct_in_out_graphs(const Alternative_Graph &alt_graph)
{
	construct_out_graph(alt_graph);
	construct_in_graph();
}

void Topological_Sorting_Utils::construct_out_graph(const Alternative_Graph &alt_graph)
{
	size_t uiVtx;
	int iVtx;
	int iNewLabel = -1; // numbering it starting with -1 is very important, note we are decrementing
	const auto& out_alt_graph = alt_graph.getGraph();
	const auto& in_alt_graph = alt_graph.getReverseGraph();
	std::unordered_map<size_t, int> str_comp_vtx_map;	// size_t, int

	// at this point, m_list_Super_Comp contains only components with 2 or more vertices
	//number the SCC
	for (auto it_list = m_list_Super_Comp.begin(); it_list != m_list_Super_Comp.end(); it_list++)
	{
		assert(1 < it_list->size());
		for (auto it_vtx = it_list->begin(); it_vtx != it_list->end(); it_vtx++)
		{
			str_comp_vtx_map.emplace(*it_vtx, iNewLabel);
		}
		iNewLabel--;
	}

	//allocate adjacency buffers for all vertices
	for (auto it = out_alt_graph.begin(); it != out_alt_graph.end(); it++)
	{
		uiVtx = it->first;
		auto it_find = str_comp_vtx_map.find(uiVtx);
		if (str_comp_vtx_map.end() != it_find)
		{
			if (m_out_graph.end() != m_out_graph.find(it_find->second)) continue;
			else m_out_graph.emplace(it_find->second, std::unordered_map<int, size_t>());
		}
		else
			m_out_graph.emplace((int)uiVtx, std::unordered_map<int, size_t>());
	}

	//populate adjacency information 
	for (auto it = out_alt_graph.begin(); it != out_alt_graph.end(); it++)
	{
		uiVtx = it->first;
		auto it_find = str_comp_vtx_map.find(uiVtx);
		if (str_comp_vtx_map.end() == it_find) iVtx = (int)(uiVtx);
		else iVtx = it_find->second;

		for (auto it_succ = out_alt_graph.at(uiVtx).begin(); it_succ != out_alt_graph.at(uiVtx).end(); it_succ++)
		{
			auto it_neigh = str_comp_vtx_map.find(it_succ->first);
			if (str_comp_vtx_map.end() != it_neigh)
			{
				if (iVtx == it_neigh->second) continue;

				// doing it this way to prevent creation of multigraph
				auto it_find = m_out_graph.at(iVtx).find(it_neigh->second);

				if (m_out_graph.at(iVtx).end() == it_find) m_out_graph.at(iVtx).emplace(it_neigh->second, it_succ->second);
				else it_find->second = std::max(it_find->second, it_succ->second);
			}
			else
			{
				// doing it this way to prevent creation of multigraph
				auto it_find = m_out_graph.at(iVtx).find((int)it_succ->first);

				if (m_out_graph.at(iVtx).end() == it_find) m_out_graph.at(iVtx).emplace((int)it_succ->first, it_succ->second);
				else it_find->second = std::max(it_find->second, it_succ->second);
			}
		}
	}
}

void Topological_Sorting_Utils::construct_in_graph()
{
	for (auto it_vtx = m_out_graph.begin(); it_vtx != m_out_graph.end(); it_vtx++)
	{
		m_in_graph.emplace(it_vtx->first, std::unordered_map<int, size_t>());
	}

	for (auto it_tail = m_out_graph.begin(); it_tail != m_out_graph.end(); it_tail++)
	{
		for (auto it_head = it_tail->second.begin(); it_head != it_tail->second.end(); it_head++)
		{
			m_in_graph.at(it_head->first).emplace(it_tail->first, it_head->second); // reverse the graph, so tail and head switched
		}
	}
}


void Topological_Sorting_Utils::Topological_sort_out_graph()
{
	assert(0 == m_list_order.size());
	std::unordered_map<int, std::string> map_seen;
	int iVtx;

	for (auto it = m_out_graph.cbegin(); it != m_out_graph.cend(); it++)
	{
		iVtx = it->first;
		if (map_seen.end() == map_seen.find(iVtx))
		{
			Topological_Dfs(iVtx, map_seen);
		}
	}
}

void Topological_Sorting_Utils::Topological_Dfs(int iVtx, std::unordered_map<int, std::string> &map_seen)
{
	auto it_insert = map_seen.emplace(iVtx, "GRAY");
	assert(true == it_insert.second);

	for (auto it = m_out_graph.at(iVtx).cbegin(); it != m_out_graph.at(iVtx).cend(); it++)
	{
		auto it_neigh = map_seen.find(it->first);
		if (map_seen.end() == it_neigh) Topological_Dfs(it->first, map_seen);
		else assert("BLACK" == it_neigh->second); // helps to check validity of DAG, note all valid loops are converted to super vertices by this step already
	}

	map_seen[iVtx] = "BLACK";
	m_list_order.push_front(iVtx);
}

bool Topological_Sorting_Utils::Check_Pos_Loop_Remove_1comp(const Alternative_Graph &alt_graph, const size_t c_uiNumRobots)
{
	size_t uiVtx1, uiVtx2, uiRobot;
	const auto& out_alt_graph = alt_graph.getGraph();

	for (auto it_list = m_list_Super_Comp.begin(); it_list != m_list_Super_Comp.end(); )
	{
		if (1 == it_list->size())
		{
			it_list = m_list_Super_Comp.erase(it_list);
			continue;
		}
		else if (it_list->size() > c_uiNumRobots) return true;  // assumes that all jobs have > 0 durations

		for (auto it_vtx1 = it_list->begin(); it_vtx1 != it_list->end(); it_vtx1++)
		{
			uiVtx1 = *it_vtx1;
			uiRobot = alt_graph.get_vertex_ownership(uiVtx1);

			for (auto it_vtx2 = out_alt_graph.at(uiVtx1).begin(); it_vtx2 != out_alt_graph.at(uiVtx1).end(); it_vtx2++)
			{
				uiVtx2 = it_vtx2->first;
				if (uiRobot == alt_graph.get_vertex_ownership(uiVtx2))
				{
					if (it_list->end() != it_list->find(uiVtx2)) return true;
				}
			}
		}
		it_list++;
	}
	return false;
}

bool Topological_Sorting_Utils::construct_graph_populate_order(const Alternative_Graph &alt_graph, const size_t c_uiNumRobots)
{
	clear_prev_info();
	Kosaraju_Algo obj;
	obj.compute_maximal_components(alt_graph.getGraph(), alt_graph.getReverseGraph(), m_list_Super_Comp);
	bool bFeasible = !(Check_Pos_Loop_Remove_1comp(alt_graph, c_uiNumRobots));
	if (false == bFeasible) return false;

	construct_in_out_graphs(alt_graph);
	Topological_sort_out_graph();
	return true;
}