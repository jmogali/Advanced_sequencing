#include "Coll_Topological_Sorting_Utils.h"

// m_out_graph(m_in_graph) graphs are different from those in alt_graph in the sense that some vertices correspond to strongly connected components
void Coll_Topological_Sorting_Utils::construct_in_out_graphs(const Alternative_Graph &alt_graph)
{
	const auto& out_alt_graph = alt_graph.getGraph();
	const auto& in_alt_graph = alt_graph.getReverseGraph();
	
	Topological_Sorting_Utils::construct_in_out_graphs(out_alt_graph, in_alt_graph);
}

bool Coll_Topological_Sorting_Utils::Check_Pos_Loop_Remove_1comp(const Alternative_Graph &alt_graph, const size_t c_uiNumRobots)
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

bool Coll_Topological_Sorting_Utils::construct_graph_populate_order(const Alternative_Graph &alt_graph, const size_t c_uiNumRobots)
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