#include "Greedy_Heuristic.h"
#include <iostream>
#include "Kosaraju_Algo.h"

bool Greedy_Heuristic::construct_Alt_Graph_STN(const std::vector<std::list<size_t>> &rob_seq)
{
	bool bFeasible = construct_Alt_Graph(rob_seq);
	if (false == bFeasible) return false;
	return true;
}

void construct_prec_graph_for_each_job(const std::vector<std::list<size_t>> &rob_seq, const Layout_LS &layout_graph, Alternative_Graph &alt_graph)
{
	size_t uiNumRobots = layout_graph.get_num_robots();
	
	for (size_t uiRobot = 0; uiRobot < uiNumRobots; uiRobot++)
	{
		size_t uiInd = 0;
		auto it_next = rob_seq[uiRobot].begin();
		for (auto it = rob_seq[uiRobot].begin(); it != rob_seq[uiRobot].end(); it++)
		{
			it_next++;
			if (it_next == rob_seq[uiRobot].end())
			{
				alt_graph.add_vertex_ownership_pos(*it, uiRobot , uiInd);
				assert(0 == layout_graph.getTime(*it));
				break;
			}
			alt_graph.add_prec_arc(*it, *it_next, layout_graph.getTime(*it));
			alt_graph.add_vertex_ownership_pos(*it, uiRobot, uiInd);
			uiInd++;
		}		
	}
}

void get_verts_not_self_enabled(size_t uiRobot, std::unordered_set<size_t> &dep_vert, const std::vector<std::list<size_t>> &rob_seq, const Layout_LS &layout_graph)
{
	std::unordered_set<size_t> comp_vert;
	size_t uiVert;
	bool bFound;
	auto &vec_enabler = layout_graph.get_Enablers();

	auto it = rob_seq[uiRobot].begin();
	comp_vert.emplace(*it);
	it++;

	for (; it != rob_seq[uiRobot].end(); it++)
	{
		bFound = false;
		uiVert = *it;

		if ("H" != layout_graph.getType(uiVert)) continue;

		for (auto it_enabler = vec_enabler[uiVert].set.begin(); it_enabler != vec_enabler[uiVert].set.end(); it_enabler++)
		{
			if (comp_vert.find(it_enabler->getInd()) != comp_vert.end())
			{
				bFound = true;
				break;
			}
		}
		if (!bFound) {
			dep_vert.emplace(uiVert);
		}
		comp_vert.emplace(uiVert);
	}
}

bool add_prec_arcs_for_dep_vert_of_job(size_t uiGivenRobot, const std::unordered_set<size_t> &dep_vert, const std::vector<std::list<size_t>> &rob_seq, const Layout_LS &layout_graph, Alternative_Graph &alt_graph)
{
	size_t uiNumRobots = layout_graph.get_num_robots();
	size_t uiVert;
	std::vector<size_t> vec_pos_enabler;
	auto &vec_enabler = layout_graph.get_Enablers();

	for (auto it = dep_vert.begin(); it != dep_vert.end(); it++)
	{
		uiVert = *it;
		if ("H" != layout_graph.getType(uiVert)) continue;

		for (size_t uiRobot = 0; uiRobot < uiNumRobots; uiRobot++)
		{
			if (uiGivenRobot == uiRobot) continue;
			for (auto it = rob_seq[uiRobot].begin(); it != rob_seq[uiRobot].end(); it++)
			{
				if (vec_enabler.at(uiVert).set.find(*it) != vec_enabler.at(uiVert).set.end())
				{
					it++;	// we need to store the vertex following the enabler
					vec_pos_enabler.emplace_back(*it);
					break;
				}
			}
		}
		if (0 == vec_pos_enabler.size()) return false;
		else if (1 == vec_pos_enabler.size()) { alt_graph.add_prec_arc(vec_pos_enabler[0], uiVert, 0); }
		vec_pos_enabler.clear();
	}
	return true;
}

bool add_enabling_cons(const std::vector<std::list<size_t>> &rob_seq, const Layout_LS &layout_graph, Alternative_Graph &alt_graph)
{
	size_t uiNumRobots = layout_graph.get_num_robots();
	bool bFeasible;

	for (size_t uiRobot = 0; uiRobot < uiNumRobots; uiRobot++)
	{
		std::unordered_set<size_t> set_dep_vert;
		get_verts_not_self_enabled(uiRobot, set_dep_vert, rob_seq, layout_graph);
		bFeasible = add_prec_arcs_for_dep_vert_of_job(uiRobot, set_dep_vert, rob_seq, layout_graph, alt_graph);
		if (false == bFeasible) return false;
	}
	return true;
}

bool add_coll_cons_bet_pair_jobs(size_t uiRobot1, size_t uiRobot2, const std::vector<std::list<size_t>> &rob_seq, const Layout_LS &layout_graph, Alternative_Graph &alt_graph)
{
	for (auto it1 = rob_seq[uiRobot1].begin(); it1 != rob_seq[uiRobot1].end(); it1++)
	{
		auto it12 = it1;
		it12++;

		for (auto it2 = rob_seq[uiRobot2].begin(); it2 != rob_seq[uiRobot2].end(); it2++)
		{
			if (layout_graph.areColliding(Coll_Pair(*it1, uiRobot1, *it2, uiRobot2)))
			{
				auto it22 = it2;
				it22++;
				
				bool bArc1 = alt_graph.containsPrecArc(arc(*it22, *it1));
				bool bArc2 = alt_graph.containsPrecArc(arc(*it12, *it2));

				if (!bArc1 & !bArc2) alt_graph.add_alt_arc(*it22, *it1, *it12, *it2); 
				else if (bArc1 & bArc2) return false;				
			}
		}
	}
	return true;
}

bool add_coll_cons(const std::vector<std::list<size_t>> &rob_seq, const Layout_LS &layout_graph, Alternative_Graph &alt_graph)
{
	size_t uiNumRobots = layout_graph.get_num_robots();
	bool bFeasible;

	for (size_t uiRobot1 = 0; uiRobot1 < uiNumRobots; uiRobot1++)
	{
		for (size_t uiRobot2 = uiRobot1+1; uiRobot2 < uiNumRobots; uiRobot2++)
		{
			bFeasible = add_coll_cons_bet_pair_jobs(uiRobot1, uiRobot2, rob_seq, layout_graph, alt_graph);
			if (false == bFeasible) return false;
		}
	}
	return true;
}

bool CheckForPositiveLoops(const std::list<std::unordered_set<size_t>> &listComp, const Alternative_Graph &alt_graph)
{
	size_t uiVtx1, uiVtx2, uiRobot;
	const auto& out_graph = alt_graph.getGraph();

	for (auto it_list = listComp.begin(); it_list != listComp.end(); it_list++)
	{
		if (1 == it_list->size()) continue;
		
		for (auto it_vtx1 = it_list->begin(); it_vtx1 != it_list->end(); it_vtx1++)
		{
			uiVtx1 = *it_vtx1;
			uiRobot = alt_graph.get_vertex_ownership(uiVtx1);
			
			for (auto it_vtx2 = out_graph.at(uiVtx1).begin(); it_vtx2 != out_graph.at(uiVtx1).end(); it_vtx2++)
			{
				uiVtx2 = it_vtx2->first;
				if (uiRobot == alt_graph.get_vertex_ownership(uiVtx2))
				{
					if (it_list->end() != it_list->find(uiVtx2)) return true;
				}
			}
		}
		/*for (auto it_vtx1 = it_list->begin(); it_vtx1 != it_list->end(); it_vtx1++)
		{
			uiVtx1 = *it_vtx1;
			for (auto it_vtx2 = it_list->begin(); it_vtx2 != it_list->end(); it_vtx2++)
			{
				uiVtx2 = *it_vtx2;
				if (uiVtx1 == uiVtx2) continue;
				
				arc inp_arc(uiVtx1, uiVtx2);
				if (true == alt_graph.containsPrecArc(inp_arc))
				{
					if (0 < alt_graph.getArcCost(inp_arc))	return true;
				}
			}
		}*/
	}
	return false;
}

bool add_enabling_coll_cons(const std::vector<std::list<size_t>> &rob_seq, const Layout_LS &layout_graph, Alternative_Graph &alt_graph)
{
	bool bFeasible = add_enabling_cons(rob_seq, layout_graph, alt_graph);
	if (false == bFeasible) return false;
	
	std::list<std::unordered_set<size_t>> listComp;
	Kosaraju_Algo obj;
	obj.compute_maximal_components(alt_graph.getGraph() , alt_graph.getReverseGraph(), listComp);
	bFeasible = !(CheckForPositiveLoops(listComp, alt_graph));
	if (false == bFeasible) return false;

	bFeasible = add_coll_cons(rob_seq, layout_graph, alt_graph);
	if (false == bFeasible) return false;
	return true;
}

bool Greedy_Heuristic::construct_Alt_Graph(const std::vector<std::list<size_t>> &rob_seq)
{
	m_alt_graph.allocate_buffer_for_graph(rob_seq);
	construct_prec_graph_for_each_job(rob_seq, m_graph, m_alt_graph);
	bool bFeasible = add_enabling_coll_cons(rob_seq, m_graph, m_alt_graph);
	return bFeasible;
}




