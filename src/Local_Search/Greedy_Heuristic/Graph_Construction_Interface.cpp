#include "Greedy_Heuristic.h"
#include "Kosaraju_Algo.h"

bool Greedy_Heuristic::construct_Alt_Graph_STN(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::list<size_t>> &new_rob_seq)
{
	bool bFeasible = construct_Alt_Graph(rob_seq, new_rob_seq);
	if (false == bFeasible) return false;
	return true;
}

void construct_prec_graph_for_each_operation(const std::vector<std::list<size_t>> &rob_seq, const Layout_LS &layout_graph, Alternative_Graph &alt_graph)
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

void get_verts_not_self_enabled(size_t uiRobot, std::unordered_set<size_t> &dep_vert, const std::vector<std::list<size_t>> &rob_seq, const Layout_LS &layout_graph, std::vector<std::list<size_t>>& vec_dep_vert)
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
			vec_dep_vert[uiRobot].emplace_back(uiVert);
		}
		comp_vert.emplace(uiVert);
	}
}

bool add_prec_arcs_for_dep_vert_of_job(size_t uiGivenRobot, const std::unordered_set<size_t> &dep_vert, const std::vector<std::list<size_t>> &rob_seq, const Layout_LS &layout_graph, Alternative_Graph &alt_graph, std::list<arc> &list_prec_arcs_betw_jobs, std::unordered_map<size_t, std::unordered_map<size_t, std::pair<size_t, size_t>>> &map_enabler_pos_vert, std::vector<std::list<size_t>> &vec_dep_vert)
{
	size_t uiNumRobots = layout_graph.get_num_robots();
	size_t uiVert;
	std::vector<size_t> vec_pos_enabler;
	auto &vec_enabler = layout_graph.get_Enablers();

	for (auto it_vert = dep_vert.begin(); it_vert != dep_vert.end(); it_vert++)
	{
		uiVert = *it_vert;
		if ("H" != layout_graph.getType(uiVert)) continue;

		for (size_t uiRobot = 0; uiRobot < uiNumRobots; uiRobot++)
		{
			if (uiGivenRobot == uiRobot) continue;
			
			size_t uiPos = 0;
			for (auto it = rob_seq[uiRobot].begin(); it != rob_seq[uiRobot].end(); it++, uiPos++)
			{
				map_enabler_pos_vert.emplace(uiVert, std::unordered_map<size_t, std::pair<size_t, size_t>>());
				if (vec_enabler.at(uiVert).set.find(*it) != vec_enabler.at(uiVert).set.end())
				{
					it++;	// we need to store the vertex following the enabler
					uiPos++; // position also needs to be reflected
					vec_pos_enabler.emplace_back(*it);
					map_enabler_pos_vert.at(uiVert).emplace(uiRobot, std::make_pair(uiPos, *it)); // we need to store the position following the vertex
					break;
				}				
			}
		}
		if (0 == vec_pos_enabler.size()) return false;
		else if (1 == vec_pos_enabler.size()) 
		{ 
			alt_graph.add_prec_arc(vec_pos_enabler[0], uiVert, 0); 
			list_prec_arcs_betw_jobs.emplace_back(arc(vec_pos_enabler[0], uiVert));
		}
		vec_pos_enabler.clear();
	}
	return true;
}

bool add_enabling_cons(const std::vector<std::list<size_t>> &rob_seq, const Layout_LS &layout_graph, Alternative_Graph &alt_graph, std::list<arc> &list_prec_arcs_betw_jobs, std::unordered_map<size_t, std::unordered_map<size_t, std::pair<size_t, size_t>>> &map_enabler_pos_vert, std::vector<std::list<size_t>> &vec_dep_vert)
{
	size_t uiNumRobots = layout_graph.get_num_robots();
	bool bFeasible;

	for (size_t uiRobot = 0; uiRobot < uiNumRobots; uiRobot++)
	{
		std::unordered_set<size_t> set_dep_vert;
		get_verts_not_self_enabled(uiRobot, set_dep_vert, rob_seq, layout_graph, vec_dep_vert);
		bFeasible = add_prec_arcs_for_dep_vert_of_job(uiRobot, set_dep_vert, rob_seq, layout_graph, alt_graph, list_prec_arcs_betw_jobs, map_enabler_pos_vert, vec_dep_vert);
		if (false == bFeasible) return false;
	}
	return true;
}
  
/*
bool add_coll_cons_bet_pair_jobs(size_t uiRobot1, size_t uiRobot2, const std::vector<std::list<size_t>> &rob_seq, const Layout_LS &layout_graph, Alternative_Graph &alt_graph, Collision_Filtering &coll_filter)
{
	size_t uiPos = 0;
	for (auto it1 = rob_seq[uiRobot1].begin(); it1 != rob_seq[uiRobot1].end(); it1++)
	{
		auto it12 = it1;
		it12++;

		auto pr1 = coll_filter.get_bounds(*it1, uiRobot2);
		auto it2_start = rob_seq[uiRobot2].begin();
		std::advance(it2_start, pr1.first);

		auto it2_end = rob_seq[uiRobot2].begin();
		std::advance(it2_end, pr1.second);
		it2_end++;

		for (auto it2 = it2_start; it2 != it2_end; it2++)
		{
			auto pr2 = coll_filter.get_bounds(*it2, uiRobot1);
			if ((uiPos < pr2.first) || (uiPos > pr2.second)) continue;

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
		uiPos++;
	}
	return true;
}
*/

bool add_coll_cons_bet_pair_jobs(size_t uiRobot1, size_t uiRobot2, const std::vector<std::list<size_t>> &rob_seq, const Layout_LS &layout_graph, Alternative_Graph &alt_graph, Collision_Filtering &coll_filter)
{
	size_t uiRob_1_Pos;

	for (auto it1 = rob_seq[uiRobot1].begin(); it1 != rob_seq[uiRobot1].end(); it1++)
	{
		auto it12 = it1;
		it12++;

		auto pr1 = coll_filter.get_lower_bound_pos(*it1, uiRobot2);
		auto it2_start = rob_seq[uiRobot2].begin();
		std::advance(it2_start, pr1);
		uiRob_1_Pos = alt_graph.get_vertex_position(*it1);

		for (auto it2 = it2_start; it2 != rob_seq[uiRobot2].end(); it2++)
		{
			auto pr2 = coll_filter.get_lower_bound_pos(*it2, uiRobot1);
			if (uiRob_1_Pos < pr2) break;			

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

/*
bool get_coll_cons_bet_pair_jobs(size_t uiRobot1, size_t uiRobot2, const std::vector<std::list<size_t>> &rob_seq, const Layout_LS &layout_graph, Alternative_Graph &alt_graph, std::unordered_set<Coll_Pair, CollHasher> &set_coll,Collision_Filtering &coll_filter)
{
	size_t uiPos = 0;
	for (auto it1 = rob_seq[uiRobot1].begin(); it1 != rob_seq[uiRobot1].end(); it1++)
	{
		auto it12 = it1;
		it12++;

		auto pr1 = coll_filter.get_bounds(*it1, uiRobot2);
		auto it2_start = rob_seq[uiRobot2].begin();
		std::advance(it2_start, pr1.first);

		auto it2_end = rob_seq[uiRobot2].begin();
		std::advance(it2_end, pr1.second);
		it2_end++;

		for (auto it2 = it2_start; it2 != it2_end; it2++)
		{
			auto pr2 = coll_filter.get_bounds(*it2, uiRobot1);
			if ((uiPos < pr2.first) || (uiPos > pr2.second)) continue;

			if (layout_graph.areColliding(Coll_Pair(*it1, uiRobot1, *it2, uiRobot2)))
			{
				auto it22 = it2;
				it22++;

				bool bArc1 = alt_graph.containsPrecArc(arc(*it22, *it1));
				bool bArc2 = alt_graph.containsPrecArc(arc(*it12, *it2));
				
				if (!bArc1 & !bArc2) set_coll.emplace(Coll_Pair(*it1, uiRobot1, *it2, uiRobot2));
				else if (bArc1 & bArc2) return false;
			}
		}
		uiPos++;
	}
	return true;
}
*/

bool get_coll_cons_bet_pair_jobs(size_t uiRobot1, size_t uiRobot2, const std::vector<std::list<size_t>> &rob_seq, const Layout_LS &layout_graph, Alternative_Graph &alt_graph, std::unordered_set<Coll_Pair, CollHasher> &set_coll, Collision_Filtering &coll_filter)
{
	size_t uiRob_1_Pos;
	for (auto it1 = rob_seq[uiRobot1].begin(); it1 != rob_seq[uiRobot1].end(); it1++)
	{
		auto it12 = it1;
		it12++;

		auto pr1 = coll_filter.get_lower_bound_pos(*it1, uiRobot2);
		auto it2_start = rob_seq[uiRobot2].begin();
		std::advance(it2_start, pr1);
		uiRob_1_Pos = alt_graph.get_vertex_position(*it1);

		for (auto it2 = it2_start; it2 != rob_seq[uiRobot2].end(); it2++)
		{
			auto pr2 = coll_filter.get_lower_bound_pos(*it2, uiRobot1);
			if (uiRob_1_Pos < pr2) break;

			if (layout_graph.areColliding(Coll_Pair(*it1, uiRobot1, *it2, uiRobot2)))
			{
				auto it22 = it2;
				it22++;

				bool bArc1 = alt_graph.containsPrecArc(arc(*it22, *it1));
				bool bArc2 = alt_graph.containsPrecArc(arc(*it12, *it2));

				if (!bArc1 & !bArc2) set_coll.emplace(Coll_Pair(*it1, uiRobot1, *it2, uiRobot2));
				else if (bArc1 & bArc2) return false;
			}
		}		
	}
	return true;
}

bool add_coll_cons(const std::vector<std::list<size_t>> &rob_seq, const Layout_LS &layout_graph, Alternative_Graph &alt_graph, Collision_Filtering &coll_filter)
{
	size_t uiNumRobots = layout_graph.get_num_robots();
	bool bFeasible;

	for (size_t uiRobot1 = 0; uiRobot1 < uiNumRobots; uiRobot1++)
	{
		for (size_t uiRobot2 = uiRobot1+1; uiRobot2 < uiNumRobots; uiRobot2++)
		{
			bFeasible = add_coll_cons_bet_pair_jobs(uiRobot1, uiRobot2, rob_seq, layout_graph, alt_graph, coll_filter);
			if (false == bFeasible) return false;
		}
	}
	return true;
}

bool get_coll_cons(const std::vector<std::list<size_t>> &rob_seq, const Layout_LS &layout_graph, Alternative_Graph &alt_graph, std::unordered_set<Coll_Pair, CollHasher> &set_coll, Collision_Filtering &coll_filter)
{
	size_t uiNumRobots = layout_graph.get_num_robots();
	bool bFeasible;

	for (size_t uiRobot1 = 0; uiRobot1 < uiNumRobots; uiRobot1++)
	{
		for (size_t uiRobot2 = uiRobot1 + 1; uiRobot2 < uiNumRobots; uiRobot2++)
		{
			bFeasible = get_coll_cons_bet_pair_jobs(uiRobot1, uiRobot2, rob_seq, layout_graph, alt_graph, set_coll, coll_filter);
			if (false == bFeasible) return false;
		}
	}
	return true;
}

bool add_imp_prec_coll_con(const std::vector<std::list<size_t>> &rob_seq, Alternative_Graph &alt_graph, const arc &inp_arc, std::list<arc> &list_prec_arcs_betw_jobs, const Layout_LS &layout_graph)
{
	size_t uiTailStartVtx = inp_arc.first;
	size_t uiHeadStartVtx = inp_arc.second;

	size_t uiRobot_Tail = alt_graph.get_vertex_ownership(uiTailStartVtx);
	size_t uiRobot_Head = alt_graph.get_vertex_ownership(uiHeadStartVtx);

	if (uiTailStartVtx == *rob_seq[uiRobot_Tail].rbegin()) return false;
	
	//if (set_coll.end() != set_coll.find(Coll_Pair(uiTailStartVtx, uiRobot_Tail, uiHeadStartVtx, uiRobot_Head)))
	if(true == layout_graph.areColliding(Coll_Pair(uiTailStartVtx, uiRobot_Tail, uiHeadStartVtx, uiRobot_Head)))
	{
		auto pr = alt_graph.get_next_vtx_same_job(uiTailStartVtx);
		assert(true == pr.first);
		size_t uiNewTailVtx = pr.second;
		if (false == alt_graph.containsPrecArc(arc(uiNewTailVtx, uiHeadStartVtx)))
		{
			alt_graph.add_prec_arc(uiNewTailVtx, uiHeadStartVtx, 0);
			list_prec_arcs_betw_jobs.emplace_back(arc(uiNewTailVtx, uiHeadStartVtx));
			return true;
		}
	}
	return false;
}

/*
bool add_imp_con_for_given_prec_arc_forward_dir(const std::vector<std::list<size_t>> &rob_seq, Alternative_Graph &alt_graph, const arc &inp_arc, std::list<arc> &list_prec_arcs_betw_jobs, std::unordered_set<Coll_Pair, CollHasher> &set_coll)
{
	bool bAdded = false;
	size_t uiTailStartVtx = inp_arc.first;
	size_t uiHeadStartVtx = inp_arc.second;

	size_t uiRobot_Tail = alt_graph.get_vertex_ownership(uiTailStartVtx);
	size_t uiRobot_Head = alt_graph.get_vertex_ownership(uiHeadStartVtx);

	if (uiTailStartVtx == *rob_seq[uiRobot_Tail].rbegin()) return false;
	if (uiHeadStartVtx == *rob_seq[uiRobot_Head].rbegin()) return false;

	auto pr_head = alt_graph.get_next_vtx_same_job(uiHeadStartVtx);
	assert(true == pr_head.first);
	size_t uiHeadVtx = pr_head.second;

	if (set_coll.end() != set_coll.find(Coll_Pair(uiTailStartVtx, uiRobot_Tail, uiHeadVtx, uiRobot_Head)))
	{
		auto pr_tail = alt_graph.get_next_vtx_same_job(uiTailStartVtx);
		assert(true == pr_tail.first);
		size_t uiTailVtx = pr_tail.second;

		if (false == alt_graph.containsPrecArc(arc(uiTailVtx, uiHeadVtx)))
		{
			alt_graph.add_prec_arc(uiTailVtx, uiHeadVtx, 0);
			list_prec_arcs_betw_jobs.emplace_back(arc(uiTailVtx, uiHeadVtx));
			bAdded = true;
		}
	}		
	return bAdded;
}*/

bool add_imp_con_for_given_prec_arc_forward_dir(const std::vector<std::list<size_t>> &rob_seq, Alternative_Graph &alt_graph, const arc &inp_arc, std::list<arc> &list_prec_arcs_betw_jobs, const Layout_LS &layout_graph)
{
	bool bAdded = false;
	const size_t c_uiTailStartVtx = inp_arc.first;
	const size_t c_uiHeadStartVtx = inp_arc.second;

	size_t uiRobot_Tail = alt_graph.get_vertex_ownership(c_uiTailStartVtx);
	size_t uiRobot_Head = alt_graph.get_vertex_ownership(c_uiHeadStartVtx);

	if (c_uiTailStartVtx == *rob_seq[uiRobot_Tail].rbegin()) return false;
	if (c_uiHeadStartVtx == *rob_seq[uiRobot_Head].rbegin()) return false;

	auto pr_head = alt_graph.get_next_vtx_same_job(c_uiHeadStartVtx);
	assert(true == pr_head.first);
	size_t uiHeadVtx = pr_head.second;

	auto pr_tail = alt_graph.get_next_vtx_same_job(c_uiTailStartVtx);
	assert(true == pr_tail.first);
	const size_t c_uiTailVtx = pr_tail.second;

	while (1)
	{
		//if (set_coll.end() != set_coll.find(Coll_Pair(c_uiTailStartVtx, uiRobot_Tail, uiHeadVtx, uiRobot_Head)))
		if(true == layout_graph.areColliding(Coll_Pair(c_uiTailStartVtx, uiRobot_Tail, uiHeadVtx, uiRobot_Head)))
		{
			if (false == alt_graph.containsPrecArc(arc(c_uiTailVtx, uiHeadVtx)))
			{
				alt_graph.add_prec_arc(c_uiTailVtx, uiHeadVtx, 0);
				list_prec_arcs_betw_jobs.emplace_back(arc(c_uiTailVtx, uiHeadVtx));
				bAdded = true;
			}
			break; // remaining constraints if added will only be redundant
		}

		pr_head = alt_graph.get_next_vtx_same_job(uiHeadVtx);
		if(false == pr_head.first) break;
		uiHeadVtx = pr_head.second;
	}
	return bAdded;
}

/*
bool add_imp_con_for_given_prec_arc_reverse_dir(const std::vector<std::list<size_t>> &rob_seq, Alternative_Graph &alt_graph, const arc &inp_arc, std::list<arc> &list_prec_arcs_betw_jobs, std::unordered_set<Coll_Pair, CollHasher> &set_coll)
{
	bool bAdded = false;
	size_t uiTailStartVtx = inp_arc.first;
	size_t uiHeadStartVtx = inp_arc.second;

	size_t uiRobot_Tail = alt_graph.get_vertex_ownership(uiTailStartVtx);
	size_t uiRobot_Head = alt_graph.get_vertex_ownership(uiHeadStartVtx);

	if (uiTailStartVtx == *rob_seq[uiRobot_Tail].begin()) return false;
	if (uiHeadStartVtx == *rob_seq[uiRobot_Head].begin()) return false;

	auto pr_head = alt_graph.get_prec_vtx_same_job(uiHeadStartVtx);
	assert(true == pr_head.first);
	size_t uiHeadVtx = pr_head.second;
	
	auto pr_tail = alt_graph.get_prec_vtx_same_job(uiTailStartVtx);
	assert(true == pr_tail.first);
	size_t uiTailVtx = pr_tail.second;

	if (set_coll.end() != set_coll.find(Coll_Pair(uiTailVtx, uiRobot_Tail, uiHeadVtx, uiRobot_Head)))
	{
		if (false == alt_graph.containsPrecArc(arc(uiTailStartVtx, uiHeadVtx)))
		{
			alt_graph.add_prec_arc(uiTailStartVtx, uiHeadVtx, 0);
			list_prec_arcs_betw_jobs.emplace_back(arc(uiTailStartVtx, uiHeadVtx));
			bAdded = true;
		}
	}
	return bAdded;
}
*/

bool add_imp_con_for_given_prec_arc_reverse_dir(const std::vector<std::list<size_t>> &rob_seq, Alternative_Graph &alt_graph, const arc &inp_arc, std::list<arc> &list_prec_arcs_betw_jobs, const Layout_LS &layout_graph)
{
	bool bAdded = false;
	const size_t c_uiTailStartVtx = inp_arc.first;
	const size_t c_uiHeadStartVtx = inp_arc.second;

	size_t uiRobot_Tail = alt_graph.get_vertex_ownership(c_uiTailStartVtx);
	size_t uiRobot_Head = alt_graph.get_vertex_ownership(c_uiHeadStartVtx);

	if (c_uiTailStartVtx == *rob_seq[uiRobot_Tail].begin()) return false;
	if (c_uiHeadStartVtx == *rob_seq[uiRobot_Head].begin()) return false;

	auto pr_head = alt_graph.get_prec_vtx_same_job(c_uiHeadStartVtx);
	assert(true == pr_head.first);
	const size_t c_uiHeadVtx = pr_head.second;

	auto pr_tail = alt_graph.get_prec_vtx_same_job(c_uiTailStartVtx);
	assert(true == pr_tail.first);
	size_t uiTailVtx = pr_tail.second;
	size_t uiTailSucc = c_uiTailStartVtx;
	
	while (1)
	{
		//if (set_coll.end() != set_coll.find(Coll_Pair(uiTailVtx, uiRobot_Tail, c_uiHeadVtx, uiRobot_Head)))
		if(true == layout_graph.areColliding(Coll_Pair(uiTailVtx, uiRobot_Tail, c_uiHeadVtx, uiRobot_Head)))
		{
			if (false == alt_graph.containsPrecArc(arc(uiTailSucc, c_uiHeadVtx)))
			{
				alt_graph.add_prec_arc(uiTailSucc, c_uiHeadVtx, 0);
				list_prec_arcs_betw_jobs.emplace_back(arc(uiTailSucc, c_uiHeadVtx));
				bAdded = true;
			}
			break; // remaining constraints if added will only be redundant
		}

		pr_tail = alt_graph.get_prec_vtx_same_job(uiTailVtx);
		if (false == pr_tail.first) break;
		uiTailSucc = uiTailVtx;
		uiTailVtx = pr_tail.second;		
	}
	return bAdded;
}

bool add_imp_con_for_given_pred_arc(const std::vector<std::list<size_t>> &rob_seq, Alternative_Graph &alt_graph, const arc &inp_arc, std::list<arc> &list_prec_arcs_betw_jobs, const Layout_LS &layout_graph)
{
	bool bAdded;
	bAdded = add_imp_prec_coll_con(rob_seq, alt_graph, inp_arc, list_prec_arcs_betw_jobs, layout_graph);
	bAdded = bAdded | add_imp_con_for_given_prec_arc_forward_dir(rob_seq, alt_graph, inp_arc, list_prec_arcs_betw_jobs, layout_graph);
	bAdded = bAdded | add_imp_con_for_given_prec_arc_reverse_dir(rob_seq, alt_graph, inp_arc, list_prec_arcs_betw_jobs, layout_graph);
	return bAdded;
}

//adds new implied precedence constraints and removes previously considered precedence constraints
bool add_impl_cons_rem_prev_cons(const std::vector<std::list<size_t>> &rob_seq, Alternative_Graph &alt_graph, std::list<arc> &list_prec_arcs_betw_jobs, const Layout_LS &layout_graph)
{
	bool bAdded = false;
	for (auto it = list_prec_arcs_betw_jobs.begin(); it != list_prec_arcs_betw_jobs.end();)
	{
		bAdded = bAdded | add_imp_con_for_given_pred_arc(rob_seq, alt_graph, *it, list_prec_arcs_betw_jobs, layout_graph);
		it = list_prec_arcs_betw_jobs.erase(it);
	}	
	return bAdded;
}

// feasibility, new arc added
std::pair<bool, bool> add_scc_check_coll_feasible(Alternative_Graph &alt_graph, const Collision_Filtering &coll_filter, std::list<arc> &list_prec_arcs_betw_jobs, const Layout_LS &layout_graph)
{
	bool bAdded = coll_filter.add_scc_comps(alt_graph, list_prec_arcs_betw_jobs);
	const auto& list_scc = coll_filter.get_scc();
	size_t uiRobot1, uiRobot2;

	for (auto it_succ_list = list_scc.begin(); it_succ_list != list_scc.end(); it_succ_list++)
	{
		for (auto it_vtx1 = it_succ_list->begin(); it_vtx1 != it_succ_list->end(); it_vtx1++)
		{
			uiRobot1 = alt_graph.get_vertex_ownership(*it_vtx1);
			auto it_vtx2 = it_vtx1;
			it_vtx2++;

			for (; it_vtx2 != it_succ_list->end(); it_vtx2++)
			{
				uiRobot2 = alt_graph.get_vertex_ownership(*it_vtx2);
				if (true == layout_graph.areColliding(Coll_Pair(*it_vtx1, uiRobot1, *it_vtx2, uiRobot2))) return std::make_pair(false, bAdded);
			}
		}
	}
	return std::make_pair(true, bAdded);
}

bool add_enabling_coll_cons(const std::vector<std::list<size_t>> &rob_seq, const Layout_LS &layout_graph, Alternative_Graph &alt_graph, Collision_Filtering &coll_filter, std::unordered_map<size_t, std::unordered_map<size_t, std::pair<size_t, size_t>>> &map_enabler_pos_vert)
{
	bool bChange = true, bFirstIter = true, bFeasible;
	std::list<arc> list_prec_arcs_betw_jobs;
	std::vector<std::list<size_t>> vec_dep_vert;

	for (size_t uiRobot = 0; uiRobot < rob_seq.size(); uiRobot++)
	{
		vec_dep_vert.emplace_back(std::list<size_t>());
	}
		
	while (bChange)
	{
		bChange = false;
		if (bFirstIter)
		{
			bFeasible = add_enabling_cons(rob_seq, layout_graph, alt_graph, list_prec_arcs_betw_jobs, map_enabler_pos_vert, vec_dep_vert);
			if (false == bFeasible) return false;
		}
		
		bFeasible = coll_filter.Check_Feasibility_Compute_Bounds_For_Each_Vertex(rob_seq, alt_graph);
		if (false == bFeasible) return false;

		/*
		if (bFirstIter)
		{
			bFeasible = get_coll_cons(rob_seq, layout_graph, alt_graph, set_coll, coll_filter);
			if (false == bFeasible) return false;
		}
		*/

		auto pr = add_scc_check_coll_feasible(alt_graph, coll_filter, list_prec_arcs_betw_jobs, layout_graph);
		bFeasible = pr.first;
		if (false == bFeasible) return false;
		bChange = bChange | pr.second;

		bChange = bChange | add_impl_cons_rem_prev_cons(rob_seq, alt_graph, list_prec_arcs_betw_jobs, layout_graph);
		
		bFirstIter = false;
	}
	return true;
}

bool Greedy_Heuristic::construct_Alt_Graph(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::list<size_t>> &new_rob_seq)
{
	m_alt_graph.allocate_buffer_for_graph(rob_seq);
	construct_prec_graph_for_each_operation(rob_seq, m_graph, m_alt_graph);
	bool bFeasible = add_enabling_coll_cons(rob_seq, m_graph, m_alt_graph, m_coll_filter, m_map_enabler_pos_vert);
	if (false == bFeasible) return false;

	new_rob_seq = rob_seq;
	bFeasible = add_coll_cons(rob_seq, m_graph, m_alt_graph, m_coll_filter);	

	if (false == bFeasible) return false;
	return bFeasible;
}




