#include "Greedy_Heuristic.h"
#include "Kosaraju_Algo.h"

bool Greedy_Heuristic::construct_Alt_Graph_STN(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::list<size_t>> &new_rob_seq, const size_t c_uiUpperBound)
{
	bool bFeasible = construct_Alt_Graph(rob_seq, new_rob_seq, c_uiUpperBound);
	if (false == bFeasible) return false;
	return true;
}

void Greedy_Heuristic::construct_prec_graph_for_each_operation(const std::vector<std::list<size_t>> &rob_seq)
{
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		size_t uiInd = 0;
		auto it_next = rob_seq[uiRobot].cbegin();
		for (auto it = rob_seq[uiRobot].cbegin(); it != rob_seq[uiRobot].cend(); it++)
		{
			it_next++;
			if (it_next == rob_seq[uiRobot].cend())
			{
				m_alt_graph.add_vertex_ownership_pos(*it, uiRobot , uiInd);
				//assert(DEPOT_TIME == getTime(*it));
				break;
			}

			m_alt_graph.add_prec_arc(*it, *it_next, getTime(*it));
			m_alt_graph.add_vertex_ownership_pos(*it, uiRobot, uiInd);
			uiInd++;
		}		
	}
}

void Greedy_Heuristic::get_verts_not_self_enabled(size_t uiRobot, const std::vector<std::list<size_t>> &rob_seq, std::list<size_t>& list_dep_vert)
{
	std::unordered_set<size_t> comp_vert;
	size_t uiVert;
	bool bFound;
	auto &vec_enabler = m_graph.get_Enablers();

	auto it = rob_seq[uiRobot].begin();
	comp_vert.emplace(*it);
	it++;

	for (; it != rob_seq[uiRobot].end(); it++)
	{
		bFound = false;
		uiVert = *it;

		if ("H" != m_graph.getType(uiVert)) continue;

		for (auto it_enabler = vec_enabler[uiVert].set.begin(); it_enabler != vec_enabler[uiVert].set.end(); it_enabler++)
		{
			if (comp_vert.find(it_enabler->getInd()) != comp_vert.end())
			{
				bFound = true;
				break;
			}
		}
		if (!bFound) {
			//dep_vert.emplace(uiVert);
			list_dep_vert.emplace_back(uiVert);
		}
		comp_vert.emplace(uiVert);
	}
}

bool Greedy_Heuristic::add_prec_arcs_for_dep_vert_of_job(size_t uiGivenRobot, const std::vector<std::list<size_t>> &rob_seq, std::list<arc> &list_prec_arcs_betw_jobs, std::list<size_t> &list_dep_vert)
{
	size_t uiNumRobots = m_graph.get_num_robots(), uiEnablerRobot;
	size_t uiVert, uiEnablerVtx;
	size_t uiEnablerPos;
	std::vector<size_t> vec_pos_enabler;
	auto &vec_enabler = m_graph.get_Enablers();
	
	//for (auto it_vert = dep_vert.begin(); it_vert != dep_vert.end(); it_vert++)
	for(auto it_vert = list_dep_vert.begin(); it_vert != list_dep_vert.end(); )  //notice for loop has been changed this way because of deletions while iterating
	{
		vec_pos_enabler.clear();
		uiVert = *it_vert;
		
		if (m_map_enabler_pos_vert.end() != m_map_enabler_pos_vert.find(uiVert)) m_map_enabler_pos_vert.at(uiVert).clear();
		else m_map_enabler_pos_vert.emplace(uiVert, std::unordered_map<size_t, std::pair<size_t, size_t>>());

		//perhaps more effecient to do this way since enablers are in general fewer than number of vertices
		for (auto it_enabler = vec_enabler.at(uiVert).set.begin(); it_enabler != vec_enabler.at(uiVert).set.end(); it_enabler++)
		{
			uiEnablerVtx = it_enabler->getInd();
			if (false == isEnablerHolePresent(uiEnablerVtx)) continue;

			uiEnablerRobot = m_alt_graph.get_vertex_ownership(uiEnablerVtx);
			if (uiGivenRobot == uiEnablerRobot) continue; 
			uiEnablerPos = m_alt_graph.get_vertex_position(uiEnablerVtx);

			uiEnablerPos++; // we need to store vertex following the enabler;
			auto res = m_alt_graph.get_next_vtx_same_job(uiEnablerVtx);
			assert(true == res.first);
			uiEnablerVtx = res.second;

			auto it_find = m_map_enabler_pos_vert.at(uiVert).find(uiEnablerRobot);
			if (m_map_enabler_pos_vert.at(uiVert).end() == it_find)
			{
				m_map_enabler_pos_vert.at(uiVert).emplace(uiEnablerRobot, std::make_pair(uiEnablerPos, uiEnablerVtx));
			}
			else if (it_find->second.first > uiEnablerPos)
			{
				m_map_enabler_pos_vert.at(uiVert)[uiEnablerRobot] = std::make_pair(uiEnablerPos, uiEnablerVtx);
			}
		}

		// this step below is crucial, we have to populate vec_pos_enabler correctly (exactly earliest enabler from each robot)
		//for robots that do not have any enablers to the given vertex, we enter infinty values
		for (size_t uiRobot = 0; uiRobot < uiNumRobots; uiRobot++)
		{
			if (uiGivenRobot == uiRobot) continue;
			auto it_find = m_map_enabler_pos_vert.at(uiVert).find(uiRobot);
			if (m_map_enabler_pos_vert.at(uiVert).end() == it_find)
			{
				m_map_enabler_pos_vert.at(uiVert).emplace(uiRobot, std::make_pair(std::numeric_limits<size_t>::max(), std::numeric_limits<size_t>::max()));
			}
			else vec_pos_enabler.emplace_back(it_find->second.second);			
		}
		
#ifdef ENABLE_FULL_CHECKING
		for (size_t uiRobot = 0; uiRobot < uiNumRobots; uiRobot++)
		{
			if (uiGivenRobot == uiRobot) continue;
			
			size_t uiPos = 0;
			for (auto it = rob_seq[uiRobot].begin(); it != rob_seq[uiRobot].end(); it++, uiPos++)
			{
				//map_enabler_pos_vert.emplace(uiVert, std::unordered_map<size_t, std::pair<size_t, size_t>>());
				if (vec_enabler.at(uiVert).set.find(*it) != vec_enabler.at(uiVert).set.end())
				{
					it++;	// we need to store the vertex following the enabler
					uiPos++; // position also needs to be reflected
					//vec_pos_enabler.emplace_back(*it);
					
					if ( (map_enabler_pos_vert.at(uiVert).at(uiRobot).first != uiPos) || (map_enabler_pos_vert.at(uiVert).at(uiRobot).second != *it))
					{
#ifdef WINDOWS
						cout << "Something wrong with enabler code above \n";
						assert(true);
#else
						cout << "Something wrong with enabler code above \n";
						exit(-1);
#endif
					}
					
					//map_enabler_pos_vert.at(uiVert).emplace(uiRobot, std::make_pair(uiPos, *it)); // we need to store the position following the vertex
					break;
				}				
			}
		}
#endif

		if (0 == vec_pos_enabler.size()) return false;
		else if (1 == vec_pos_enabler.size()) 
		{ 
			m_alt_graph.add_prec_arc(vec_pos_enabler[0], uiVert, 0);
			list_prec_arcs_betw_jobs.emplace_back(arc(vec_pos_enabler[0], uiVert));
			it_vert = list_dep_vert.erase(it_vert);	
			size_t uiErase = m_map_enabler_pos_vert.erase(uiVert); //because this vertex is no longer dependent, we can skip storing its enabler positions
#ifdef WINDOWS
			assert(1 == uiErase);
#else
			if (1 != uiErase)
			{
				cout << "Something wrong with enabling\n";
				exit(-1);
			}
#endif
			continue;
		}		
		it_vert++;
	}
	return true;
}

bool Greedy_Heuristic::add_enabling_cons(const std::vector<std::list<size_t>> &rob_seq, std::list<arc> &list_prec_arcs_betw_jobs, std::vector<std::list<size_t>> &vec_dep_vert)
{
	size_t uiNumRobots = m_graph.get_num_robots();
	bool bFeasible;

	for (size_t uiRobot = 0; uiRobot < uiNumRobots; uiRobot++)
	{
		get_verts_not_self_enabled(uiRobot, rob_seq, vec_dep_vert[uiRobot]);
		bFeasible = add_prec_arcs_for_dep_vert_of_job(uiRobot, rob_seq, list_prec_arcs_betw_jobs, vec_dep_vert[uiRobot]);
		if (false == bFeasible) return false;
	}
	return true;
}

bool Greedy_Heuristic::check_if_new_precedences_can_be_added(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::list<size_t>> &vec_dep_vert)
{
	size_t uiNumRobots = m_graph.get_num_robots();
	size_t uiVtx, uiVtxPos, uiEnablerPos, uiEnablerVtx;
	bool bResolved, bUpperBounded;

	for (size_t uiRobot1 = 0; uiRobot1 < uiNumRobots; uiRobot1++)
	{
		for (auto it = vec_dep_vert[uiRobot1].begin(); it != vec_dep_vert[uiRobot1].end(); )
		{
			uiVtx = *it;
			bResolved = false;
			bUpperBounded = true;  // checks if all the enablers of vtx V are guaranteed to occur after V
			uiVtxPos = m_alt_graph.get_vertex_position(*it);

			for (size_t uiRobot2 = 0; uiRobot2 < uiNumRobots; uiRobot2++)
			{
				if (uiRobot1 == uiRobot2) continue;
				
				auto it_enabler = m_map_enabler_pos_vert.at(uiVtx).at(uiRobot2);
				uiEnablerPos = it_enabler.first;
				uiEnablerVtx = it_enabler.second;

				if (m_coll_filter.get_lower_bound_pos(uiVtx, uiRobot2) >= uiEnablerPos)
				{
					bResolved = true;
					break;
				}
				else if (m_coll_filter.get_lower_bound_pos(uiEnablerVtx, uiRobot1) <= uiVtxPos) bUpperBounded = false;
			}

			if (true == bUpperBounded) return false; //infeasible, implies all enablers are bound to occur later

			if (bResolved) it = vec_dep_vert[uiRobot1].erase(it);
			else it++;
		}
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

bool Greedy_Heuristic::add_coll_cons_bet_pair_jobs(size_t uiRobot1, size_t uiRobot2, const std::vector<std::list<size_t>> &rob_seq, const std::vector<std::vector<size_t>> &vec_cost_from_source, const std::vector<std::vector<size_t>> &vec_cost_to_go, const size_t c_uiUpperBound)
{
	size_t uiRob_1_Pos, uiRob_2_Pos;

	for (auto it1 = rob_seq[uiRobot1].begin(); it1 != rob_seq[uiRobot1].end(); it1++)
	{
		auto it12 = it1;
		it12++;

		auto pr1 = m_coll_filter.get_lower_bound_pos(*it1, uiRobot2);
		auto it2_start = rob_seq[uiRobot2].begin();
		std::advance(it2_start, pr1);
		
		uiRob_1_Pos = m_alt_graph.get_vertex_position(*it1);
		uiRob_2_Pos = pr1;

		for (auto it2 = it2_start; it2 != rob_seq[uiRobot2].end(); it2++ , uiRob_2_Pos++)
		{
			auto pr2 = m_coll_filter.get_lower_bound_pos(*it2, uiRobot1);
			if (uiRob_1_Pos < pr2) break;			

			if (m_graph.areColliding(Coll_Pair(*it1, uiRobot1, *it2, uiRobot2)))
			{
				auto it22 = it2;
				it22++;

				bool bArc1 = m_alt_graph.containsPrecArc(arc(*it22, *it1));
				bool bArc2 = m_alt_graph.containsPrecArc(arc(*it12, *it2));

				if (!bArc1 & !bArc2)
				{
					//note - definitions of bArc1 and bArc2 are somewhat different from the usage above
					if (vec_cost_from_source[uiRobot2][uiRob_2_Pos + 1] + vec_cost_to_go[uiRobot1][uiRob_1_Pos] > c_uiUpperBound) bArc1 = true;
					if (vec_cost_from_source[uiRobot1][uiRob_1_Pos + 1] + vec_cost_to_go[uiRobot2][uiRob_2_Pos] > c_uiUpperBound) bArc2 = true;

					if (bArc1 & bArc2) return false;  // infeasible by bounding
					else if (bArc1 & !bArc2) m_alt_graph.add_prec_arc(*it12, *it2, 0); // pruned, arc(*it22, *it1) will shoot upper bound
					else if (!bArc1 & bArc2) m_alt_graph.add_prec_arc(*it22, *it1, 0); // pruned, arc(*it12, *it2) will shoot upper bound
					else m_alt_graph.add_alt_arc(*it22, *it1, *it12, *it2);
				}
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

bool Greedy_Heuristic::get_coll_cons_bet_pair_jobs(size_t uiRobot1, size_t uiRobot2, const std::vector<std::list<size_t>> &rob_seq, std::unordered_set<Coll_Pair, CollHasher> &set_coll)
{
	size_t uiRob_1_Pos;
	for (auto it1 = rob_seq[uiRobot1].begin(); it1 != rob_seq[uiRobot1].end(); it1++)
	{
		auto it12 = it1;
		it12++;

		auto pr1 = m_coll_filter.get_lower_bound_pos(*it1, uiRobot2);
		auto it2_start = rob_seq[uiRobot2].begin();
		std::advance(it2_start, pr1);
		uiRob_1_Pos = m_alt_graph.get_vertex_position(*it1);

		for (auto it2 = it2_start; it2 != rob_seq[uiRobot2].end(); it2++)
		{
			auto pr2 = m_coll_filter.get_lower_bound_pos(*it2, uiRobot1);
			if (uiRob_1_Pos < pr2) break;

			if (m_graph.areColliding(Coll_Pair(*it1, uiRobot1, *it2, uiRobot2)))
			{
				auto it22 = it2;
				it22++;

				bool bArc1 = m_alt_graph.containsPrecArc(arc(*it22, *it1));
				bool bArc2 = m_alt_graph.containsPrecArc(arc(*it12, *it2));

				if (!bArc1 & !bArc2) set_coll.emplace(Coll_Pair(*it1, uiRobot1, *it2, uiRobot2));
				else if (bArc1 & bArc2) return false;
			}
		}		
	}
	return true;
}

bool Greedy_Heuristic::add_coll_cons(const std::vector<std::list<size_t>> &rob_seq, const std::vector<std::vector<size_t>> &vec_cost_from_source, const std::vector<std::vector<size_t>> &vec_cost_to_go, const size_t c_uiUpperBound)
{
	size_t uiNumRobots = m_graph.get_num_robots();
	bool bFeasible;

	for (size_t uiRobot1 = 0; uiRobot1 < uiNumRobots; uiRobot1++)
	{
		for (size_t uiRobot2 = uiRobot1+1; uiRobot2 < uiNumRobots; uiRobot2++)
		{
			bFeasible = add_coll_cons_bet_pair_jobs(uiRobot1, uiRobot2, rob_seq, vec_cost_from_source, vec_cost_to_go, c_uiUpperBound);
			if (false == bFeasible) return false;
		}
	}
	return true;
}

bool Greedy_Heuristic::get_coll_cons(const std::vector<std::list<size_t>> &rob_seq, std::unordered_set<Coll_Pair, CollHasher> &set_coll)
{
	size_t uiNumRobots = m_graph.get_num_robots();
	bool bFeasible;

	for (size_t uiRobot1 = 0; uiRobot1 < uiNumRobots; uiRobot1++)
	{
		for (size_t uiRobot2 = uiRobot1 + 1; uiRobot2 < uiNumRobots; uiRobot2++)
		{
			bFeasible = get_coll_cons_bet_pair_jobs(uiRobot1, uiRobot2, rob_seq, set_coll);
			if (false == bFeasible) return false;
		}
	}
	return true;
}

bool Greedy_Heuristic::add_imp_prec_coll_con(const std::vector<std::list<size_t>> &rob_seq, const arc &inp_arc, std::list<arc> &list_prec_arcs_betw_jobs)
{
	size_t uiTailStartVtx = inp_arc.first;
	size_t uiHeadStartVtx = inp_arc.second;

	const size_t c_uiRobot_Tail = m_alt_graph.get_vertex_ownership(uiTailStartVtx);
	const size_t c_uiRobot_Head = m_alt_graph.get_vertex_ownership(uiHeadStartVtx);

	if (uiTailStartVtx == *rob_seq[c_uiRobot_Tail].rbegin()) return false;
	
	//if (set_coll.end() != set_coll.find(Coll_Pair(uiTailStartVtx, uiRobot_Tail, uiHeadStartVtx, uiRobot_Head)))
	if(true == m_graph.areColliding(Coll_Pair(uiTailStartVtx, c_uiRobot_Tail, uiHeadStartVtx, c_uiRobot_Head)))
	{
		auto pr = m_alt_graph.get_next_vtx_same_job(uiTailStartVtx);
		assert(true == pr.first);
		size_t uiNewTailVtx = pr.second;
		if (false == m_alt_graph.containsPrecArc(arc(uiNewTailVtx, uiHeadStartVtx)))
		{
			m_alt_graph.add_prec_arc(uiNewTailVtx, uiHeadStartVtx, 0);
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

bool Greedy_Heuristic::add_imp_con_for_given_prec_arc_forward_dir(const std::vector<std::list<size_t>> &rob_seq, const arc &inp_arc, std::list<arc> &list_prec_arcs_betw_jobs)
{
	bool bAdded = false;
	const size_t c_uiTailStartVtx = inp_arc.first;
	const size_t c_uiHeadStartVtx = inp_arc.second;

	const size_t c_uiRobot_Tail = m_alt_graph.get_vertex_ownership(c_uiTailStartVtx);
	const size_t c_uiRobot_Head = m_alt_graph.get_vertex_ownership(c_uiHeadStartVtx);

	if (c_uiTailStartVtx == *rob_seq[c_uiRobot_Tail].rbegin()) return false;
	if (c_uiHeadStartVtx == *rob_seq[c_uiRobot_Head].rbegin()) return false;

	auto pr_head = m_alt_graph.get_next_vtx_same_job(c_uiHeadStartVtx);
	assert(true == pr_head.first);
	size_t uiHeadVtx = pr_head.second;

	auto pr_tail = m_alt_graph.get_next_vtx_same_job(c_uiTailStartVtx);
	assert(true == pr_tail.first);
	const size_t c_uiTailVtx = pr_tail.second;

	while (1)
	{
		//if (set_coll.end() != set_coll.find(Coll_Pair(c_uiTailStartVtx, uiRobot_Tail, uiHeadVtx, uiRobot_Head)))
		if(true == m_graph.areColliding(Coll_Pair(c_uiTailStartVtx, c_uiRobot_Tail, uiHeadVtx, c_uiRobot_Head)))
		{
			if (false == m_alt_graph.containsPrecArc(arc(c_uiTailVtx, uiHeadVtx)))
			{
				m_alt_graph.add_prec_arc(c_uiTailVtx, uiHeadVtx, 0);
				list_prec_arcs_betw_jobs.emplace_back(arc(c_uiTailVtx, uiHeadVtx));
				bAdded = true;
			}
			break; // remaining constraints if added will only be redundant
		}

		pr_head = m_alt_graph.get_next_vtx_same_job(uiHeadVtx);
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

bool Greedy_Heuristic::add_imp_con_for_given_prec_arc_reverse_dir(const std::vector<std::list<size_t>> &rob_seq, const arc &inp_arc, std::list<arc> &list_prec_arcs_betw_jobs)
{
	bool bAdded = false;
	const size_t c_uiTailStartVtx = inp_arc.first;
	const size_t c_uiHeadStartVtx = inp_arc.second;

	const size_t c_uiRobot_Tail = m_alt_graph.get_vertex_ownership(c_uiTailStartVtx);
	const size_t c_uiRobot_Head = m_alt_graph.get_vertex_ownership(c_uiHeadStartVtx);

	if (c_uiTailStartVtx == *rob_seq[c_uiRobot_Tail].begin()) return false;
	if (c_uiHeadStartVtx == *rob_seq[c_uiRobot_Head].begin()) return false;

	auto pr_head = m_alt_graph.get_prec_vtx_same_job(c_uiHeadStartVtx);
	assert(true == pr_head.first);
	const size_t c_uiHeadVtx = pr_head.second;

	auto pr_tail = m_alt_graph.get_prec_vtx_same_job(c_uiTailStartVtx);
	assert(true == pr_tail.first);
	size_t uiTailVtx = pr_tail.second;
	size_t uiTailSucc = c_uiTailStartVtx;
	
	while (1)
	{
		//if (set_coll.end() != set_coll.find(Coll_Pair(uiTailVtx, uiRobot_Tail, c_uiHeadVtx, uiRobot_Head)))
		if(true == m_graph.areColliding(Coll_Pair(uiTailVtx, c_uiRobot_Tail, c_uiHeadVtx, c_uiRobot_Head)))
		{
			if (false == m_alt_graph.containsPrecArc(arc(uiTailSucc, c_uiHeadVtx)))
			{
				m_alt_graph.add_prec_arc(uiTailSucc, c_uiHeadVtx, 0);
				list_prec_arcs_betw_jobs.emplace_back(arc(uiTailSucc, c_uiHeadVtx));
				bAdded = true;
			}
			break; // remaining constraints if added will only be redundant
		}

		pr_tail = m_alt_graph.get_prec_vtx_same_job(uiTailVtx);
		if (false == pr_tail.first) break;
		uiTailSucc = uiTailVtx;
		uiTailVtx = pr_tail.second;		
	}
	return bAdded;
}

bool Greedy_Heuristic::add_imp_con_for_given_pred_arc(const std::vector<std::list<size_t>> &rob_seq, const arc &inp_arc, std::list<arc> &list_prec_arcs_betw_jobs)
{
	bool bAdded;
	bAdded = add_imp_prec_coll_con(rob_seq, inp_arc, list_prec_arcs_betw_jobs);
	bAdded = bAdded | add_imp_con_for_given_prec_arc_forward_dir(rob_seq, inp_arc, list_prec_arcs_betw_jobs);
	bAdded = bAdded | add_imp_con_for_given_prec_arc_reverse_dir(rob_seq, inp_arc, list_prec_arcs_betw_jobs);
	return bAdded;
}

//adds new implied precedence constraints and removes previously considered precedence constraints
bool Greedy_Heuristic::add_impl_cons_rem_prev_cons(const std::vector<std::list<size_t>> &rob_seq, std::list<arc> &list_prec_arcs_betw_jobs)
{
	bool bAdded = false;
	for (auto it = list_prec_arcs_betw_jobs.begin(); it != list_prec_arcs_betw_jobs.end();)
	{
		bAdded = bAdded | add_imp_con_for_given_pred_arc(rob_seq, *it, list_prec_arcs_betw_jobs);
		it = list_prec_arcs_betw_jobs.erase(it);
	}	
	return bAdded;
}

// feasibility, new arc added
std::pair<bool, bool> Greedy_Heuristic::add_scc_check_coll_feasible(std::list<arc> &list_prec_arcs_betw_jobs)
{
	bool bAdded = m_coll_filter.add_scc_comps(m_alt_graph, list_prec_arcs_betw_jobs);
	const auto& list_scc = m_coll_filter.get_scc();
	size_t uiRobot1, uiRobot2;

	for (auto it_succ_list = list_scc.begin(); it_succ_list != list_scc.end(); it_succ_list++)
	{
		for (auto it_vtx1 = it_succ_list->begin(); it_vtx1 != it_succ_list->end(); it_vtx1++)
		{
			uiRobot1 = m_alt_graph.get_vertex_ownership(*it_vtx1);
			auto it_vtx2 = it_vtx1;
			it_vtx2++;

			for (; it_vtx2 != it_succ_list->end(); it_vtx2++)
			{
				uiRobot2 = m_alt_graph.get_vertex_ownership(*it_vtx2);
				if (true == m_graph.areColliding(Coll_Pair(*it_vtx1, uiRobot1, *it_vtx2, uiRobot2))) return std::make_pair(false, bAdded);
			}
		}
	}
	return std::make_pair(true, bAdded);
}

bool Greedy_Heuristic::add_enabling_fix_coll_cons(const std::vector<std::list<size_t>> &rob_seq)
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
			bFeasible = add_enabling_cons(rob_seq, list_prec_arcs_betw_jobs, vec_dep_vert);
			if (false == bFeasible) return false;
			bFirstIter = false;
		}
		else
		{
			//check if new implications can be added- useful for > 2 robots case
			bFeasible = check_if_new_precedences_can_be_added(rob_seq, vec_dep_vert);
			if (false == bFeasible) return false;
		}
		
		bFeasible = m_coll_filter.Check_Feasibility_Compute_Bounds_For_Each_Vertex(rob_seq, m_alt_graph);
		if (false == bFeasible) return false;

		/*
		if (bFirstIter)
		{
			bFeasible = get_coll_cons(rob_seq, layout_graph, alt_graph, set_coll, coll_filter);
			if (false == bFeasible) return false;
		}
		*/

		auto pr = add_scc_check_coll_feasible(list_prec_arcs_betw_jobs);
		bFeasible = pr.first;
		if (false == bFeasible) return false;
		bChange = bChange | pr.second;

		bChange = bChange | add_impl_cons_rem_prev_cons(rob_seq, list_prec_arcs_betw_jobs);		
	}
	return true;
}

void Check_costs(const std::vector<std::vector<size_t>> &vec_cost_from_source, const std::vector<std::vector<size_t>> vec_cost_to_go)
{
	size_t c_uiNumRobots = vec_cost_from_source.size();
	size_t uiMaxFrom = std::numeric_limits<size_t>::min(), uiMaxGo = std::numeric_limits<size_t>::min();

	for (size_t uiRobot = 0; uiRobot < c_uiNumRobots; uiRobot++)
	{
		uiMaxFrom = std::max(uiMaxFrom , vec_cost_from_source[uiRobot][vec_cost_from_source[uiRobot].size()-1]);
		uiMaxGo = std::max(uiMaxGo , vec_cost_to_go[uiRobot][0]);
	}

#ifdef WINDOWS
	assert(uiMaxFrom == uiMaxGo);
#else
	if (uiMaxFrom != uiMaxGo)
	{
		cout << "Cost computations incorrect";
		exit(-1);
	}
#endif


	for (size_t uiRobot = 0; uiRobot < c_uiNumRobots; uiRobot++)
	{
		for (size_t uiCount = 0; uiCount < vec_cost_from_source[uiRobot].size()-1; uiCount++)
		{
#ifdef WINDOWS
			assert(vec_cost_from_source[uiRobot][uiCount] < vec_cost_from_source[uiRobot][uiCount+1]);
			assert(vec_cost_to_go[uiRobot][uiCount + 1] < vec_cost_to_go[uiRobot][uiCount]);
#else
			if ((vec_cost_from_source[uiRobot][uiCount] >= vec_cost_from_source[uiRobot][uiCount + 1]) || (vec_cost_to_go[uiRobot][uiCount + 1] >= vec_cost_to_go[uiRobot][uiCount]))
			{
				cout << "Cost computations incorrect \n";
				exit(-1);
			}
#endif
		}
	}
}

bool Greedy_Heuristic::construct_Alt_Graph(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::list<size_t>> &new_rob_seq, const size_t c_uiUpperBound)
{
	m_alt_graph.allocate_buffer_for_graph(rob_seq);
	construct_prec_graph_for_each_operation(rob_seq);
	bool bFeasible = add_enabling_fix_coll_cons(rob_seq);
	if (false == bFeasible) return false;

	//buffers for cost based filtering
	// this filtering is not done here instead of in the loop because we prefer not to enumerate all collisions earlier
	std::vector<std::vector<size_t>> vec_cost_from_source;
	std::vector<std::vector<size_t>> vec_cost_to_go;
	size_t uiMakeSpan = m_coll_filter.Compute_costs_for_each_Vertex(rob_seq, m_alt_graph, vec_cost_from_source, vec_cost_to_go);
	if (uiMakeSpan > c_uiUpperBound) return false;

#ifdef ENABLE_FULL_CHECKING	
	Check_costs(vec_cost_from_source, vec_cost_to_go);
#endif

	new_rob_seq = rob_seq;
	bFeasible = add_coll_cons(rob_seq, vec_cost_from_source, vec_cost_to_go, c_uiUpperBound);	

	if (false == bFeasible) return false;
	return bFeasible;
}




