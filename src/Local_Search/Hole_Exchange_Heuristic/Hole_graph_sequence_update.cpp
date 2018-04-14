#include "Hole_exchanges.h"

bool Hole_Exchange::update_sequence_graphs_for_removal(const size_t c_uiHole, const size_t c_uiRobot, const std::vector<std::list<size_t>> &rob_sub_seq, std::pair<size_t, size_t> &taboo_hole_pair)
{
	// it is required that m_rob_seq is not updated with the removed hole at this step
	update_out_in_graphs_for_removal(c_uiHole, c_uiRobot, rob_sub_seq);
	//update m_rob_seq
	auto res = remove_hole_update_rob_sequence(c_uiHole, c_uiRobot);
	taboo_hole_pair = res;

	//resolve collisions and enablers by adding arcs
	bool bFeasible = make_solution_feasible(rob_sub_seq);
	return bFeasible;
}

bool Hole_Exchange::update_sequence_graphs_for_insertion(const size_t c_uiHole, const std::pair<size_t, size_t> pr_hole_pair, const size_t c_uiRobot, const std::vector<std::list<size_t>> &rob_sub_seq)
{
	// it is required that m_rob_seq is not updated with the removed hole at this step
	update_out_in_graphs_for_insertion(c_uiHole, pr_hole_pair, c_uiRobot, rob_sub_seq);
	//update m_rob_seq
	insert_hole_update_rob_sequence(c_uiHole, pr_hole_pair, c_uiRobot);

	//resolve collisions and enablers by adding arcs
	bool bFeasible = make_solution_feasible(rob_sub_seq);
	return bFeasible;
}

std::pair<size_t, size_t> Hole_Exchange::remove_hole_update_rob_sequence(const size_t c_uiHole, const size_t c_uiRobot)
{
	return remove_INP_HOLE_in_rob_sub_seq(c_uiHole, c_uiRobot, m_rob_seq, m_graph);	
}

void Hole_Exchange::insert_hole_update_rob_sequence(const size_t c_uiHole, const std::pair<size_t, size_t> pr_hole_pair, const size_t c_uiRobot)
{
	insert_INP_HOLE_in_rob_seq(c_uiHole, c_uiRobot, pr_hole_pair, m_rob_seq, m_graph);
}

void Hole_Exchange::populate_removed_hole_prev_next_iv(const size_t c_uiHole, const size_t c_uiRobot, std::set<size_t> &set_vts)
{
	set_vts.emplace(c_uiHole);
	auto it = std::find(m_rob_seq[c_uiRobot].begin(), m_rob_seq[c_uiRobot].end(), c_uiHole);
	auto it_prev_IV = it;
	it_prev_IV--;
	while ("IV" == m_graph.getType(*it_prev_IV))
	{
		set_vts.emplace(*it_prev_IV);
		it_prev_IV--;
	}
	
	auto it_next_IV = it;
	it_next_IV++;
	while ("IV" == m_graph.getType(*it_next_IV))
	{
		set_vts.emplace(*it_next_IV);
		it_next_IV++;
	}
}

void Hole_Exchange::populate_IV_insertion_betw_holes(const std::pair<size_t, size_t> pr_hole_pair, const size_t c_uiRobot, std::set<size_t> &set_vts)
{
	assert(0 == set_vts.size());
	const auto &vec_rob_iv = m_graph.get_IV_Vec();
	const auto &vec_iv = vec_rob_iv[c_uiRobot].map.at(pr_hole_pair.first).map.at(pr_hole_pair.second).vec;
	for (auto it_iv = vec_iv.begin(); it_iv != vec_iv.end(); it_iv++)
	{
		set_vts.emplace(it_iv->getInd());
	}
}

void Hole_Exchange::update_out_in_graphs_for_removal(const size_t c_uiHole, const size_t c_uiRobot, const std::vector<std::list<size_t>> &rob_sub_seq)
{
	std::set<size_t> set_rob_sub_seq_vts;
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		std::copy(rob_sub_seq[uiRobot].begin(), rob_sub_seq[uiRobot].end(), std::inserter(set_rob_sub_seq_vts, set_rob_sub_seq_vts.end()));
	}

	// rob_sub_seq does not contain IV - c_uiHole- IV, bur m_rob_seq does and by extension the graph also does.
	std::set<size_t> set_addtl_vts_to_remove;
	populate_removed_hole_prev_next_iv(c_uiHole, c_uiRobot, set_addtl_vts_to_remove);

	//copy the removed holes and its imeediate predecessor sucessor IVs into set_rob_sub_seq verts
	//the basic function of set_rob_sub_seq is to identify those vertices that need to be first removed
	//fromt the original graph.
	std::copy(set_addtl_vts_to_remove.begin(), set_addtl_vts_to_remove.end(), std::inserter(set_rob_sub_seq_vts, set_rob_sub_seq_vts.end()));

	perform_out_in_graph_modifications(set_rob_sub_seq_vts, rob_sub_seq);
}

void Hole_Exchange::update_out_in_graphs_for_insertion(const size_t c_uiHole, const std::pair<size_t, size_t> pr_hole_pair, const size_t c_uiRobot, const std::vector<std::list<size_t>> &rob_sub_seq)
{
	std::set<size_t> set_rob_sub_seq_vts;
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		std::copy(rob_sub_seq[uiRobot].begin(), rob_sub_seq[uiRobot].end(), std::inserter(set_rob_sub_seq_vts, set_rob_sub_seq_vts.end()));
	}

	// rob_sub_seq does not contain IV - c_uiHole- IV, bur m_rob_seq does and by extension the graph also does.
	std::set<size_t> set_addtl_vts_to_remove;
	populate_IV_insertion_betw_holes(pr_hole_pair, c_uiRobot, set_addtl_vts_to_remove);
	
	//copy the removed holes and its imeediate predecessor sucessor IVs into set_rob_sub_seq verts
	//the basic function of set_rob_sub_seq is to identify those vertices that need to be first removed
	//fromt the original graph.
	std::copy(set_addtl_vts_to_remove.begin(), set_addtl_vts_to_remove.end(), std::inserter(set_rob_sub_seq_vts, set_rob_sub_seq_vts.end()));
	perform_out_in_graph_modifications(set_rob_sub_seq_vts, rob_sub_seq);
}

void Hole_Exchange::perform_out_in_graph_modifications(const std::set<size_t> &set_rob_sub_seq_vts, const std::vector<std::list<size_t>> &rob_sub_seq)
{
	//gather arcs between start vertices. These need to be put back when constructing the new graph
	std::vector<arc> vec_arcs_betw_rob_sub_seq_start_vtcs;
	gather_arcs_betw_rob_sub_seq_start_vtcs(vec_arcs_betw_rob_sub_seq_start_vtcs, rob_sub_seq);
	perform_del_vtx_arcs_in_update(set_rob_sub_seq_vts);
	int iConstructionOption = 3;
	perform_patch_rob_sub_seq_graph(iConstructionOption, rob_sub_seq, vec_arcs_betw_rob_sub_seq_start_vtcs);
}

void Hole_Exchange::gather_arcs_betw_rob_sub_seq_start_vtcs(std::vector<arc> &vec_arcs,const std::vector<std::list<size_t>> &rob_sub_seq)
{
	assert(0 == vec_arcs.size());
	size_t uiStartVtx1, uiStartVtx2;
	for (size_t uiRobot1 = 0; uiRobot1 < m_uiNumRobots; uiRobot1++)
	{
		uiStartVtx1 = *rob_sub_seq[uiRobot1].begin();
		for (size_t uiRobot2 = uiRobot1 + 1; uiRobot2 < m_uiNumRobots; uiRobot2++)
		{
			uiStartVtx2 = *rob_sub_seq[uiRobot2].begin();
			if (m_alt_out_graph.at(uiStartVtx1).end() != m_alt_out_graph.at(uiStartVtx1).find(uiStartVtx2))
			{
				vec_arcs.emplace_back(arc(uiStartVtx1, uiStartVtx2));
			}
			if (m_alt_out_graph.at(uiStartVtx2).end() != m_alt_out_graph.at(uiStartVtx2).find(uiStartVtx1))
			{
				vec_arcs.emplace_back(arc(uiStartVtx2, uiStartVtx1));
			}
		}
	}
}

void Hole_Exchange::perform_del_vtx_arcs_in_update(const std::set<size_t> &set_rob_sub_seq_vts)
{
	//remove arcs that need to be removed
	for (auto it_sub_seq_verts = set_rob_sub_seq_vts.begin(); it_sub_seq_verts != set_rob_sub_seq_vts.end(); it_sub_seq_verts++)
	{
		size_t uiVert = *it_sub_seq_verts;
		
		auto it_find = m_alt_in_graph.find(uiVert);
		if (m_alt_in_graph.end() == it_find) continue;

		for (auto it_in = m_alt_in_graph.at(uiVert).begin(); it_in != m_alt_in_graph.at(uiVert).end(); )
		{
			m_alt_out_graph.at(it_in->first).erase(uiVert);
			it_in = m_alt_in_graph.at(uiVert).erase(it_in);
		}

		for (auto it_out = m_alt_out_graph.at(uiVert).begin(); it_out != m_alt_out_graph.at(uiVert).end();)
		{
			m_alt_in_graph.at(it_out->first).erase(uiVert);
			it_out = m_alt_out_graph.at(uiVert).erase(it_out);
		}
	}

	//remove vertices in the new sub sequence and hole that was removed and its predecessor and sucessor IVs
	for (auto it_vtx = set_rob_sub_seq_vts.begin(); it_vtx != set_rob_sub_seq_vts.end(); it_vtx++)
	{
		m_alt_in_graph.erase(*it_vtx);
		m_alt_out_graph.erase(*it_vtx);
	}	
}

void Hole_Exchange::perform_patch_rob_sub_seq_graph(int iOption, const std::vector<std::list<size_t>> &rob_sub_seq, const std::vector<arc> &vec_arcs)
{
	const auto& alt_graph = m_ls_heur.get_complete_alt_graph(iOption);
	//append alternative graph to out graph
	std::copy(alt_graph.getGraph().begin(), alt_graph.getGraph().end(), std::inserter(m_alt_out_graph, m_alt_out_graph.end()));
	// append reverse alternative graph to in graph
	std::copy(alt_graph.getReverseGraph().begin(), alt_graph.getReverseGraph().end(), std::inserter(m_alt_in_graph, m_alt_in_graph.end()));
	
	// add connectors between rob_sub_seq to old graph
	size_t uiStart, uiEnd, uiTime;
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		uiStart = *rob_sub_seq[uiRobot].begin();
		auto it_find = std::find(m_rob_seq[uiRobot].begin(), m_rob_seq[uiRobot].end(), uiStart);
		if (m_rob_seq[uiRobot].begin() != it_find)
		{
			auto it_prev = it_find;
			std::advance(it_prev, -1);
			uiTime = m_graph.getTime(*it_prev);
			auto res = m_alt_out_graph.at(*it_prev).emplace(uiStart, uiTime);
			assert(true == res.second);
			res = m_alt_in_graph.at(uiStart).emplace(*it_prev, uiTime);
			assert(true == res.second);
		}

		uiEnd = *rob_sub_seq[uiRobot].rbegin();
		it_find = std::find(m_rob_seq[uiRobot].begin(), m_rob_seq[uiRobot].end(), uiEnd);
		if (*m_rob_seq[uiRobot].rbegin() != *it_find)
		{
			auto it_next = it_find;
			std::advance(it_next, 1);
			uiTime = m_graph.getTime(uiEnd);
			auto res = m_alt_out_graph.at(uiEnd).emplace(*it_next, uiTime);
			assert(true == res.second);
			res = m_alt_in_graph.at(*it_next).emplace(uiEnd, uiTime);
			assert(true == res.second);
		}
	}

	//add arcs between start vertices of rob_sub_seq. Note all the arcs between rob_sub_seq END already exists because of alt_graph.getGraph()
	size_t uiTail, uiHead;
	for (auto it_start_arcs = vec_arcs.begin(); it_start_arcs != vec_arcs.end(); it_start_arcs++)
	{
		uiTail = it_start_arcs->first;
		uiHead = it_start_arcs->second;

		auto it_insert = m_alt_out_graph.at(uiTail).emplace(uiHead, 0);
		assert(true == it_insert.second);

		it_insert = m_alt_in_graph.at(uiHead).emplace(uiTail, 0);
		assert(true == it_insert.second);
	}
}

bool Hole_Exchange::make_solution_feasible(const std::vector<std::list<size_t>> &rob_sub_seq)
{
	std::set<size_t> set_vts_before_sub_seq;
	std::set<size_t> set_vts_sub_seq;
	std::set<size_t> set_vts_after_sub_seq;
	gather_vertices_before_sub_seq_after(set_vts_before_sub_seq, set_vts_sub_seq, set_vts_after_sub_seq, rob_sub_seq);

	std::unordered_map<size_t, size_t> map_old_start_times = m_map_start_times;
	std::unordered_map<size_t, size_t> map_old_completion_times = m_map_completion_times;

	bool bFeasible = false;
	size_t uiIters = 0;

	while (!bFeasible)
	{
		if (uiIters > c_uiMaxInfeasibleIters) break;
		
		m_map_start_times.clear();

		//if for any reason we decide to remove this step here, take care that vertex slack occuring later 
		//needs it. Currently since we are computing topological ordering here, we do not repeat there
		m_top_order_dist.construct_graph_populate_order_with_dist(m_alt_out_graph, m_alt_in_graph, m_map_start_times);
		
		m_map_completion_times.clear();
		compute_completion_times();

		construct_vertex_schedule();
		construct_state_transition_path();

		bFeasible = resolve_collisions_unenabled_vts_dynamically(set_vts_before_sub_seq, set_vts_sub_seq, set_vts_after_sub_seq, map_old_start_times, map_old_completion_times);
		uiIters++;
	}
	return bFeasible;
}

bool Hole_Exchange::resolve_collisions_unenabled_vts_dynamically(const std::set<size_t> &set_vts_before_sub_seq, const std::set<size_t> &set_vts_sub_seq, const std::set<size_t> &set_vts_after_sub_seq, const std::unordered_map<size_t, size_t> &map_old_start_times, const std::unordered_map<size_t, size_t> &map_old_completion_times)
{
	std::vector<std::list<size_t>::iterator> vec_rob_vtx_itr;
	bool bFeasible;

	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++) vec_rob_vtx_itr.emplace_back(m_rob_seq[uiRobot].begin());

	for (size_t uiState = 0; uiState < m_vec_state_path.size(); uiState++)
	{
		const auto& vec_vtx_in_state = m_vec_state_path[uiState].m_vec_rob_vtx;

		for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
		{
			while (*vec_rob_vtx_itr[uiRobot] != vec_vtx_in_state[uiRobot]) vec_rob_vtx_itr[uiRobot]++;
		}

		check_and_resolve_collision(vec_rob_vtx_itr, set_vts_before_sub_seq, set_vts_sub_seq, set_vts_after_sub_seq, map_old_start_times);
		bFeasible = check_and_resolve_enablers(vec_rob_vtx_itr, map_old_completion_times);
		if (false == bFeasible) return false;
	}
	return true;
}

void Hole_Exchange::check_and_resolve_collision(const std::vector<std::list<size_t>::iterator>& vec_rob_vtx_itr, const std::set<size_t> &set_vts_before_sub_seq, const std::set<size_t> &set_vts_sub_seq, const std::set<size_t> &set_vts_after_sub_seq, const std::unordered_map<size_t, size_t> &map_old_start_times)
{
	size_t uiVtx1, uiVtx2, uiVtx1Next, uiVtx2Next;
	bool bVtx1PrecVtx2;
	for (size_t uiRobot1 = 0; uiRobot1 < m_uiNumRobots; uiRobot1++)
	{
		uiVtx1 = *vec_rob_vtx_itr[uiRobot1];
		for (size_t uiRobot2 = uiRobot1 + 1; uiRobot2 < m_uiNumRobots; uiRobot2++)
		{
			uiVtx2 = *vec_rob_vtx_itr[uiRobot2];
			if (true == m_graph.areColliding(Coll_Pair(uiVtx1, uiRobot1, uiVtx2, uiRobot2)))
			{
				auto it_find1 = map_old_start_times.find(uiVtx1);
				auto it_find2 = map_old_start_times.find(uiVtx2);

				if ((map_old_start_times.end() != it_find1) && (map_old_start_times.end() != it_find2))
				{
					if (it_find1->second <= it_find2->second) bVtx1PrecVtx2 = true;
					else if (it_find2->second <= it_find1->second) bVtx1PrecVtx2 = false;
				}
				else
				{
					bVtx1PrecVtx2 = check_if_vtx1_prec_vtx2_sequence_partition(uiVtx1, uiVtx2, set_vts_before_sub_seq, set_vts_sub_seq, set_vts_after_sub_seq);
				}
			}

			if (bVtx1PrecVtx2)
			{
				auto it_nxt1 = vec_rob_vtx_itr[uiRobot1];
				it_nxt1++;
				uiVtx1Next = *it_nxt1;
				m_alt_out_graph.at(uiVtx1Next).emplace(uiVtx2, 0);
				m_alt_in_graph.at(uiVtx2).emplace(uiVtx1Next, 0);
			}
			else
			{
				auto it_nxt2 = vec_rob_vtx_itr[uiRobot2];
				it_nxt2++;
				uiVtx2Next = *it_nxt2;
				m_alt_out_graph.at(uiVtx2Next).emplace(uiVtx1, 0);
				m_alt_in_graph.at(uiVtx1).emplace(uiVtx2Next, 0);
			}
		}		
	}
}

bool Hole_Exchange::check_if_vtx1_prec_vtx2_sequence_partition(const size_t c_uiVtx1, const size_t c_uiVtx2, const std::set<size_t> &set_vts_before_sub_seq, const std::set<size_t> &set_vts_sub_seq, const std::set<size_t> &set_vts_after_sub_seq)
{
	if (set_vts_sub_seq.end() != set_vts_sub_seq.find(c_uiVtx1))
	{
		if (set_vts_after_sub_seq.end() != set_vts_after_sub_seq.find(c_uiVtx2)) return true;
		if (set_vts_sub_seq.end() != set_vts_sub_seq.find(c_uiVtx2)) assert(false); //m_ls_heur should have resolved this collision
	}
	else if (set_vts_sub_seq.end() != set_vts_sub_seq.find(c_uiVtx2))
	{
		if (set_vts_after_sub_seq.end() != set_vts_after_sub_seq.find(c_uiVtx1)) return false;
		if (set_vts_sub_seq.end() != set_vts_sub_seq.find(c_uiVtx1)) assert(false); //m_ls_heur should have resolved this collision
	}
	
	assert(false); // this case should never occur 
	return false;
}

bool Hole_Exchange::check_and_resolve_enablers(const std::vector<std::list<size_t>::iterator>& vec_rob_vtx_itr, const std::unordered_map<size_t, size_t> &map_old_completion_times)
{
	bool bEnabled = false, bFound;
	size_t uiVtx, uiEnabler, uiMinTime, uiOldTime;
	const auto &vec_enablers = m_graph.get_Enablers();

	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		uiVtx = *vec_rob_vtx_itr[uiRobot];
		if ("H" != m_graph.getType(uiVtx)) continue;
		bEnabled = false;

		for (auto it = vec_enablers[uiVtx].set.begin(); it != vec_enablers[uiVtx].set.end(); it++)
		{
			if (m_map_completion_times.at(it->getInd()) <= m_map_start_times.at(uiVtx))
			{
				bEnabled = true;
				break;
			}
		}

		if (false == bEnabled)
		{
			uiMinTime = std::numeric_limits<size_t>::max();
			for (auto it = vec_enablers[uiVtx].set.begin(); it != vec_enablers[uiVtx].set.end(); it++)
			{
				uiOldTime = map_old_completion_times.at(it->getInd());
				if (uiOldTime < uiMinTime)
				{
					uiMinTime = uiOldTime;
					uiEnabler = it->getInd();
				}
			}
			
			if (m_alt_out_graph.end() == m_alt_out_graph.find(uiEnabler)) return false;

			bFound = false;
			for (size_t uiOtherRobot = 0; uiOtherRobot < m_uiNumRobots; uiOtherRobot++)
			{
				if (uiOtherRobot == uiRobot) continue;
				auto it_find = std::find(m_rob_seq[uiOtherRobot].begin(), m_rob_seq[uiOtherRobot].end(), uiEnabler);
				if (m_rob_seq[uiOtherRobot].end() == it_find) continue;
				bFound = true;
				auto it_next = it_find;
				it_next++;
				if (m_rob_seq[uiOtherRobot].end() == it_next) return false;
				m_alt_out_graph.at(*it_next).emplace(uiVtx, 0);
				m_alt_in_graph.at(uiVtx).emplace(*it_next, 0);
			}
		}
	}
	return true;
}