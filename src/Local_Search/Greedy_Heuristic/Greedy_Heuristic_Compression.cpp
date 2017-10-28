#include "Greedy_Heuristic.h"

#ifdef COMPRESSION_ENABLE

//initializes to false if a vertex has an incoming or outgoing precedence arc, true otherwise
size_t initialize_buffer_compr_verts(const Alternative_Graph &alt_graph, const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<bool>> &vec_compress_status)
{
	assert(0 == vec_compress_status.size());
	size_t uiMaxVertInd = std::numeric_limits<size_t>::min();

	const auto& out_graph = alt_graph.getGraph();
	const auto& in_graph = alt_graph.getReverseGraph();
	size_t uiVert;

	vec_compress_status.resize(rob_seq.size());
	for (size_t uiRobot = 0; uiRobot < rob_seq.size(); uiRobot++)
	{
		// at this stage, alt_graph contains only precedence constraints
		for (auto it = rob_seq[uiRobot].begin() ; it != rob_seq[uiRobot].end() ; it++)
		{
			uiVert = *it;
			if (uiMaxVertInd < uiVert) uiMaxVertInd = uiVert;
			
			//checks if there is an incoming or outgoing precedence arc for the vertex
			if ((out_graph.at(uiVert).size() > 1) || (in_graph.at(uiVert).size() > 1))
				vec_compress_status[uiRobot].push_back(false);
			else 
				vec_compress_status[uiRobot].push_back(true);
		}
		vec_compress_status[uiRobot][0] = false; // starting vertex should not be compressed				
		vec_compress_status[uiRobot][vec_compress_status[uiRobot].size()-1] = false;	// ending vertex should not be compressed
	}
	return uiMaxVertInd;
}

bool Greedy_Heuristic::add_colls_compress_graph(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::list<size_t>> &new_rob_seq)
{
	std::vector<std::vector<bool>> vec_compress_status;
	
	m_uiSuperVtxThresh = initialize_buffer_compr_verts(m_alt_graph, rob_seq, vec_compress_status);
	update_compr_verts_by_unself_enabled_and_deps(vec_compress_status);
	bool bFeasible = gather_coll_cons_update_compr_verts(rob_seq, vec_compress_status);
	if (false == bFeasible) return false;
	construct_new_rob_sequence(rob_seq, vec_compress_status, new_rob_seq);
	m_alt_graph.compress_graph(m_graph, m_map_superVtx_vecVtx, m_map_vtx_super_vtx, m_set_coll, new_rob_seq, m_map_super_vtx_proc_time);
	reassign_enablers();
	return true;
}

/*
bool Greedy_Heuristic::gather_coll_cons_update_compr_verts(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<bool>> &vec_compress_status)
{
	assert(std::numeric_limits<size_t>::min() != m_uiSuperVtxThresh);
	bool bFeasible;
	for (size_t uiRobot1 = 0; uiRobot1 < rob_seq.size(); uiRobot1++)
	{
		for (size_t uiRobot2 = uiRobot1 + 1; uiRobot2 < rob_seq.size(); uiRobot2++)
		{
			bFeasible = gather_coll_cons_compr_verts_rob_pair(uiRobot1, uiRobot2, rob_seq, vec_compress_status);
			if (false == bFeasible) return false;
		}
	}
	return true;
}*/

bool Greedy_Heuristic::gather_coll_cons_update_compr_verts(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<bool>> &vec_compress_status)
{
	size_t uiVtx1, uiVtx2, uiRobot1, uiRobot2, uiPos1, uiPos2, uiVtx_1_nxt, uiVtx_2_nxt;

	for (auto it = m_set_coll.begin(); it != m_set_coll.end(); )
	{
		auto pr1 = it->getPair1();
		auto pr2 = it->getPair2();

		uiVtx1 = pr1.getInd1();
		uiRobot1 = pr1.getInd2();
		uiPos1 = m_alt_graph.get_vertex_position(uiVtx1);

		uiVtx2 = pr2.getInd1();
		uiRobot2 = pr2.getInd2();
		uiPos2 = m_alt_graph.get_vertex_position(uiVtx2);

		auto bounds1 = m_coll_filter.get_bounds(uiVtx1, uiRobot2);
		if ((uiPos2 < bounds1.first) || (uiPos2 > bounds1.second))
		{
			it = m_set_coll.erase(it);
			continue;
		}

		auto bounds2 = m_coll_filter.get_bounds(uiVtx2, uiRobot1);
		if ((uiPos1 < bounds2.first) || (uiPos1 > bounds2.second))
		{
			it = m_set_coll.erase(it);
			continue;
		}

		auto nxt_vtx1 = m_alt_graph.get_next_vtx_same_job(uiVtx1);
		assert(true == nxt_vtx1.first);
		uiVtx_1_nxt = nxt_vtx1.second;

		auto nxt_vtx2 = m_alt_graph.get_next_vtx_same_job(uiVtx2);
		assert(true == nxt_vtx2.first);
		uiVtx_2_nxt = nxt_vtx2.second;

		bool bArc1 = m_alt_graph.containsPrecArc(arc(uiVtx_2_nxt, uiVtx1));
		bool bArc2 = m_alt_graph.containsPrecArc(arc(uiVtx_1_nxt, uiVtx2));

		if (!bArc1 & !bArc2)
		{
			vec_compress_status[uiRobot1][m_alt_graph.get_vertex_position(uiVtx1)] = false;
			vec_compress_status[uiRobot2][m_alt_graph.get_vertex_position(uiVtx2)] = false;
		}
		else if (bArc1 & bArc2) return false;

		it++;
	}
	return true;
}

//At this stage, m_alt_graph is in uncompressed form
void Greedy_Heuristic::update_compr_verts_by_unself_enabled_and_deps(std::vector<std::vector<bool>> &vec_compress_status)
{
	for (auto it = m_map_enabler_pos_vert.begin(); it != m_map_enabler_pos_vert.end(); it++)
	{
		size_t uiUnselfEnab = it->first;
		size_t uiPos = m_alt_graph.get_vertex_position(uiUnselfEnab);
		size_t uiRobot = m_alt_graph.get_vertex_ownership(uiUnselfEnab);
		
		if (2 == m_uiNumRobots) assert(false == vec_compress_status[uiRobot][uiPos]);
		vec_compress_status[uiRobot][uiPos] = false;

		for (auto it_enablers = it->second.begin(); it_enablers != it->second.end(); it_enablers++)
		{
			if (2 == m_uiNumRobots) assert(false == vec_compress_status[it_enablers->first][it_enablers->second.first]);
			vec_compress_status[it_enablers->first][it_enablers->second.first] = false;			
		} 
	}
}

//at this stage, the m_alt_graph has been compressed and vertex positions have been updated
void Greedy_Heuristic::reassign_enablers()
{
	for (auto it = m_map_enabler_pos_vert.begin(); it != m_map_enabler_pos_vert.end(); it++)
	{
		size_t uiDepVtx = it->first;
		assert(true == m_alt_graph.containsVertex(uiDepVtx));
		for (auto it_enablers = it->second.begin(); it_enablers != it->second.end(); it_enablers++)
		{
			size_t uiEnabler = it_enablers->second.second;
			assert(true == m_alt_graph.containsVertex(uiEnabler));
			it_enablers->second = std::make_pair(m_alt_graph.get_vertex_position(uiEnabler), uiEnabler);
		}
	}
}

/*bool Greedy_Heuristic::gather_coll_cons_compr_verts_rob_pair(size_t uiRobot1, size_t uiRobot2, const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<bool>> &vec_compress_status, std::vector<std::pair<arc, arc>> &alt_coll_edges)
{
	size_t uiPos = 0;
	for (auto it1 = rob_seq[uiRobot1].begin(); it1 != rob_seq[uiRobot1].end(); it1++)
	{
		auto it12 = it1;
		it12++;

		auto pr1 = m_coll_filter.get_bounds(*it1, uiRobot2);
		auto it2_start = rob_seq[uiRobot2].begin();
		std::advance(it2_start, pr1.first);

		auto it2_end = rob_seq[uiRobot2].begin();
		std::advance(it2_end, pr1.second);
		it2_end++;

		for (auto it2 = it2_start; it2 != it2_end; it2++)
		{
			auto pr2 = m_coll_filter.get_bounds(*it2, uiRobot1);
			if ((uiPos < pr2.first) || (uiPos > pr2.second)) continue;

			if (m_graph.areColliding(Coll_Pair(*it1, uiRobot1, *it2, uiRobot2)))
			{
				auto it22 = it2;
				it22++;

				bool bArc1 = m_alt_graph.containsPrecArc(arc(*it22, *it1));
				bool bArc2 = m_alt_graph.containsPrecArc(arc(*it12, *it2));

				if (!bArc1 & !bArc2)
				{
					alt_coll_edges.emplace_back(std::make_pair(arc(*it22, *it1), arc(*it12, *it2)));
					vec_compress_status[uiRobot1][m_alt_graph.get_vertex_position(*it1)] = false;
					vec_compress_status[uiRobot2][m_alt_graph.get_vertex_position(*it2)] = false;
				}
				else if (bArc1 & bArc2) return false;
			}
		}
		uiPos++;
	}
	return true;
}
*/

void Greedy_Heuristic::construct_new_rob_sequence(const std::vector<std::list<size_t>> &rob_seq, const std::vector<std::vector<bool>> &vec_compress_status, std::vector<std::list<size_t>> &new_rob_seq)
{
	assert(m_uiSuperVtxThresh != std::numeric_limits<size_t>::min());
	size_t uiSuperVtxInd = m_uiSuperVtxThresh;
	uiSuperVtxInd++;

	new_rob_seq.resize(rob_seq.size());
	for (size_t uiRobot = 0; uiRobot < rob_seq.size(); uiRobot++)
	{
		size_t uiPos = 0, uiLen = 0;
		bool bCompress = false;
		std::vector<size_t> vec_comp_vts;
		assert(false == vec_compress_status[uiRobot][0]);
		assert(false == vec_compress_status[uiRobot][vec_compress_status[uiRobot].size()-1]);

		for (auto it = rob_seq[uiRobot].begin(); it != rob_seq[uiRobot].end(); it++ , uiPos++)
		{
			if (false == vec_compress_status[uiRobot][uiPos])
			{
				if (uiLen > 0)
				{
					if (uiLen > 1)
					{
						new_rob_seq[uiRobot].emplace_back(uiSuperVtxInd);
						for (auto it_compr = vec_comp_vts.begin(); it_compr != vec_comp_vts.end(); it_compr++)
						{
							m_map_vtx_super_vtx.emplace(*it_compr, uiSuperVtxInd);
						}
						m_map_superVtx_vecVtx.emplace(uiSuperVtxInd, vec_comp_vts);
						uiSuperVtxInd++;
					}
					else
					{
						assert(1 == vec_comp_vts.size());
						new_rob_seq[uiRobot].emplace_back(vec_comp_vts[0]);
					}
					vec_comp_vts.clear();
				}

				new_rob_seq[uiRobot].emplace_back(*it);
				uiLen = 0;
			}
			else
			{
				vec_comp_vts.emplace_back(*it);
				if (true == bCompress) uiLen++;
				else
				{
					bCompress = true;
					uiLen = 1;
				}
			}
		}
	}
}

#endif