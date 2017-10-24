#include "Alternative_Graph.h"
#include <assert.h>

void Alternative_Graph::reallocate_buffers_compressed_vertices(const std::unordered_map<size_t, std::vector<size_t>> &map_superVtx_vecVtx)
{
	for (auto it_super = map_superVtx_vecVtx.begin(); it_super != map_superVtx_vecVtx.end(); it_super++)
	{
		m_vec_adj_set_out.emplace(it_super->first, std::unordered_map<size_t, size_t>());
		m_vec_adj_set_in.emplace(it_super->first, std::unordered_map<size_t, size_t>());
		m_vec_vtx_alt_edge_ind_in.emplace(it_super->first, std::unordered_map<size_t, size_t>());
		m_vec_vtx_alt_edge_ind_out.emplace(it_super->first, std::unordered_map<size_t, size_t>());		
	}
}

void Alternative_Graph::remove_buffer_redundant_vtx(size_t uiVert)
{
	assert(m_vec_adj_set_out[uiVert].size() <= 1);
	assert(m_vec_adj_set_in[uiVert].size() <= 1);
	assert(0 == m_vec_vtx_alt_edge_ind_in[uiVert].size());
	assert(0 == m_vec_vtx_alt_edge_ind_out[uiVert].size());

	m_vec_adj_set_out.erase(uiVert);
	m_vec_adj_set_in.erase(uiVert);
	m_vec_vtx_alt_edge_ind_in.erase(uiVert);
	m_vec_vtx_alt_edge_ind_out.erase(uiVert);
	m_map_vertex_robot_pos_map.erase(uiVert);
}

void Alternative_Graph::reassign_vertex_ownership_pos(size_t uiVtx, size_t uiRobot, size_t uiPos)
{
	if (m_map_vertex_robot_pos_map.end() == m_map_vertex_robot_pos_map.find(uiVtx))
		m_map_vertex_robot_pos_map.emplace(uiVtx, std::make_pair(uiRobot, uiPos));
	else
		m_map_vertex_robot_pos_map[uiVtx] = std::make_pair(uiRobot, uiPos);
}

void Alternative_Graph::reassign_vertex_ownership_positions(std::vector<std::list<size_t>> &new_rob_seq)
{
	size_t uiNumRobots = new_rob_seq.size();

	for (size_t uiRobot = 0; uiRobot < uiNumRobots; uiRobot++)
	{
		size_t uiInd = 0;
		auto it_next = new_rob_seq[uiRobot].begin();
		for (auto it = new_rob_seq[uiRobot].begin(); it != new_rob_seq[uiRobot].end(); it++)
		{
			it_next++;
			if (it_next == new_rob_seq[uiRobot].end())
			{
				reassign_vertex_ownership_pos(*it, uiRobot, uiInd);
				break;
			}			
			reassign_vertex_ownership_pos(*it, uiRobot, uiInd);
			uiInd++;
		}
	}
}

size_t get_total_num_verts(std::vector<std::list<size_t>> &new_rob_seq)
{
	size_t uiVert = 0;
	for (size_t uiRobot = 0; uiRobot < new_rob_seq.size(); uiRobot++)
	{
		uiVert += new_rob_seq[uiRobot].size();
	}
	return uiVert;
}

void Alternative_Graph::sanity_check_compression(std::vector<std::list<size_t>> &new_rob_seq)
{
#ifdef WINDOWS
	size_t uiVertNum = get_total_num_verts(new_rob_seq);
	assert(uiVertNum == m_vec_adj_set_out.size());
	assert(uiVertNum == m_vec_adj_set_in.size());
	assert(uiVertNum == m_vec_vtx_alt_edge_ind_in.size());
	assert(uiVertNum == m_vec_vtx_alt_edge_ind_out.size());
	assert(uiVertNum == m_map_vertex_robot_pos_map.size());
#else
	size_t uiVertNum = get_total_num_verts(new_rob_seq);
	if (uiVertNum != m_vec_adj_set_out.size())
	{
		cout << "Graph compression, mismatch 1";
		exit(1);
	}
	if (uiVertNum != m_vec_adj_set_in.size())
	{
		cout << "Graph compression, mismatch 2";
		exit(1);
	}
	assert(uiVertNum == m_vec_vtx_alt_edge_ind_in.size())
	{
		cout << "Graph compression, mismatch 3";
		exit(1);
	}
	assert(uiVertNum == m_vec_vtx_alt_edge_ind_out.size())
	{
		cout << "Graph compression, mismatch 4";
		exit(1);
	}
	assert(uiVertNum == m_map_vertex_robot_pos_map.size())
	{
		cout << "Graph compression, mismatch 5";
		exit(1);
	}
#endif
}

void Alternative_Graph::compress_graph(const Layout_LS &layout_graph, const std::unordered_map<size_t, std::vector<size_t>> &map_superVtx_vecVtx, const std::unordered_map<size_t, size_t> &map_vtx_super_vtx, const std::vector<std::pair<arc, arc>> &alt_coll_edges, std::vector<std::list<size_t>> &new_rob_seq, std::unordered_map<size_t, size_t> &map_super_vtx_proc_time)
{
	assert(0 == m_alt_edges.size());
	reallocate_buffers_compressed_vertices(map_superVtx_vecVtx);
	compress_prec_graph(layout_graph, map_superVtx_vecVtx, map_super_vtx_proc_time);
	add_compressed_alt_edges(map_vtx_super_vtx, alt_coll_edges);
	reassign_vertex_ownership_positions(new_rob_seq);

	sanity_check_compression(new_rob_seq);
}

void Alternative_Graph::compress_prec_graph(const Layout_LS &layout_graph, const std::unordered_map<size_t, std::vector<size_t>> &map_superVtx_vecVtx, std::unordered_map<size_t, size_t> &map_super_vtx_proc_time)
{
	size_t uiSuperVtx;
	for (auto it_vtx_vec = map_superVtx_vecVtx.begin(); it_vtx_vec != map_superVtx_vecVtx.end(); it_vtx_vec++)
	{
		uiSuperVtx = it_vtx_vec->first;
		size_t uiStart = *it_vtx_vec->second.begin();
		auto pr_start = get_prec_vtx_same_job(uiStart);
		assert(true == pr_start.first);
		size_t uiStartPrec = pr_start.second;

		size_t uiEnd = *it_vtx_vec->second.rbegin();
		auto pr_end = get_next_vtx_same_job(uiEnd);
		assert(true == pr_end.first);
		size_t uiEndFoll = pr_end.second;

		assert(((m_vec_adj_set_in[uiStart].size() <= 1) ? true : false) && ((m_vec_adj_set_out[uiStart].size() <= 1) ? true : false));
		assert(((m_vec_adj_set_in[uiEnd].size() <= 1) ? true : false) && ((m_vec_adj_set_out[uiEnd].size() <= 1) ? true : false));

		size_t uiStartCost = getArcCost(arc(uiStartPrec, uiStart));
		remove_prec_arc(arc(uiStartPrec, uiStart), false);
		remove_prec_arc(arc(uiEnd, uiEndFoll), false);

		size_t uiSuperVtxCost = 0;
		for (auto it_vert = it_vtx_vec->second.begin(); it_vert != it_vtx_vec->second.end(); it_vert++)
		{
			uiSuperVtxCost += layout_graph.getTime(*it_vert);
			remove_buffer_redundant_vtx(*it_vert);
		}

		add_prec_arc(arc(uiStartPrec, uiSuperVtx), uiStartCost);
		add_prec_arc(arc(uiSuperVtx, uiEndFoll), uiSuperVtxCost);
		map_super_vtx_proc_time.emplace(uiSuperVtx, uiSuperVtxCost);
	}
}

void Alternative_Graph::add_compressed_alt_edges(const std::unordered_map<size_t, size_t> &map_vtx_super_vtx, const std::vector<std::pair<arc, arc>> &alt_coll_edges)
{
	size_t uiVtx11, uiVtx12, uiVtx21, uiVtx22;
	for (auto it = alt_coll_edges.begin(); it != alt_coll_edges.end(); it++)
	{
		uiVtx11 = it->first.first;
		auto it_find = map_vtx_super_vtx.find(uiVtx11);
		if (map_vtx_super_vtx.end() != it_find) uiVtx11 = it_find->second;

		uiVtx12 = it->first.second;
		it_find = map_vtx_super_vtx.find(uiVtx12);
		if (map_vtx_super_vtx.end() != it_find) uiVtx12 = it_find->second;

		uiVtx21 = it->second.first;
		it_find = map_vtx_super_vtx.find(uiVtx21);
		if (map_vtx_super_vtx.end() != it_find) uiVtx21 = it_find->second;

		uiVtx22 = it->second.second;
		it_find = map_vtx_super_vtx.find(uiVtx22);
		if (map_vtx_super_vtx.end() != it_find) uiVtx22 = it_find->second;

		add_alt_arc(uiVtx11, uiVtx12, uiVtx21, uiVtx22);
	}
}
