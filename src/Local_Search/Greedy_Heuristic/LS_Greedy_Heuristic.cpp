#include "LS_Greedy_Heuristic.h"
#include "Sequence_Visualizer.h"

size_t LS_Greedy_Heuristic::getTime(size_t uiVert)
{
	auto it_find = m_map_rob_start_vtx_time.find(uiVert);
	if (m_map_rob_start_vtx_time.end() != it_find) return it_find->second;
	else return Greedy_Heuristic::getTime(uiVert);
}

LS_Greedy_Heuristic::LS_Greedy_Heuristic(const size_t uiRobotNum, const Layout_LS &graph, Power_Set &power):Greedy_Heuristic(uiRobotNum, graph, power)
{ }

void LS_Greedy_Heuristic::clear_prev_info_buffers()
{
	m_map_rob_start_vtx_time.clear();
	m_set_skip_enabling.clear();
	Greedy_Heuristic::clear_prev_info_buffers();
}

void LS_Greedy_Heuristic::populate_rob_start_times(const std::vector<std::list<size_t>> &rob_seq, const std::vector<size_t>& vec_start_times)
{
	assert(0 == m_map_rob_start_vtx_time.size());

	for (size_t uiRobot = 0; uiRobot < rob_seq.size(); uiRobot++)
	{
		m_map_rob_start_vtx_time.emplace(*rob_seq[uiRobot].begin() , vec_start_times[uiRobot]);
	}
}

void LS_Greedy_Heuristic::populate_enabled_verts(const std::unordered_set<size_t> &set_enabled_vertices)
{
	for (auto it = set_enabled_vertices.begin(); it != set_enabled_vertices.end(); it++)
	{
		m_set_skip_enabling.emplace(*it);
	}
}

void LS_Greedy_Heuristic::get_verts_not_self_enabled(size_t uiRobot, const std::vector<std::list<size_t>> &rob_seq, std::list<size_t>& list_dep_vert)
{
	Greedy_Heuristic::get_verts_not_self_enabled(uiRobot, rob_seq, list_dep_vert);
	
	//remove skip enabling vertices from list_dep_vert
	for (auto it = list_dep_vert.begin(); it != list_dep_vert.end(); )
	{
		if (m_set_skip_enabling.end() != m_set_skip_enabling.find(*it)) it = list_dep_vert.erase(it);
		else it++;
	}
}

bool LS_Greedy_Heuristic::perform_initializations(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::list<size_t>> &new_rob_seq, const size_t c_uiUpperBound, const std::vector<size_t>& vec_start_times, const std::unordered_set<size_t> &set_enabled_verts)
{
	populate_enabled_verts(set_enabled_verts);
	populate_rob_start_times(rob_seq, vec_start_times);
	return Greedy_Heuristic::perform_initializations(rob_seq, new_rob_seq, c_uiUpperBound);
}

int LS_Greedy_Heuristic::compute_greedy_sol(const std::vector<std::list<size_t>> &rob_seq, const std::vector<size_t>& vec_start_times, const std::unordered_set<size_t> &set_enabled_verts, std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, std::string strPlotFolder, const size_t c_uiUpperBound)
{
	//print_sequence(rob_seq);
	Sequence_Visualization obj_vis;
	clear_prev_info_buffers();
	std::vector<std::list<size_t>> new_rob_seq;
	bool bFeasible = perform_initializations(rob_seq, new_rob_seq, c_uiUpperBound, vec_start_times, set_enabled_verts);
	if (false == bFeasible)
	{
#ifdef PLOT_INFEASIBLE_CASES
		print_sequence(rob_seq);
		obj_vis.plot_alternative_graph(strPlotFolder, m_alt_graph, m_map_states_feas);
#endif
		return -1;
	}

	State root(m_uiNumRobots);
	populate_root_node_info(root, new_rob_seq);
	int iRetVal = compute_DFS(root, 0, 0);
	if (1 == iRetVal) { vectorize_schedule(new_rob_seq, vec_rob_sch, rob_seq); }
	else  m_set_to_do_verts.clear();

	if (iRetVal < 0)
	{
		iRetVal = -1;   // readjusting to 1, so local search will not get affected

#ifdef PLOT_INFEASIBLE_CASES
		obj_vis.plot_alternative_graph(strPlotFolder, m_alt_graph, m_map_states_feas);
		print_sequence(rob_seq);
#endif
	}

	return iRetVal;
}

//not all info regarding the pseudo end depots are purged, just the ones necessary are purged (from rob_seq and alt graph (selective info))
void LS_Greedy_Heuristic::minimally_purge_end_depot_info(const std::vector<bool> &vec_end_depot_psuedo)
{
	purge_pseudo_end_depots_from_alt_graphs(vec_end_depot_psuedo);

	//this has to occur in the end of this function
	purge_pseudo_end_depots_from_robot_sequence(vec_end_depot_psuedo);
}

void LS_Greedy_Heuristic::purge_pseudo_end_depots_from_robot_sequence(const std::vector<bool> &vec_end_depot_psuedo)
{
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		if (true == vec_end_depot_psuedo[uiRobot]) continue;
		m_rob_seq[uiRobot].pop_back();
	}
}

//we are not however modifying m_alt_edges, m_vec_vtx_alt_edge_ind_in, m_vec_vtx_alt_edge_ind_out
void LS_Greedy_Heuristic::purge_pseudo_end_depot_from_alt_graphs(size_t uiVtx)
{
	for (auto it_in = m_alt_graph.m_vec_adj_set_in.at(uiVtx).begin(); it_in != m_alt_graph.m_vec_adj_set_in.at(uiVtx).end(); )
	{
		size_t uiErase = m_alt_graph.m_vec_adj_set_out.at(it_in->first).erase(uiVtx);
		assert(1 == uiErase);
		it_in = m_alt_graph.m_vec_adj_set_in.at(uiVtx).erase(it_in);		
	}

	m_alt_graph.m_vec_adj_set_out.erase(uiVtx);
	m_alt_graph.m_vec_adj_set_in.erase(uiVtx);
	m_alt_graph.m_map_vertex_robot_pos_map.erase(uiVtx);
}

void LS_Greedy_Heuristic::purge_pseudo_end_depots_from_alt_graphs(const std::vector<bool> &vec_end_depot_psuedo)
{
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		if (true == vec_end_depot_psuedo[uiRobot]) continue;
		purge_pseudo_end_depot_from_alt_graphs(*m_rob_seq[uiRobot].rbegin());
	}
}

bool LS_Greedy_Heuristic::isVtxPreEnabled(size_t uiVtx)
{
	if (m_set_skip_enabling.end() != m_set_skip_enabling.find(uiVtx)) return true;
	return false;
}