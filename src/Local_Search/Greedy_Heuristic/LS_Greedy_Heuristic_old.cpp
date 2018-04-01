#include "LS_Greedy_Heuristic_old.h"

size_t LS_Greedy_Heuristic_Old::getTime(size_t uiVert)
{
	auto it_find = m_map_rob_start_vtx.find(uiVert);
	if (m_map_rob_start_vtx.end() != it_find) return it_find->second;
	else return getTime(uiVert);
}

LS_Greedy_Heuristic_Old::LS_Greedy_Heuristic_Old(const size_t uiRobotNum, const Layout_LS &graph, Power_Set &power):Greedy_Heuristic_old(uiRobotNum, graph, power)
{ }

void LS_Greedy_Heuristic_Old::clear_prev_info_buffers()
{
	m_map_rob_start_vtx.clear();
	Greedy_Heuristic_old::clear_prev_info_buffers();
}

void LS_Greedy_Heuristic_Old::populate_rob_start_times(const std::vector<std::list<size_t>> &rob_seq, const std::vector<size_t>& vec_start_times)
{
	assert(0 == m_map_rob_start_vtx.size());

	for (size_t uiRobot = 0; uiRobot < rob_seq.size(); uiRobot++)
	{
		m_map_rob_start_vtx.emplace(*rob_seq[uiRobot].begin() , vec_start_times[uiRobot]);
	}
}

void LS_Greedy_Heuristic_Old::populate_enabled_verts(const std::unordered_set<size_t> &set_enabled_vertices)
{
	for (auto it = set_enabled_vertices.begin(); it != set_enabled_vertices.end(); it++)
	{
		m_set_skip_enabling.emplace(*it);
	}
}

int LS_Greedy_Heuristic_Old::check_if_enabling_feasible(const State& state)
{
	bool bEnabled;
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		if (m_set_skip_enabling.end() != m_set_skip_enabling.find(*state.m_vec_rob_pos[uiRobot])) continue;

		bEnabled = check_if_self_enabling(uiRobot, state);
		if (bEnabled) continue;
		bEnabled = check_if_other_enabling(uiRobot, state);
		if (!bEnabled) return -1;
	}
	return 0;
}

void LS_Greedy_Heuristic_Old::perform_initializations(const std::vector<std::list<size_t>> &rob_seq, const std::vector<size_t>& vec_start_times, const std::unordered_set<size_t> &set_enabled_verts)
{
	populate_enabled_verts(set_enabled_verts);
	populate_rob_start_times(rob_seq, vec_start_times);
	Greedy_Heuristic_old::perform_initializations(rob_seq);
}

int LS_Greedy_Heuristic_Old::compute_greedy_sol(const std::vector<std::list<size_t>> &rob_seq, const std::vector<size_t>& vec_start_times, const std::unordered_set<size_t> &set_enabled_verts, std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch)
{
	clear_prev_info_buffers();	
	perform_initializations(rob_seq, vec_start_times, set_enabled_verts);
	State root(m_uiNumRobots);
	populate_root_node_info(root, rob_seq);
	int iRetVal = compute_DFS(root, 0, 0);
	if (1 == iRetVal) { vectorize_schedule(rob_seq, vec_rob_sch); }
	return iRetVal;
}