#include "LS_Greedy_Heuristic_old.h"

size_t LS_Greedy_Heuristic_Old::getTime(size_t uiVert)
{
	auto it_find = m_map_rob_start_vtx.find(uiVert);
	if (m_map_rob_start_vtx.end() != it_find) return it_find->second;
	else return Greedy_Heuristic_old::getTime(uiVert);
}

LS_Greedy_Heuristic_Old::LS_Greedy_Heuristic_Old(const size_t uiRobotNum, const Layout_LS &graph, Power_Set &power) :Greedy_Heuristic_old(uiRobotNum, graph, power)
{
	m_rob_seq.resize(m_uiNumRobots);
}

void LS_Greedy_Heuristic_Old::clear_prev_info_buffers()
{
	m_map_rob_start_vtx.clear();
	m_set_skip_enabling.clear();

	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++) m_rob_seq[uiRobot].clear();

	Greedy_Heuristic_old::clear_prev_info_buffers();
}

void LS_Greedy_Heuristic_Old::populate_rob_start_times(const std::vector<std::list<size_t>> &rob_seq, const std::vector<size_t>& vec_start_times)
{
	assert(0 == m_map_rob_start_vtx.size());

	for (size_t uiRobot = 0; uiRobot < rob_seq.size(); uiRobot++)
	{
		m_map_rob_start_vtx.emplace(*rob_seq[uiRobot].begin(), vec_start_times[uiRobot]);
	}
}

void LS_Greedy_Heuristic_Old::populate_enabled_verts(const std::unordered_set<size_t> &set_enabled_vertices)
{
	assert(0 == m_set_skip_enabling.size());
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

	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		std::copy(rob_seq[uiRobot].begin(), rob_seq[uiRobot].end(), std::inserter(m_rob_seq[uiRobot], m_rob_seq[uiRobot].end()));
	}

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

void LS_Greedy_Heuristic_Old::populate_vertices(std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &out_graph, std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &in_graph)
{
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		for (auto it = m_rob_seq[uiRobot].begin(); it != m_rob_seq[uiRobot].end(); it++)
		{
			out_graph.emplace(*it, std::unordered_map<size_t, size_t>());
			in_graph.emplace(*it, std::unordered_map<size_t, size_t>());
		}
	}
}

void LS_Greedy_Heuristic_Old::populate_rob_seq_edges(std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &out_graph, std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &in_graph)
{
	size_t uiCost;
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		auto it = m_rob_seq[uiRobot].begin();
		auto it_next = it;
		it_next++;
		for (; it != m_rob_seq[uiRobot].end(); it++, it_next++)
		{
			if (m_rob_seq[uiRobot].end() == it_next) break;
			uiCost = getTime(*it);
			out_graph.at(*it).emplace(*it_next, uiCost);
			in_graph.at(*it_next).emplace(*it, uiCost);
		}
	}
}

void LS_Greedy_Heuristic_Old::populate_graphs(std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &out_graph, std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &in_graph)
{
	assert(true == out_graph.empty());
	assert(true == in_graph.empty());

	populate_vertices(out_graph, in_graph);
	populate_rob_seq_edges(out_graph, in_graph);
	populate_enabling_edges(out_graph, in_graph);
	populate_collision_edges(out_graph, in_graph);
}

void LS_Greedy_Heuristic_Old::populate_enabling_edges(std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &out_graph, std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &in_graph)
{
	const auto& vec_enablers = m_graph.get_Enablers();
	size_t uiMinTime , uiEnablerVtx;

	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		for (auto it = m_rob_seq[uiRobot].begin(); it != m_rob_seq[uiRobot].end(); it++)
		{
			if ("H" != m_graph.getType(*it)) continue;
			auto it_find = m_map_self_enabling.find(*it);
			if (m_map_self_enabling.end() != it_find)
			{
				if (true == it_find->second) continue;
			}			
			if (m_set_skip_enabling.end() != m_set_skip_enabling.find(*it)) continue;

			for (size_t uiOtherRobot = 0; uiOtherRobot < m_uiNumRobots; uiOtherRobot++)
			{
				if (uiOtherRobot == uiRobot) continue;
				uiMinTime = std::numeric_limits<size_t>::max();

				for (auto it_other = m_rob_seq[uiOtherRobot].begin(); it_other != m_rob_seq[uiOtherRobot].end(); it_other++)
				{
					if (vec_enablers[*it].set.end() != vec_enablers[*it].set.find(*it_other))
					{
						auto it_enabler = it_other;
						it_enabler++;
						if (m_rob_seq[uiOtherRobot].end() == it_enabler) break;
						if (uiMinTime > m_rob_hole_times[uiOtherRobot].at(*it_enabler).m_uiStartTime)
						{
							uiMinTime = m_rob_hole_times[uiOtherRobot].at(*it_enabler).m_uiStartTime;
							uiEnablerVtx = *it_enabler;
						}
						break;
					}
				}
				assert(std::numeric_limits<size_t>::max() != uiMinTime);
				out_graph.at(uiEnablerVtx).emplace(*it, 0);
				in_graph.at(*it).emplace(uiEnablerVtx, 0);
			}
		}
	}
}

void LS_Greedy_Heuristic_Old::populate_collision_edges(std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &out_graph, std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &in_graph)
{
	bool bVtx1PrecVtx2;
	size_t uiTail, uiHead;
	for (size_t uiRobot1 = 0; uiRobot1 < m_uiNumRobots; uiRobot1++)
	{
		for (auto it1 = m_rob_seq[uiRobot1].begin(); it1 != m_rob_seq[uiRobot1].end(); it1++)
		{
			for (size_t uiRobot2 = uiRobot1 + 1; uiRobot2 < m_uiNumRobots; uiRobot2++)
			{
				for (auto it2 = m_rob_seq[uiRobot2].begin(); it2 != m_rob_seq[uiRobot2].end(); it2++)
				{
					if (false == m_graph.areColliding(Coll_Pair(*it1, uiRobot1, *it2, uiRobot2))) continue;

					if (m_rob_hole_times[uiRobot1].at(*it1).m_uiStartTime < m_rob_hole_times[uiRobot2].at(*it2).m_uiStartTime) bVtx1PrecVtx2 = true;
					else bVtx1PrecVtx2 = false;

					if (bVtx1PrecVtx2)
					{
						auto it1_next = it1;
						it1_next++;
						if (m_rob_seq[uiRobot1].end() == it1_next) continue;
						uiTail = *it1_next;
						uiHead = *it2;
					}
					else
					{
						auto it2_next = it2;
						it2_next++;
						if (m_rob_seq[uiRobot2].end() == it2_next) continue;
						uiTail = *it2_next;
						uiHead = *it1;
					}
					out_graph[uiTail].emplace(uiHead, 0);
					in_graph[uiHead].emplace(uiTail, 0);
				}
			}
		}
	}
}