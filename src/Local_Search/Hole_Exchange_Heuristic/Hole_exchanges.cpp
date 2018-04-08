#include "Hole_exchanges.h"

Hole_Exchange::Hole_Exchange(size_t uiNumRobots, const Layout_LS &graph, Power_Set &power, const Enabling_Graph &en_graph) : m_uiNumRobots(uiNumRobots), m_graph(graph), m_ls_heur(uiNumRobots, graph, power), m_en_graph(en_graph)
{
	m_rob_seq.resize(m_uiNumRobots);
	m_vec_full_rob_sch.resize(m_uiNumRobots);
}

void Hole_Exchange::clear_prev_info()
{
	m_vec_state_path.clear();
	m_vec_full_rob_sch.clear();
	m_alt_in_graph.clear();
	m_alt_out_graph.clear();
	m_rob_seq.clear();
	m_hole_rob_owner.clear();
}

void Hole_Exchange::populate_rob_graphs(const Alternative_Graph &alt_graph)
{
	m_alt_in_graph = alt_graph.getReverseGraph();
	m_alt_out_graph = alt_graph.getGraph();
}

void Hole_Exchange::populate_robot_owners(const std::vector<std::list<size_t>> &rob_seq)
{
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		for (auto it = rob_seq[uiRobot].cbegin(); it != rob_seq[uiRobot].cend(); it++)
		{
			if ("H" != m_graph.getType(*it)) continue;
			m_hole_rob_owner.emplace(*it, uiRobot);
		}
	}
}

void Hole_Exchange::perform_heuristic_moves(const std::vector<std::list<size_t>> &rob_seq, const Alternative_Graph &alt_graph, const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch)
{
	assert(rob_seq.size() == m_uiNumRobots);
	clear_prev_info();
	//construct_graph_populate_order_with_dist(alt_graph.getGraph(), alt_graph.getReverseGraph());
		
	//populates m_vec_state_path
	m_rob_seq = rob_seq;
	m_vec_full_rob_sch = full_rob_sch;
	construct_state_transition_path(full_rob_sch);
	populate_rob_graphs(alt_graph);
	populate_robot_owners(rob_seq);	

	perform_swap_operation();
}

void clear_buffers(std::list<size_t> &critical_path, std::list<std::tuple<size_t, size_t, size_t>> &list_best_cand, std::unordered_map<size_t, size_t> &map_start_times)
{
	critical_path.clear();
	list_best_cand.clear();
	map_start_times.clear();
}

void Hole_Exchange::perform_swap_operation()
{
	size_t uiIter = 0;
	bool bFeasible, bImproving;
	std::list<size_t> critical_path;
	std::list<std::tuple<size_t, size_t, size_t>> list_best_cand;

	while (uiIter < c_uiMaxNumSwaps)
	{
		clear_buffers(critical_path, list_best_cand, m_map_start_times);
		
		//computes start time of all vertices
		compute_start_times(m_alt_out_graph, m_alt_in_graph, m_map_start_times);
		
		compute_critical_path(m_alt_in_graph, m_map_start_times, critical_path);
		get_cand_vertex_critical_path(1, critical_path, list_best_cand);

		for (auto it_cand = list_best_cand.begin(); it_cand != list_best_cand.end(); it_cand++)
		{
			bImproving = check_if_candidate_improving(std::get<0>(*it_cand), std::get<1>(*it_cand), std::get<2>(*it_cand));
			//if (bImproving) break;
		}

	}
}

bool Hole_Exchange::check_if_candidate_improving(const size_t c_uiVtx, const size_t c_uiMinTime, const size_t c_uiMaxTime)
{
	bool bFeasible = check_if_retraction_feasible(c_uiVtx, m_hole_rob_owner.at(c_uiVtx), m_vec_state_path, m_rob_seq, m_vec_full_rob_sch);
	if (false == bFeasible) return false;
}

