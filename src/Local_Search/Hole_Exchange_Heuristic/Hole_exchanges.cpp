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
}

void Hole_Exchange::populate_rob_graphs(const Alternative_Graph &alt_graph)
{
	m_alt_in_graph = alt_graph.getReverseGraph();
	m_alt_out_graph = alt_graph.getGraph();
}

void Hole_Exchange::perform_heuristic_moves(const std::vector<std::list<size_t>> &rob_seq, const Alternative_Graph &alt_graph, const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch)
{
	assert(rob_seq.size() == m_uiNumRobots);
	clear_prev_info();
	//construct_graph_populate_order_with_dist(alt_graph.getGraph(), alt_graph.getReverseGraph());
		
	//populates m_vec_state_path
	construct_state_transition_path(full_rob_sch);
	populate_rob_graphs(alt_graph);
	m_rob_seq = rob_seq;
	m_vec_full_rob_sch = full_rob_sch;

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

			//if (bImproving) break;
		}

	}
}

