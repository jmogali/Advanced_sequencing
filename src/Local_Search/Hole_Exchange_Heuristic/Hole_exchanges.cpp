#include "Hole_exchanges.h"

Hole_Exchange::Hole_Exchange(size_t uiNumRobots, const Layout_LS &graph, Power_Set &power) : m_uiNumRobots(uiNumRobots), m_graph(graph), ls_heur(uiNumRobots, graph, power)
{}

void Hole_Exchange::clear_prev_info()
{
	m_vec_state_path.clear();
}

void Hole_Exchange::perform_heuristic_moves(const std::vector<std::list<size_t>> &rob_seq, const Alternative_Graph &alt_graph, const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch)
{
	assert(rob_seq.size() == m_uiNumRobots);
	clear_prev_info();
	bool bFeasible = construct_graph_populate_order(alt_graph, m_uiNumRobots);
	assert(true == bFeasible); // we will assume that we are always given a feasible sequence as input
	construct_state_transition_path(full_rob_sch);

}

