#include "Local_Search.h"
#include "VLNS_Interface.h"

void Local_Search::populate_new_sequence(const std::vector<std::list<size_t>> &rob_sequence_with_IV, std::vector<std::list<size_t>> &rob_sequence_without_IV)
{
	assert(true == rob_sequence_without_IV.empty());
	const size_t c_uiNumRobots = rob_sequence_with_IV.size();

	rob_sequence_without_IV.resize(c_uiNumRobots);

	for (size_t uiRobot = 0; uiRobot < c_uiNumRobots; uiRobot++)
	{
		for (auto it = rob_sequence_with_IV[uiRobot].begin(); it != rob_sequence_with_IV[uiRobot].end(); it++)
		{
			if ("IV" == m_graph.getType(*it)) continue;
			rob_sequence_without_IV[uiRobot].emplace_back(*it);
		}
	}
}

bool Local_Search::gen_seq_hole_exchange(Hole_Exchange &hole_exchange, Greedy_Heuristic &heur, const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, std::vector<std::list<size_t>> &rob_seq, size_t &c_uiTargetMakeSpan)
{
	std::vector<std::list<size_t>> full_rob_seq;
	convert_hole_seq_to_full_seq(rob_seq, full_rob_seq);

	const Alternative_Graph &alt_graph = heur.get_complete_alt_graph(1);
	bool bImproving = hole_exchange.perform_heuristic_moves(full_rob_seq, alt_graph, full_rob_sch, (size_t) (c_uiTargetMakeSpan));
	if (true == bImproving)
	{
		rob_seq.clear();
		populate_new_sequence(hole_exchange.get_robot_sequence(), rob_seq);
		c_uiTargetMakeSpan = hole_exchange.get_best_makespan_found();		
	}
	return bImproving;
}

void Local_Search::gen_seq_TSP(std::string strTSPFolder, Greedy_Heuristic &heur, const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, std::vector<std::list<size_t>> &rob_seq, size_t &c_uiTargetMakeSpan, const size_t c_uiKVal)
{
	std::vector<std::list<size_t>> full_rob_seq;
	convert_hole_seq_to_full_seq(rob_seq, full_rob_seq);

	c_uiTargetMakeSpan = perform_TSP_Move(strTSPFolder, full_rob_seq, full_rob_sch, heur, m_graph, m_en_graph, (int)c_uiKVal);

	rob_seq.clear();
	populate_new_sequence(full_rob_seq, rob_seq);
}

void Local_Search::free_VLNS_buffers()
{
	free_TSP_buffers();
}