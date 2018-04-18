#include "Local_Search.h"

bool Local_Search::generate_new_sequence_conservative(Hole_Exchange &hole_exchange, Greedy_Heuristic &heur, const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, std::vector<std::list<size_t>> &rob_seq, size_t &c_uiTargetMakeSpan)
{
	std::vector<std::list<size_t>> full_rob_seq;
	convert_hole_seq_to_full_seq(rob_seq, full_rob_seq);

	const Alternative_Graph &alt_graph = heur.get_complete_alt_graph(1);
	bool bImproving = hole_exchange.perform_heuristic_moves(full_rob_seq, alt_graph, full_rob_sch, (size_t) (c_uiTargetMakeSpan));
	if (true == bImproving)
	{
		rob_seq.clear();
		hole_exchange.populate_new_sequence(rob_seq);
		c_uiTargetMakeSpan = hole_exchange.get_best_makespan_found();		
	}
	return bImproving;
}