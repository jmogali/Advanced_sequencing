#pragma once
#ifndef COLLISION_TOPOLOGICAL_SORTING_UTILS_H
#define COLLISION_TOPOLOGICAL_SORTING_UTILS_H
#include "Topological_Sorting_Utils.h"

class Coll_Topological_Sorting_Utils : public Topological_Sorting_Utils
{
	private:
		void construct_in_out_graphs(const Alternative_Graph &alt_graph);
		bool Check_Pos_Loop_Remove_1comp(const Alternative_Graph &alt_graph, const size_t c_uiNumRobots);

	protected:
		bool construct_graph_populate_order(const Alternative_Graph &alt_graph, const size_t c_uiNumRobots);
};

#endif
