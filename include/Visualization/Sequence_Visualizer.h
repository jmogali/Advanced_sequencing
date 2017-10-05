#ifndef SEQUENCE_VISUALIZATION_H
#define SEQUENCE_VISUALIZATION_H

#include "Windows_Linux.h"
#include "Alternative_Graph.h"

void plot_infeasible_sequence();
void plot_alternative_graph(std::string strFilePath, const Alternative_Graph &graph);

#endif