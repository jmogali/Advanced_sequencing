#pragma once
#ifndef VLNS_INTERFACE_H
#define VLNS_INTERFACE_H

size_t perform_TSP_Move(std::string strTSPFileFolder, std::vector<std::list<size_t>> &rob_seq, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const Greedy_Heuristic &heur, const Layout_LS &graph, const Enabling_Graph &en_graph, int kVal);
void free_TSP_buffers();

#endif
