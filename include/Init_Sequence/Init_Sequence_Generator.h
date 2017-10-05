#pragma once
#ifndef INIT_SEQUENCE_GENERATOR_H
#define INIT_SEQUENCE_GENERATOR_H
#include <random>
#include <list>

void bias_node_search_by_weights(std::mt19937 &rng, std::list<size_t> &list_order, std::list<std::pair<double, size_t>> &list_neighs);


#endif
