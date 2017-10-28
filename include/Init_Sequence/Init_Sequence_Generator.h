#pragma once
#ifndef INIT_SEQUENCE_GENERATOR_H
#define INIT_SEQUENCE_GENERATOR_H
#include <random>
#include <list>
#include <unordered_map>

//void bias_node_search_by_weights(std::mt19937 &rng, std::list<size_t> &list_order, std::list<std::pair<double, size_t>> &list_neighs);
//T can be list<std::pair<size_t, double>> or vector<std::pair<size_t double>>
template <class T>
size_t rand_select_list_pair_with_bias(std::mt19937 &rng, const T &ind_dist_container, std::string strPriority, double dFact) //works for <size_t, double>, selection returned must be first variable
{
	assert(dFact > 0);
	double sum_of_weight = 0, dRand;
	std::vector<double> vecSum;
	
	for (auto it = ind_dist_container.begin(); it != ind_dist_container.end(); it++)
	{
		if ("LOW_DIST" == strPriority) sum_of_weight += exp(-dFact * it->second);		// being done this way because lower weight should be given higher probability
		else if ("HIGH_DIST" == strPriority) sum_of_weight += exp(dFact * it->second);
		else assert(false);

		vecSum.push_back(sum_of_weight);
	}

	std::uniform_real_distribution<double> unif_len(0, sum_of_weight);
	dRand = unif_len(rng);

	size_t uiCount = 0, uiSel;
	for (auto it = ind_dist_container.begin(); it != ind_dist_container.end(); it++, uiCount++)
	{
		if (dRand <= vecSum[uiCount])
		{
			uiSel = it->first;
			break;
		}
	}
	return uiSel;
}


#endif
