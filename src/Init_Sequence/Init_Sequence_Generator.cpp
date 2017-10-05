#include "Init_Sequence_Generator.h"
#include <assert.h>

void bias_node_search_by_weights(std::mt19937 &rng, std::list<size_t> &list_order, std::list<std::pair<double, size_t>> &list_neighs)
{
	double sum_of_weight, dRand;
	size_t uiCount, uiNumChoice = list_neighs.size();

	for (size_t uiChoice = 0; uiChoice < uiNumChoice; uiChoice++)
	{
		if (uiChoice == uiNumChoice - 1)
		{
			assert(1 == list_neighs.size());
			list_order.push_back(list_neighs.begin()->second);
			list_neighs.clear();
			break;
		}

		sum_of_weight = 0;
		std::vector<double> vecSum;

		for (auto it = list_neighs.begin(); it != list_neighs.end(); it++)
		{
			sum_of_weight += it->first;
			vecSum.push_back(sum_of_weight);
		}
		std::uniform_real_distribution<double> unif_len(0, sum_of_weight);
		dRand = unif_len(rng);

		uiCount = 0;
		for (auto it = list_neighs.begin(); it != list_neighs.end();)
		{
			if (dRand <= vecSum[uiCount])
			{
				list_order.push_back(it->second);
				list_neighs.erase(it);
				break;
			}
			it++;
			uiCount++;
		}
	}
}