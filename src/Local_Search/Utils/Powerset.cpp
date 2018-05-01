#include "Powerset.h"
#include <algorithm>

void Power_Set::computeSubsets(const std::vector<size_t> &set)
{
	std::vector< std::vector<size_t> > subset;
	std::vector<size_t> empty;
	subset.push_back(empty);
	
	for (size_t i = 0; i < set.size(); i++)
	{
		std::vector< std::vector<size_t> > subsetTemp = subset;

		for (size_t j = 0; j < subsetTemp.size(); j++)
			subsetTemp[j].push_back(set[i]);

		for (size_t j = 0; j < subsetTemp.size(); j++)
			subset.push_back(subsetTemp[j]);
	}	
	subset.erase(subset.begin());
	m_map_power_set.emplace(set, subset);
}

const std::vector<std::vector<size_t>>& Power_Set::get_power_set(const std::vector<size_t> &inp_set)
{
	std::vector<size_t> set(inp_set);
	std::sort(set.begin(), set.end());

	if (m_map_power_set.find(set) == m_map_power_set.end())
	{
		computeSubsets(set);
	}
	return m_map_power_set.at(set);
}