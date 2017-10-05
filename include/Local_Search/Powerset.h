#pragma once
#ifndef POWER_SET_H
#define POWER_SET_H

#include <unordered_map>
#include <vector>
#include "Hashing_Utils.h"

struct Set_Hasher
{
	std::size_t operator() (const std::vector<size_t> &set) const
	{
		size_t seed = 0;
		for (size_t uiCount = 0; uiCount < set.size(); uiCount++)
		{
			Hash_It(seed, set[uiCount]);
		}
		return seed;
	}
};

class Power_Set
{
	private:
		std::unordered_map<std::vector<size_t>, std::vector<std::vector<size_t>>, Set_Hasher> m_map_power_set; 
		void computeSubsets(const std::vector<size_t> &set);
		
	public:
		const std::vector<std::vector<size_t>>& get_power_set(const std::vector<size_t> &set);
		Power_Set() {};
};

#endif
