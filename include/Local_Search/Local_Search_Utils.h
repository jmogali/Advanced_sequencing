#pragma once
#ifndef LOCAL_SEARCH_UTILS_H
#define LOCAL_SEARCH_UTILS_H

#include "Hashing_Utils.h"

struct Node_Set
{
	std::unordered_set<size_t> set;
};

struct Local_Robots
{
	std::unordered_set<size_t> set;
};

class Or_Pair
{
	private:
		size_t uiInd1, uiInd2;
	public:
		Or_Pair(size_t ind1, size_t ind2) : uiInd1(ind1 < ind2 ? ind1 : ind2), uiInd2(ind1 < ind2 ? ind2 : ind1) {};
		inline size_t getInd1() const { return uiInd1; };
		inline size_t getInd2() const { return uiInd2; };
};

inline bool operator== (const Or_Pair& lhs, const Or_Pair& rhs)
{
	return (lhs.getInd1() == rhs.getInd1()) && (lhs.getInd2() == rhs.getInd2());
}

struct Or_Pair_Hasher
{
	std::size_t operator() (const Or_Pair &v) const
	{
		size_t seed = 0;
		::Hash_It(seed, v.getInd1());
		::Hash_It(seed, v.getInd2());
		return seed;
	}
};



#endif