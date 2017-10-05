#pragma once
#ifndef HASHING_UTILS_H
#define HASHING_UTILS_H

template <class T>
inline void hash_combine(size_t & seed, const T & v)
{
	std::hash<T> hasher;
	seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

inline size_t Hash_It(size_t &seed, size_t uiData)
{
	::hash_combine(seed, uiData);
	return seed;
}

#endif
