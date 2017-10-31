#pragma once
#ifndef GREEDY_HEURISTIC_UTILS_H
#define GREEDY_HEURISTIC_UTILS_H

#include <list>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include "Layout_LS.h"

#define COMPRESSION_ENABLE

struct ST_Time
{
	const static size_t UNSET = std::numeric_limits<size_t>::max();
	size_t m_uiStartTime;
	ST_Time(size_t uiStartTime);
};

struct Vertex_Schedule
{
	size_t m_uiInd;
	size_t m_uiStart, m_uiEnd, m_uiWait;
	Vertex_Schedule(size_t uiInd, size_t uiStart, size_t uiEnd, size_t uiWait) : m_uiInd(uiInd), m_uiStart(uiStart), m_uiEnd(uiEnd), m_uiWait(uiWait) {}
	void print_schedule() const;	
};

//no constraint earliest finish time
struct NoConstraint_EFT
{
	std::unordered_map<N_Ind, size_t, IndHasher> m_map_eft;
	size_t m_uiNC_Makespan;
};

struct State
{
	std::vector<std::list<size_t>::const_iterator> m_vec_rob_pos;
	State(size_t uiNumRobots) { m_vec_rob_pos.resize(uiNumRobots); };
	State(const State& state);
	inline size_t getRobotNum() const { return m_vec_rob_pos.size(); };
	void get_vertices(std::unordered_set<size_t> &set_vert) const;
};

bool operator== (const State& lhs, const State& rhs);

struct StateHasher
{
	std::size_t operator()(const State& state) const
	{
		size_t seed = 0;
		for (size_t uiRobot = 0; uiRobot < state.getRobotNum(); uiRobot++)
		{
			Hash_It(seed, *state.m_vec_rob_pos[uiRobot]);
		}
		return seed;
	}
};

struct Comparison_Object
{
	size_t uiDispatchTime;
	size_t uiCompSize;	// Number of robots in the selected components
	size_t uiExpMakeSpan;
	size_t uiMaxDelay;
	Comparison_Object(size_t dispatch_time, size_t comp_size, size_t makespan, size_t delay);
};

void print_sequence(const std::vector<std::list<size_t>> &rob_seq);

#endif
