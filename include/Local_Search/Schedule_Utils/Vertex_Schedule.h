#pragma once
#ifndef VERTEX_SCHEDULE_H
#define VERTEX_SCHEDULE_H

struct Vertex_Schedule
{
	size_t m_uiInd;
	size_t m_uiStart, m_uiEnd, m_uiWait;
	Vertex_Schedule(size_t uiInd, size_t uiStart, size_t uiEnd, size_t uiWait) : m_uiInd(uiInd), m_uiStart(uiStart), m_uiEnd(uiEnd), m_uiWait(uiWait) {}
	void print_schedule() const;
};

bool doIntervalsOverlap(const Vertex_Schedule& v1, const Vertex_Schedule& v2);

#endif