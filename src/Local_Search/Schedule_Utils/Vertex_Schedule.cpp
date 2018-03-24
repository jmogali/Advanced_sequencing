#include "Vertex_Schedule.h"

Vertex_Schedule::Vertex_Schedule(size_t uiInd, size_t uiStart, size_t uiEnd, size_t uiWait) : m_uiInd(uiInd), m_uiStart(uiStart), m_uiEnd(uiEnd), m_uiWait(uiWait)
{}

bool doIntervalsOverlap(const Vertex_Schedule& v1, const Vertex_Schedule& v2)
{
	// does open interval overlap
	if ((v2.m_uiStart < v1.m_uiEnd) && (v1.m_uiStart < v2.m_uiEnd))
		return true;
	else
		return false;
}