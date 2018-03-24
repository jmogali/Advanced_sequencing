#include "Vertex_Schedule.h"

bool doIntervalsOverlap(const Vertex_Schedule& v1, const Vertex_Schedule& v2)
{
	// does open interval overlap
	if ((v2.m_uiStart < v1.m_uiEnd) && (v1.m_uiStart < v2.m_uiEnd))
		return true;
	else
		return false;
}