#pragma once
#ifndef LAYOUT_BOEING_H
#define LAYOUT_BOEING_H

#include "Layout_Graph.h"

class Layout_Boeing : public Layout_Graph
{
	public:
		Layout_Boeing(size_t uiNumRobots , size_t uiNumHoles);
		void add_depot(size_t uiRobot, size_t uiTime, size_t uiFromDepotInd, size_t uiToDepotInd, const Coordinates &loc);
		void add_hole(size_t uiHoleInd, size_t uiTime , const Coordinates &loc);
		void add_edge(size_t uiRobot , size_t uiSvtx, size_t uiDvtx, size_t uiVal);
		void add_iv(size_t uiRobot, size_t uiSvtx, size_t uiDvtx, size_t uiIndex , size_t uiVal);
		void add_conflict(N_Ind s1, R_Ind r1, N_Ind s2, R_Ind r2);	
		void add_enabler(N_Ind src, N_Ind neigh);
};

#endif
