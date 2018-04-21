#pragma once
#ifndef FREE_INTERVALS_H
#define FREE_INTERVALS_H

#include "VLNS_Constants.h"

// Structure to represent an interval
struct Interval
{
	int m_uiLow, m_uiHigh;
};

struct Hole_Overlap_Enabler_Info
{
	unsigned int m_uiNumIntervals; //number of free intervals, have to be non-overlapping and sorted
	struct Interval* m_parIntervals; // free intervals
	int* m_piEnablers; // contains which holes enables the given hole, the array must be sorted
	unsigned int m_uiNumEnablers;
	int m_uiOtherRobotEnableTime; // time it is enabled by some hole from a different robot 	
};

int getEST_Vtx(struct Hole_Overlap_Enabler_Info* pInfo, int uiStartTime, int uiProcTime, struct Dyn_Node_Desc* pstNodeInfo, int c_iTourPos, int c_iTourLen);
unsigned int check_if_vtx_enabled(struct Hole_Overlap_Enabler_Info* pInfo, struct Dyn_Node_Desc* pstNodeInfo, int c_iTourPos, int c_iTourLen);


#endif
