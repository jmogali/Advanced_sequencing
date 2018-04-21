#ifndef COSTS_CONTAINER_H
#define COSTS_CONTAINER_H

#include "Free_Intervals.h"

struct Costs_Container
{
	int m_iNumVtx;
	struct Hole_Overlap_Enabler_Info* m_parTimeWindows;
	int* m_parProcTime;
	int** m_pparTravTime;
};

int getEST(struct Costs_Container* pCost, unsigned int uiVtx, int iCurrTime, struct Dyn_Node_Desc* pstAuxNodeInfo, int c_iTourPos, unsigned int n);
int getProcTime(struct Costs_Container* pCost, unsigned int uiVtx);
int getTravTime(struct Costs_Container* pCost, unsigned int uiVtx1, unsigned int uiVtx2, unsigned int n);

#endif



