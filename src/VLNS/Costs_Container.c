#include "Costs_Container.h"
#include "VLNS_Constants.h"

int getEST(struct Costs_Container* pCost, unsigned int uiVtx, int iCurrTime, struct Dyn_Node_Desc* pstAuxNodeInfo, int c_iTourPos, unsigned int n)
{
	if (uiVtx > n) return inFinity;
	else if (inFinity <= iCurrTime) return inFinity;
	return getEST_Vtx(&(pCost->m_parTimeWindows[uiVtx]), iCurrTime, pCost->m_parProcTime[uiVtx], pstAuxNodeInfo, c_iTourPos, n);
}

int getProcTime(struct Costs_Container* pCost, unsigned int uiVtx)
{
	return pCost->m_parProcTime[uiVtx];
}

int getTravTime(struct Costs_Container* pCost, unsigned int uiVtx1, unsigned int uiVtx2, unsigned int n)
{
	if ((uiVtx1 > n) || (uiVtx2 > n)) return inFinity;
	return pCost->m_pparTravTime[uiVtx1][uiVtx2];
}
