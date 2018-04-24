#include "Free_Intervals.h"
#include "Dyn_Node_Desc.h"

// A utility function to check if Interval2 fits in Interval1
unsigned int doIntervalFit(struct Interval* i1, struct Interval* i2)
{
	if ((i1->m_uiLow <= i2->m_uiLow) && (i1->m_uiHigh >= i2->m_uiHigh))
		return 1;
	else return 0;
}

int getEST_Vtx(struct Hole_Overlap_Enabler_Info* pInfo, int uiStartTime, int uiProcTime, struct Dyn_Node_Desc* pstAuxNodeInfo, int c_iTourPos, int c_iTourLen)
{
	int iStart;
	struct Interval i;
	
	unsigned int uiSelfEnabled = check_if_vtx_enabled(pInfo, pstAuxNodeInfo, c_iTourPos, c_iTourLen);

	for (unsigned int uiCount = 0; uiCount < pInfo->m_uiNumIntervals; uiCount++)
	{
		iStart = pInfo->m_parIntervals[uiCount].m_uiLow >= uiStartTime ? pInfo->m_parIntervals[uiCount].m_uiLow : uiStartTime;
		if(0 == uiSelfEnabled) iStart = iStart >= pInfo->m_uiOtherRobotEnableTime ? iStart : pInfo->m_uiOtherRobotEnableTime;

		i.m_uiLow = iStart;
		i.m_uiHigh = iStart + uiProcTime;
		if (1 == doIntervalFit( &(pInfo->m_parIntervals[uiCount]), &i))	return iStart;
	}

	return inFinity;
}


unsigned int check_if_vtx_enabled(struct Hole_Overlap_Enabler_Info* pInfo, struct Dyn_Node_Desc *pstAuxNodeInfo, int c_iTourPos, int c_iTourLen)
{
	//first check for S-
	int iIndexEnabler = 0, iSminusIndex = 0 , iSplusIndex = 0;
	
	if (NULL == pInfo->m_piEnablers)
	{
		if (inFinity == pInfo->m_uiOtherRobotEnableTime) return 1;
		else return 0;
	}

	const int c_iEnablerSize = pInfo->m_uiNumEnablers;
	const int c_iSplusSize = pstAuxNodeInfo->m_ui_S_Size;
	const int c_iSminusSize = pstAuxNodeInfo->m_ui_S_Size;

	int iCurrEnablerVtx, iSminusVtx, iSplusVtx;
	
	//check for S+
	while (1)
	{
		if (c_iEnablerSize == iIndexEnabler) break;
		else if (c_iSplusSize == iSplusIndex) break;

		iCurrEnablerVtx = pInfo->m_piEnablers[iIndexEnabler];
		iSplusVtx = c_iTourPos + pstAuxNodeInfo->m_Splus[iSplusIndex];

		//check this
		if (iSplusVtx < 0)
		{
			iSplusIndex++;
			continue;
		}

		if (iCurrEnablerVtx < iSplusVtx) return 1;
		else if (iCurrEnablerVtx >= c_iTourPos) break;  // iCurrEnablerVtx is guaranteed not to be present in Splus
		else if (iCurrEnablerVtx == iSplusVtx)
		{
			iIndexEnabler++;
			iSplusIndex++;
		}
		else if (iSplusVtx < iCurrEnablerVtx)
		{
			while (c_iTourPos + pstAuxNodeInfo->m_Splus[iSplusIndex] < iCurrEnablerVtx)
			{
				iSplusIndex++;
				if (c_iSplusSize == iSplusIndex) break;
			}
		}
	}

	//check for S-
	while (1)
	{
		if (c_iEnablerSize == iIndexEnabler) break;
		else if (c_iSminusSize == iSminusIndex) break;

		iCurrEnablerVtx = pInfo->m_piEnablers[iIndexEnabler];
		iSminusVtx = c_iTourPos + pstAuxNodeInfo->m_Sminus[iSminusIndex];

		//check this
		if (iSminusVtx > c_iTourLen) break;

		if (iCurrEnablerVtx == iSminusVtx) return 1;
		else if (iSminusVtx < iCurrEnablerVtx)
		{
			while (c_iTourPos + pstAuxNodeInfo->m_Sminus[iSminusIndex] < iCurrEnablerVtx)
			{
				iSminusIndex++;
				if (c_iSminusSize == iSminusIndex) break;
			}
		}
		else if (iCurrEnablerVtx < iSminusVtx)
		{
			while (pInfo->m_piEnablers[iIndexEnabler] < iSminusVtx)
			{
				iIndexEnabler++;
				if (c_iEnablerSize == iIndexEnabler) break;					
			}
		}
	}
	return 0;
}

