#include "Layout_utils.h"

Node_Desc::Node_Desc(size_t uiTime, const Coordinates &loc): m_uiTime(uiTime) , m_loc(loc)
{}

Node_Desc::Node_Desc(const Node_Desc &obj): m_loc(obj.m_loc) , m_uiTime(obj.m_uiTime)
{}

Depo_Desc::Depo_Desc(size_t uiTime, size_t uiFromInd, size_t uiToInd, const Coordinates &loc): Node_Desc(uiTime , loc)
{
	m_uiFromInd = uiFromInd;
	m_uiToInd = uiToInd;
}

Depo_Desc::Depo_Desc(const Depo_Desc &obj): Node_Desc(obj)
{
	m_uiFromInd = obj.m_uiFromInd;
	m_uiToInd = obj.m_uiToInd;
}