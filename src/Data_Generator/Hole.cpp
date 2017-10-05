#include "Hole.h"

Hole::Hole(Coordinates loc, std::string strType , size_t uiFrame):m_loc(loc)
{
	m_strType = strType;
	m_uiFrame = uiFrame;
}