#pragma once
#ifndef DYN_NODE_DESC_H
#define DYN_NODE_DESC_H

#include <stdlib.h>

struct Dyn_Node_Desc
{	
	int m_iOffset;
	int* m_Splus;
	int* m_Sminus;	
};

#endif
