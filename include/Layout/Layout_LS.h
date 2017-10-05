#pragma once
#ifndef LAYOUT_LS_H
#define LAYOUT_LS_H

#include "Layout_Graph.h"
#include <unordered_map>

class Layout_LS : public Layout_Graph
{
	private:
		
	public:
		Layout_LS(size_t uiNumRobots, size_t uiNumHoles);
		size_t getTime(N_Ind Ind) const;
		std::string getType(N_Ind Ind) const;
		Coordinates getLoc(N_Ind Ind) const;
		size_t getEdgeDist(R_Ind uiRobot, N_Ind Ind1 , N_Ind Ind2) const;
		bool doesEdgeExist(R_Ind uiRobot, N_Ind Ind1, N_Ind Ind2) const;
		void get_nearest_robots_for_hole(N_Ind Ind, std::unordered_set<size_t> &set_near_robots) const; 
};	

#endif
