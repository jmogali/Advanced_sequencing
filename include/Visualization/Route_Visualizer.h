#pragma once
#ifndef ROUTE_VISUALIZER_H
#define ROUTE_VISUALIZER_H

#include <vector>
#include "Schedule_Validity_Check.h"
#include <fstream>

class Route_Visualization
{
	public:
		static void plot_route(std::string strFolderPath, const Layout_LS &graph, const std::vector<std::vector<Vertex_Schedule>> &vec_schedule);
};

#endif
