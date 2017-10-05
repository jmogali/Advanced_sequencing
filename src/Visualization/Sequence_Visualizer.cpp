#include "Sequence_Visualizer.h"
#include <iostream>
#include <sstream> // stringstream


void plot_alternative_graph(std::string strFilePath, const Alternative_Graph &graph)
{
	const auto& graph = alt_graph.getGraph();
	size_t uiVtx1, uiVtx2, uiPos1, uiPos2, uiRobot1, uiRobot2;
	ofstream altGraphFile;
	
	altGraphFile.open(strFilePath.c_str());
	
	for(auto it_vtx = graph.begin(); it_vtx != graph.end(); it_vtx++)
	{
		uiVtx1 = it_vtx->first;
		uiRobot1 = alt_graph.get_vertex_ownership(uiVtx1);
		uiPos1 = alt_graph.get_vertex_position(uiVtx1);
		
		for(auto it_neighs = it_vtx->second.begin(); it_neighs != it_vtx->second.end(); it_vtx++)
		{
			uiVtx2 = it_neighs->first;
			uiRobot2 = alt_graph.get_vertex_ownership(uiVtx2);
			uiPos2 = alt_graph.get_vertex_position(uiVtx2);
			
			altGraphFile << uiPos1 << " " << uiRobot1 <<" "<< uiPos2 - uiPos1 <<" " <<uiRobot2 - uiRobot1<<"\n";			
		}
	}
	
	altGraphFile.close();
}