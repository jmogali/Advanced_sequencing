#include "Sequence_Visualizer.h"

Sequence_Visualization::Sequence_Visualization()
{}

void Sequence_Visualization::plot_alternative_graph(std::string strFolderPath, const Alternative_Graph &alt_graph)
{
#ifdef WINDOWS
	_mkdir(strFolderPath.c_str());
#else
	mkdir(strFolderPath.c_str(), S_IRWXU);
#endif

	std::string strFileEnabling = strFolderPath + "enabling.dat";
	plot_enabling_cons_alt_graph(strFileEnabling, alt_graph);

	std::string strFileCollisions = strFolderPath + "collisions.dat";
	plot_coll_cons_alt_graph(strFileCollisions, alt_graph);
}

void Sequence_Visualization::plot_enabling_cons_alt_graph(std::string strFilePath, const Alternative_Graph &alt_graph)
{
	const auto& graph = alt_graph.getGraph();
	int uiVtx1, uiVtx2, uiPos1, uiPos2, uiRobot1, uiRobot2;
	std::ofstream altGraphFile;

	altGraphFile.open(strFilePath.c_str());

	for (auto it_vtx = graph.begin(); it_vtx != graph.end(); it_vtx++)
	{
		uiVtx1 = it_vtx->first;
		uiRobot1 = alt_graph.get_vertex_ownership(uiVtx1);
		uiPos1 = alt_graph.get_vertex_position(uiVtx1);

		for (auto it_neighs = it_vtx->second.begin(); it_neighs != it_vtx->second.end(); it_neighs++)
		{
			uiVtx2 = it_neighs->first;
			uiRobot2 = alt_graph.get_vertex_ownership(uiVtx2);
			uiPos2 = alt_graph.get_vertex_position(uiVtx2);

			altGraphFile << uiPos1 << " " << uiRobot1 << " " << uiPos2 - uiPos1 << " " << uiRobot2 - uiRobot1 << "\n";
		}
	}
	altGraphFile.close();
}

void Sequence_Visualization::plot_coll_cons_alt_graph(std::string strFilePath, const Alternative_Graph &alt_graph)
{
	int uiVtx1, uiVtx2, uiPos1, uiPos2, uiRobot1, uiRobot2;
	std::ofstream altGraphFile;
	altGraphFile.open(strFilePath.c_str());
	
	for (auto it_vtx = alt_graph.m_vec_vtx_alt_edge_ind_out.begin(); it_vtx != alt_graph.m_vec_vtx_alt_edge_ind_out.end(); it_vtx++)
	{
		uiVtx1 = it_vtx->first;
		uiRobot1 = alt_graph.get_vertex_ownership(uiVtx1);
		uiPos1 = alt_graph.get_vertex_position(uiVtx1);

		for (auto it_neigh = it_vtx->second.begin(); it_neigh != it_vtx->second.end(); it_neigh++)
		{
			uiVtx2 = it_neigh->first;
			uiRobot2 = alt_graph.get_vertex_ownership(uiVtx2);
			uiPos2 = alt_graph.get_vertex_position(uiVtx2);

			altGraphFile << uiPos1 << " " << uiRobot1 << " " << uiPos2 - uiPos1 << " " << uiRobot2 - uiRobot1 << "\n";
		}
	}
	altGraphFile.close();
}



