#include "Route_Visualizer.h"

void Route_Visualization::plot_route(std::string strFolderPath, const Layout_LS &graph, const std::vector<std::vector<Vertex_Schedule>> &vec_schedule)
{
	for (size_t uiRobot = 0; uiRobot < vec_schedule.size(); uiRobot++)
	{
		stringstream stream_R;
		stream_R << uiRobot;
		std::string strFilePath = strFolderPath + "Tour_"+ stream_R.str() + ".dat";
		
		std::ofstream visTourFile;
		visTourFile.open(strFilePath.c_str());

		auto loc = graph.getLoc(vec_schedule[uiRobot][0].m_uiInd);
		double dPrev_X = loc.get_X_loc(), dPrev_Y = loc.get_Y_loc();
		double dCurr_X, dCurr_Y;

		for (size_t uiInd = 1; uiInd < vec_schedule[uiRobot].size(); uiInd++)
		{
			if ("IV" == graph.getType(vec_schedule[uiRobot][uiInd].m_uiInd)) continue;

			loc = graph.getLoc(vec_schedule[uiRobot][uiInd].m_uiInd);
			dCurr_X = loc.get_X_loc();
			dCurr_Y = loc.get_Y_loc();

			visTourFile << dPrev_X << " " << dPrev_Y << " " << dCurr_X - dPrev_X << " " << dCurr_Y - dPrev_Y << "\n";

			dPrev_X = dCurr_X;
			dPrev_Y = dCurr_Y;
		}

		visTourFile.close();
	}
}