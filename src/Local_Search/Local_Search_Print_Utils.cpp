#include "Local_Search.h"

void Local_Search::print_state_transition_path(std::string strFilePath, const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch)
{
	assert(false == full_rob_sch.empty());
	const size_t c_uiNumRobots = full_rob_sch.size();
	std::vector<State_vtx_time> vec_state_path;
	std::vector<size_t> vec_rob_pos_index;
	vec_rob_pos_index.resize(c_uiNumRobots, 0);
	
	size_t uiNextTime, uiCurrTime = std::numeric_limits<size_t>::min();
	bool bEnd = false;

	for (size_t uiRobot = 0; uiRobot < c_uiNumRobots; uiRobot++)
	{
		uiCurrTime = std::max(uiCurrTime, full_rob_sch[uiRobot][0].m_uiStart);
	}

	while (1)
	{
		vec_state_path.emplace_back(State_vtx_time(c_uiNumRobots));
		State_vtx_time &newState = vec_state_path[vec_state_path.size() - 1];
		uiNextTime = std::numeric_limits<size_t>::max();

		bEnd = true;
		for (size_t uiRobot = 0; uiRobot < c_uiNumRobots; uiRobot++)
		{
			newState.m_vec_rob_vtx[uiRobot] = full_rob_sch[uiRobot][vec_rob_pos_index[uiRobot]].m_uiInd;

			if (vec_rob_pos_index[uiRobot] == full_rob_sch[uiRobot].size() - 1) continue;
			else bEnd = false;

			uiNextTime = std::min(uiNextTime, full_rob_sch[uiRobot][vec_rob_pos_index[uiRobot]].m_uiEnd);
		}
		newState.m_uiTime = uiCurrTime;
		if (bEnd) break;

		for (size_t uiRobot = 0; uiRobot < c_uiNumRobots; uiRobot++)
		{
			if (vec_rob_pos_index[uiRobot] == full_rob_sch[uiRobot].size() - 1) continue;

			if (full_rob_sch[uiRobot][vec_rob_pos_index[uiRobot]].m_uiEnd == uiNextTime)
			{
				vec_rob_pos_index[uiRobot] = vec_rob_pos_index[uiRobot] + 1;
#ifdef WINDOWS
				assert(uiNextTime == full_rob_sch[uiRobot][vec_rob_pos_index[uiRobot]].m_uiStart);
#else
				if (uiNextTime != full_rob_sch[uiRobot][vec_rob_pos_index[uiRobot]].m_uiStart)
				{
					cout << "Computation of state transitions is incorrect \n";
					exit(-1);
				}
#endif
			}
		}
		uiCurrTime = uiNextTime;
	}

	ofstream myFile;
	std::string str_hole = strFilePath;
	myFile.open(str_hole.c_str());

	for (size_t uiState = 0; uiState < vec_state_path.size(); uiState++)
	{
		for (size_t uiRobot = 0; uiRobot < c_uiNumRobots; uiRobot++)
		{
			myFile << vec_state_path[uiState].m_vec_rob_vtx[uiRobot] << " ";
		}
		myFile << endl;
	}
	myFile.close();
}