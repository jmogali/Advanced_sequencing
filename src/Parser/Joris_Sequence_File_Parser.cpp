#include "Joris_Sequence_File_Parser.h"

bool parse_sequence_file(std::string strFile, size_t uiNumRobots, std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const Layout_LS &graph)
{
	std::ifstream myFile(strFile);
	std::string line;
	
	rob_seq.resize(uiNumRobots);
	vec_rob_sch.resize(uiNumRobots);
	std::unordered_map<size_t, size_t> map_vtx_time;

	if (myFile.is_open())
	{
		while (getline(myFile, line))
		{
			std::vector<std::string> elems;
			boost::split(elems, line, boost::is_any_of(" "));

			if ("OBJECTIVE:" == elems[0]) continue;
			parse_robot_sequence(myFile, uiNumRobots, rob_seq, vec_rob_sch);
			parse_schedule_info(myFile, uiNumRobots, map_vtx_time);
		}
	}
	myFile.close();
	bool bValid = populate_schedule(map_vtx_time, vec_rob_sch, graph);
	return bValid;
}

void parse_robot_sequence(std::ifstream &myFile, size_t uiNumRobots, std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch)
{
	std::string line;
	size_t uiRobot = 0, uiInd;
	size_t c_uiMax_Num = std::numeric_limits<size_t>::max();

	while (getline(myFile, line))
	{
		if (line == "SCHEDULE:") return;
		std::vector<std::string> elems;
		boost::split(elems, line, boost::is_any_of(","));

		for (size_t uiCount = 0; uiCount < elems.size(); uiCount++)
		{
			uiInd = (size_t)atoi(elems[uiCount].c_str());
			if (0 == uiCount % 2) rob_seq[uiRobot].push_back(uiInd);
			vec_rob_sch[uiRobot].emplace_back(uiInd, c_uiMax_Num, c_uiMax_Num, c_uiMax_Num);
		}
		uiRobot++;
	}
}

void parse_schedule_info(std::ifstream &myFile, size_t uiNumRobots, std::unordered_map<size_t, size_t> &map_vtx_time)
{
	std::string line;
	while (getline(myFile, line))
	{
		std::vector<std::string> elems;
		boost::split(elems, line, boost::is_any_of(" "));

		map_vtx_time.emplace((size_t)atoi(elems[0].c_str()) , (size_t)atoi(elems[1].c_str()));
	}
}

bool populate_schedule(const std::unordered_map<size_t, size_t> &map_vtx_time, std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const Layout_LS &graph)
{
	const auto &vec_rob_iv = graph.get_IV_Vec();
	int iWait;

	for (size_t uiRobot = 0; uiRobot < vec_rob_sch.size(); uiRobot++)
	{
		size_t uiCount1 = 0;
		size_t uiCount2 = 1;

		for(; uiCount2 <= vec_rob_sch[uiRobot].size(); uiCount2++ , uiCount1++)
		{
			vec_rob_sch[uiRobot][uiCount1].m_uiStart = map_vtx_time.at(vec_rob_sch[uiRobot][uiCount1].m_uiInd);
			vec_rob_sch[uiRobot][uiCount1].m_uiEnd = map_vtx_time.at(vec_rob_sch[uiRobot][uiCount2].m_uiInd);
			iWait = (int)vec_rob_sch[uiRobot][uiCount1].m_uiEnd - (int)(vec_rob_sch[uiRobot][uiCount1].m_uiStart + graph.getTime(vec_rob_sch[uiRobot][uiCount1].m_uiInd));
			if (iWait < 0) return false;
			vec_rob_sch[uiRobot][uiCount1].m_uiWait = (size_t)iWait;

			if (uiCount2 == vec_rob_sch[uiRobot].size() - 1)
			{
				vec_rob_sch[uiRobot][uiCount2].m_uiStart = map_vtx_time.at(vec_rob_sch[uiRobot][uiCount2].m_uiInd);
				vec_rob_sch[uiRobot][uiCount2].m_uiEnd = map_vtx_time.at(vec_rob_sch[uiRobot][uiCount2].m_uiInd);
				vec_rob_sch[uiRobot][uiCount2].m_uiWait = 0;
				break;
			}
		}
	}
	return true;
}