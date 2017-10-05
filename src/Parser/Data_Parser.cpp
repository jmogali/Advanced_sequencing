#include "Data_Parser.h"

Data_Parser::Data_Parser(std::string strFile) : m_strFile(strFile)
{}

int Data_Parser::read_header_info(std::pair<size_t, size_t> &pr)
{
	ifstream myFile(m_strFile);
	std::string line;
	int iOffset = 0;

	if (myFile.is_open())
	{
		while (getline(myFile , line))
		{
			std::vector<std::string> elems;
			boost::split(elems, line, boost::is_any_of(" "));

			if (elems[0] == "VERTICES:")
				pr.first = (size_t)(atoi)(elems[1].c_str());

			if (elems[0] == "ROBOTS:")
				pr.second = (size_t)(atoi)(elems[1].c_str());

			iOffset++;
			if (elems[0] == "NODE_DESCRIPTION_SECTION:")
				break;
		}
	}
	else
	{
		iOffset = -1;
	}
	pr.first = pr.first - (2 * pr.second);
	return iOffset;
}

void Data_Parser::populate_info(int iOffset, Layout_Graph &graph)
{
	ifstream myFile(m_strFile);
	std::string line;
	int iLine = 1;
	std::unordered_map<size_t, size_t> map_iv_vec;

	if (myFile.is_open())
	{
		while (getline(myFile, line))
		{
			if (iLine < iOffset)
			{
				iLine++;
				continue;
			}

			add_vertices(myFile, graph , map_iv_vec);
			add_edges_iv(myFile, graph, map_iv_vec);
			add_collisons(myFile, graph);
			add_enablers(myFile, graph);
			add_depots(myFile, graph);
		}
	}
}

void Data_Parser::add_vertices(ifstream &myFile , Layout_Graph &graph, std::unordered_map<size_t, size_t> &map_iv_vec)
{
	std::string line;
	size_t uiVtx , uiTime;
	double dLocx, dLocy;

	while (getline(myFile, line))
	{
		if (line == "EDGE_DESCRIPTION_SECTION:") return;

		std::vector<std::string> elems;
		boost::split(elems, line, boost::is_any_of(" "));
		uiVtx = (size_t) atoi(elems[1].c_str());

		if ("V" == elems[0])
		{	
			dLocx = atof(elems[2].c_str());
			dLocy = atof(elems[3].c_str());
			uiTime = (size_t)atoi(elems[4].c_str());
			graph.m_map_V_H.emplace(uiVtx, Node_Desc(uiTime, Coordinates(dLocx, dLocy, 0)));
		}
		else if ("IV" == elems[0])
		{
			uiTime = (size_t)atoi(elems[2].c_str());
			map_iv_vec.emplace(uiVtx, uiTime);
		}
	}
}

void Data_Parser::add_edges_iv(ifstream &myFile, Layout_Graph &graph, std::unordered_map<size_t, size_t> &map_iv_vec)
{
	std::string line;
	size_t uiRobot, uiVtx1, uiVtx2, uiIV , uiTime;
	
	while (getline(myFile, line))
	{
		if (line == "COLLISION_DESCRIPTION_SECTION:") return;		
		std::vector<std::string> elems;
		boost::split(elems, line, boost::is_any_of(" "));
		if (elems[0] == "ROBOT:") 
		{ 
			uiRobot = (size_t)atoi(elems[1].c_str()); 
			continue;
		}

		uiVtx1 = (size_t)atoi(elems[0].c_str());
		uiVtx2 = (size_t)atoi(elems[elems.size()-1].c_str());

		uiTime = 0;
		for (size_t uiCount = 1; uiCount < elems.size() - 1; uiCount++)
		{
			uiIV = (size_t)atoi(elems[uiCount].c_str());
			graph.m_edge_inter_vert[uiRobot].map[uiVtx1].map[uiVtx2].vec.push_back(IV_Ind(uiIV, map_iv_vec.at(uiIV)));
			uiTime += map_iv_vec.at(uiIV);
			graph.m_map_IV_hole_pair[uiRobot].emplace(uiIV, IV_Hole_Pair(uiVtx1, uiVtx2));
		}
		graph.m_in_edge[uiRobot].map[uiVtx2].map[uiVtx1] = uiTime;	
	}
}

void Data_Parser::add_collisons(ifstream &myFile, Layout_Graph &graph)
{
	std::string line;
	size_t uiRobot1, uiRobot2, uiVtx1, uiVtx2;

	while (getline(myFile, line))
	{
		if (line == "ENABLER_DESCRIPTION_SECTION:") return;
		std::vector<std::string> elems;
		boost::split(elems, line, boost::is_any_of(" "));

		uiVtx1 = atoi(elems[0].c_str());
		uiRobot1 = atoi(elems[1].c_str());
		uiVtx2 = atoi(elems[2].c_str());
		uiRobot2 = atoi(elems[3].c_str());
		graph.m_conf_map.emplace(Coll_Pair(uiVtx1, uiRobot1, uiVtx2, uiRobot2));
	}
}

void Data_Parser::add_enablers(ifstream &myFile, Layout_Graph &graph)
{
	std::string line;
	size_t uiVtx, uiNeigh;

	while (getline(myFile, line))
	{
		if (line == "START_END_DESCRIPTION_SECTION:") return;
		std::vector<std::string> elems;
		boost::split(elems, line, boost::is_any_of(" "));
		uiVtx = (size_t) atoi(elems[0].c_str());

		for (size_t uiCount = 1; uiCount < elems.size(); uiCount++)
		{
			uiNeigh = (size_t)atoi(elems[uiCount].c_str());
			graph.m_vec_set_enablers[uiVtx].set.emplace(uiNeigh);
		}
	}
}

void Data_Parser::add_depots(ifstream &myFile, Layout_Graph &graph)
{
	std::string line;
	size_t uiRobot, uiVtx1, uiVtx2, uiTime;
	double dLocx, dLocy;
	std::unordered_map<size_t, std::pair<size_t, size_t>> map_depots;

	while (getline(myFile, line))
	{
		std::vector<std::string> elems;
		boost::split(elems, line, boost::is_any_of(" "));

		uiRobot = atoi(elems[0].c_str());
		uiVtx1 = atoi(elems[1].c_str());
		uiVtx2 = atoi(elems[2].c_str());

		map_depots.emplace(uiRobot , std::make_pair(uiVtx1, uiVtx2));
	}

	for (size_t uiRobot = 0; uiRobot < graph.get_num_robots(); uiRobot++)
	{
		uiVtx1 = map_depots.at(uiRobot).first;
		
		for (auto it = graph.m_map_V_H.begin() ; it != graph.m_map_V_H.end() ; it++)
		{
			if (it->first.getInd() == uiVtx1)
			{
				uiTime = it->second.getTime();
				dLocx = it->second.getLoc().get_X_loc();
				dLocy = it->second.getLoc().get_Y_loc();
				graph.m_map_V_H.erase(it);
				break;
			}
		}
		
		uiVtx2 = map_depots.at(uiRobot).second;

		for (auto it = graph.m_map_V_H.begin(); it != graph.m_map_V_H.end(); it++)
		{
			if (it->first.getInd() == uiVtx2)
			{
				assert(dLocx == it->second.getLoc().get_X_loc());
				assert(dLocy == it->second.getLoc().get_Y_loc());
				assert(uiTime == it->second.getTime());
				graph.m_map_V_H.erase(it);
				break;
			}
		}
		graph.m_vec_V_D.emplace(uiRobot, Depo_Desc(uiTime, uiVtx1 , uiVtx2, Coordinates(dLocx, dLocy, 0)));
	}
}
