#include "Data_Generator.h"

struct less_than_V_Ind
{
	inline bool operator() (const std::pair<N_Ind, Node_Desc>& var1, const std::pair<N_Ind, Node_Desc>& var2)
	{
		return (var1.first.getInd() < var2.first.getInd());
	}
};

struct less_than_IV_Ind
{
	inline bool operator() (const std::pair<Ind, size_t>& var1, const std::pair<Ind, size_t>& var2)
	{
		return (var1.first < var2.first);
	}
};


void Data_Generator::print_data_files(std::string strFolder, std::string strFileName)
{
	ofstream myFile;
	std::string str_hole = strFolder + "/" + strFileName;
	myFile.open(str_hole.c_str());

	print_header_info(strFileName, myFile);
	print_node_description(myFile);
	print_edges(myFile);
	print_collisions(myFile);
	print_enablers(myFile);
	print_start_end_des(myFile);

	myFile.close();
}

void Data_Generator::print_header_info(std::string strFileName, ofstream &myFile)
{
	myFile << "NAME: " << strFileName << endl;
	myFile << "VERTICES: " << (2 * m_handle.get_num_robots()) + m_handle.get_num_holes() <<endl;
	myFile << "ROBOTS: " << m_handle.get_num_robots() << endl;
	myFile << "NODE_TYPE: " << "V IV" <<endl;
	myFile << "EDGE_DATA_FORMAT: " << "ADJ_LIST" << endl;
}

void Data_Generator::print_node_description(ofstream &myFile)
{
	myFile << "NODE_DESCRIPTION_SECTION: " << endl;
	const auto &map_depo = m_handle.getDepotMap();
	
	for (auto it = map_depo.begin(); it != map_depo.end() ; it++)
	{
		myFile << "V " << it->second.getFromInd() << " " << it->second.getLoc().get_X_loc() << " " <<it->second.getLoc().get_Y_loc()<<" "<<it->second.getTime() << endl;
		myFile << "V " << it->second.getToInd() << " " << it->second.getLoc().get_X_loc() << " " << it->second.getLoc().get_Y_loc() << " " << it->second.getTime() << endl;
	}

	const auto &map_holes = m_handle.getHolesMap();
	std::map<size_t, Node_Desc> sorted_map_holes;
	for (auto it = map_holes.begin(); it != map_holes.end(); it++)
	{
		sorted_map_holes.emplace(it->first.getInd() , it->second);
	}
	
	for (auto it = sorted_map_holes.begin() ; it != sorted_map_holes.end() ; it++)
	{
		myFile << "V " << it->first<< " " << it->second.getLoc().get_X_loc() <<" "<< it->second.getLoc().get_Y_loc() <<" "<< it->second.getTime() <<endl;
	}

	const auto &vec_rob_iv = m_handle.get_IV_Vec();
	std::vector<std::pair<Ind, size_t>> vec_iv;
	for (size_t uiRobot = 0; uiRobot < vec_rob_iv.size(); uiRobot++)
	{
		for (auto it1 = vec_rob_iv[uiRobot].map.begin(); it1 != vec_rob_iv[uiRobot].map.end(); it1++)
		{
			for (auto it2 = it1->second.map.begin(); it2 != it1->second.map.end(); it2++)
			{
				for (auto it3 = it2->second.vec.begin(); it3 != it2->second.vec.end(); it3++)
				{
					vec_iv.push_back(std::make_pair(it3->getInd() , it3->getTime()));					
				}
			}			
		}
	}

	std::sort(vec_iv.begin() , vec_iv.end() , less_than_IV_Ind());
	for (auto it = vec_iv.begin(); it != vec_iv.end(); it++)
	{
		myFile << "IV " << it->first.getInd() << " " << it->second << endl;
	}
}

void Data_Generator::print_edges(ofstream &myFile)
{
	myFile << "EDGE_DESCRIPTION_SECTION:" <<endl;
	size_t uiStart, uiEnd;

	const auto &vec_rob_iv = m_handle.get_IV_Vec();
	
	for (size_t uiRobot = 0; uiRobot < vec_rob_iv.size(); uiRobot++)
	{
		myFile << "ROBOT: " << uiRobot << endl;
		for (auto it1 = vec_rob_iv[uiRobot].map.begin(); it1 != vec_rob_iv[uiRobot].map.end(); it1++)
		{
			uiStart = it1->first.getInd();
			for (auto it2 = it1->second.map.begin(); it2 != it1->second.map.end() ; it2++)
			{
				uiEnd = it2->first.getInd();
				myFile << uiStart << " ";
				for (auto it3 = it2->second.vec.begin(); it3 != it2->second.vec.end(); it3++)
				{
					myFile << it3->getInd() << " ";
				}				
				myFile << uiEnd << endl;				
			}			
		}		
	}
}

void Data_Generator::print_collisions(ofstream &myFile)
{
	myFile << "COLLISION_DESCRIPTION_SECTION:" << endl;
	const auto &coll_map = m_handle.getHoleCollMap();

	for (auto it = coll_map.begin(); it != coll_map.end(); it++)
	{
		myFile << it->getPair1().getInd1() << " " << it->getPair1().getInd2() << " " << it->getPair2().getInd1() << " " << it->getPair2().getInd2() << endl;
	}
}

void Data_Generator::print_enablers(ofstream &myFile)
{
	myFile << "ENABLER_DESCRIPTION_SECTION:" << endl;
	const auto &vec_enablers = m_handle.get_Enablers();
	
	for (size_t uiVert = 0; uiVert < vec_enablers.size() ; uiVert++)
	{
		if (0 == vec_enablers[uiVert].set.size()) continue;
		myFile << uiVert;
		for (auto it = vec_enablers[uiVert].set.begin(); it != vec_enablers[uiVert].set.end(); it++)
		{
			myFile <<" "<< it->getInd();
		}
		myFile << endl;
	}
}

void Data_Generator::print_start_end_des(ofstream &myFile)
{
	myFile << "START_END_DESCRIPTION_SECTION:" <<endl;
	
	for (size_t uiRobot = 0; uiRobot < m_handle.get_num_robots(); uiRobot++)
	{
		myFile << uiRobot << " " << 2*uiRobot << " " << (2 * uiRobot) + 1 << endl;
	}
}