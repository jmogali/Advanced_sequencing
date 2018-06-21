#include "Data_Generator.h"

bool isProximalEnabling_Neigh_Strategy(const Coordinates &loc1, const Coordinates &loc2, double dNormX, double dNormY)
{
	double dist = abs( (loc1.get_X_loc() - loc2.get_X_loc()) / dNormX) + abs( ( loc1.get_Y_loc() - loc2.get_Y_loc() ) / dNormY);
	if (dist <= 2.0) return true;
	return false;
}

Data_Generator::Data_Generator(size_t uiNumRobots , size_t uiNumHoles , const std::vector<Robot> &vec_robots): m_handle(uiNumRobots , uiNumHoles)
{ 
	for (auto it = vec_robots.begin(); it != vec_robots.end(); it++)
	{
		m_vec_robots.push_back(*it);
	}
}

void Data_Generator::populate_data(const Boeing_Fuesalage &boeing)
{
	std::set<size_t> set_st_vert;
	compute_start_locs(set_st_vert, boeing);
	std::vector<map_rob_ind_id_pair> map_iv_inds;
	map_iv_inds.resize(m_handle.get_num_robots());

	add_vertex_data(boeing); 
	add_edge_iv_info(set_st_vert, map_iv_inds, boeing);
	compute_collisions(map_iv_inds, boeing);
	compute_enablers(set_st_vert, boeing);
}

void Data_Generator::add_vertex_data(const Boeing_Fuesalage &boeing)
{
	add_depot_info(boeing);
	add_hole_info(boeing);	
}

void Data_Generator::add_depot_info(const Boeing_Fuesalage &boeing)
{
	size_t c_ui_num_robots = m_handle.get_num_robots();

	for (size_t uiRobot = 0; uiRobot < c_ui_num_robots; uiRobot++)
	{
		m_handle.add_depot(uiRobot , DEPOT_TIME, 2 * uiRobot , 2 * uiRobot + 1 , boeing.m_vec_depots[uiRobot]);
	}
}

void Data_Generator::add_hole_info(const Boeing_Fuesalage &boeing)
{
	size_t uiIndex = 2 * boeing.m_vec_depots.size();
	size_t c_uiHoleNum = boeing.m_vec_holes.size();

	for (size_t uiCount = 0; uiCount < c_uiHoleNum; uiCount++)
	{
		m_handle.add_hole(uiIndex++, SCALE_TIME(TIME_PER_HOLE) , boeing.m_vec_holes[uiCount].getLoc());
	}
}

void Data_Generator::add_edge_iv_info(const std::set<size_t> &set_st_vert, std::vector<map_rob_ind_id_pair> &map_iv_inds, const Boeing_Fuesalage &boeing)
{
	size_t uiIndex = boeing.m_vec_holes.size() + (2*boeing.m_vec_depots.size());

	for (size_t uiRobot = 0; uiRobot < m_vec_robots.size(); uiRobot++)
	{
		add_edge_iv_info_robot(uiRobot, set_st_vert, uiIndex, map_iv_inds, boeing);
	}
}

void Data_Generator::compute_start_locs(std::set<size_t> &set_st_vert , const Boeing_Fuesalage &boeing)
{
	const size_t c_uiNumRobots = m_handle.get_num_robots();
	std::vector<Coordinates> vec_tacks;
	bool bEnable;

	for (size_t uiFrame = 0; uiFrame < boeing.m_vec_tacks.size(); uiFrame++)
	{
		for (auto it = boeing.m_vec_tacks[uiFrame].begin(); it != boeing.m_vec_tacks[uiFrame].end(); it++)
		{
			vec_tacks.emplace_back(*it);
		}
	}

	for (size_t uiCount = 0; uiCount < boeing.m_vec_holes.size(); uiCount++)
	{
		for (auto it1 = vec_tacks.begin(); it1 != vec_tacks.end(); it1++)
		{
			bEnable = isProximalEnabling_Neigh_Strategy(*it1, boeing.m_vec_holes[uiCount].getLoc(), boeing.m_dHorSpacing, boeing.m_dVertSpacing);
			if (bEnable) 
			{
				set_st_vert.emplace( (2 * c_uiNumRobots) + uiCount );
				break;
			}
		}
	}
}

void Data_Generator::add_edge_iv_info_robot(size_t uiRobot, const std::set<size_t> &set_st_vert, size_t &uiIndex, std::vector<map_rob_ind_id_pair> &map_iv_inds, const Boeing_Fuesalage &boeing)
{
	const size_t c_uiNumRobots = m_handle.get_num_robots();
	bool bToolChange;

	for (auto it = set_st_vert.begin(); it!= set_st_vert.end(); it++)
	{
		size_t uiHoleInd = *it - (2 * c_uiNumRobots);
		if (false == m_vec_robots[uiRobot].isHoleReachable(boeing.m_vec_holes[uiHoleInd])) continue;
		add_edge_iv_info(uiRobot, boeing.m_vec_depots[uiRobot], boeing.m_vec_holes[uiHoleInd].getLoc(), 2 * uiRobot, *it, false, uiIndex, boeing);
		map_iv_inds[uiRobot].emplace(uiIndex , std::make_pair(std::make_pair(uiRobot , "D") , std::make_pair(uiHoleInd, "H")));
		uiIndex++;
	}

	for (size_t uiHole1 = 0; uiHole1 < boeing.m_vec_holes.size(); uiHole1++)
	{
		if (false == m_vec_robots[uiRobot].isHoleReachable(boeing.m_vec_holes[uiHole1])) continue;

		for (size_t uiHole2 = 0; uiHole2 < boeing.m_vec_holes.size(); uiHole2++)
		{
			if (uiHole1 == uiHole2) continue;
			if (false == m_vec_robots[uiRobot].isHoleReachable(boeing.m_vec_holes[uiHole2])) continue;
			bToolChange = boeing.m_vec_holes[uiHole1].getType() != boeing.m_vec_holes[uiHole2].getType() ? true : false;
			add_edge_iv_info(uiRobot, boeing.m_vec_holes[uiHole1].getLoc(), boeing.m_vec_holes[uiHole2].getLoc(), (2 * c_uiNumRobots) + uiHole1, (2 * c_uiNumRobots) + uiHole2, bToolChange, uiIndex, boeing);
			map_iv_inds[uiRobot].emplace(uiIndex, std::make_pair(std::make_pair(uiHole1, "H"), std::make_pair(uiHole2, "H")));
			uiIndex++;
		}
	}

	for (size_t uiHole = 0; uiHole < boeing.m_vec_holes.size(); uiHole++)
	{
		if (false == m_vec_robots[uiRobot].isHoleReachable(boeing.m_vec_holes[uiHole])) continue;
		add_edge_iv_info(uiRobot, boeing.m_vec_holes[uiHole].getLoc(), boeing.m_vec_depots[uiRobot], (2 * c_uiNumRobots) + uiHole, (2 * uiRobot) + 1, false, uiIndex, boeing);
		map_iv_inds[uiRobot].emplace(uiIndex, std::make_pair(std::make_pair(uiHole, "H"), std::make_pair( uiRobot , "D")));
		uiIndex++;
	}
}

void Data_Generator::add_edge_iv_info(size_t uiRobot , const Coordinates &loc1, const Coordinates &loc2, size_t uiGraphInd1, size_t uiGraphInd2, bool bToolChange, size_t uiIndex, const Boeing_Fuesalage &boeing)
{
	double dTime = 0;
	
	if (bToolChange)
	{
		dTime = m_vec_robots[uiRobot].compute_time(loc1, boeing.m_vec_tool_change_loc[uiRobot]);
		dTime += m_vec_robots[uiRobot].compute_time(boeing.m_vec_tool_change_loc[uiRobot], loc2);
	}
	else
	{
		dTime = m_vec_robots[uiRobot].compute_time(loc1, loc2);
	}

	size_t uiScaledTime = SCALE_TIME(dTime);
	m_handle.add_edge(uiRobot, uiGraphInd1, uiGraphInd2, uiScaledTime);
	m_handle.add_iv(uiRobot, uiGraphInd1, uiGraphInd2, uiIndex, uiScaledTime);	
}

void Data_Generator::compute_collisions(const std::vector<map_rob_ind_id_pair> &map_iv_inds, const Boeing_Fuesalage &boeing)
{
	compute_hole_pair_colls(boeing);
	//compute_iv_pair_colls(map_iv_inds, boeing);
	//compute_v_iv_pair_colls(map_iv_inds, boeing);
}

void Data_Generator::compute_hole_pair_colls(const Boeing_Fuesalage &boeing)
{
	for (size_t uiRobot1 = 0; uiRobot1 < m_vec_robots.size()-1; uiRobot1++)
	{
		for (size_t uiRobot2 = uiRobot1+1; uiRobot2 < m_vec_robots.size(); uiRobot2++)
		{
			compute_hole_pair_colls(uiRobot1, uiRobot2, boeing);
		}
	}
}

void Data_Generator::compute_hole_pair_colls(size_t uiRobot1, size_t uiRobot2, const Boeing_Fuesalage &boeing)
{
	const size_t c_uiNumRobots = m_vec_robots.size();

	for (size_t uiHole1 = 0; uiHole1 < boeing.m_vec_holes.size(); uiHole1++)
	{
		if (false == m_vec_robots[uiRobot1].isHoleReachable(boeing.m_vec_holes[uiHole1])) continue;
		Polygon poly1;
		m_vec_robots[uiRobot1].getPoseToPoint(boeing.m_vec_holes[uiHole1].getLoc(), poly1);
		
		for (size_t uiHole2 = 0; uiHole2 < boeing.m_vec_holes.size(); uiHole2++)
		{
			if (uiHole1 == uiHole2) continue;
			if (false == m_vec_robots[uiRobot2].isHoleReachable(boeing.m_vec_holes[uiHole2])) continue;
			Polygon poly2;
			m_vec_robots[uiRobot2].getPoseToPoint(boeing.m_vec_holes[uiHole2].getLoc(), poly2);
			
			if (boost::geometry::intersects(poly1, poly2))
			{
				m_handle.add_conflict((2 * c_uiNumRobots) + uiHole1, uiRobot1, (2 * c_uiNumRobots) + uiHole2, uiRobot2);
			}
		}
	}
}

void Data_Generator::compute_iv_pair_colls(const std::vector<map_rob_ind_id_pair> &map_iv_inds, const Boeing_Fuesalage &boeing)
{
	for (size_t uiRobot1 = 0; uiRobot1 < m_vec_robots.size() - 1; uiRobot1++)
	{
		for (size_t uiRobot2 = uiRobot1 + 1; uiRobot2 < m_vec_robots.size(); uiRobot2++)
		{
			compute_iv_pair_colls(map_iv_inds, uiRobot1, uiRobot2, boeing);
		}
	}
}

void Data_Generator::compute_iv_pair_colls(const std::vector<map_rob_ind_id_pair> &map_iv_inds, size_t uiRobot1, size_t uiRobot2, const Boeing_Fuesalage &boeing)
{
	for (auto it1 = map_iv_inds[uiRobot1].begin(); it1 != map_iv_inds[uiRobot1].end(); it1++)
	{
		size_t uiInd11 = it1->second.first.first , uiInd12 = it1->second.second.first;
		std::vector<Coordinates> vecCoords1;
		if (it1->second.first.second == "H")
		{
			if (false == m_vec_robots[uiRobot1].isHoleReachable(boeing.m_vec_holes[uiInd11])) continue;
			vecCoords1.push_back(boeing.m_vec_holes[uiInd11].getLoc());
		}
		else
		{
			vecCoords1.push_back(boeing.m_vec_depots[uiInd11]);
		}

		if ((it1->second.first.second == "H") && (it1->second.second.second == "H"))
		{
			if (boeing.m_vec_holes[uiInd11].getType() != boeing.m_vec_holes[uiInd12].getType())
			{
				vecCoords1.push_back(boeing.m_vec_tool_change_loc[uiRobot1]);
			}
		}

		if (it1->second.second.second == "H")
		{
			if (false == m_vec_robots[uiRobot1].isHoleReachable(boeing.m_vec_holes[uiInd12])) continue;
			vecCoords1.push_back(boeing.m_vec_holes[uiInd12].getLoc());
		}
		else
		{
			vecCoords1.push_back(boeing.m_vec_depots[uiInd12]);
		}
		
		Polygon poly1;		
		m_vec_robots[uiRobot1].getPoseEnvelope(vecCoords1, poly1);	

		for (auto it2 = map_iv_inds[uiRobot2].begin(); it2 != map_iv_inds[uiRobot2].end(); it2++)
		{
			size_t uiInd21 = it2->second.first.first, uiInd22 = it2->second.second.first;
			std::vector<Coordinates> vecCoords2;
			
			if (it2->second.first.second == "H")
			{
				if (false == m_vec_robots[uiRobot2].isHoleReachable(boeing.m_vec_holes[uiInd21])) continue;
				vecCoords2.push_back(boeing.m_vec_holes[uiInd21].getLoc());
			}
			else
			{
				vecCoords2.push_back(boeing.m_vec_depots[uiInd21]);
			}

			if ((it2->second.first.second == "H") && (it2->second.second.second == "H"))
			{
				if (boeing.m_vec_holes[uiInd21].getType() != boeing.m_vec_holes[uiInd22].getType())
				{
					vecCoords2.push_back(boeing.m_vec_tool_change_loc[uiRobot2]);
				}
			}

			if (it2->second.second.second == "H")
			{
				if (false == m_vec_robots[uiRobot2].isHoleReachable(boeing.m_vec_holes[uiInd22])) continue;
				vecCoords2.push_back(boeing.m_vec_holes[uiInd22].getLoc());
			}
			else
			{
				vecCoords2.push_back(boeing.m_vec_depots[uiInd22]);
			}
			
			Polygon poly2;
			m_vec_robots[uiRobot2].getPoseEnvelope(vecCoords2, poly2);

			if (boost::geometry::intersects(poly1, poly2))
			{
				m_handle.add_conflict(it1->first, uiRobot1, it2->first, uiRobot2);
			}
		}
	}
}

void Data_Generator::compute_v_iv_pair_colls(const std::vector<map_rob_ind_id_pair> &map_iv_inds, const Boeing_Fuesalage &boeing)
{
	for (size_t uiRobot1 = 0; uiRobot1 < m_vec_robots.size() - 1; uiRobot1++)
	{
		for (size_t uiRobot2 = uiRobot1 + 1; uiRobot2 < m_vec_robots.size(); uiRobot2++)
		{
			compute_v_iv_pair_colls(map_iv_inds, uiRobot1 , uiRobot2, boeing);
			compute_v_iv_pair_colls(map_iv_inds, uiRobot2, uiRobot1, boeing);
		}
	}
}

void Data_Generator::compute_v_iv_pair_colls(const std::vector<map_rob_ind_id_pair> &map_iv_inds, size_t uiRobot1, size_t uiRobot2, const Boeing_Fuesalage &boeing)
{
	size_t c_uiNumRobots = m_handle.get_num_robots();

	for (size_t uiHole1 = 0; uiHole1 < boeing.m_vec_holes.size(); uiHole1++)
	{
		if (false == m_vec_robots[uiRobot1].isHoleReachable(boeing.m_vec_holes[uiHole1])) continue;
		Polygon poly1;
		m_vec_robots[uiRobot1].getPoseToPoint(boeing.m_vec_holes[uiHole1].getLoc(), poly1);
		
		for (auto it2 = map_iv_inds[uiRobot2].begin(); it2 != map_iv_inds[uiRobot2].end(); it2++)
		{
			size_t uiInd21 = it2->second.first.first, uiInd22 = it2->second.second.first;
			std::vector<Coordinates> vecCoords2;

			if (it2->second.first.second == "H")
			{
				if (false == m_vec_robots[uiRobot2].isHoleReachable(boeing.m_vec_holes[uiInd21])) continue;
				vecCoords2.push_back(boeing.m_vec_holes[uiInd21].getLoc());
			}
			else
			{
				vecCoords2.push_back(boeing.m_vec_depots[uiInd21]);
			}

			if ((it2->second.first.second == "H") && (it2->second.second.second == "H"))
			{
				if (boeing.m_vec_holes[uiInd21].getType() != boeing.m_vec_holes[uiInd22].getType())
				{
					vecCoords2.push_back(boeing.m_vec_tool_change_loc[uiRobot2]);
				}
			}

			if (it2->second.second.second == "H")
			{
				if (false == m_vec_robots[uiRobot2].isHoleReachable(boeing.m_vec_holes[uiInd22])) continue;
				vecCoords2.push_back(boeing.m_vec_holes[uiInd22].getLoc());
			}
			else
			{
				vecCoords2.push_back(boeing.m_vec_depots[uiInd22]);
			}

			Polygon poly2;
			m_vec_robots[uiRobot2].getPoseEnvelope(vecCoords2, poly2);			

			if (boost::geometry::intersects(poly1, poly2))
			{
				m_handle.add_conflict(uiHole1 + (2* c_uiNumRobots), uiRobot1, it2->first, uiRobot2);
			}
		}
	}
}

void Data_Generator::compute_enablers(const std::set<size_t> &set_st_vert, const Boeing_Fuesalage &boeing)
{
	const size_t c_uiNumRobots = m_handle.get_num_robots();
	for (size_t uiHole1 = 0; uiHole1 < boeing.m_vec_holes.size(); uiHole1++)
	{
		if (set_st_vert.find(uiHole1 + (2 * c_uiNumRobots)) != set_st_vert.end())
		{
			for (size_t uiRobot = 0; uiRobot < m_vec_robots.size(); uiRobot++)
			{
				if (m_vec_robots[uiRobot].isHoleReachable(boeing.m_vec_holes[uiHole1]))
				{
					m_handle.add_enabler( uiHole1 + (2 * c_uiNumRobots) , 2*uiRobot );
					//m_handle.add_enabler( uiHole1 + (2 * c_uiNumRobots), (2 * uiRobot) + 1);
				}
			}
			continue;
		}

		for (size_t uiHole2 = 0; uiHole2 < boeing.m_vec_holes.size(); uiHole2++)
		{
			if (uiHole1 == uiHole2) continue;
			if (false == isProximalEnabling_Neigh_Strategy(boeing.m_vec_holes[uiHole1].getLoc(), boeing.m_vec_holes[uiHole2].getLoc(), boeing.m_dHorSpacing, boeing.m_dVertSpacing)) continue;
		
			m_handle.add_enabler(uiHole1 + (2 * c_uiNumRobots), uiHole2 + (2 * c_uiNumRobots));
		}
	}
}

#ifdef SINGLE_ROBOT_MODE

Data_Generator::Data_Generator(size_t uiNumRobots, size_t uiNumHoles):m_handle(uiNumRobots, uiNumHoles)
{ }

void Data_Generator::add_hole_info(const Special_Parser &boeing)
{
	const size_t c_uiOffset = 2 * m_handle.m_uiNumRobots;
	
	for (size_t uiIndex = 0; uiIndex < boeing.m_uiNumHoles; uiIndex++)
	{
		m_handle.add_hole(c_uiOffset + uiIndex, SCALE_TIME(TIME_PER_HOLE), boeing.m_vec_hole_coords[uiIndex]);
	}

	m_handle.add_hole(c_uiOffset + boeing.m_uiNumHoles, SCALE_TIME(TIME_PER_HOLE), Coordinates(0, 0, 0)); // dummy hole
}

void Data_Generator::set_start_locs(std::set<size_t> &set_st_vert, const Special_Parser &boeing)
{
	const size_t c_uiHoleOffset = 2 * m_handle.m_uiNumRobots;
	std::string strHole;

	for (auto it = boeing.m_set_comp_holes.begin(); it != boeing.m_set_comp_holes.end(); it++)
	{
		strHole = *it;
		for (auto it_enabled = boeing.m_set_adj_buffer.at(strHole).begin(); it_enabled != boeing.m_set_adj_buffer.at(strHole).end(); it_enabled++)
		{
			auto pr = boeing.isValidHole(*it_enabled);
			if (false == pr.first) continue;

			set_st_vert.emplace(c_uiHoleOffset + pr.second);
		}
	}	
}

void Data_Generator::parse_single_robot_case(const Special_Parser &boeing)
{
	m_handle.add_depot(0, DEPOT_TIME, 0, 1, Coordinates(0,0,0)); // actual data
	m_handle.add_depot(1, DEPOT_TIME, 2, 3, Coordinates(0, 0, 0)); // dummy robot
	
	add_hole_info(boeing);
	std::set<size_t> set_st_vert;
	set_start_locs(set_st_vert, boeing);

	add_edge_iv_info(boeing, set_st_vert);
	compute_enablers(boeing ,set_st_vert);
}

void Data_Generator::add_edge_iv_info(size_t uiRobot, double dTime, size_t uiGraphInd1, size_t uiGraphInd2, size_t uiIndex)
{
	size_t uiScaledTime = SCALE_TIME(dTime);
	m_handle.add_edge(uiRobot, uiGraphInd1, uiGraphInd2, uiScaledTime);
	m_handle.add_iv(uiRobot, uiGraphInd1, uiGraphInd2, uiIndex, uiScaledTime);
}

void Data_Generator::add_edge_iv_info(const Special_Parser &boeing, const std::set<size_t> &set_st_vert)
{
	const size_t c_uiNumRobots = m_handle.get_num_robots(), c_ui_Hole_Offset = (2 * c_uiNumRobots) ,c_ui_IV_Offset = (2 * c_uiNumRobots) + (m_handle.m_uiNumHoles);
	size_t uiRobot = 0, uiIndex = c_ui_IV_Offset;
	
	for (auto it = set_st_vert.begin(); it != set_st_vert.end(); it++)
	{
		double dTime = m_handle.getDepotMap().at(uiRobot).getLoc().getDist_XYZ(m_handle.getHolesMap().at(*it).getLoc());
		add_edge_iv_info(0, dTime, 2 * uiRobot, *it, uiIndex);
		uiIndex++;
	}

	for (size_t uiHole1 = 0; uiHole1 < boeing.m_uiNumHoles; uiHole1++)
	{
		for (size_t uiHole2 = 0; uiHole2 < boeing.m_uiNumHoles; uiHole2++)
		{
			if (uiHole1 == uiHole2) continue;
			add_edge_iv_info(uiRobot, boeing.m_vec_dist_buffer[uiHole1][uiHole2], c_ui_Hole_Offset + uiHole1, c_ui_Hole_Offset + uiHole2, uiIndex);
			uiIndex++;
		}
	}

	for (size_t uiHole = 0; uiHole < boeing.m_uiNumHoles; uiHole++)
	{
		//MAKESPAN CHANGE: change dTime variable to a small positive integer if return time to depot is not to be included
		double dTime = m_handle.getDepotMap().at(uiRobot).getLoc().getDist_XYZ(m_handle.getHolesMap().at(uiHole + (2*c_uiNumRobots)).getLoc());
		add_edge_iv_info(uiRobot, dTime, c_ui_Hole_Offset + uiHole, (2 * uiRobot) + 1, uiIndex);
		uiIndex++;
	}

	//add edge info for dummy robot and vertex
	size_t uiDummyrobot = 1;
	add_edge_iv_info(uiDummyrobot, 1, 2* uiDummyrobot, c_ui_Hole_Offset + m_handle.m_uiNumHoles-1, uiIndex);
	
	uiIndex++;

	add_edge_iv_info(uiDummyrobot, 1, c_ui_Hole_Offset + m_handle.m_uiNumHoles - 1, (2 * uiDummyrobot) + 1, uiIndex);
}

void Data_Generator::compute_enablers(const Special_Parser &boeing, const std::set<size_t> &set_st_vert)
{
	const size_t c_uiNumRobots = m_handle.get_num_robots();
	const size_t c_uiHoleOffset = 2 * c_uiNumRobots;
	size_t uiRobot = 0;
	std::string strHole;

	for (auto it = set_st_vert.begin(); it != set_st_vert.end(); it++)
	{
		m_handle.add_enabler(*it, 2* uiRobot);
	}

	for (size_t uiHole = 0; uiHole < boeing.m_uiNumHoles; uiHole++)
	{
		strHole = boeing.m_map_index_hole.at(uiHole);
		
		for (auto it_enable = boeing.m_set_adj_buffer.at(strHole).begin(); it_enable != boeing.m_set_adj_buffer.at(strHole).end(); it_enable++)
		{
			auto pr = boeing.isValidHole(*it_enable);
			if (false == pr.first) continue;
			
			if (set_st_vert.end() != set_st_vert.find(pr.second + c_uiHoleOffset)) continue; // we have enablers for these before
			m_handle.add_enabler(pr.second + c_uiHoleOffset, uiHole + c_uiHoleOffset);			
		}
	}	

	//for dummy hole
	size_t uiDummyRobot = 1;
	m_handle.add_enabler(c_uiHoleOffset + m_handle.m_uiNumHoles - 1, 2 * uiDummyRobot);
}

#endif