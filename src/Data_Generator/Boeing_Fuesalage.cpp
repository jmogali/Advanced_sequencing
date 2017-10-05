#include "Boeing_Fuesalage.h"
#include <fstream>

using namespace std;

Boeing_Fuesalage::Boeing_Fuesalage(size_t uiFrames, size_t uiRobots, double dWidth, double dHeight, double dHorSpace, double dVertSpace, std::string strategy = "DEFAULT"):m_uiNumFrames(uiFrames), m_uiNumRobots(uiRobots), m_dWidth(dWidth) , m_dHeight(dHeight), m_dHorSpacing(dHorSpace), m_dVertSpacing(dVertSpace)
{
	assert(uiRobots <= 4);
	construct_layout(strategy);
}

void Boeing_Fuesalage::construct_layout(std::string strategy)
{
	addDepots();
	addToolChangeLocs();

	if ("DEFAULT" == strategy)
	{
		add_Tacks_Holes_Default();
	}
	else
	{
		//To be filled later;
	}
}

void Boeing_Fuesalage::addDepots()
{
	std::vector<Coordinates> vec_locs;
	vec_locs.push_back(Coordinates(0.0, 0.0 , 0.0));
	vec_locs.push_back(Coordinates(m_dWidth, 0.0 , 0.0));
	vec_locs.push_back(Coordinates(0.0, m_dHeight, 0.0));
	vec_locs.push_back(Coordinates(m_dWidth, m_dHeight, 0.0));

	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		m_vec_depots.push_back(vec_locs[uiRobot]);
	}
}

void Boeing_Fuesalage::addToolChangeLocs()
{
	std::vector<Coordinates> vec_locs;
	vec_locs.push_back(Coordinates(0.0, 1.0, 0.0));
	vec_locs.push_back(Coordinates(m_dWidth, 1.0, 0.0));
	vec_locs.push_back(Coordinates(0.0, m_dHeight-1, 0.0));
	vec_locs.push_back(Coordinates(m_dWidth, m_dHeight-1, 0.0));

	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		m_vec_tool_change_loc.push_back(vec_locs[uiRobot]);
	}
}

void Boeing_Fuesalage::add_Tacks_Holes_Default()
{
	double d_x , d_y;
	size_t ui_Row , uiMiddle, uiFrame_Holes, uiHoleInd = 0;

	m_vec_tacks.resize(m_uiNumFrames);

	for (size_t uiFrame = 0; uiFrame < m_uiNumFrames; uiFrame++)
	{
		d_x = (uiFrame * 1.0 * (m_dWidth - m_dHorSpacing)) / (m_uiNumFrames-1);
		d_y = 0;

		uiFrame_Holes = 0;
		ui_Row = 0;
		uiMiddle = (size_t)(1.0 * m_dHeight / (2.0 * m_dVertSpacing));
		
		while (d_y < m_dWidth)
		{
			if (uiMiddle == ui_Row)
			{
				m_vec_tacks[uiFrame].push_back(Coordinates(d_x, d_y, 0.0));
			}
			else
			{
				m_vec_holes.push_back(Hole(Coordinates(d_x, d_y, 0.0), "SMALL", uiFrame));
				uiFrame_Holes++;
			}		

			uiHoleInd++;

			m_vec_holes.push_back(Hole(Coordinates(d_x + m_dHorSpacing, d_y , 0.0), "SMALL", uiFrame));
			uiFrame_Holes++;

			uiHoleInd++;
			ui_Row++;

			d_y = d_y + m_dVertSpacing;
		}		
	}
}

void Boeing_Fuesalage::print_layout(std::string strFolder, std::string strFileName)
{
	ofstream hole_file;
	std::string strHoleFile = strFolder + "/" + strFileName + "_holes.dat";
	hole_file.open(strHoleFile.c_str());

	for (size_t uiCount = 0; uiCount < m_vec_holes.size(); uiCount++)
	{
		Coordinates loc = m_vec_holes[uiCount].getLoc();
		hole_file << loc.get_X_loc() << " " << loc.get_Y_loc()<<"\n";
	}
	hole_file.close();

	ofstream tack_file;
	std::string strTackFile = strFolder + "/" + strFileName + "_tacks.dat";
	tack_file.open(strTackFile.c_str());

	for (size_t uiFrame = 0; uiFrame < m_vec_tacks.size(); uiFrame++)
	{
		for (size_t uiCount = 0; uiCount < m_vec_tacks[uiFrame].size(); uiCount++)
		{
			tack_file << m_vec_tacks[uiFrame][uiCount].get_X_loc() << " " << m_vec_tacks[uiFrame][uiCount].get_Y_loc() << "\n";
		}		
	}
	tack_file.close();
}

void Boeing_Fuesalage::help_robot_set_up(std::vector<Robot> &vec_Robots) const
{
	assert(m_uiNumRobots == 2);

	std::set<size_t> set_frames_0;
	size_t ui_Max_Frame_0 = (size_t)floor(ROBOT_FRACTION_COVERAGE * m_uiNumFrames);
	for (size_t uiCount = 0; uiCount < ui_Max_Frame_0; uiCount++)
		set_frames_0.emplace(uiCount);

	vec_Robots.push_back(Robot(m_vec_depots[0], set_frames_0));

	std::set<size_t> set_frames_1;
	size_t ui_Min_Frame_1 = m_uiNumFrames - (size_t)ceil(ROBOT_FRACTION_COVERAGE * m_uiNumFrames);
	for (size_t uiCount = ui_Min_Frame_1; uiCount < m_uiNumFrames; uiCount++)
		set_frames_1.emplace(uiCount);

	vec_Robots.push_back(Robot(Coordinates(m_vec_depots[1]) , set_frames_1));
}