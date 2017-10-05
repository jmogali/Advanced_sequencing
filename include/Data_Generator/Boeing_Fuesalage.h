#pragma once
#ifndef BOEING_LAYOUT_H
#define BOEING_LAYOUT_H

#include "Hole.h"
#include "Robot.h"
#include <vector>

class Data_Generator;

class Boeing_Fuesalage
{
	private:
		const size_t m_uiNumFrames , m_uiNumRobots;
		const double m_dWidth, m_dHeight;
		const double m_dHorSpacing, m_dVertSpacing;
		std::vector<Coordinates> m_vec_depots;
		std::vector<Hole> m_vec_holes;
		std::vector<std::vector<Coordinates>> m_vec_tacks;
		std::vector<Coordinates> m_vec_tool_change_loc;
		void construct_layout(std::string strategy);
		void addDepots();
		void addToolChangeLocs();
		void add_Tacks_Holes_Default();		

	public:
		Boeing_Fuesalage(size_t uiFrames, size_t uiRobots, double dWidth, double dHeight, double dHorSpace, double dVertSpace, std::string strtegy);
		void print_layout(std::string strFolder, std::string strFile);
		inline size_t get_num_robots() const { return m_uiNumRobots; };
		inline size_t get_num_holes() const { return m_vec_holes.size(); };
		void help_robot_set_up(std::vector<Robot> &vec_Robots) const;
		friend class Data_Generator;
};

#endif
