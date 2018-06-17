#pragma once
#ifndef SPECIAL_PARSER_H
#define SPECIAL_PARSER_H

#include <unordered_map>
#include <set>
#include <vector>
#include <fstream>

#include <boost/algorithm/string.hpp>
using namespace std;
using namespace boost;

#include "Layout_utils.h"
class Special_Parser
{
	private:
		size_t m_uiNumHoles;
		std::set<std::string> m_set_comp_holes;
		std::unordered_map<size_t, std::string> m_map_index_hole;
		std::unordered_map< std::string, size_t> m_map_hole_index;
		std::unordered_map<std::string, std::set<std::string>> m_set_adj_buffer;
		std::vector<std::vector<double>> m_vec_dist_buffer;
		std::vector<Coordinates> m_vec_hole_coords;

		void populate_comp_holes(std::string strFilledHoles);
		void populate_adjacency_info(std::string strEnablerFile);
		void populate_reachable_holes();
		void populate_distance_buffer(std::string strDistFile);
		void populate_hole_coord(std::string strHoleFile);		

	public:
		void parse_files(std::string strHoleFile, std::string strDistFile, std::string strEnablerFile, std::string strFilledHoles);
		inline size_t get_num_holes() { return m_uiNumHoles; };
		std::pair<bool, size_t> isValidHole(std::string strHole) const;
		Special_Parser();
		friend class Data_Generator;
};

#endif
