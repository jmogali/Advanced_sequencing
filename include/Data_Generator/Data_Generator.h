#pragma once
#ifndef DATA_GENERATOR_H
#define DATA_GENERATOR_H

#include <vector>
#include "Typedefs.h"
#include "Robot.h"
#include "Boeing_Fuesalage.h"
#include "Layout_Boeing.h"
#include <fstream>
#include "Special_Parser.h"
#include "Windows_Linux.h"

class Data_Generator 
{
	private:
		typedef std::pair<size_t, std::string> ind_name;
		typedef std::pair<ind_name, ind_name> rob_ind_id_pair;
		typedef std::unordered_map<size_t, rob_ind_id_pair> map_rob_ind_id_pair;

		Layout_Boeing m_handle;
		std::vector<Robot> m_vec_robots;
		void add_vertex_data(const Boeing_Fuesalage &boeing);
		void add_depot_info(const Boeing_Fuesalage &boeing);
		void add_hole_info(const Boeing_Fuesalage &boeing);
		void add_edge_iv_info(const std::set<size_t> &set_st_vert, std::vector<map_rob_ind_id_pair> &map_iv_inds, const Boeing_Fuesalage &boeing);
		void add_edge_iv_info_robot(size_t uiRobot , const std::set<size_t> &set_st_vert, size_t &uiIndex, std::vector<map_rob_ind_id_pair> &map_iv_inds, const Boeing_Fuesalage &boeing);
		void compute_start_locs(std::set<size_t> &set_st_vert, const Boeing_Fuesalage &boeing);
		void add_edge_iv_info(size_t uiRobot, const Coordinates &loc1, const Coordinates &loc2, size_t uiGraphInd1, size_t uiGraphInd2, bool bToolChange, size_t uiIndex, const Boeing_Fuesalage &boeing);
		void compute_collisions(const std::vector<map_rob_ind_id_pair> &map_iv_inds, const Boeing_Fuesalage &boeing);
		void compute_hole_pair_colls(const Boeing_Fuesalage &boeing);
		void compute_hole_pair_colls(size_t uiRobot1, size_t uiRobot2, const Boeing_Fuesalage &boeing);
		void compute_iv_pair_colls(const std::vector<map_rob_ind_id_pair> &map_iv_inds, const Boeing_Fuesalage &boeing);
		void compute_iv_pair_colls(const std::vector<map_rob_ind_id_pair> &map_iv_inds, size_t uiRobot1, size_t uiRobot2, const Boeing_Fuesalage &boeing);
		void compute_v_iv_pair_colls(const std::vector<map_rob_ind_id_pair> &map_iv_inds, const Boeing_Fuesalage &boeing);
		void compute_v_iv_pair_colls(const std::vector<map_rob_ind_id_pair> &map_iv_inds, size_t uiRobot1, size_t uiRobot2, const Boeing_Fuesalage &boeing);
		void compute_enablers(const std::set<size_t> &set_st_vert, const Boeing_Fuesalage &boeing);
	
#ifdef SINGLE_ROBOT_MODE
		void add_hole_info(const Special_Parser &boeing);
		void set_start_locs(std::set<size_t> &set_st_vert, const Special_Parser &boeing);
		void add_edge_iv_info(const Special_Parser &boeing, const std::set<size_t> &set_st_vert);
		void add_edge_iv_info(size_t uiRobot, double dTime, size_t uiGraphInd1, size_t uiGraphInd2, size_t uiIndex);
		void compute_enablers(const Special_Parser &boeing, std::set<size_t> &set_st_vert);
#endif

	//Print files
		void print_header_info(std::string strFileName, ofstream &myFile);
		void print_node_description(ofstream &myFile);
		void print_edges(ofstream &myFile);
		void print_collisions(ofstream &myFile);
		void print_enablers(ofstream &myFile);
		void print_start_end_des(ofstream &myFile);

	public:
		Data_Generator(size_t uiNumRobots , size_t uiNumHoles , const std::vector<Robot> &vec_robots);
		void populate_data(const Boeing_Fuesalage &boeing);
		void print_data_files(std::string strFolder, std::string strFileName);

#ifdef SINGLE_ROBOT_MODE
		Data_Generator(size_t uiNumRobots, size_t uiNumHoles);
		void parse_single_robot_case(const Special_Parser &boeing);
#endif
};

#endif