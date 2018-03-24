#pragma once
#ifndef SCHEDULE_VALIDITY_CHECK_H
#define SCHEDULE_VALIDITY_CHECK_H

#include "Vertex_Schedule.h"
#include <vector>
#include <list>
#include <unordered_map>
#include "Layout_LS.h"
#include "Windows_Linux.h"

class Schedule_Validity_Check
{
	private:
		void populate_start_timemap(const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, std::unordered_map<size_t, std::pair<size_t, size_t>> &map_start_times);

		bool check_sequence_info(const std::vector<std::list<size_t>> &rob_seq, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const Layout_LS &graph); //checks if timing and intermediate vertices are correct
		bool check_sequence_info_robot(size_t uiRobot, const std::vector<std::list<size_t>> &rob_seq, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const Layout_LS &graph);
				
		bool check_collision_enabling(const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const Layout_LS &graph);
		
		bool check_enabling_info(const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const Layout_LS &graph, const std::unordered_map<size_t, std::pair<size_t, size_t>> &map_start_times);
		bool check_enabling_info_robot(size_t uiRobot, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const Layout_LS &graph, const std::unordered_map<size_t, std::pair<size_t, size_t>> &map_start_times);
		
		bool check_collision_info(const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const Layout_LS &graph, const std::unordered_map<size_t, std::pair<size_t, size_t>> &map_start_times);
		bool check_collision_info_robot(size_t uiRobot, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const Layout_LS &graph, const std::unordered_map<size_t, std::pair<size_t, size_t>> &map_start_times);
		bool check_collision_info_robot_robot(size_t uiRobot, size_t uiOtherRobot, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const Layout_LS &graph, const std::unordered_map<size_t, std::pair<size_t, size_t>> &map_start_times);

	public:
		bool check_vertex_schedule(const std::vector<std::list<size_t>> &rob_seq, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const Layout_LS &graph);		
};

#endif