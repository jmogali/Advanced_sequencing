#pragma once
#ifndef JORIS_SEQUENCE_FILE_PARSER_H
#define JORIS_SEQUENCE_FILE_PARSER_H

#include "Typedefs.h"
#include "Vertex_Schedule.h"
#include "Schedule_Validity_Check.h"
#include <string>
#include <vector>
#include <unordered_map>
#include "fstream"
#include "Layout_LS.h"
#include "Windows_Linux.h"

void parse_robot_sequence(std::ifstream &myFile, size_t uiNumRobots, std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch);
void parse_schedule_info(std::ifstream &myFile, size_t uiNumRobots, std::unordered_map<size_t, size_t> &map_vtx_time);
bool populate_schedule(const std::unordered_map<size_t, size_t> &map_vtx_time, std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const Layout_LS &graph);

bool parse_sequence_file(std::string strFile, size_t uiNumRobots, std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const Layout_LS &graph);
#endif
