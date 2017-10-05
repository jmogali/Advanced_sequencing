#pragma once
#ifndef DATA_PARSER_H
#define DATA_PARSER_H

#include "fstream"
#include "Layout_Graph.h"

class Data_Parser
{
	private:
		std::string m_strFile;
		void add_vertices(ifstream &myFile, Layout_Graph &graph , std::unordered_map<size_t, size_t> &map_iv_vec);
		void add_edges_iv(ifstream &myFile, Layout_Graph &graph, std::unordered_map<size_t, size_t> &map_iv_vec);
		void add_collisons(ifstream &myFile, Layout_Graph &graph);
		void add_enablers(ifstream &myFile, Layout_Graph &graph);
		void add_depots(ifstream &myFile, Layout_Graph &graph);

	public:
		Data_Parser(std::string strFile);
		int read_header_info(std::pair<size_t, size_t> &pr);
		void populate_info(int iOffset, Layout_Graph &graph);
};

#endif
