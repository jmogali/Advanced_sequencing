#pragma once
#ifndef ALTERNATING_GRAPH_COMPRESSION_H
#define ALTERNATING_GRAPH_COMPRESSION_H

#include "Alternative_Graph.h"

class Alternative_Graph_Compression : public Alternative_Graph
{
	private:
		void compress_prec_graph(const Layout_LS &layout_graph, const std::unordered_map<size_t, std::vector<size_t>> &map_superVtx_vecVtx, std::unordered_map<size_t, size_t> &map_super_vtx_proc_time);
		void add_compressed_alt_edges(const std::unordered_map<size_t, size_t> &map_vtx_super_vtx, const std::vector<std::pair<arc, arc>> &alt_coll_edges);
		void remove_buffer_redundant_vtx(size_t uiVert);
		void reallocate_buffers_compressed_vertices(const std::unordered_map<size_t, std::vector<size_t>> &map_superVtx_vecVtx);
		void reassign_vertex_ownership_positions(std::vector<std::list<size_t>> &new_rob_seq);
		void reassign_vertex_ownership_pos(size_t uiVtx, size_t uiRobot, size_t uiPos);
		void sanity_check_compression(std::vector<std::list<size_t>> &new_rob_seq);

	public:
		Alternative_Graph_Compression();
		void compress_graph(const Layout_LS &layout_graph, const std::unordered_map<size_t, std::vector<size_t>> &map_superVtx_vecVtx, const std::unordered_map<size_t, size_t> &map_vtx_super_vtx, const std::vector<std::pair<arc, arc>> &alt_coll_edges, std::vector<std::list<size_t>> &new_rob_seq, std::unordered_map<size_t, size_t> &map_super_vtx_proc_time);
};

#endif