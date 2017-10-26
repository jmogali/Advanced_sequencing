#pragma once
#ifndef GREEDY_HEURISTIC_COMPRESSION_H
#define GREEDY_HEURISTIC_COMPRESSION_H

#include "Greedy_Heuristic.h"

class Greedy_Heuristic_Compression: public Greedy_Heuristic
{
	private:
		std::unordered_map<size_t, std::vector<size_t>> m_map_superVtx_vecVtx;
		std::unordered_map<size_t, size_t> m_map_vtx_super_vtx;
		std::unordered_map<size_t, size_t> m_map_super_vtx_proc_time;

		void clear_prev_info_buffers();
		bool add_colls_compress_graph(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::list<size_t>> &new_rob_seq);
		bool gather_coll_cons_update_compr_verts(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<bool>> &vec_compress_status, std::vector<std::pair<arc, arc>> &alt_coll_edges);
		bool gather_coll_cons_compr_verts_rob_pair(size_t uiRobot1, size_t uiRobot2, const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<bool>> &vec_compress_status, std::vector<std::pair<arc, arc>> &alt_coll_edges);
		void construct_new_rob_sequence(const std::vector<std::list<size_t>> &rob_seq, const std::vector<std::vector<bool>> &vec_compress_status, std::vector<std::list<size_t>> &new_rob_seq);
		void update_compr_verts_by_unself_enabled_and_deps(std::vector<std::vector<bool>> &vec_compress_status);
		void reassign_enablers();

	public:
		Greedy_Heuristic_Compression(const size_t uiRobotNum, const Layout_LS &graph, Power_Set &power);
		int compute_greedy_sol(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, std::string strFolder);
};

#endif
