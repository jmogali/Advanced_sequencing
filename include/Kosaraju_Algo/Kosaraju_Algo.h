#pragma once
#ifndef KOSARAJU_ALGO_H
#define KOSARAJU_ALGO_H

#include <unordered_map>
#include <unordered_set>
#include <list>
#include <stack>

class Kosaraju_Algo
{
	private:
		std::unordered_map<size_t, bool> m_dfs_data;
		std::stack<size_t> m_stack_comp_vts;

		void perform_dfs(const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &graph);
		void initialize_DFS_container(const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &graph);
		void dfs(size_t uiVtx, const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &graph, std::unordered_set<size_t> *pComp = NULL);
		void reverse_graph(std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &rev_graph, const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &graph);
		void get_components(const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &rev_graph, std::list<std::unordered_set<size_t>> &listComp);

	public:
		void compute_maximal_components(const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &graph, std::list<std::unordered_set<size_t>> &listComp); //graph - vertex, cost
		void compute_maximal_components(const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &graph, const std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &rev_graph, std::list<std::unordered_set<size_t>> &listComp);
		Kosaraju_Algo();
};

#endif
