#pragma once
#ifndef ALTERNATIVE_GRAPH_H
#define ALTERNATIVE_GRAPH_H

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include "Hashing_Utils.h"
#include <list>

namespace Alternative_Graphs
{
	const std::string SELECTED = "SELECTED";
	const std::string UNSELECTED = "UNSELECTED";
	const std::string FORBIDDEN = "FORBIDDEN";

	typedef std::pair<size_t, size_t> arc;	//tail = uiVtx1 , head = uiVtx2	

	inline bool operator== (const arc& lhs, const arc& rhs)
	{
		return (lhs.first == rhs.first) && (lhs.second == rhs.second);
	}
};

using namespace Alternative_Graphs;

class Arc_status
{
	private:
		arc m_arc;
		std::string m_str_status;
	
	public:
		Arc_status(size_t uiVtx1, size_t uiVtx2);
		inline std::string getStatus() { return m_str_status; };
		inline void setStatus(std::string strSel) { m_str_status = strSel; };
		bool areArcsSame(arc inp_arc);		
		inline size_t getTail() { return m_arc.first; };
		inline size_t getHead() { return m_arc.second; };
		inline const arc getArc() { return m_arc; };
};


class Alterative_arc
{
	private:
		Arc_status m_arc1, m_arc2;		
	public:
		Alterative_arc(size_t uiVtx11, size_t uiVtx12, size_t uiVtx21, size_t uiVtx22);
		void setStatus(size_t uiVtx1, size_t uiVtx2, std::string strStatus);
		Arc_status& getArc(size_t uiVtx1, size_t uiVtx2);
		Arc_status& getAltArc(size_t uiVtx1, size_t uiVtx2);
		Arc_status& getArc(arc inp_arc);
		bool isSelected();		
};

class Alternative_Graph
{
	private:
		std::unordered_map<size_t, std::unordered_map<size_t , size_t>> m_vec_adj_set_out;		// vtx , <adj vertex, cost>
		std::unordered_map<size_t, std::unordered_map<size_t, size_t>> m_vec_adj_set_in;		// vtx , <adj vertex, cost>
		std::unordered_map<size_t, std::pair<size_t , size_t> > m_map_vertex_robot_pos_map;			// vertex , <robot_owner, position>

		std::vector<Alterative_arc> m_alt_edges;
		std::unordered_map<size_t, std::unordered_map<size_t, size_t>> m_vec_vtx_alt_edge_ind_in;   // Vertex, Index - contains indices for m_alt_edges
		std::unordered_map<size_t, std::unordered_map<size_t, size_t>> m_vec_vtx_alt_edge_ind_out;   // Vertex, Index - contains indices for m_alt_edges

		void construct_induced_sub_graph(std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &graph, const std::unordered_set<size_t> &B_Q);
		void filter_components(const std::unordered_set<size_t> &B_Q, const std::unordered_set<size_t> &Q, std::list<std::unordered_set<size_t>> &listComp);
		bool contains_incoming_edge(const std::unordered_set<size_t> &B_Q, const std::unordered_set<size_t> &Q, const std::unordered_set<size_t> &comp);

		void add_prec_arc(arc new_arc, size_t uiCost);
		void remove_prec_arc(size_t uiVtx1, size_t uiVtx2);
		void remove_prec_arc(arc new_arc);
		
		//bool get_topological_order_robot_pairwise(std::unordered_map<size_t, std::string> &map_visited_state, std::list<size_t> &stack_topo_order, size_t uiVtx) const;

	public:
		Alternative_Graph();
		void add_prec_arc(size_t uiVtx1, size_t uiVtx2, size_t uiCost);
		void add_alt_arc(size_t uiVtx11, size_t uiVtx12, size_t uiVtx21, size_t uiVtx22);	
		
		void allocate_buffer_for_graph(const std::vector<std::list<size_t>> &rob_seq);
		void clear_prev_info();

		bool get_arcs_to_make_sel_positional(const std::unordered_set<size_t> &R, std::unordered_map<size_t, std::pair<std::pair<arc, std::string>, std::pair<arc, std::string>>> &map_new_sel_arcs);
		void get_next_strongly_conn_components(const std::unordered_set<size_t> &B_Q, const std::unordered_set<size_t> &Q, std::list<std::unordered_set<size_t>> &listComp);
		void make_selection_positional(const std::unordered_map<size_t, std::pair<std::pair<arc, std::string>, std::pair<arc, std::string>>> &map_new_sel_arcs);
		void unselect_positional_arcs(const std::unordered_map<size_t, std::pair<std::pair<arc, std::string>, std::pair<arc, std::string>>> &map_new_sel_arcs);
		
		inline size_t getArcCost(arc inp_arc) const { return m_vec_adj_set_out.at(inp_arc.first).at(inp_arc.second); };
		inline bool containsPrecArc(arc inp_arc) const { return m_vec_adj_set_out.at(inp_arc.first).find(inp_arc.second) != m_vec_adj_set_out.at(inp_arc.first).end() ? true : false; };

		const std::unordered_map<size_t, std::unordered_map<size_t, size_t>>& getGraph() { return m_vec_adj_set_out; }
		const std::unordered_map<size_t, std::unordered_map<size_t, size_t>>& getReverseGraph() { return m_vec_adj_set_in; }

		void add_vertex_ownership_pos(size_t uiVtx, size_t uiRobot, size_t uiPos);
		inline size_t get_vertex_ownership(size_t uiVtx) const { return m_map_vertex_robot_pos_map.at(uiVtx).first; };
		inline size_t get_vertex_position(size_t uiVtx) const { return m_map_vertex_robot_pos_map.at(uiVtx).second; };
		//bool get_topological_order(size_t uiRobot1, size_t uiRobot2, std::list<size_t> &stack_topo_order) const;
		std::pair<bool, size_t> get_best_preceding_arc(size_t uiVtx, size_t uiOtherRobot) const;			
};

#endif 

