#include "Alternative_Graph.h"
#include "Kosaraju_Algo.h"
#include <assert.h>
#include <algorithm>
#include "Windows_Linux.h"

Arc_status::Arc_status(size_t uiVtx1, size_t uiVtx2) : m_arc(uiVtx1, uiVtx2), m_str_status(UNSELECTED)
{ }

bool Arc_status::areArcsSame(arc inp_arc)
{
	return m_arc == inp_arc;
}

Arc_status& Alterative_arc::getArc(size_t uiVtx1, size_t uiVtx2)
{
	arc inp_arc(uiVtx1, uiVtx2);
	if (true == m_arc1.areArcsSame(inp_arc)) return m_arc1;
	else if (true == m_arc2.areArcsSame(inp_arc)) return m_arc2;
	else
	{
		assert(false);
		return m_arc1; // should not occur
	}
}

Arc_status& Alterative_arc::getArc(arc inp_arc)
{
	return getArc(inp_arc.first , inp_arc.second);
}

Arc_status& Alterative_arc::getAltArc(size_t uiVtx1, size_t uiVtx2)
{
	arc inp_arc(uiVtx1, uiVtx2);
	if (true == m_arc1.areArcsSame(inp_arc)) return m_arc2;
	else if (true == m_arc2.areArcsSame(inp_arc)) return m_arc1;
	else
	{
		assert(false);
		return m_arc1; // should not occur
	}
}

Alterative_arc::Alterative_arc(size_t uiVtx11, size_t uiVtx12, size_t uiVtx21, size_t uiVtx22):m_arc1(uiVtx11, uiVtx12) , m_arc2(uiVtx21, uiVtx22)
{ }

void Alterative_arc::setStatus(size_t uiVtx1, size_t uiVtx2, std::string strSel)
{
	arc inp_arc(uiVtx1 , uiVtx2);
	if (true == m_arc1.areArcsSame(inp_arc)) { m_arc1.setStatus(strSel); }
	else if (true == m_arc2.areArcsSame(inp_arc)) {	m_arc2.setStatus(strSel); }
	else 
	{	
		assert(false);  // This should not occur 
	}
}

bool Alterative_arc::isSelected()
{
	if((UNSELECTED == m_arc1.getStatus()) && (UNSELECTED == m_arc2.getStatus())) return false;
	if (((UNSELECTED == m_arc1.getStatus()) && (UNSELECTED != m_arc2.getStatus())) || ((UNSELECTED != m_arc1.getStatus()) && (UNSELECTED == m_arc2.getStatus()))) assert(false);
	return true;
}

Alternative_Graph::Alternative_Graph()
{}

void Alternative_Graph::add_prec_arc(size_t uiVtx1, size_t uiVtx2, size_t uiCost)
{
	if (m_vec_adj_set_out[uiVtx1].end() != m_vec_adj_set_out[uiVtx1].find(uiVtx2))
	{
#ifdef WINDOWS		
		assert(false);	// need to analyze this state
#else
		exit(1);
#endif
	}

	m_vec_adj_set_out[uiVtx1].emplace(uiVtx2 , uiCost);
	m_vec_adj_set_in[uiVtx2].emplace(uiVtx1, uiCost);
}

void Alternative_Graph::add_prec_arc(arc new_arc, size_t uiCost)
{
	return add_prec_arc(new_arc.first, new_arc.second, uiCost);
}

void Alternative_Graph::remove_prec_arc(size_t uiVtx1, size_t uiVtx2, bool bAltGraphBackTrack)
{
	if(bAltGraphBackTrack) assert(0 == m_vec_adj_set_out[uiVtx1].at(uiVtx2));
	
	m_vec_adj_set_out[uiVtx1].erase(uiVtx2);
	m_vec_adj_set_in[uiVtx2].erase(uiVtx1);
}

void Alternative_Graph::remove_prec_arc(arc new_arc, bool bAltGraphBackTrack)
{
	return remove_prec_arc(new_arc.first , new_arc.second, bAltGraphBackTrack);
}

void Alternative_Graph::add_vertex_ownership_pos(size_t uiVtx, size_t uiRobot, size_t uiPos)
{
	m_map_vertex_robot_pos_map.emplace(uiVtx , std::make_pair(uiRobot , uiPos));
}

void Alternative_Graph::add_alt_arc(size_t uiVtx11, size_t uiVtx12, size_t uiVtx21, size_t uiVtx22)
{
	m_alt_edges.emplace_back(uiVtx11 , uiVtx12 , uiVtx21, uiVtx22);
	size_t uiInd = m_alt_edges.size() - 1;
		
	m_vec_vtx_alt_edge_ind_out[uiVtx11].emplace(uiVtx12, uiInd);
	m_vec_vtx_alt_edge_ind_in[uiVtx12].emplace(uiVtx11, uiInd);
	
	m_vec_vtx_alt_edge_ind_out[uiVtx21].emplace(uiVtx22, uiInd);
	m_vec_vtx_alt_edge_ind_in[uiVtx22].emplace(uiVtx21, uiInd);	
}

void Alternative_Graph::get_next_strongly_conn_components(const std::unordered_set<size_t> &B_Q, const std::unordered_set<size_t> &Q, std::list<std::unordered_set<size_t>> &listComp)
{
	assert(true == listComp.empty());
	std::unordered_map<size_t, std::unordered_map<size_t, size_t>> graph;
	construct_induced_sub_graph(graph, B_Q);
	Kosaraju_Algo obj_strong_conn;
	obj_strong_conn.compute_maximal_components(graph, listComp);
	filter_components(B_Q, Q, listComp);
}

void Alternative_Graph::construct_induced_sub_graph(std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &graph, const std::unordered_set<size_t> &B_Q)
{
	size_t uiVtx1, uiVtx2;
	
	for (auto it = B_Q.begin(); it != B_Q.end(); it++)
	{
		graph.emplace(*it, std::unordered_map<size_t, size_t>());
	}

	for (auto it1 = B_Q.begin(); it1 != B_Q.end(); it1++)
	{
		uiVtx1 = *it1;

		for (auto it2 = B_Q.begin(); it2 != B_Q.end(); it2++)
		{
			uiVtx2 = *it2;
			if (uiVtx1 == uiVtx2) continue;

			if (m_vec_adj_set_out[uiVtx1].find(uiVtx2) != m_vec_adj_set_out[uiVtx1].end())
			{
				graph.at(uiVtx1).emplace(uiVtx2, m_vec_adj_set_out[uiVtx1].at(uiVtx2));
			}
		}	
	}
}

void Alternative_Graph::filter_components(const std::unordered_set<size_t> &B_Q, const std::unordered_set<size_t> &Q, std::list<std::unordered_set<size_t>> &listComp)
{
	for (auto it = listComp.begin(); it != listComp.end(); )
	{
		if (true == contains_incoming_edge(B_Q, Q, *it))
		{
			it = listComp.erase(it);			// it is okay to erase because the list components form a DAG
		}
		else
			it++;
	}	
}

bool Alternative_Graph::contains_incoming_edge(const std::unordered_set<size_t> &B_Q, const std::unordered_set<size_t> &Q, const std::unordered_set<size_t> &comp)
{
	size_t uiHeadVtx , uiTailVtx;
	for (auto it1 = comp.begin(); it1 != comp.end(); it1++)
	{
		uiHeadVtx = *it1;
		for (auto it2 = m_vec_adj_set_in[uiHeadVtx].begin(); it2 != m_vec_adj_set_in[uiHeadVtx].end(); it2++)
		{
			uiTailVtx = it2->first;
			if (comp.find(uiTailVtx) != comp.end()) continue;
			if (B_Q.find(uiTailVtx) != B_Q.end()) return true;
			if (Q.find(uiTailVtx) != Q.end()) return true;
		}
	}
	return false;
}

// note we have eused B_P instead of R (refer blocking job shop paper for terminology)
bool Alternative_Graph::get_arcs_to_make_sel_positional(const std::unordered_set<size_t> &B_P, std::unordered_map<size_t, std::pair<std::pair<arc, std::string>, std::pair<arc, std::string>>> &map_new_sel_arcs)
{
	size_t uiVtx1, uiVtx2, uiInd;

	for (auto it = B_P.begin(); it != B_P.end(); it++)
	{
		uiVtx2 = *it;
		for (auto it = m_vec_vtx_alt_edge_ind_in[uiVtx2].begin(); it != m_vec_vtx_alt_edge_ind_in[uiVtx2].end(); it++)
		{
			uiVtx1 = it->first;
			uiInd = it->second;

			if (true == m_alt_edges[uiInd].isSelected()) continue;

			auto &arc_status1 = m_alt_edges[uiInd].getArc(uiVtx1, uiVtx2);
			auto &arc_status2 = m_alt_edges[uiInd].getAltArc(uiVtx1, uiVtx2);

			auto it_arc = map_new_sel_arcs.find(uiInd);
			if (it_arc != map_new_sel_arcs.end())
			{
				if ((arc_status2.areArcsSame(it_arc->second.first.first)) || (arc_status1.areArcsSame(it_arc->second.second.first)))
				{
					map_new_sel_arcs.clear();
					return false;
				}
			}
			map_new_sel_arcs.emplace(uiInd, std::make_pair(std::make_pair(arc_status1.getArc(), FORBIDDEN), std::make_pair(arc_status2.getArc(), SELECTED)));
		}
	}
	return true;
}

void Alternative_Graph::make_selection_positional(const std::unordered_map<size_t, std::pair<std::pair<arc, std::string>, std::pair<arc, std::string>>> &map_new_sel_arcs)
{
	size_t uiInd;
	for (auto it = map_new_sel_arcs.begin(); it != map_new_sel_arcs.end(); it++)
	{
		uiInd = it->first;
		auto &arc_status1 = m_alt_edges[uiInd].getArc(it->second.first.first);
		auto &arc_status2 = m_alt_edges[uiInd].getArc(it->second.second.first);

		assert(FORBIDDEN == it->second.first.second);
		assert(SELECTED == it->second.second.second);

		arc_status1.setStatus(it->second.first.second);		// verify, has to be FORBIDDED
		arc_status2.setStatus(it->second.second.second);    // has to be SELECTED

		add_prec_arc(arc_status2.getArc(), 0);
	}
}

void Alternative_Graph::unselect_positional_arcs(const std::unordered_map<size_t, std::pair<std::pair<arc, std::string>, std::pair<arc, std::string>>> &map_new_sel_arcs)
{
	size_t uiInd;
	for (auto it = map_new_sel_arcs.begin(); it != map_new_sel_arcs.end(); it++)
	{
		uiInd = it->first;
		auto &arc_status1 = m_alt_edges[uiInd].getArc(it->second.first.first);
		auto &arc_status2 = m_alt_edges[uiInd].getArc(it->second.second.first);

		arc_status1.setStatus(UNSELECTED);
		arc_status2.setStatus(UNSELECTED);

		assert(SELECTED == it->second.second.second);
		remove_prec_arc(arc_status2.getArc());
	}
}

std::pair<bool, size_t> Alternative_Graph::get_best_preceding_arc(size_t uiVtx, size_t uiOtherRobot) const
{
	size_t uiPrevVtx, uiBestVtx = std::numeric_limits<size_t>::max(), uiPos = std::numeric_limits<size_t>::min();
	bool bFound = false;

	for (auto it = m_vec_adj_set_in.at(uiVtx).begin(); it != m_vec_adj_set_in.at(uiVtx).end(); it++)
	{
		uiPrevVtx = it->first;
		if (m_map_vertex_robot_pos_map.at(uiVtx).first == uiOtherRobot)
		{
			if (uiPos < m_map_vertex_robot_pos_map.at(uiVtx).second)
			{
				uiPos = m_map_vertex_robot_pos_map.at(uiVtx).second;
				uiBestVtx = uiPrevVtx;
				bFound = true;
			}			
		}
	}
	return std::make_pair(bFound , uiBestVtx);
}

void Alternative_Graph::allocate_buffer_for_graph(const std::vector<std::list<size_t>> &rob_seq)
{
	for (size_t uiRobot = 0; uiRobot < rob_seq.size(); uiRobot++)
	{
		for (auto it = rob_seq[uiRobot].begin(); it != rob_seq[uiRobot].end(); it++)
		{
			m_vec_adj_set_out.emplace(*it, std::unordered_map<size_t, size_t>());
			m_vec_adj_set_in.emplace(*it, std::unordered_map<size_t, size_t>());
			m_vec_vtx_alt_edge_ind_in.emplace(*it, std::unordered_map<size_t, size_t>());
			m_vec_vtx_alt_edge_ind_out.emplace(*it, std::unordered_map<size_t, size_t>());
		}
	}
}

void Alternative_Graph::clear_prev_info()
{
	m_vec_adj_set_out.clear();
	m_vec_adj_set_in.clear();
	m_vec_vtx_alt_edge_ind_in.clear();
	m_vec_vtx_alt_edge_ind_out.clear();	
	m_map_vertex_robot_pos_map.clear();
	m_alt_edges.clear();
}

std::pair<bool, size_t> Alternative_Graph::get_next_vtx_same_job(size_t uiVtx) const
{
	if (false == containsVertex(uiVtx)) return std::make_pair(false, std::numeric_limits<size_t>::max());

	size_t uiRobot = get_vertex_ownership(uiVtx);
	size_t uiNext = std::numeric_limits<size_t>::max();

	for (auto it = m_vec_adj_set_out.at(uiVtx).cbegin(); it != m_vec_adj_set_out.at(uiVtx).cend(); it++)
	{
		if (uiRobot == get_vertex_ownership(it->first))
		{
			uiNext = it->first;
			break;
		}
	}
	if (std::numeric_limits<size_t>::max() == uiNext) return std::make_pair(false, uiNext);
	else return std::make_pair(true, uiNext);
}

std::pair<bool, size_t> Alternative_Graph::get_prec_vtx_same_job(size_t uiVtx) const
{
	if (false == containsVertex(uiVtx)) return std::make_pair(false, std::numeric_limits<size_t>::max());

	size_t uiRobot = get_vertex_ownership(uiVtx);
	size_t uiPrev = std::numeric_limits<size_t>::max();

	for (auto it = m_vec_adj_set_in.at(uiVtx).begin(); it != m_vec_adj_set_in.at(uiVtx).end(); it++)
	{
		if (uiRobot == get_vertex_ownership(it->first))
		{
			uiPrev = it->first;
			break;
		}
	}
	if (std::numeric_limits<size_t>::max() == uiPrev) return std::make_pair(false, uiPrev);
	else return std::make_pair(true, uiPrev);
}

bool Alternative_Graph::check_if_all_collisions_backwards(size_t uiVtx1, size_t uiRobot1, const std::vector<size_t>& vec_rob_vertpos)
{
#ifdef WINDOWS
	assert(uiRobot1 == get_vertex_ownership(uiVtx1));	
#else
	if (uiRobot1 != get_vertex_ownership(uiVtx1)) exit(-1);	
#endif

	size_t uiRobot, uiVtx, uiPos;
	for (auto it = m_vec_vtx_alt_edge_ind_in[uiVtx1].begin(); it != m_vec_vtx_alt_edge_ind_in[uiVtx1].end(); it++)
	{
		uiVtx = it->first;
		uiRobot = get_vertex_ownership(uiVtx);
		uiPos = get_vertex_position(uiVtx) - 1; // -1 is because this is an incoming alternating edge, the true collision vertex is 1 behind
		
		if (vec_rob_vertpos[uiRobot] < uiPos) return false; //false implies forward edge
	}
	
	return true;
}