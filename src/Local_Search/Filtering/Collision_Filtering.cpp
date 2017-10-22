#include "Collision_Filtering.h"
#include <assert.h>
#include <algorithm>
#include "Windows_Linux.h"


void Collision_Filtering::clear_prev_bounds_related_info()
{
	m_out_graph.clear();
	m_in_graph.clear();
	m_list_order.clear();
	m_list_Comp.clear();	
	m_map_bounds.clear();	
}

//m_out_graph(m_in_graph) graphs are different from those in alt_graph in the sense that some vertices correspond to strongly connected components
void Collision_Filtering::construct_in_out_graphs(const Alternative_Graph &alt_graph)
{
	construct_out_graph(alt_graph);
	construct_in_graph();
}

void Collision_Filtering::construct_out_graph(const Alternative_Graph &alt_graph)
{
	copy_to_out_graph(alt_graph.getGraph());
	replace_strong_conn_comp_out_graph(alt_graph);
}

void Collision_Filtering::copy_to_out_graph(const std::unordered_map<size_t, std::unordered_map<size_t, size_t>>& inp_graph)
{
	int iVtx;
	for (auto it = inp_graph.begin(); it != inp_graph.end(); it++)
	{
		iVtx = (int) it->first;
		m_out_graph.emplace(iVtx, std::unordered_set<int>());
		for (auto it_neighs = it->second.begin(); it_neighs != it->second.end(); it_neighs++)
		{
			m_out_graph.at(iVtx).emplace((int)(it_neighs->first));
		}
	}
}

void Collision_Filtering::construct_in_graph()
{
	for (auto it_vtx = m_out_graph.begin(); it_vtx != m_out_graph.end(); it_vtx++)
	{
		m_in_graph.emplace(it_vtx->first, std::unordered_set<int>());
	}

	for (auto it_tail = m_out_graph.begin(); it_tail != m_out_graph.end(); it_tail++)
	{
		for (auto it_head = it_tail->second.begin(); it_head != it_tail->second.end(); it_head++)
		{
			m_in_graph.at(*it_head).emplace(it_tail->first); // reverse the graph, so tail and head switched
		}
	}
}

// Also add redundant strongly connected component arcs
bool Collision_Filtering::Check_Feasibility_Compute_Bounds_For_Each_Vertex(const std::vector<std::list<size_t>> &rob_seq, const Alternative_Graph &alt_graph)
{
	clear_prev_bounds_related_info();
	
	Kosaraju_Algo obj;
	obj.compute_maximal_components(alt_graph.getGraph(), alt_graph.getReverseGraph(), m_list_Comp);
	bool bFeasible = !(Check_Pos_Loop_Remove_1comp(alt_graph));
	if (false == bFeasible) return false;
	
	construct_in_out_graphs(alt_graph);
	Topological_sort_out_graph();	

	Compute_bounds(alt_graph, rob_seq);
	bFeasible = check_bounds_validity(rob_seq);
	if (false == bFeasible) return false;

	return true;
}

bool Collision_Filtering::Check_Pos_Loop_Remove_1comp(const Alternative_Graph &alt_graph)
{
	size_t uiVtx1, uiVtx2, uiRobot;
	const auto& out_alt_graph = alt_graph.getGraph();

	for (auto it_list = m_list_Comp.begin(); it_list != m_list_Comp.end(); )
	{
		if (1 == it_list->size())
		{
			it_list = m_list_Comp.erase(it_list);
			continue;
		}

		for (auto it_vtx1 = it_list->begin(); it_vtx1 != it_list->end(); it_vtx1++)
		{
			uiVtx1 = *it_vtx1;
			uiRobot = alt_graph.get_vertex_ownership(uiVtx1);

			for (auto it_vtx2 = out_alt_graph.at(uiVtx1).begin(); it_vtx2 != out_alt_graph.at(uiVtx1).end(); it_vtx2++)
			{
				uiVtx2 = it_vtx2->first;
				if (uiRobot == alt_graph.get_vertex_ownership(uiVtx2))
				{
					if (it_list->end() != it_list->find(uiVtx2)) return true;
				}
			}
		}
		it_list++;
	}
	return false;
}

void Collision_Filtering::replace_strong_conn_comp_out_graph(const Alternative_Graph &alt_graph)
{
	size_t uiVtx;
	int iNewVtx = -1; // numbering it starting with -1 is very important, note we are decrementing
	const auto& out_alt_graph = alt_graph.getGraph();
	const auto& in_alt_graph = alt_graph.getReverseGraph();

	for (auto it_list = m_list_Comp.begin(); it_list != m_list_Comp.end(); it_list++)
	{
		assert(1 < it_list->size()); //at this stage, m_list_Comp contains only strongly connected components having more than 1 vertex
		m_out_graph.emplace(iNewVtx, std::unordered_set<int>());
		
		for (auto it_vtx = it_list->begin(); it_vtx != it_list->end(); it_vtx++)
		{
			uiVtx = *it_vtx;
			for (auto it_succ = out_alt_graph.at(uiVtx).begin(); it_succ != out_alt_graph.at(uiVtx).end(); it_succ++)
			{
				m_out_graph.at(iNewVtx).emplace((int)it_succ->first);
			}
			
			for (auto it_prec = in_alt_graph.at(uiVtx).begin(); it_prec != in_alt_graph.at(uiVtx).end(); it_prec++)
			{
				size_t uiErase = m_out_graph.at((int)it_prec->first).erase((int)uiVtx);
				assert(1 == uiErase);
			}
			m_out_graph.erase((int)uiVtx);
		}
		iNewVtx--;
	}
}

void Collision_Filtering::Topological_sort_out_graph()
{
	assert(0 == m_list_order.size());
	std::unordered_map<int, std::string> map_seen;
	int iVtx;
		
	for (auto it = m_out_graph.begin(); it != m_out_graph.end(); it++)
	{
		iVtx = it->first;
		if (map_seen.end() == map_seen.find(iVtx))
		{
			Topological_Dfs(iVtx, map_seen);
		}
	}
}

void Collision_Filtering::Topological_Dfs(int iVtx, std::unordered_map<int, std::string> &map_seen)
{
	auto it_insert = map_seen.emplace(iVtx , "GRAY");
	assert(true == it_insert.second);
	
	for (auto it = m_out_graph.at(iVtx).begin(); it != m_out_graph.at(iVtx).end(); it++)
	{
		auto it_neigh = map_seen.find(*it);
		if (map_seen.end() == it_neigh) Topological_Dfs(*it, map_seen);
		else assert("BLACK" == it_neigh->second); // helps to check validity of DAG, note all valid loops are converted to super vertices by this step already
	}

	map_seen[iVtx] = "BLACK"; 
	m_list_order.push_front(iVtx);
}

void Collision_Filtering::Initialize_bounds_map(std::unordered_map<size_t, std::unordered_map<size_t, std::pair<size_t, size_t>>> &m_map_bounds, const std::vector<std::list<size_t>> &rob_seq)
{
	// initialize bounds map
	for (size_t uiRobot = 0; uiRobot < rob_seq.size(); uiRobot++)
	{
		for (auto it = rob_seq[uiRobot].begin(); it != rob_seq[uiRobot].end(); it++)
		{
			m_map_bounds.emplace(*it, std::unordered_map<size_t, std::pair<size_t, size_t>>());
		}
	}
}

void Collision_Filtering::Compute_bounds(const Alternative_Graph &alt_graph, const std::vector<std::list<size_t>> &rob_seq)
{
	size_t uiNumRobots = rob_seq.size();
	Initialize_bounds_map(m_map_bounds, rob_seq);	

	for (size_t uiRobot1 = 0; uiRobot1 < uiNumRobots; uiRobot1++)
	{
		for (size_t uiRobot2 = uiRobot1+1; uiRobot2 < uiNumRobots; uiRobot2++)
		{
			std::list<int> rob_pair_list_order;
			compute_robot_pair_list_order(uiRobot1, uiRobot2, rob_pair_list_order, alt_graph);
			Compute_lower_bounds(uiRobot1, uiRobot2, rob_pair_list_order, rob_seq, alt_graph);
			Compute_upper_bounds(uiRobot1, uiRobot2, rob_pair_list_order, rob_seq, alt_graph);
		}
	}	
}

void Collision_Filtering::Compute_lower_bounds(size_t uiRobot1, size_t uiRobot2, const std::list<int> &rob_list_order, const std::vector<std::list<size_t>> &rob_seq, const Alternative_Graph &alt_graph)
{
	int iVtx;
	size_t uiStart_Vtx_1 = *rob_seq[uiRobot1].begin();
	size_t uiStart_Vtx_2 = *rob_seq[uiRobot2].begin();
	initialize_lower_bounds(uiRobot1, uiRobot2, rob_seq);	

	for (auto it = rob_list_order.begin(); it != rob_list_order.end(); it++)
	{
		iVtx = *it;		
		if (iVtx < 0)
		{
			// will only handle the case where both start vertices can be a maximal component or both are not, if only one of the start vertices is, we detect it as infeasibility
			size_t uiComp = (size_t)(-1 * iVtx) - 1;
			auto pr1 = containsVertex(uiRobot1, uiComp, alt_graph);
			auto pr2 = containsVertex(uiRobot2, uiComp, alt_graph);
						
			if ((pr1.second == uiStart_Vtx_1) && (pr2.second == uiStart_Vtx_2)) continue;
			else if ( (pr1.second == uiStart_Vtx_1) || (pr2.second == uiStart_Vtx_2) ) 
			{
#ifdef WINDOWS
				assert(false);
#else 
				exit(1);
#endif
			}
			
			if ( (true == pr1.first) && (true == pr2.first))
			{
				m_map_bounds.at(pr1.second).emplace(uiRobot2, std::make_pair(alt_graph.get_vertex_position(pr2.second), std::numeric_limits<size_t>::max()));
				m_map_bounds.at(pr2.second).emplace(uiRobot1, std::make_pair(alt_graph.get_vertex_position(pr1.second), std::numeric_limits<size_t>::max()));
			}
			else if (true == pr1.first)
			{
				insert_LB_non_r1_r2_comp_vtx(iVtx, pr1.second, uiRobot1, uiRobot2, alt_graph);
			}
			else if (true == pr2.first)
			{
				insert_LB_non_r1_r2_comp_vtx(iVtx, pr2.second, uiRobot2, uiRobot1, alt_graph);
			}
		}
		else
		{	
			size_t uiRobot, uiOtherRobot;
			if ((iVtx == (int)uiStart_Vtx_1) || (iVtx == (int)uiStart_Vtx_2)) continue;
			uiRobot = alt_graph.get_vertex_ownership((size_t)iVtx);
			uiOtherRobot = uiRobot1 == uiRobot ? uiRobot2 : uiRobot1;
			insert_LB_non_r1_r2_comp_vtx(iVtx, (size_t)iVtx, uiRobot, uiOtherRobot, alt_graph);
		}
	}
}

//Note iVtx and uiVtx can be different
void Collision_Filtering::insert_LB_non_r1_r2_comp_vtx(int iVtx, size_t uiVtx, size_t uiRobot, size_t uiOtherRobot, const Alternative_Graph &alt_graph)
{
	size_t uiPos, uiVal;
	uiPos = std::numeric_limits<size_t>::min();

	for (auto it_tail = m_in_graph.at(iVtx).begin(); it_tail != m_in_graph.at(iVtx).end(); it_tail++)
	{
		auto pr = get_LB_from_pred(*it_tail, uiRobot, uiOtherRobot, alt_graph);
		if (false == pr.first) continue;
		uiVal = pr.second;
		if (uiPos <= uiVal) uiPos = uiVal;	// Required this way for correctness and to handle redundant precedence constraints
	}

	auto pr = m_map_bounds.at(uiVtx).emplace(uiOtherRobot, std::make_pair(uiPos, std::numeric_limits<size_t>::max()));
	assert(true == pr.second);	
}

std::pair<bool, size_t> Collision_Filtering::get_LB_from_pred(int iVtx, size_t uiRobot, size_t uiOtherRobot, const Alternative_Graph &alt_graph)
{
	if (iVtx >= 0)
	{
		size_t uiRobotOwner = alt_graph.get_vertex_ownership((size_t)iVtx);
		
		if (uiRobotOwner == uiOtherRobot)
		{
			return std::make_pair(true, alt_graph.get_vertex_position((size_t)iVtx));
		}
		else if (uiRobotOwner == uiRobot)
		{
			assert(m_map_bounds.at(iVtx).find(uiOtherRobot) != m_map_bounds.at(iVtx).end());
			return std::make_pair(true, m_map_bounds.at(iVtx).at(uiOtherRobot).first);
		}
		else
			return std::make_pair(false, std::numeric_limits<size_t>::max());	//setting it this way sets an invalid bound
	}
	else
	{
		//indicates it is coming from a strongly connected component vertex
		bool bValidPred = false;
		auto it_comp = m_list_Comp.begin();
		size_t uiComp = (size_t)(-1 * iVtx) - 1, uiPos = std::numeric_limits<size_t>::min(), uiVal;
		std::advance(it_comp, uiComp);

		//uiVal is introduced for both correctness and handling redundant precedence constraints
		for (auto it_vtx = it_comp->begin(); it_vtx != it_comp->end(); it_vtx++)
		{
			size_t uiRobotOwner = alt_graph.get_vertex_ownership(*it_vtx);
			if (uiOtherRobot == uiRobotOwner)
			{
				uiVal = alt_graph.get_vertex_position(*it_vtx);
				if (uiVal >= uiPos) uiPos = uiVal;
				bValidPred = true;
			}
			else if (uiRobot == uiRobotOwner)
			{
				assert(m_map_bounds.at(*it_vtx).end() != m_map_bounds.at(*it_vtx).find(uiOtherRobot));
				uiVal = m_map_bounds.at(*it_vtx).at(uiOtherRobot).first;
				if (uiVal >= uiPos) uiPos = uiVal;
				bValidPred = true;
			}
		}

		if (bValidPred) return std::make_pair(true, uiPos);
		else return std::make_pair(false, std::numeric_limits<size_t>::max());	//setting it this way sets an invalid bound
	}
}

void Collision_Filtering::initialize_lower_bounds(size_t uiRobot1, size_t uiRobot2, const std::vector<std::list<size_t>> &rob_seq)
{
	size_t uiStart1 = *rob_seq[uiRobot1].begin();
	size_t uiStart2 = *rob_seq[uiRobot2].begin();
	m_map_bounds.at(uiStart1).emplace(uiRobot2, std::make_pair(0, std::numeric_limits<size_t>::max()));
	m_map_bounds.at(uiStart2).emplace(uiRobot1, std::make_pair(0, std::numeric_limits<size_t>::max()));
}

void Collision_Filtering::initialize_upper_bounds(size_t uiRobot1, size_t uiRobot2, const std::vector<std::list<size_t>> &rob_seq)
{
	size_t uiEnd1 = *rob_seq[uiRobot1].rbegin();
	size_t uiEnd2 = *rob_seq[uiRobot2].rbegin();
	m_map_bounds.at(uiEnd1).at(uiRobot2).second = rob_seq[uiRobot2].size() - 1;
	m_map_bounds.at(uiEnd2).at(uiRobot1).second = rob_seq[uiRobot1].size() - 1;
}

void Collision_Filtering::Compute_upper_bounds(size_t uiRobot1, size_t uiRobot2, const std::list<int> &rob_list_order, const std::vector<std::list<size_t>> &rob_seq, const Alternative_Graph &alt_graph)
{
	int iVtx;
	size_t uiEnd_Vtx_1 = *rob_seq[uiRobot1].rbegin();
	size_t uiEnd_Vtx_2 = *rob_seq[uiRobot2].rbegin();
	initialize_upper_bounds(uiRobot1, uiRobot2, rob_seq);
	std::unordered_map<size_t, size_t> set_marked_nodes; // vertex , position

	for (auto it = rob_list_order.rbegin(); it != rob_list_order.rend(); it++)
	{
		iVtx = *it;

		if (iVtx < 0)
		{
			size_t uiComp = (size_t)(-1 * iVtx) - 1;
			auto pr1 = containsVertex(uiRobot1, uiComp, alt_graph);
			auto pr2 = containsVertex(uiRobot2, uiComp, alt_graph);
			if ((true == pr1.first) && (true == pr2.first))
			{
				auto it_insert = set_marked_nodes.emplace(pr1.second, alt_graph.get_vertex_position(pr2.second));
				assert(true == it_insert.second);
				it_insert = set_marked_nodes.emplace(pr2.second, alt_graph.get_vertex_position(pr1.second));
				assert(true == it_insert.second);
			}
			if (true == pr1.first) insert_UB_non_r1_r2_comp_vtx(iVtx, pr1.second, uiRobot1, uiRobot2, alt_graph, set_marked_nodes);
			if (true == pr2.first) insert_UB_non_r1_r2_comp_vtx(iVtx, pr2.second, uiRobot2, uiRobot1, alt_graph, set_marked_nodes);			
		}
		else
		{
			size_t uiRobot, uiOtherRobot;
			if ((iVtx == (int)uiEnd_Vtx_1) || (iVtx == (int)uiEnd_Vtx_2)) continue;
			uiRobot = alt_graph.get_vertex_ownership((size_t)iVtx);
			uiOtherRobot = uiRobot1 == uiRobot ? uiRobot2 : uiRobot1;
			insert_UB_non_r1_r2_comp_vtx(iVtx, (size_t)iVtx, uiRobot, uiOtherRobot, alt_graph, set_marked_nodes);
		}
	}
}

// note iVtx can be different from uiVtx in the case when iVtx < 0 (maximal component)
void Collision_Filtering::insert_UB_non_r1_r2_comp_vtx(int iVtx, size_t uiVtx, size_t uiRobot, size_t uiOtherRobot, const Alternative_Graph &alt_graph, std::unordered_map<size_t, size_t> &set_marked_nodes)
{
	size_t uiPos, uiVal;
	uiPos = std::numeric_limits<size_t>::max();

	for (auto it_head = m_out_graph.at(iVtx).begin(); it_head != m_out_graph.at(iVtx).end(); it_head++)
	{
		auto pr = get_UB_from_succ(*it_head, uiRobot, uiOtherRobot, alt_graph, set_marked_nodes);
		if (true == pr.second.first)	// component corresponding to upper bound
		{
			uiVal = pr.second.second;
			if (uiPos >= uiVal) uiPos = uiVal;	// being done this way for correctness and filtering redundant constraints
		}
		if (true == pr.first.first)		// component corresponding to whether a blocking vertex or not
		{
			if (set_marked_nodes.end() == set_marked_nodes.find(uiVtx)) set_marked_nodes.emplace(uiVtx , pr.first.second);
			else if (set_marked_nodes.at(uiVtx) >= pr.first.second) set_marked_nodes.at(uiVtx) = pr.first.second;
		}
	}
	m_map_bounds.at(uiVtx).at(uiOtherRobot).second = uiPos;
}

std::pair<std::pair<bool, size_t>, std::pair<bool,size_t>> Collision_Filtering::get_UB_from_succ(int iVtx, size_t uiRobot, size_t uiOtherRobot, const Alternative_Graph &alt_graph, const std::unordered_map<size_t, size_t> &set_marked_nodes)
{
	if (iVtx >= 0)
	{
		size_t uiRobotOwner = alt_graph.get_vertex_ownership((size_t)iVtx);

		if (uiRobotOwner == uiOtherRobot)
		{
			return std::make_pair( std::make_pair(true, alt_graph.get_vertex_position((size_t)iVtx)), std::make_pair(false, std::numeric_limits<size_t>::min()));
		}
		else if (uiRobotOwner == uiRobot)
		{
			if (set_marked_nodes.find((size_t)iVtx) != set_marked_nodes.end())
			{
				assert(m_map_bounds.at((size_t)iVtx).at(uiOtherRobot).second != std::numeric_limits<size_t>::max());	// simply asserts that this vertex was previously seen
				return std::make_pair(std::make_pair(false, std::numeric_limits<size_t>::min()), std::make_pair(true, std::min(set_marked_nodes.at((size_t)iVtx)-1 , m_map_bounds.at((size_t)iVtx).at(uiOtherRobot).second)));	// -1 is needed since iVtx is a blocker vertex
			}
			else
			{
				assert(m_map_bounds.at((size_t)iVtx).at(uiOtherRobot).second != std::numeric_limits<size_t>::max());
				return std::make_pair(std::make_pair(false, std::numeric_limits<size_t>::min()), std::make_pair(true, m_map_bounds.at((size_t)iVtx).at(uiOtherRobot).second));
			}
		}
		else
			return std::make_pair(std::make_pair(false, std::numeric_limits<size_t>::max()), std::make_pair(false, std::numeric_limits<size_t>::max()));	//setting it this way sets an invalid bound
	}
	else
	{
		bool bMark = false, bValidSucc = false;
		auto it_comp = m_list_Comp.begin();
		size_t uiComp = (size_t)(-1 * iVtx) - 1, uiPos = std::numeric_limits<size_t>::min(), uiVal = std::numeric_limits<size_t>::max();
		std::advance(it_comp, uiComp);

		for (auto it_vtx = it_comp->cbegin(); it_vtx != it_comp->cend(); it_vtx++)
		{
			size_t uiRobotOwner = alt_graph.get_vertex_ownership(*it_vtx);
			if (uiOtherRobot == uiRobotOwner)
			{
				bMark = true;
				uiVal = alt_graph.get_vertex_position(*it_vtx);
			}
			else if (uiRobot == uiRobotOwner)
			{
				if (set_marked_nodes.find(*it_vtx) != set_marked_nodes.end())
				{
					assert(std::numeric_limits<size_t>::max() != m_map_bounds.at(*it_vtx).at(uiOtherRobot).second);
					uiPos = std::min(set_marked_nodes.at(*it_vtx) - 1 , m_map_bounds.at(*it_vtx).at(uiOtherRobot).second);	// this way because *it_Vtx is a blocker vertex				
				}
				else
				{
					assert(std::numeric_limits<size_t>::max() != m_map_bounds.at(*it_vtx).at(uiOtherRobot).second);
					uiPos = m_map_bounds.at(*it_vtx).at(uiOtherRobot).second;					
				}
				bValidSucc = true;
			}
		}
		return std::make_pair(std::make_pair(bMark, uiVal), std::make_pair(bValidSucc, uiPos));
	}
}

void Collision_Filtering::compute_robot_pair_list_order(size_t uiRobot1, size_t uiRobot2, std::list<int> &rob_pair_list_order, const Alternative_Graph &alt_graph)
{
	size_t uiRobot , uiComp;
	int iVtx;
	for(auto it = m_list_order.begin(); it != m_list_order.end(); it++)
	{
		iVtx = *it;
		if (iVtx < 0)
		{
			uiComp = (size_t)(-1 * iVtx) - 1;
			auto res1 = containsVertex(uiRobot1, uiComp, alt_graph);
			auto res2 = containsVertex(uiRobot2, uiComp, alt_graph);

			if ( (res1.first == true) || (res2.first == true))
				rob_pair_list_order.push_back(iVtx);			
		}
		else
		{
			uiRobot = alt_graph.get_vertex_ownership((size_t) iVtx);
			if ((uiRobot == uiRobot1) || (uiRobot == uiRobot2))
				rob_pair_list_order.push_back(iVtx);
		}		
	}
}

std::pair<bool, size_t> Collision_Filtering::containsVertex(size_t uiRobot, size_t uiComp, const Alternative_Graph &alt_graph)
{
	auto it_comp = m_list_Comp.begin();
	std::advance(it_comp, uiComp);
	return containsVertex(uiRobot , alt_graph, *it_comp);
}

std::pair<bool, size_t> Collision_Filtering::containsVertex(size_t uiRobot, const Alternative_Graph &alt_graph, const std::unordered_set<size_t> &comp)
{
	for (auto it = comp.begin(); it != comp.end(); it++)
	{
		if (alt_graph.get_vertex_ownership(*it) == uiRobot)
		{
			return std::make_pair(true, *it);
		}
	}
	return std::make_pair(false, std::numeric_limits<int>::min());	// returns an invalid vertex number
}

bool Collision_Filtering::check_bounds_validity(const std::vector<std::list<size_t>> &rob_seq)
{
	assert(false == m_map_bounds.empty());
	bool bFeasible;
	for (size_t uiRobot = 0; uiRobot < rob_seq.size(); uiRobot++)
	{
		bFeasible = check_bounds_validity(uiRobot, rob_seq);
		if (false == bFeasible) return false;
	}
	return true;
}

bool Collision_Filtering::check_bounds_validity(size_t uiRobot, const std::vector<std::list<size_t>> &rob_seq)
{
	for (size_t uiOtherRobot = 0; uiOtherRobot < rob_seq.size(); uiOtherRobot++)
	{
		if (uiOtherRobot == uiRobot) continue;
		for (auto it = rob_seq[uiRobot].cbegin(); it != rob_seq[uiRobot].cend(); it++)
		{
			auto it_bound = m_map_bounds.at(*it).find(uiOtherRobot);
			if (it_bound->second.first > it_bound->second.second) return false;
		}
	}
	return true;
}

std::pair<size_t , size_t> Collision_Filtering::get_bounds(size_t uiVtx, size_t uiOtherRobot)
{
	auto it_find = m_map_bounds.at(uiVtx).find(uiOtherRobot);
	assert(m_map_bounds.at(uiVtx).end() != it_find);
	return it_find->second;
}

bool Collision_Filtering::add_scc_comps(Alternative_Graph &alt_graph, std::list<arc> &list_prec_arcs_betw_jobs) const
{
	bool bAdded = false;
	for (auto it_comp = m_list_Comp.begin(); it_comp != m_list_Comp.end(); it_comp++)
	{
		for (auto it_vtx1 = it_comp->begin(); it_vtx1 != it_comp->end(); it_vtx1++)
		{
			auto it_vtx2 = it_vtx1;
			it_vtx2++;
			for (; it_vtx2 != it_comp->end(); it_vtx2++)
			{
				if (false == alt_graph.containsPrecArc(arc(*it_vtx1, *it_vtx2)))
				{
					alt_graph.add_prec_arc(*it_vtx1, *it_vtx2, 0);
					list_prec_arcs_betw_jobs.emplace_back(arc(*it_vtx1, *it_vtx2));
					bAdded = true;
				}
				if (false == alt_graph.containsPrecArc(arc(*it_vtx2, *it_vtx1)))
				{
					alt_graph.add_prec_arc(*it_vtx2, *it_vtx1, 0);
					list_prec_arcs_betw_jobs.emplace_back(arc(*it_vtx2, *it_vtx1));
					bAdded = true;
				}
			}
		}
	}
	return bAdded;
}