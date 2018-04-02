#include "Collision_Filtering.h"
#include <assert.h>
#include <algorithm>
#include <vector>
#include "Windows_Linux.h"
#include "Greedy_Heuristic_Utils.h"

void Collision_Filtering::clear_prev_info()
{
	m_map_lower_bounds.clear();
	m_map_bounds.clear();		
}

void Collision_Filtering::Initialize_lower_bounds_map(const std::vector<std::list<size_t>> &rob_seq)
{
	for (size_t uiRobot1 = 0; uiRobot1 < rob_seq.size(); uiRobot1++)
	{
		for (auto it = rob_seq[uiRobot1].begin(); it != rob_seq[uiRobot1].end(); it++)
		{
			m_map_lower_bounds.emplace(*it, std::vector<size_t>());
		}

		for (size_t uiRobot2 = 0; uiRobot2 < rob_seq.size(); uiRobot2++)
		{
			m_map_lower_bounds[*rob_seq[uiRobot1].begin()].push_back(0);
		}
	}
}

// Also add redundant strongly connected component arcs
bool Collision_Filtering::Check_Feasibility_Compute_Bounds_For_Each_Vertex(const std::vector<std::list<size_t>> &rob_seq, const Alternative_Graph &alt_graph)
{
	clear_prev_info();
	bool bFeasible = construct_graph_populate_order(alt_graph, rob_seq.size());
	if (false == bFeasible) return false;

	/*
	Kosaraju_Algo obj;
	obj.compute_maximal_components(alt_graph.getGraph(), alt_graph.getReverseGraph(), m_list_Super_Comp);
	bool bFeasible = !(Check_Pos_Loop_Remove_1comp(alt_graph , rob_seq.size()));
	if (false == bFeasible) return false;
	
	construct_in_out_graphs(alt_graph);
	Topological_sort_out_graph();	
	*/

	Compute_lower_bounds(alt_graph, rob_seq);
	
	/*
	Compute_bounds(alt_graph, rob_seq);
	for (size_t uiRobot = 0; uiRobot < rob_seq.size(); uiRobot++)
	{
		for (auto it = rob_seq[uiRobot].begin(); it != rob_seq[uiRobot].end(); it++)
		{
			for (size_t uiRobot1 = 0; uiRobot1 < rob_seq.size(); uiRobot1++)
			{
				if (uiRobot1 == uiRobot) continue;
				if (m_map_lower_bounds[*it][uiRobot1] != m_map_bounds.at(*it).at(uiRobot1).first)
				{
					cout << "Mistake detected \n\n";
					print_sequence(rob_seq);
					exit(-1);
				}
			}
		}
	}	
	//bFeasible = check_bounds_validity(rob_seq);
	//if (false == bFeasible) return false;
	*/

	return true;
}

void Collision_Filtering::Compute_lower_bounds(const Alternative_Graph &alt_graph, const std::vector<std::list<size_t>> &rob_seq)
{
	const size_t c_uiNumRobots = rob_seq.size();
	Initialize_lower_bounds_map(rob_seq);
	int iVtx;

	for (auto it = m_list_order.begin(); it != m_list_order.end(); it++)
	{
		iVtx = *it;
		compute_lower_bound_for_component(alt_graph, iVtx, rob_seq.size());
	}
}

void Collision_Filtering::compute_lower_bound_for_component(const Alternative_Graph &alt_graph, int iVtx, const size_t c_uiNumRobots)
{
	std::vector<size_t> vec_comp_vtx;  // contains all the vertices that are (potentially) super vertex in the DAG
	std::vector<size_t> vec_comp_robots; // contains the corresponding robots that own the vertex in each (potentially) super vertex
	std::vector<size_t> vec_vtx_pos; // positions that need to be computed

	if (iVtx >= 0)
	{
		vec_comp_vtx.emplace_back(iVtx);
		vec_comp_robots.emplace_back(alt_graph.get_vertex_ownership(iVtx));
	}
	else
	{
		auto it_comp = m_list_Super_Comp.begin();
		size_t uiComp = (size_t)(-1 * iVtx) - 1;
		std::advance(it_comp, uiComp);

		for (auto it_vtx = it_comp->begin(); it_vtx != it_comp->end(); it_vtx++)
		{
			vec_comp_vtx.emplace_back(*it_vtx);
			vec_comp_robots.emplace_back(alt_graph.get_vertex_ownership(*it_vtx));
		}
	}

	size_t uiPredVtx;
	
	for (size_t uiRobot = 0; uiRobot < c_uiNumRobots; uiRobot++)
	{
		vec_vtx_pos.push_back(std::numeric_limits<size_t>::min());
	}

	for (auto it_tail = m_in_graph.at(iVtx).begin(); it_tail != m_in_graph.at(iVtx).end(); it_tail++)
	{
		if (it_tail->first < 0)
		{
			auto it_comp = m_list_Super_Comp.begin();
			size_t uiComp = (size_t)(-1 * (it_tail->first)) - 1;
			std::advance(it_comp, uiComp);
			uiPredVtx = *(it_comp->begin()); // this is sufficient because pretty much all the bounds will be the same for all synch vertices
		}
		else uiPredVtx = it_tail->first;

		for (size_t uiRobot = 0; uiRobot < c_uiNumRobots; uiRobot++)
		{
			vec_vtx_pos[uiRobot] = std::max(vec_vtx_pos[uiRobot], m_map_lower_bounds.at(uiPredVtx)[uiRobot]);
		}
	}	

	// populate the values for the (potentially) super vertex 

	size_t uiCount = 0;
	for (auto it_vtx = vec_comp_vtx.begin(); it_vtx != vec_comp_vtx.end(); it_vtx++, uiCount++)
	{
		for (size_t uiRobot = 0; uiRobot < c_uiNumRobots; uiRobot++)
		{
			if(vec_comp_robots[uiCount] == uiRobot) m_map_lower_bounds.at(*it_vtx).push_back(alt_graph.get_vertex_position(*it_vtx));
			else m_map_lower_bounds.at(*it_vtx).push_back(vec_vtx_pos[uiRobot]);
		}
	}
	
	if (vec_comp_vtx.size() > 1)
	{
		size_t uiCount1 = 0;
		for (auto it_vtx1 = vec_comp_vtx.begin(); it_vtx1 != vec_comp_vtx.end(); it_vtx1++, uiCount1++)
		{
			auto it_vtx2 = it_vtx1;
			it_vtx2++;
			
			size_t uiCount2 = uiCount1 + 1;

			for ( ; it_vtx2 != vec_comp_vtx.end(); it_vtx2++ , uiCount2++)
			{
				m_map_lower_bounds.at(*it_vtx1)[vec_comp_robots[uiCount2]] = m_map_lower_bounds.at(*it_vtx2)[vec_comp_robots[uiCount2]];
				m_map_lower_bounds.at(*it_vtx2)[vec_comp_robots[uiCount1]] = m_map_lower_bounds.at(*it_vtx1)[vec_comp_robots[uiCount1]];
			}
		}
	}
}

size_t Collision_Filtering::get_lower_bound_pos(size_t uiVtx, size_t uiOtherRobot) const
{
	auto it_find = m_map_lower_bounds.find(uiVtx);
	assert(m_map_lower_bounds.end() != it_find);
	return it_find->second[uiOtherRobot];
}

size_t Collision_Filtering::Compute_costs_for_each_Vertex(const std::vector<std::list<size_t>> &rob_seq, const Alternative_Graph &alt_graph, std::vector<std::vector<size_t>> &vec_cost_from_source, std::vector<std::vector<size_t>> &vec_cost_to_go)
{
	size_t uiFromMakeSpan, uiGoMakeSpan;
	if (0 == m_list_order.size())
	{
		cout << "Topological order was not previously computed \n";
		exit(-1);
	}

#ifdef WINDOWS
	assert(0 == vec_cost_from_source.size());
	assert(0 == vec_cost_to_go.size());
#else
	if ((0 != vec_cost_from_source.size()) || (0 != vec_cost_to_go.size()))
	{
		cout << "Wrongly placed cost based filtering\n";
		exit(-1);
	}
#endif

	vec_cost_from_source.resize(rob_seq.size());
	vec_cost_to_go.resize(rob_seq.size());

	for (size_t uiRobot = 0; uiRobot < rob_seq.size(); uiRobot++)
	{
		vec_cost_from_source[uiRobot].resize(rob_seq[uiRobot].size(), 0);
		vec_cost_to_go[uiRobot].resize(rob_seq[uiRobot].size(), 0);
	}	

	uiFromMakeSpan = Compute_FROM_costs_each_Vertex(alt_graph, vec_cost_from_source);
	uiGoMakeSpan = Compute_GO_costs_each_Vertex(alt_graph, vec_cost_to_go);

#ifdef WINDOWS
	assert(uiFromMakeSpan == uiGoMakeSpan);	
#else
	if (uiFromMakeSpan != uiGoMakeSpan)
	{
		cout << "From and To Makespans do not match\n";
		exit(-1);
	}
#endif

	return uiFromMakeSpan;
}

void Update_cost(size_t uiCost, int iVtx, std::vector<std::vector<size_t>> &vec_cost, const Alternative_Graph &alt_graph, const std::list<std::unordered_set<size_t>> &list_Comp)
{
	size_t uiRobot, uiPos;
	if (iVtx < 0)
	{
		auto it_comp = list_Comp.begin();
		size_t uiComp = (size_t)(-1 * iVtx) - 1;
		std::advance(it_comp, uiComp);

		//uiVal is introduced for both correctness and handling redundant precedence constraints
		for (auto it_vtx = it_comp->begin(); it_vtx != it_comp->end(); it_vtx++)
		{
			uiRobot = alt_graph.get_vertex_ownership(*it_vtx);
			uiPos = alt_graph.get_vertex_position(*it_vtx);
			
			assert(0 == vec_cost[uiRobot][uiPos]);			
			vec_cost[uiRobot][uiPos] = uiCost;			
		}
	}
	else
	{
		uiRobot = alt_graph.get_vertex_ownership((size_t) iVtx);
		uiPos = alt_graph.get_vertex_position((size_t)iVtx);
		assert(0 == vec_cost[uiRobot][uiPos]);
		vec_cost[uiRobot][uiPos] = uiCost;
	}	
}

size_t get_any_vertex_from_scc(int iVtx, const std::list<std::unordered_set<size_t>> &list_Comp)
{
#ifdef WINDOWS
	assert(iVtx < 0);
#else
	if (iVtx >= 0)
	{
		cout << "Incorrect usage of vertices from scc \n";
		exit(-1);
	}
#endif

	auto it_comp = list_Comp.begin();
	size_t uiComp = (size_t)(-1 * iVtx) - 1;
	std::advance(it_comp, uiComp);

#ifdef WINDOWS
	assert(it_comp->size() > 1);
#else
	if (it_comp->size() <= 1)
	{
		cout << "SCC were not correctly filtered earlier \n";
		exit(-1);
	}
#endif
	return *(it_comp->begin());
}

size_t Collision_Filtering::Compute_FROM_costs_each_Vertex(const Alternative_Graph &alt_graph, std::vector<std::vector<size_t>> &vec_cost_from_source)
{
	int iVtx, iPrev; 
	size_t uiPrev, uiPrevPos, uiRobot, uiCost;
	size_t uiMakeSpan = std::numeric_limits<size_t>::min();

	for (auto it_curr = m_list_order.begin(); it_curr != m_list_order.end(); it_curr++)
	{
		iVtx = *it_curr;
		uiCost = std::numeric_limits<size_t>::min();

		if (0 == m_in_graph.at(iVtx).size())
		{
			Update_cost(0, iVtx, vec_cost_from_source, alt_graph, m_list_Super_Comp);
			continue;
		}

		for (auto it_prev = m_in_graph.at(iVtx).begin(); it_prev != m_in_graph.at(iVtx).end(); it_prev++)
		{
			iPrev = it_prev->first;
			
			if(iPrev < 0) uiPrev = get_any_vertex_from_scc(iPrev, m_list_Super_Comp);  // notice that uiPrev and iPrev can be different
			else uiPrev = (size_t)iPrev;
			
			uiRobot = alt_graph.get_vertex_ownership(uiPrev);
			uiPrevPos = alt_graph.get_vertex_position(uiPrev);

			uiCost = std::max(uiCost , vec_cost_from_source[uiRobot][uiPrevPos] + it_prev->second);
		}

		Update_cost(uiCost, iVtx, vec_cost_from_source, alt_graph, m_list_Super_Comp);
		uiMakeSpan = std::max(uiMakeSpan, uiCost);
	}
	return uiMakeSpan;
}

size_t Collision_Filtering::Compute_GO_costs_each_Vertex(const Alternative_Graph &alt_graph, std::vector<std::vector<size_t>> &vec_cost_to_go)
{
	int iVtx, iNext;
	size_t uiNext, uiNextPos, uiRobot, uiCost;
	size_t uiMakeSpan = std::numeric_limits<size_t>::min();

	for (auto it_curr = m_list_order.rbegin(); it_curr != m_list_order.rend(); it_curr++)
	{
		iVtx = *it_curr;
		uiCost = std::numeric_limits<size_t>::min();

		if (0 == m_out_graph.at(iVtx).size())
		{
			Update_cost(0, iVtx, vec_cost_to_go, alt_graph, m_list_Super_Comp);
			continue;
		}

		for (auto it_next = m_out_graph.at(iVtx).begin(); it_next != m_out_graph.at(iVtx).end(); it_next++)
		{
			iNext = it_next->first;

			if(iNext < 0) uiNext = get_any_vertex_from_scc(iNext, m_list_Super_Comp);  // notice that uiPrev and iPrev can be different
			else uiNext = (size_t)iNext;

			uiRobot = alt_graph.get_vertex_ownership(uiNext);
			uiNextPos = alt_graph.get_vertex_position(uiNext);

			uiCost = std::max(uiCost, vec_cost_to_go[uiRobot][uiNextPos] + it_next->second);
		}
		Update_cost(uiCost, iVtx, vec_cost_to_go, alt_graph, m_list_Super_Comp);
		uiMakeSpan = std::max(uiMakeSpan, uiCost);
	}
	return uiMakeSpan;
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
		auto pr = get_LB_from_pred(it_tail->first, uiRobot, uiOtherRobot, alt_graph);
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
		auto it_comp = m_list_Super_Comp.begin();
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
		auto pr = get_UB_from_succ(it_head->first, uiRobot, uiOtherRobot, alt_graph, set_marked_nodes);
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
		auto it_comp = m_list_Super_Comp.begin();
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
	auto it_comp = m_list_Super_Comp.begin();
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
	for (auto it_comp = m_list_Super_Comp.begin(); it_comp != m_list_Super_Comp.end(); it_comp++)
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