#include "Greedy_Heuristic.h"
#include "Sequence_Visualizer.h"

void print_sequence(const std::vector<std::list<size_t>> &rob_seq)
{
	for (size_t uiRobot = 0; uiRobot < rob_seq.size(); uiRobot++)
	{
		for (auto it = rob_seq[uiRobot].begin(); it != rob_seq[uiRobot].end(); it++)
		{
			cout << *it << ",";
		}
		cout << endl;
	}

	cout << "\n\n";
}

bool greedy_heuristic(const std::pair<Comparison_Object, State>& lhs, const std::pair<Comparison_Object, State>& rhs)
{
	//early dispatch
	if (lhs.first.uiDispatchTime < rhs.first.uiDispatchTime) return true;
	else if (lhs.first.uiDispatchTime > rhs.first.uiDispatchTime) return false;

	//minimize tardiness
	if (lhs.first.uiMaxDelay < rhs.first.uiMaxDelay) return true;
	else if (lhs.first.uiMaxDelay > rhs.first.uiMaxDelay) return false;

	//minimize expected makespan
	if (lhs.first.uiExpMakeSpan < rhs.first.uiExpMakeSpan) return true;
	else if (lhs.first.uiExpMakeSpan > rhs.first.uiExpMakeSpan) return false;

	//maximize number of dispatched robots
	if (lhs.first.uiCompSize > rhs.first.uiCompSize) return true;
	else if (lhs.first.uiCompSize < rhs.first.uiCompSize) return false;

	boost::function<size_t(const State &x)> f;
	f = StateHasher{};
	return f(lhs.second) < f(rhs.second);
}

Greedy_Heuristic::Greedy_Heuristic(const size_t uiRobotNum, const Layout_LS &graph, Power_Set &power) : m_uiNumRobots(uiRobotNum), m_graph(graph), m_power(power)
{
	m_rob_hole_times.resize(uiRobotNum);
	m_set_prev_HD_states.resize(uiRobotNum);
	m_set_prev_all_states.resize(uiRobotNum);
	m_vec_nc_eft.resize(uiRobotNum);
}

bool Greedy_Heuristic::perform_initializations(const std::vector<std::list<size_t>> &rob_seq)
{
	bool bFeasible = construct_Alt_Graph_STN(rob_seq);
	if (false == bFeasible) return false;
	allocate_buffers(rob_seq);		
	initialize_to_do_verts(rob_seq);
	compute_NC_makespan(rob_seq);
	return true;
}

void Greedy_Heuristic::clear_prev_info_buffers()
{
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		m_rob_hole_times[uiRobot].clear();
		m_set_prev_HD_states[uiRobot].clear();
		m_set_prev_all_states[uiRobot].clear();	
		m_vec_nc_eft[uiRobot].m_map_eft.clear();
		m_vec_nc_eft[uiRobot].m_uiNC_Makespan = std::numeric_limits<size_t>::max();
	}

	m_map_states_feas.clear();
	m_map_self_enabling.clear();
	m_alt_graph.clear_prev_info();
	m_vec_map_new_sel_alt_arcs.clear();
	assert(0 == m_set_to_do_verts.size());
	m_set_to_do_verts.clear();	
}

// populates buffers for storing timing information
void Greedy_Heuristic::allocate_buffers(const std::vector<std::list<size_t>> &rob_seq)
{
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		for (auto it = rob_seq[uiRobot].begin(); it != rob_seq[uiRobot].end(); it++)
		{
			m_rob_hole_times[uiRobot].emplace(*it, ST_Time::UNSET);
		}		
	}
}

void Greedy_Heuristic::initialize_to_do_verts(const std::vector<std::list<size_t>> &rob_seq)
{
	size_t uiVerts = 0;
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		for (auto it = rob_seq[uiRobot].begin(); it != rob_seq[uiRobot].end(); it++)
		{
			auto it_insert = m_set_to_do_verts.emplace(*it);
			assert(true == it_insert.second);
		}
	}
}

// NC -: No - Collision
void Greedy_Heuristic::compute_NC_makespan(const std::vector<std::list<size_t>> &rob_seq)
{
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		size_t uiMakespan = 0;
		for (auto it  = rob_seq[uiRobot].begin() ; it != rob_seq[uiRobot].end() ; it++)
		{
			uiMakespan += m_graph.getTime(*it);
			m_vec_nc_eft[uiRobot].m_map_eft.emplace(*it, uiMakespan);
		}
		m_vec_nc_eft[uiRobot].m_uiNC_Makespan = uiMakespan;
	}
}

void Greedy_Heuristic::populate_root_node_info(State &root, const std::vector<std::list<size_t>> &rob_seq)
{
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		root.m_vec_rob_pos[uiRobot] = rob_seq[uiRobot].begin();
	}	
}

int Greedy_Heuristic::compute_greedy_sol(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, std::string strFolder)
{
#ifdef WINDOWS
	print_sequence(rob_seq);
#endif
	//print_sequence(rob_seq);
		
	Sequence_Visualization obj_vis;
	clear_prev_info_buffers();
	
	bool bFeasible = perform_initializations(rob_seq);
	if (false == bFeasible) 
	{ 
#ifdef PLOT_INFEASIBLE_CASES
		print_sequence(rob_seq);
		obj_vis.plot_alternative_graph(strFolder , m_alt_graph, m_map_states_feas);
#endif
		return -1; 
	}

	State root(m_uiNumRobots);
	populate_root_node_info(root , rob_seq);
	int iRetVal = compute_DFS(root, 0, 0);
	if (1 == iRetVal) { vectorize_schedule(rob_seq , vec_rob_sch); }
	else { m_set_to_do_verts.clear(); }

	if (-1 == iRetVal)
	{
#ifdef PLOT_INFEASIBLE_CASES
		obj_vis.plot_alternative_graph(strFolder, m_alt_graph, m_map_states_feas);
		print_sequence(rob_seq);
#endif
	}
		
	return iRetVal;
}

int Greedy_Heuristic::compute_DFS(const State& state , size_t uiDepth, size_t uiStartTime)
{
	//print_state(uiDepth , uiStartTime, state);
	bool bSeen = wasStatePreviouslySeen(state);
	if (true == bSeen) return -1;

	safe_initialize(uiStartTime, uiDepth, state);

	int iRetVal = make_selection_pos_and_check_if_feasible(uiDepth, state);
	if (-1 == iRetVal)
	{
		safe_backtrack(uiDepth, state);
		return -1;
	}

	iRetVal = check_if_final_state(state);
	if (1 == iRetVal) return 1;
	
	//expected makespan, max delay, num_robots_schedule, next time, state
	std::vector<std::pair<Comparison_Object, State>> vec_children;
	compute_succ_nodes(uiStartTime, state, vec_children);
	if(0 == vec_children.size())
	{
		safe_backtrack(uiDepth, state);
		return -1;
	}

	size_t uiInFeasChildCount = 0;
	for (size_t uiCount = 0; uiCount < vec_children.size(); uiCount++)
	{
		iRetVal = compute_DFS(vec_children[uiCount].second, uiDepth + 1, vec_children[uiCount].first.uiDispatchTime);
		if (1 == iRetVal) return iRetVal;
		assert(-1 == iRetVal);
		uiInFeasChildCount++;
	}

	assert(vec_children.size() == uiInFeasChildCount);
	safe_backtrack(uiDepth, state);
	return -1;
}

bool Greedy_Heuristic::wasStatePreviouslySeen(const State &state)
{
	auto it = m_map_states_feas.find(state);
	if (it != m_map_states_feas.end())
	{
		assert(-1 == it->second);
		return true;
	}
	return false;
}

//checks if last element is depot of that robot
int Greedy_Heuristic::check_if_final_state(const State& state)
{
	const auto &vec_depo = m_graph.getDepotMap();
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		if (*state.m_vec_rob_pos[uiRobot] != vec_depo.at(uiRobot).getToInd())
			return 0;
	}
	return 1;
}

int Greedy_Heuristic::make_selection_pos_and_check_if_feasible(size_t uiDepth, const State& state)
{
	m_map_states_feas.emplace(state, 1);

	bool bFeasible = make_state_positional_sel(uiDepth, state);
	if (false == bFeasible) return -1;
	
	int iRetVal = check_if_coll_feasible(state);
	assert(-1 != iRetVal);
	if (-1 == iRetVal) return -1;
	
	iRetVal = check_if_enabling_feasible(state);
	
	// this condition because, for more robots enabling constraints are not necessarily precedence constraints
	if (2 == m_uiNumRobots)
	{ 
#ifdef WINDOWS
		assert(-1 != iRetVal); 
#else
		if (-1 == iRetVal)
		{
			cout << "Failed enabling constraints\n";
			exit(1);
		}
#endif
	}
	return iRetVal;
}

int Greedy_Heuristic::check_if_coll_feasible(const State& state)
{
	for (size_t uiRobot1 = 0; uiRobot1 < m_uiNumRobots; uiRobot1++)
	{
		N_Ind Ind1(*state.m_vec_rob_pos[uiRobot1]);
		for (size_t uiRobot2 = uiRobot1 + 1; uiRobot2 < m_uiNumRobots; uiRobot2++)
		{
			N_Ind Ind2(*state.m_vec_rob_pos[uiRobot2]);
			if (m_graph.areColliding(Coll_Pair(Ind1, uiRobot1, Ind2, uiRobot2)))
				return -1;
		}
	}
	return 0;
}

int Greedy_Heuristic::check_if_enabling_feasible(const State& state)
{
	bool bEnabled;
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		bEnabled = check_if_self_enabling(uiRobot, state);
		if (bEnabled) continue;
		bEnabled = check_if_other_enabling(uiRobot, state);		
		if (!bEnabled) return -1;
	}
	return 0;
}

bool Greedy_Heuristic::check_if_self_enabling(size_t uiRobot, const State& state)
{
	N_Ind Ind(*state.m_vec_rob_pos[uiRobot]);

	if ("H" != m_graph.getType(Ind)) return true;

	auto it_self = m_map_self_enabling.find(Ind);
	if (m_map_self_enabling.find(Ind) != m_map_self_enabling.end())
	{
		return it_self->second;
	}

	const auto &vec_Enablers = m_graph.get_Enablers();
	if (vec_Enablers.at(Ind.getInd()).set.size() > m_set_prev_HD_states[uiRobot].size())
	{
		for (auto it = m_set_prev_HD_states[uiRobot].begin(); it != m_set_prev_HD_states[uiRobot].end(); it++)
		{
			if (vec_Enablers.at(Ind.getInd()).set.find(it->first) != vec_Enablers.at(Ind.getInd()).set.end())
			{
				m_map_self_enabling.emplace(Ind, true);
				return true;
			}
		}
	}
	else
	{
		for (auto it = vec_Enablers.at(Ind.getInd()).set.begin(); it != vec_Enablers.at(Ind.getInd()).set.end(); it++)
		{
			if (m_set_prev_HD_states[uiRobot].find(*it) != m_set_prev_HD_states[uiRobot].end())
			{
				m_map_self_enabling.emplace(Ind, true);
				return true;
			}
		}
	}

	m_map_self_enabling.emplace(Ind, false);
	return false;
}

bool Greedy_Heuristic::check_if_other_enabling(size_t uiRobot, const State& state)
{
	N_Ind Ind(*state.m_vec_rob_pos[uiRobot]);
	const auto &vec_Enablers = m_graph.get_Enablers();
	
	for (size_t uiOtherRobot = 0; uiOtherRobot < m_uiNumRobots; uiOtherRobot++)
	{
		if (uiOtherRobot == uiRobot) continue;
		if (vec_Enablers.size() < m_set_prev_HD_states[uiOtherRobot].size())
		{
			for (auto it = vec_Enablers.at(Ind.getInd()).set.begin(); it != vec_Enablers.at(Ind.getInd()).set.end(); it++)
			{
				if (m_set_prev_HD_states[uiOtherRobot].find(*it) != m_set_prev_HD_states[uiOtherRobot].end())
					return true;
			}
		}
		else
		{
			for (auto it = m_set_prev_HD_states[uiOtherRobot].begin(); it != m_set_prev_HD_states[uiOtherRobot].end(); it++)
			{
				if (vec_Enablers.at(Ind.getInd()).set.find(it->first) != vec_Enablers.at(Ind.getInd()).set.end())
					return true;
			}
		}
	}
	return false;
}

// minimize expected makespan
std::pair<size_t, size_t> Greedy_Heuristic::compute_exp_Mkspn_delay(const size_t uiCurrTime, const State& state)
{
	size_t maxMakespan = std::numeric_limits<size_t>::min(), uiMaxDelay = std::numeric_limits<size_t>::min();
	size_t ui_comp_Time, uiDelay, uiVal;
	
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		auto it = state.m_vec_rob_pos[uiRobot];
		assert(m_rob_hole_times[uiRobot].at(*it).m_uiStartTime != ST_Time::UNSET);
		ui_comp_Time = m_rob_hole_times[uiRobot].at(*it).m_uiStartTime + m_graph.getTime(*it);
		if (uiCurrTime >= ui_comp_Time)
		{
			uiDelay = uiCurrTime - m_vec_nc_eft[uiRobot].m_map_eft.at(*it);
			maxMakespan = std::max(m_vec_nc_eft[uiRobot].m_uiNC_Makespan + uiDelay, maxMakespan);			
		}
		else
		{
			const auto &vec_depo = m_graph.getDepotMap();
			if ( *it == vec_depo.at(uiRobot).getFromInd())
			{
				maxMakespan = std::max(m_vec_nc_eft[uiRobot].m_uiNC_Makespan , maxMakespan);
				uiDelay = 0;
			}
			else
			{
				uiVal = m_rob_hole_times[uiRobot].at(*it).m_uiStartTime;
				it--;
				uiDelay = uiVal - m_vec_nc_eft[uiRobot].m_map_eft.at(*it);
				maxMakespan = std::max(m_vec_nc_eft[uiRobot].m_uiNC_Makespan + uiDelay, maxMakespan);
			}			
		}
		uiMaxDelay = std::max(uiMaxDelay , uiDelay);
	}
	return std::make_pair(maxMakespan, uiMaxDelay);
}

const std::vector<std::vector<size_t>>& Greedy_Heuristic::get_child_states_iter_inc(const State& state)
{
	std::vector<size_t> set_robots;
	get_updatable_robots(state, set_robots);
	return m_power.get_power_set(set_robots);
}

void Greedy_Heuristic::get_updatable_robots(const State& state, std::vector<size_t> &set_robots)
{
	const auto &vec_depots = m_graph.getDepotMap();
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		if (*state.m_vec_rob_pos[uiRobot] != vec_depots.at(uiRobot).getToInd())
			set_robots.push_back(uiRobot);
	}
}

bool Greedy_Heuristic::make_state_positional_sel(size_t uiDepth , const State& state)
{
	std::unordered_set<size_t> B_P;	// note although different from R this is still correct
	state.get_vertices(B_P);
	bool bFeasible = m_alt_graph.get_arcs_to_make_sel_positional(B_P, m_vec_map_new_sel_alt_arcs[uiDepth]);
	if (false == bFeasible)
	{
		return false;
	}
	m_alt_graph.make_selection_positional(m_vec_map_new_sel_alt_arcs[uiDepth]);
	return true;
}

void Greedy_Heuristic::allocate_positional_arc_buffer(size_t uiDepth)
{
	assert(uiDepth == m_vec_map_new_sel_alt_arcs.size());
	m_vec_map_new_sel_alt_arcs.emplace_back(std::unordered_map<size_t, std::pair<std::pair<arc, std::string>, std::pair<arc, std::string>>>());
}

void Greedy_Heuristic::deallocate_positional_arc_buffer(size_t uiDepth)
{
	assert(uiDepth + 1 == m_vec_map_new_sel_alt_arcs.size());
	m_alt_graph.unselect_positional_arcs(m_vec_map_new_sel_alt_arcs[uiDepth]);
	m_vec_map_new_sel_alt_arcs.pop_back();
}

void Greedy_Heuristic::update_vis_unvis_states_schedule(size_t uiCurrTime, size_t uiDepth, const State& state)
{
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		update_visited_all_states_schedule(uiRobot, uiCurrTime, uiDepth, state);
	}	
}

void Greedy_Heuristic::update_visited_all_states_schedule(size_t uiRobot, size_t uiCurrTime, size_t uiDepth, const State& state)
{
	auto it = state.m_vec_rob_pos[uiRobot];
	N_Ind Ind(*it);
	if (m_set_prev_all_states[uiRobot].find(Ind) == m_set_prev_all_states[uiRobot].end())
	{
		auto it_insert_Prev = m_set_prev_all_states[uiRobot].emplace(Ind, uiDepth);
		assert(true == it_insert_Prev.second);

		size_t uiErase = m_set_to_do_verts.erase(*it);
		assert(1 == uiErase);
		
		assert(m_rob_hole_times[uiRobot].at(Ind.getInd()).m_uiStartTime == ST_Time::UNSET);
		m_rob_hole_times[uiRobot].at(Ind.getInd()).m_uiStartTime = uiCurrTime;		

		const auto &vec_depo = m_graph.getDepotMap();
		if (*it == vec_depo.at(uiRobot).getFromInd())
		{
			return;
		}
		it--;		
		auto it_insert_HD_states = m_set_prev_HD_states[uiRobot].emplace(*it, uiDepth);
		assert(true == it_insert_HD_states.second);		
	}
}

void Greedy_Heuristic::remove_vis_unvis_state_and_schedule(size_t uiDepth, const State& state)
{
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		remove_visited_all_states_schedule(uiRobot, uiDepth, state);		
	}
}

void Greedy_Heuristic::remove_visited_all_states_schedule(size_t uiRobot, size_t uiDepth, const State& state)
{
	auto it = state.m_vec_rob_pos[uiRobot];
	N_Ind Ind(*it);
	if (uiDepth == m_set_prev_all_states[uiRobot].at(Ind))
	{
		size_t uiErase = m_set_prev_all_states[uiRobot].erase(Ind);
		assert(1 == uiErase);

		auto it_incomp_insert = m_set_to_do_verts.emplace(Ind.getInd());
		assert(true == it_incomp_insert.second);

		m_rob_hole_times[uiRobot].at(Ind.getInd()).m_uiStartTime = ST_Time::UNSET;

		const auto &vec_depo = m_graph.getDepotMap();
		if (*it == vec_depo.at(uiRobot).getFromInd()) return;
		it--;
		uiErase = m_set_prev_HD_states[uiRobot].erase(*it);
		assert(1 == uiErase);
	}
}

void Greedy_Heuristic::safe_initialize(size_t uiStartTime, size_t uiDepth, const State& state)
{
	update_vis_unvis_states_schedule(uiStartTime, uiDepth, state);
	allocate_positional_arc_buffer(uiDepth);
}

void Greedy_Heuristic::safe_backtrack(size_t uiDepth, const State& state)
{
	remove_vis_unvis_state_and_schedule(uiDepth, state);
	deallocate_positional_arc_buffer(uiDepth);
	m_map_states_feas[state] = -1;     // need to do this way instead of emplace, because this state key is already inserted
}

void Greedy_Heuristic::vectorize_schedule(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch)
{
	vec_rob_sch.resize(m_uiNumRobots);
	size_t uiStart, uiEnd, uiWait;

	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		auto it1 = rob_seq[uiRobot].begin();
		auto it2 = it1;
		it2++;
		uiStart = m_rob_hole_times[uiRobot].at(*it1).m_uiStartTime;
		for (; it2 != rob_seq[uiRobot].end(); it2++)
		{
			uiEnd = m_rob_hole_times[uiRobot].at(*it2).m_uiStartTime;
			uiWait = uiEnd - m_graph.getTime(*it1) - uiStart;
			vec_rob_sch[uiRobot].emplace_back(*it1, uiStart , uiEnd, uiWait);
			uiStart = uiEnd;
			it1++;
		}
		vec_rob_sch[uiRobot].emplace_back(*it1, uiStart, uiStart, 0 );
	}
}

void Greedy_Heuristic::compute_B_Q(const State& state, std::unordered_set<size_t> &B_Q)
{
	std::vector<size_t> set_robots;
	get_updatable_robots(state, set_robots);
	
	for (auto it_robot = set_robots.begin(); it_robot != set_robots.end(); it_robot++)
	{
		auto it = state.m_vec_rob_pos[*it_robot];
		it++;
		B_Q.emplace(*it);		
	}	
}

void Greedy_Heuristic::compute_succ_nodes(const size_t uiCurrTime, const State& state, std::vector<std::pair<Comparison_Object, State>> &vec_children)
{
	assert(0 == vec_children.size());
	size_t uiRobot;
	std::unordered_set<size_t> B_Q;	// when a robot has completed, B_Q does not include that end depot for that robot
	compute_B_Q(state, B_Q);
	std::list<std::unordered_set<size_t>> listStrConnComp;	//vertices
	m_alt_graph.get_next_strongly_conn_components(B_Q , m_set_to_do_verts, listStrConnComp);
	if (0 == listStrConnComp.size()) return;
	
	for (auto it1 = listStrConnComp.begin() ; it1 != listStrConnComp.end() ; it1++)
	{
		State child(state);
		size_t uiTime = uiCurrTime;

		for (auto it2 = it1->begin(); it2 != it1->end() ; it2++)
		{
			uiRobot = m_alt_graph.get_vertex_ownership(*it2);
			child.m_vec_rob_pos[uiRobot]++;
			N_Ind Ind(*state.m_vec_rob_pos[uiRobot]);
			assert(m_rob_hole_times[uiRobot].at(Ind).m_uiStartTime != ST_Time::UNSET);
			uiTime = std::max(m_rob_hole_times[uiRobot].at(Ind).m_uiStartTime + m_graph.getTime(Ind), uiTime);
		}
		auto pr = compute_exp_Mkspn_delay(uiTime, state);
		vec_children.emplace_back(Comparison_Object(uiTime, it1->size(), pr.first, pr.second), child);
	}
	std::sort(vec_children.begin(), vec_children.end(), greedy_heuristic);
}

/*void Greedy_Heuristic::compute_succ_nodes(const size_t uiCurrTime, const State& state, std::vector<std::tuple<int, int, int, size_t, State>> &vec_children)
{
	const auto &vec_robot_incr = get_child_states_iter_inc(state);
	size_t uiRobot;

	for (size_t uiCount1 = 0; uiCount1 < vec_robot_incr.size(); uiCount1++)
	{
		State child(state);
		size_t uiTime = uiCurrTime;

		for (size_t uiCount2 = 0; uiCount2 < vec_robot_incr[uiCount1].size(); uiCount2++)
		{
			uiRobot = vec_robot_incr[uiCount1][uiCount2];
			child.m_vec_rob_pos[uiRobot]++;
			N_Ind Ind(*state.m_vec_rob_pos[uiRobot]);
			assert(m_rob_hole_times[uiRobot].at(Ind).m_uiStartTime != ST_Time::UNSET);
			uiTime = std::max(m_rob_hole_times[uiRobot].at(Ind).m_uiStartTime + m_graph.getTime(Ind), uiTime);
		}
		auto pr = compute_exp_Mkspn_delay(uiTime, state);
		vec_children.emplace_back(std::make_tuple(pr.first, pr.second, (int)vec_robot_incr[uiCount1].size(), uiTime, child)); //need to change this
	}
	std::sort(vec_children.begin(), vec_children.end(), greedy_heuristic);
}*/

void Greedy_Heuristic::print_state(size_t uiDepth, size_t uiTime, const State &state)
{
	cout << "Depth: " << uiDepth << " , Robots loc:";
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		cout << *state.m_vec_rob_pos[uiRobot] << " , ";
	}
	cout << " , Time: " << uiTime << endl;
}

