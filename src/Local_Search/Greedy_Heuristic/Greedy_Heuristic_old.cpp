#include "Greedy_Heuristic_old.h"
#include <iostream>

bool compareMakespan(const std::tuple<int, int, int, size_t, State>& lhs, const std::tuple<int, int, int, size_t, State>& rhs)
{
	if (std::get<3>(lhs) < std::get<3>(rhs)) return true;
	if (std::get<3>(lhs) > std::get<3>(rhs)) return false;

	if (std::get<0>(lhs) < std::get<0>(rhs)) return true;
	else if (std::get<0>(lhs) > std::get<0>(rhs)) return false;

	if (std::get<1>(lhs) < std::get<1>(rhs)) return true;
	else if (std::get<1>(lhs) > std::get<1>(rhs)) return false;

	if (std::get<2>(lhs) > std::get<2>(rhs)) return true;   //notice that change in sign, more the robots the better
	else if (std::get<2>(lhs) < std::get<2>(rhs)) return false;   //notice that change in sign, more the robots the better	

	boost::function<size_t(const State &x)> f;
	f = StateHasher{};
	return f(std::get<4>(lhs)) < f(std::get<4>(rhs));
}

Greedy_Heuristic_old::Greedy_Heuristic_old(const size_t uiRobotNum, const Layout_LS &graph, Power_Set &power) : m_uiNumRobots(uiRobotNum), m_graph(graph), m_power(power)
{
	m_rob_hole_times.resize(uiRobotNum);
	m_set_prev_HD_states.resize(uiRobotNum);
	m_set_prev_all_states.resize(uiRobotNum);
	m_vec_nc_eft.resize(uiRobotNum);
}

void Greedy_Heuristic_old::perform_initializations(const std::vector<std::list<size_t>> &rob_seq)
{
	allocate_interval_buffer(rob_seq);
	compute_NC_makespan(rob_seq);
}

void Greedy_Heuristic_old::clear_prev_info_buffers()
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
}

// populates buffers for storing timing information
void Greedy_Heuristic_old::allocate_interval_buffer(const std::vector<std::list<size_t>> &rob_seq)
{
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		for (auto it = rob_seq[uiRobot].begin(); it != rob_seq[uiRobot].end(); it++)
		{
			m_rob_hole_times[uiRobot].emplace(*it, ST_Time::UNSET);
		}
	}
}

// NC -: No - Collision
void Greedy_Heuristic_old::compute_NC_makespan(const std::vector<std::list<size_t>> &rob_seq)
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

void Greedy_Heuristic_old::populate_root_node_info(State &root, const std::vector<std::list<size_t>> &rob_seq)
{
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		root.m_vec_rob_pos[uiRobot] = rob_seq[uiRobot].begin();
	}	
}

int Greedy_Heuristic_old::compute_greedy_sol(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch)
{
	clear_prev_info_buffers();
	perform_initializations(rob_seq);
	State root(m_uiNumRobots);
	populate_root_node_info(root , rob_seq);
	int iRetVal = compute_DFS(root, 0, 0);
	if (1 == iRetVal) { vectorize_schedule(rob_seq , vec_rob_sch); }
	return iRetVal;
}

int Greedy_Heuristic_old::compute_DFS(const State& state , size_t uiDepth, size_t uiStartTime)
{
	//print_state(uiDepth , uiStartTime, state);
	update_visited_states_schedule(uiStartTime, uiDepth, state);

	int iRetVal = check_if_feasible(state);
	if (-1 == iRetVal)
	{
		remove_visited_state_and_schedule(uiDepth, state);
		return -1;
	}

	iRetVal = check_if_final_state(state);
	if (1 == iRetVal) return 1;
	
	//expected makespan, max delay, num_robots_schedule, next time, state
	std::vector<std::tuple<int, int, int, size_t, State>> vec_children;
	compute_succ_nodes(uiStartTime, state, vec_children);

	size_t uiInFeasChildCount = 0;
	for (size_t uiCount = 0; uiCount < vec_children.size(); uiCount++)
	{
		iRetVal = compute_DFS(std::get<4>(vec_children[uiCount]), uiDepth + 1, std::get<3>(vec_children[uiCount]));
		if (1 == iRetVal) return iRetVal;
		assert(-1 == iRetVal);
		uiInFeasChildCount++;
	}

	assert(vec_children.size() == uiInFeasChildCount);
	
	m_map_states_feas[state] = -1;     // need to do this way instead of emplace, because this state key is already inserted
	remove_visited_state_and_schedule(uiDepth, state);

	return -1;
}

//checks if last element is depot of that robot
int Greedy_Heuristic_old::check_if_final_state(const State& state)
{
	const auto &vec_depo = m_graph.getDepotMap();
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		if (*state.m_vec_rob_pos[uiRobot] != vec_depo.at(uiRobot).getToInd())
			return 0;
	}
	return 1;
}

int Greedy_Heuristic_old::check_if_feasible(const State& state)
{
	auto it = m_map_states_feas.find(state);
	if (it != m_map_states_feas.end())
	{
		return it->second;
	}
	
	int iRetVal = check_if_coll_feasible(state);
	if (-1 == iRetVal)
	{
		m_map_states_feas.emplace(state , -1);
		return -1;
	}
	
	iRetVal = check_if_enabling_feasible(state);
	// at this point both collision and enabling are checked, so we can emplace it, if all children of this state are 
	//infeasible, then DFS function changes it to infeasible in last line
	m_map_states_feas.emplace(state , iRetVal);     
	return iRetVal;
}

int Greedy_Heuristic_old::check_if_coll_feasible(const State& state)
{
	//const auto &coll_map = m_graph.getCollMap();

	for (size_t uiRobot1 = 0; uiRobot1 < m_uiNumRobots; uiRobot1++)
	{
		N_Ind Ind1(*state.m_vec_rob_pos[uiRobot1]);
		for (size_t uiRobot2 = uiRobot1 + 1; uiRobot2 < m_uiNumRobots; uiRobot2++)
		{
			N_Ind Ind2(*state.m_vec_rob_pos[uiRobot2]);
			if (true == m_graph.areColliding(Coll_Pair(Ind1, uiRobot1, Ind2, uiRobot2)))
				return -1;
		}
	}
	return 0;
}

int Greedy_Heuristic_old::check_if_enabling_feasible(const State& state)
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

bool Greedy_Heuristic_old::check_if_self_enabling(size_t uiRobot, const State& state)
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

bool Greedy_Heuristic_old::check_if_other_enabling(size_t uiRobot, const State& state)
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

void Greedy_Heuristic_old::compute_succ_nodes(const size_t uiCurrTime, const State& state, std::vector<std::tuple<int, int, int, size_t, State>> &vec_children)
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
			uiTime = std::max( m_rob_hole_times[uiRobot].at(Ind).m_uiStartTime + m_graph.getTime(Ind) , uiTime);
		}
		auto pr = compute_greedy_heursitic1(uiTime, state);
		vec_children.emplace_back(std::make_tuple(pr.first, pr.second, (int)vec_robot_incr[uiCount1].size(), uiTime, child)); //need to change this
	}
	std::sort(vec_children.begin(), vec_children.end(), compareMakespan);
}

// minimize expected makespan
std::pair<int, int> Greedy_Heuristic_old::compute_greedy_heursitic1(const size_t uiTime, const State& state)
{
	size_t maxMakespan = std::numeric_limits<size_t>::min(), uiMaxDelay = std::numeric_limits<size_t>::min();
	size_t ui_comp_Time, uiDelay, uiVal;
	
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		auto it = state.m_vec_rob_pos[uiRobot];
		assert(m_rob_hole_times[uiRobot].at(*it).m_uiStartTime != ST_Time::UNSET);
		ui_comp_Time = m_rob_hole_times[uiRobot].at(*it).m_uiStartTime + m_graph.getTime(*it);
		if (uiTime >= ui_comp_Time)
		{
			uiDelay = uiTime - m_vec_nc_eft[uiRobot].m_map_eft.at(*it);
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
	return std::make_pair((int)maxMakespan, (int)uiMaxDelay);
}

const std::vector<std::vector<size_t>>& Greedy_Heuristic_old::get_child_states_iter_inc(const State& state)
{
	std::vector<size_t> set_robots;
	get_updatable_robots(state, set_robots);
	return m_power.get_power_set(set_robots);
}

void Greedy_Heuristic_old::get_updatable_robots(const State& state, std::vector<size_t> &set_robots)
{
	const auto &vec_depots = m_graph.getDepotMap();
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		if (*state.m_vec_rob_pos[uiRobot] != vec_depots.at(uiRobot).getToInd())
			set_robots.push_back(uiRobot);
	}
}

void Greedy_Heuristic_old::update_visited_states_schedule(size_t uiCurrTime, size_t uiDepth, const State& state)
{
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		update_visited_all_states_schedule(uiRobot, uiCurrTime, uiDepth, state);
	}
}

void Greedy_Heuristic_old::update_visited_all_states_schedule(size_t uiRobot, size_t uiCurrTime, size_t uiDepth, const State& state)
{
	auto it = state.m_vec_rob_pos[uiRobot];
	N_Ind Ind(*it);
	if (m_set_prev_all_states[uiRobot].find(Ind) == m_set_prev_all_states[uiRobot].end())
	{
		auto it_insert_Prev = m_set_prev_all_states[uiRobot].emplace(Ind, uiDepth);
		assert(true == it_insert_Prev.second);
		assert(m_rob_hole_times[uiRobot].at(Ind.getInd()).m_uiStartTime == ST_Time::UNSET);
		m_rob_hole_times[uiRobot].at(Ind.getInd()).m_uiStartTime = uiCurrTime;
		
		const auto &vec_depo = m_graph.getDepotMap();
		if (*it == vec_depo.at(uiRobot).getFromInd()) return;
		it--;		
		auto it_insert_HD_states = m_set_prev_HD_states[uiRobot].emplace(*it, uiDepth);
		assert(true == it_insert_HD_states.second);
	}
}

void Greedy_Heuristic_old::remove_visited_state_and_schedule(size_t uiDepth, const State& state)
{
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		remove_visited_all_states_schedule(uiRobot, uiDepth, state);		
	}
}

void Greedy_Heuristic_old::remove_visited_all_states_schedule(size_t uiRobot, size_t uiDepth, const State& state)
{
	auto it = state.m_vec_rob_pos[uiRobot];
	N_Ind Ind(*it);
	if (uiDepth == m_set_prev_all_states[uiRobot].at(Ind))
	{
		size_t uiErase = m_set_prev_all_states[uiRobot].erase(Ind);
		assert(uiErase == 1);
		m_rob_hole_times[uiRobot].at(Ind.getInd()).m_uiStartTime = ST_Time::UNSET;

		const auto &vec_depo = m_graph.getDepotMap();
		if (*it == vec_depo.at(uiRobot).getFromInd()) return;
		it--;
		uiErase = m_set_prev_HD_states[uiRobot].erase(*it);
		assert(uiErase == 1);
	}
}

void Greedy_Heuristic_old::vectorize_schedule(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch)
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

void Greedy_Heuristic_old::print_state(size_t uiDepth, size_t uiTime, const State &state)
{
	cout << "Depth: " << uiDepth << " , Robots loc:";
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		cout << *state.m_vec_rob_pos[uiRobot] << " , ";
	}
	cout << " , Time: " << uiTime<<endl;
}