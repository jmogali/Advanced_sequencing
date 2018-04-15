#include "Hole_exchanges.h"

Cand_for_insertion::Cand_for_insertion(size_t uiHD1, size_t uiHD2, size_t uiRobot, int iVal) :m_uiHD1(uiHD1), m_uiHD2(uiHD2), m_uiRobot(uiRobot), m_iVal(iVal)
{}

Hole_Exchange::Hole_Exchange(size_t uiNumRobots, const Layout_LS &graph, Power_Set &power, const Enabling_Graph &en_graph) : m_uiNumRobots(uiNumRobots), m_graph(graph), m_ls_heur(uiNumRobots, graph, power), m_en_graph(en_graph)
{
	m_rob_seq.resize(m_uiNumRobots);
	m_vec_full_rob_sch.resize(m_uiNumRobots);
}

void Hole_Exchange::clear_prev_info()
{
	m_vec_state_path.clear();
	m_vec_full_rob_sch.clear();
	m_alt_in_graph.clear();
	m_alt_out_graph.clear();
	m_rob_seq.clear();
	m_hole_rob_owner.clear();
	m_map_start_times.clear();
	m_map_completion_times.clear();
	m_map_vertex_slack.clear();
}

void Hole_Exchange::populate_rob_graphs(const Alternative_Graph &alt_graph)
{
	m_alt_in_graph = alt_graph.getReverseGraph();
	m_alt_out_graph = alt_graph.getGraph();
}

void Hole_Exchange::populate_robot_owners(const std::vector<std::list<size_t>> &rob_seq)
{
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		for (auto it = rob_seq[uiRobot].cbegin(); it != rob_seq[uiRobot].cend(); it++)
		{
			if ("H" != m_graph.getType(*it)) continue;
			m_hole_rob_owner.emplace(*it, uiRobot);
		}
	}
}

bool Hole_Exchange::perform_heuristic_moves(const std::vector<std::list<size_t>> &rob_seq, const Alternative_Graph &alt_graph, const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, size_t uiTargetMakeSpan)
{
	assert(rob_seq.size() == m_uiNumRobots);
	clear_prev_info();
		
	//populates m_vec_state_path
	m_rob_seq = rob_seq;
	m_vec_full_rob_sch = full_rob_sch;
	construct_state_transition_path(full_rob_sch);
	populate_rob_graphs(alt_graph);
	populate_robot_owners(rob_seq);	
	m_uiTargetMakeSpan = uiTargetMakeSpan;

	return perform_swap_operation();
}

void Hole_Exchange::copy_to_temp_buffers(std::vector<std::list<size_t>> &rob_seq, std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &out_graph, std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &in_graph, std::vector<State_vtx_time> &vec_state_path, 
										std::vector<std::vector<Vertex_Schedule>> &vec_full_rob_sch, std::unordered_map<size_t, size_t> &map_start_times, std::unordered_map<size_t, size_t> &map_completion_times)
{
	rob_seq = m_rob_seq;
	out_graph = m_alt_out_graph;
	in_graph = m_alt_in_graph;
	vec_state_path = m_vec_state_path;
	vec_full_rob_sch = m_vec_full_rob_sch;
	map_start_times = m_map_start_times;
	map_completion_times = m_map_completion_times;
}

void Hole_Exchange::copy_from_temp_buffers(std::vector<std::list<size_t>> &rob_seq, std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &out_graph, std::unordered_map<size_t, std::unordered_map<size_t, size_t>> &in_graph, std::vector<State_vtx_time> &vec_state_path, std::vector<std::vector<Vertex_Schedule>> &vec_full_rob_sch, std::unordered_map<size_t, size_t> &map_start_times, std::unordered_map<size_t, size_t> &map_completion_times)
{
	m_rob_seq = rob_seq;
	m_alt_out_graph = out_graph;
	m_alt_in_graph = in_graph;
	m_vec_state_path = vec_state_path;
	m_vec_full_rob_sch = vec_full_rob_sch;
	m_map_start_times = map_start_times;
	m_map_completion_times = map_completion_times;
}

bool Hole_Exchange::perform_swap_operation()
{
	size_t uiIter = 0, uiRobotOwner;
	bool bImproving;

	std::vector<std::list<size_t>> rob_seq_temp;
	std::vector<State_vtx_time> vec_state_path_temp;
	std::vector<std::vector<Vertex_Schedule>> vec_full_rob_sch_temp;
	std::unordered_map<size_t, std::unordered_map<size_t, size_t>> alt_out_graph_temp;   // <vtx, <out_vtx, arc cost>>
	std::unordered_map<size_t, std::unordered_map<size_t, size_t>> alt_in_graph_temp;	//  <vtx, <in_vtx, arc cost>> 
	std::unordered_map<size_t, size_t> map_start_times; 
	std::unordered_map<size_t, size_t> map_completion_times; 

	std::list<size_t> critical_path;
	std::list<std::tuple<N_Ind, size_t, size_t>> list_best_cand;

	while (uiIter < c_uiMaxNumSwaps)
	{
		copy_to_temp_buffers(rob_seq_temp, alt_out_graph_temp, alt_in_graph_temp, vec_state_path_temp, vec_full_rob_sch_temp, map_start_times, map_completion_times);

		compute_start_completion_times_from_schedule();
		compute_critical_path(critical_path);
		get_cand_vertex_critical_path(1, critical_path, list_best_cand);

		for (auto it_cand = list_best_cand.begin(); it_cand != list_best_cand.end(); it_cand++)
		{
			uiRobotOwner = m_hole_rob_owner.at(std::get<0>(*it_cand).getInd());
			bImproving = check_if_candidate_improving(std::get<0>(*it_cand).getInd(), std::get<1>(*it_cand), std::get<2>(*it_cand));
			if (bImproving) break;

			copy_from_temp_buffers(rob_seq_temp, alt_out_graph_temp, alt_in_graph_temp, vec_state_path_temp, vec_full_rob_sch_temp, map_start_times, map_completion_times);
			assign_robo_hole_owner(std::get<0>(*it_cand).getInd(), uiRobotOwner);
		}

		if (false == bImproving) break;
		uiIter++;
	}
	return bImproving;
}

bool Hole_Exchange::check_if_candidate_improving(const size_t c_uiHole, const size_t c_uiMinTime, const size_t c_uiMaxTime)
{
	std::vector<std::list<size_t>> rob_sub_seq;
	const size_t c_uiRobot = m_hole_rob_owner.at(c_uiHole);
	bool bFeasible = check_if_retraction_feasible(c_uiHole, c_uiRobot, rob_sub_seq);
	if (false == bFeasible) return false;

	std::pair<size_t, size_t> taboo_hole_pair; //should not insert new hole between the taboo hole pair
	bFeasible = update_sequence_graphs_for_removal(c_uiHole, c_uiRobot, rob_sub_seq, taboo_hole_pair);
	if (false == bFeasible) return false;

	remove_robo_hole_owner(c_uiHole);
	compute_vertex_slack();

	std::vector<std::list<size_t>> rob_seq_temp;
	std::vector<State_vtx_time> vec_state_path_temp;
	std::vector<std::vector<Vertex_Schedule>> vec_full_rob_sch_temp;
	std::unordered_map<size_t, std::unordered_map<size_t, size_t>> alt_out_graph_temp;   // <vtx, <out_vtx, arc cost>>
	std::unordered_map<size_t, std::unordered_map<size_t, size_t>> alt_in_graph_temp;	//  <vtx, <in_vtx, arc cost>> 
	std::unordered_map<size_t, size_t> map_start_times;
	std::unordered_map<size_t, size_t> map_completion_times;

	std::list<Cand_for_insertion> list_cand_insertion;
	get_cand_for_insertion(c_uiHole, c_uiMinTime, c_uiMaxTime, list_cand_insertion, taboo_hole_pair);
	const size_t c_uiDeletionMakeSpan = m_top_order_dist.get_makespan();

	//will add insertion of hole code here

	size_t uiIter = 0;
	bool bImproving = false;
	
	copy_to_temp_buffers(rob_seq_temp, alt_out_graph_temp, alt_in_graph_temp, vec_state_path_temp, vec_full_rob_sch_temp, map_start_times, map_completion_times);

	for(auto it_cand = list_cand_insertion.begin(); it_cand != list_cand_insertion.end(); it_cand++)
	{
		if (c_uiDeletionMakeSpan + it_cand->m_iVal > m_uiTargetMakeSpan) return false;
		
		bFeasible = check_if_insertion_feasible(c_uiHole, it_cand->m_uiRobot.getInd(), std::make_pair(it_cand->m_uiHD1.getInd(), it_cand->m_uiHD2.getInd()), rob_sub_seq);

		if (bFeasible)
		{
			//need to update graphs and then check whether it_cand improves
			bFeasible = update_sequence_graphs_for_insertion(c_uiHole, std::make_pair(it_cand->m_uiHD1.getInd(), it_cand->m_uiHD2.getInd()), it_cand->m_uiRobot.getInd(), rob_sub_seq);
			if (false == bFeasible) break;

			if (m_uiTargetMakeSpan > m_top_order_dist.get_makespan())
			{
				assign_robo_hole_owner(c_uiHole, it_cand->m_uiRobot.getInd());
				return true;
			}
			else bImproving = false;
		}

		if (!bFeasible || !bImproving)
		{
			copy_from_temp_buffers(rob_seq_temp, alt_out_graph_temp, alt_in_graph_temp, vec_state_path_temp, vec_full_rob_sch_temp, map_start_times, map_completion_times);
		}

		if (uiIter > c_uiMaxNumInsertTries) break;
		uiIter++;
	}
	return false;
}

void Hole_Exchange::compute_vertex_slack()
{
	m_map_vertex_slack.clear();

	//no need to recompute latest topological order since it_cand was already done during removal
	m_top_order_dist.compute_vertex_slack(m_map_vertex_slack);
}

int Hole_Exchange::compute_desirability_of_insertion(const size_t c_uiHD1, const size_t c_uiHD2, const size_t c_uiRobot, const size_t c_uiHole)
{
	int iVal;
	size_t uiOldPath, uiNewPath;
	uiOldPath = uiNewPath = m_graph.getTime(c_uiHD1);

	const auto &vec_rob_iv = m_graph.get_IV_Vec();
	const auto &vec_iv_1 = vec_rob_iv[c_uiRobot].map.at(c_uiHD1).map.at(c_uiHD2).vec;
	for (auto it_iv = vec_iv_1.begin(); it_iv != vec_iv_1.end(); it_iv++)
	{
		uiOldPath += m_graph.getTime(*it_iv);
	}

	const auto &vec_iv_2 = vec_rob_iv[c_uiRobot].map.at(c_uiHD1).map.at(c_uiHole).vec;
	for (auto it_iv = vec_iv_2.begin(); it_iv != vec_iv_2.end(); it_iv++)
	{
		uiNewPath += m_graph.getTime(*it_iv);
	}

	uiNewPath += m_graph.getTime(c_uiHole);
	const auto &vec_iv_3 = vec_rob_iv[c_uiRobot].map.at(c_uiHole).map.at(c_uiHD2).vec;
	for (auto it_iv = vec_iv_3.begin(); it_iv != vec_iv_3.end(); it_iv++)
	{
		uiNewPath += m_graph.getTime(*it_iv);
	}

	iVal = (int)(m_map_vertex_slack.at(c_uiHD2) + uiOldPath) - (int)uiNewPath;

	return iVal;
}

void Hole_Exchange::populate_new_sequence(std::vector<std::list<size_t>> &new_rob_sequence)
{
	assert(true == new_rob_sequence.empty());
	new_rob_sequence.resize(m_uiNumRobots);

	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		std::copy(m_rob_seq[uiRobot].begin(), m_rob_seq[uiRobot].end(), new_rob_sequence[uiRobot].begin());
	}
}