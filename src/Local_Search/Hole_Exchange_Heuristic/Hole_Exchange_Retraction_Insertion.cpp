#include "Hole_exchanges.h"

void modify_to_add_end_depot_sub_seq(std::vector<bool> &vec_end_depot, std::vector<std::list<size_t>> &rob_sub_seq, const Layout_LS &graph)
{
	const auto &map_depo = graph.getDepotMap();
	for (size_t uiRobot = 0; uiRobot < rob_sub_seq.size(); uiRobot++)
	{
		size_t uiToDepot = map_depo.at(uiRobot).getToInd();
		if (*rob_sub_seq[uiRobot].rbegin() != uiToDepot)
		{
			rob_sub_seq[uiRobot].emplace_back(uiToDepot);
			vec_end_depot[uiRobot] = false;
		}
		else vec_end_depot[uiRobot] = true;
	}
}

void remove_pseudo_end_depot_sub_seq(const std::vector<bool> &vec_end_depot, std::vector<std::list<size_t>> &rob_sub_seq)
{
	for (size_t uiRobot = 0; uiRobot < rob_sub_seq.size(); uiRobot++)
	{
		if (false == vec_end_depot[uiRobot]) rob_sub_seq[uiRobot].pop_back();
	}
}

size_t get_first_occurence_index(size_t uiVtx, size_t uiRobot, const std::vector<State_vtx_time> &vec_state_path)
{
	for (size_t uiIndex = 0; uiIndex < vec_state_path.size(); uiIndex++)
	{
		if (uiVtx == vec_state_path[uiIndex].m_vec_rob_vtx[uiRobot]) return uiIndex;
	}
	return std::numeric_limits<size_t>::max();
}

void get_new_left_right_hole_state_offset(const size_t c_uiHole, const size_t c_uiHole_Robot, size_t &uiLeft, size_t &uiRight, const std::vector<State_vtx_time> &vec_state_path, const Layout_LS &graph, const size_t c_uiIter)
{
	size_t uiNewLeft, uiNewRight;
	bool bValid;
	const size_t c_uiLeftStart = std::max( (int)((int)(uiLeft) - (int)c_uiHoleExcOffsetJump), 0);
	const size_t c_uiRightStart = std::min(uiRight + c_uiHoleExcOffsetJump, vec_state_path.size()-1);

	for (uiNewLeft = c_uiLeftStart; uiNewLeft >=0; uiNewLeft--)
	{
		if (0 == uiNewLeft) break;
		bValid = true;			

		if (vec_state_path[uiNewLeft].m_vec_rob_vtx[c_uiHole_Robot] == c_uiHole) bValid = false;
		else if (0 == c_uiIter)
		{
			if ("H" != graph.getType(vec_state_path[uiNewLeft].m_vec_rob_vtx[c_uiHole_Robot])) bValid = false;					
		}
		if (bValid) break;		
	}

	for (uiNewRight = c_uiRightStart; uiNewRight < vec_state_path.size(); uiNewRight++)
	{			
		bValid = true;
		if (vec_state_path[uiNewRight].m_vec_rob_vtx[c_uiHole_Robot] == c_uiHole) bValid = false;
		else if (0 == c_uiIter)
		{
			if ("H" != graph.getType(vec_state_path[uiNewRight].m_vec_rob_vtx[c_uiHole_Robot])) bValid = false;
		}			
		if (bValid) break;	
	}

	uiLeft = uiNewLeft;
	uiRight = std::min(uiNewRight, vec_state_path.size()-1);
}

void Hole_Exchange::check_ifsub_seq_construction_correct(std::vector<std::list<size_t>> &rob_sub_seq, const size_t c_uiLeft, const size_t c_uiRight, const std::vector<State_vtx_time> &vec_state_path)
{
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
#ifdef WINDOWS
		//assert(*rob_sub_seq[uiRobot].begin() == vec_state_path[c_uiLeft].m_vec_rob_vtx[uiRobot]);
		//assert(*rob_sub_seq[uiRobot].rbegin() == vec_state_path[c_uiRight].m_vec_rob_vtx[uiRobot]);
#else
		if (*rob_sub_seq[uiRobot].begin() != vec_state_path[c_uiLeft].m_vec_rob_vtx[uiRobot])
		{
			cout << "Sub sequence construction begin incorrect\n";
			exit(-1);
		}
		if (*rob_sub_seq[uiRobot].rbegin() != vec_state_path[c_uiRight].m_vec_rob_vtx[uiRobot])
		{
			cout << "Sub sequence construction end incorrect\n";
			exit(-1);
		}
#endif
		auto it_curr = rob_sub_seq[uiRobot].begin();
		auto it_next = it_curr;
		it_next++;
		for (; it_curr != rob_sub_seq[uiRobot].end(); it_curr++, it_next++)
		{
			if (it_next == rob_sub_seq[uiRobot].end()) break;

			if (*it_curr == *it_next)
			{
#ifdef WINDOWS
				assert(false);				
#else
				cout << "Construction incorrect";
				exit(-1);
#endif
			}			
		}
	}
}

std::tuple<size_t, size_t, bool> remove_INP_HOLE_in_rob_sub_seq(size_t c_uiHole, const size_t c_uiRobot, std::vector<std::list<size_t>> &rob_sub_seq, const Layout_LS &graph)
{
	const size_t c_uiInitialSize = rob_sub_seq[c_uiRobot].size();
	const auto &vec_rob_iv = graph.get_IV_Vec();
	size_t uiIV_FOLL_PREV_HD, uiPrevHD;
	
	auto it = std::find(rob_sub_seq[c_uiRobot].begin(), rob_sub_seq[c_uiRobot].end(), c_uiHole);	

#ifdef WINDOWS
	assert(it != rob_sub_seq[c_uiRobot].end());
#else
	if (it == rob_sub_seq[c_uiRobot].end())
	{
		cout << "Hole subsequence construction incorrect \n";
		exit(-1);
	}
#endif

	auto it_prevHD = it; //it points to c_uiHole
	auto it_IV_foll_PrevHD = it; //first IV following previous HD
	
	it_prevHD--;
	while ("IV" == graph.getType(*it_prevHD))
	{
		it_IV_foll_PrevHD = it_prevHD;
		it_prevHD--;		
	}
	uiPrevHD = *it_prevHD;

	uiIV_FOLL_PREV_HD = *it_IV_foll_PrevHD;

	auto it_next_HD = it;
	it_next_HD++;
	while ("IV" == graph.getType(*it_next_HD)) it_next_HD++; // points to HD following c_uiHole in rob_seq
	
	rob_sub_seq[c_uiRobot].erase(it_IV_foll_PrevHD, it_next_HD); //note erase [begin, last)

	//note at this this stage "it iterator" is perhaps invalidated
	if (vec_rob_iv[c_uiRobot].map.at(uiPrevHD).map.end() == vec_rob_iv[c_uiRobot].map.at(uiPrevHD).map.find(*it_next_HD))
	{
		return std::make_tuple(std::numeric_limits<size_t>::max(), std::numeric_limits<size_t>::max(), false);
	}

	const auto &vec_iv = vec_rob_iv[c_uiRobot].map.at(uiPrevHD).map.at(*it_next_HD).vec;
	for (auto it_iv = vec_iv.begin(); it_iv != vec_iv.end(); it_iv++)
	{
		rob_sub_seq[c_uiRobot].insert(it_next_HD, it_iv->getInd());
	}	
	assert(c_uiInitialSize - rob_sub_seq[c_uiRobot].size() == 2);
	return std::make_tuple(uiPrevHD, *it_next_HD, true);
}


bool insert_INP_HOLE_in_rob_seq(size_t c_uiHole, const size_t c_uiRobot, const std::pair<size_t, size_t> pr_hole_pair, std::vector<std::list<size_t>> &rob_sub_seq, const Layout_LS &graph)
{
	const size_t c_uiInitialSize = rob_sub_seq[c_uiRobot].size();
	auto it_HD2 = std::find(rob_sub_seq[c_uiRobot].begin(), rob_sub_seq[c_uiRobot].end(), pr_hole_pair.second);
	assert(it_HD2 != rob_sub_seq[c_uiRobot].end());

	auto it_HD1 = it_HD2;
	it_HD1--;
	while ("IV" == graph.getType(*it_HD1))
	{
		it_HD1 = rob_sub_seq[c_uiRobot].erase(it_HD1);
		assert(*it_HD2 == *it_HD1);
		it_HD1--;
	}
	
	assert(pr_hole_pair.first == *it_HD1);
	const auto &vec_rob_iv = graph.get_IV_Vec();

	if (vec_rob_iv[c_uiRobot].map.at(pr_hole_pair.first).map.end() == vec_rob_iv[c_uiRobot].map.at(pr_hole_pair.first).map.find(c_uiHole)) return false;

	const auto &vec_iv_1 = vec_rob_iv[c_uiRobot].map.at(pr_hole_pair.first).map.at(c_uiHole).vec;
	for (auto it_iv = vec_iv_1.begin(); it_iv != vec_iv_1.end(); it_iv++)
	{
		rob_sub_seq[c_uiRobot].insert(it_HD2, it_iv->getInd());
	}

	rob_sub_seq[c_uiRobot].insert(it_HD2, c_uiHole);

	if (vec_rob_iv[c_uiRobot].map.at(c_uiHole).map.end() == vec_rob_iv[c_uiRobot].map.at(c_uiHole).map.find(pr_hole_pair.second)) return false;

	const auto &vec_iv_2 = vec_rob_iv[c_uiRobot].map.at(c_uiHole).map.at(pr_hole_pair.second).vec;
	for (auto it_iv = vec_iv_2.begin(); it_iv != vec_iv_2.end(); it_iv++)
	{
		rob_sub_seq[c_uiRobot].insert(it_HD2, it_iv->getInd());
	}
		
	assert(rob_sub_seq[c_uiRobot].size() - c_uiInitialSize == 2);
	return true;
}

//Here we assume inp_seq contains c_uiHole, so care must be taken to remove it when checking for feaibility
bool Hole_Exchange::check_if_retraction_feasible(const size_t c_uiHole, const size_t c_uiRobot, std::vector<std::list<size_t>> &rob_sub_seq)
{
	bool bFeasible = false;
	size_t uiIter = 0;
	std::vector<std::tuple<std::list<size_t>::const_iterator, std::list<size_t>::const_iterator, size_t>> vec_start_end_itr_start_pos;
	std::unordered_set<size_t> set_comp_HD;
	std::unordered_set<size_t> set_enabled_holes;
	std::vector<size_t> vec_start_times_first_vertex; 
	//std::vector<bool> vec_end_depot;
	
	size_t uiIndex = get_first_occurence_index(c_uiHole, c_uiRobot, m_vec_state_path);
	size_t uiLeftPathIndex = uiIndex, uiRightPathIndex = uiIndex;	

	rob_sub_seq.resize(m_uiNumRobots);
	vec_start_times_first_vertex.resize(m_uiNumRobots);
	//vec_end_depot.resize(m_uiNumRobots, false);

	while (!bFeasible)
	{
		if (uiRightPathIndex - uiLeftPathIndex >= std::min(c_uiHoleExchMaxLen, m_vec_state_path.size() - 1)) break;
		get_new_left_right_hole_state_offset(c_uiHole, c_uiRobot, uiLeftPathIndex, uiRightPathIndex, m_vec_state_path, m_graph, uiIter);
		
		if (0 == uiIter)
		{
			construct_rob_sub_sequences(rob_sub_seq, uiLeftPathIndex, uiRightPathIndex, m_rob_seq, m_vec_state_path, vec_start_end_itr_start_pos, set_comp_HD);
			auto res = remove_INP_HOLE_in_rob_sub_seq(c_uiHole, c_uiRobot, rob_sub_seq, m_graph);
			if (false == std::get<2>(res)) return false;
			uiIter++;
		}		
		else
		{
			construct_rob_sub_sequences_with_iterators(rob_sub_seq, uiLeftPathIndex, uiRightPathIndex, m_vec_state_path, vec_start_end_itr_start_pos, set_comp_HD);
		}

		//modify_to_add_end_depot_sub_seq(vec_end_depot, rob_sub_seq, m_graph);
		
		check_ifsub_seq_construction_correct(rob_sub_seq, uiLeftPathIndex, uiRightPathIndex, m_vec_state_path);

		set_enabled_holes.clear();
		compute_enabled_holes_for_rob_sub_seq(rob_sub_seq, set_comp_HD, set_enabled_holes);
		
		//need to adjust start times to take into the fact that some portion of the work for the hole must have been completed already
		for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
		{
			int iRemTime = (int)m_graph.getTime(*std::get<0>(vec_start_end_itr_start_pos[uiRobot]))+ (int)m_vec_full_rob_sch[uiRobot][std::get<2>(vec_start_end_itr_start_pos[uiRobot])].m_uiStart - (int)m_vec_state_path[uiLeftPathIndex].m_uiTime;
			//note iRemTime can be negative, this occurs when the robot is waiting at a vertex
			vec_start_times_first_vertex[uiRobot] = 1 + (size_t)std::max(0, iRemTime);
		}

		std::vector<std::vector<Vertex_Schedule>> new_vec_rob_sub_seq_sch;
		int iRetVal = m_ls_heur_old.compute_greedy_sol(rob_sub_seq, vec_start_times_first_vertex, set_enabled_holes, new_vec_rob_sub_seq_sch);
		
		//remove_pseudo_end_depot_sub_seq(vec_end_depot, rob_sub_seq);
		
		if (1 == iRetVal)
		{
			//m_ls_heur.minimally_purge_end_depot_info(vec_end_depot);
			bFeasible = true;
		}
		else bFeasible = false;
	}
	return bFeasible;
}

//here obviously inp_seq does not contain c_uiHole
bool Hole_Exchange::check_if_insertion_feasible(const size_t c_uiHole, const size_t c_uiRobot, const std::pair<size_t, size_t> pr_hole_pair, std::vector<std::list<size_t>> &rob_sub_seq)
{
	bool bFeasible = false;
	size_t uiIter = 0;
	std::vector<std::tuple<std::list<size_t>::const_iterator, std::list<size_t>::const_iterator, size_t>> vec_start_end_itr_start_pos;
	std::unordered_set<size_t> set_comp_HD;
	std::unordered_set<size_t> set_enabled_verts;
	std::vector<size_t> vec_start_times_first_vertex;
	vec_start_times_first_vertex.resize(m_uiNumRobots);
	//std::vector<bool> vec_end_depot;

	size_t uiLeftPathIndex = get_first_occurence_index(pr_hole_pair.first, c_uiRobot, m_vec_state_path);
	size_t uiRightPathIndex = get_first_occurence_index(pr_hole_pair.second, c_uiRobot, m_vec_state_path);

	rob_sub_seq.clear();
	rob_sub_seq.resize(m_uiNumRobots);
	//vec_end_depot.resize(m_uiNumRobots, false);

	while (!bFeasible)
	{
		get_new_left_right_hole_state_offset(c_uiHole, c_uiRobot, uiLeftPathIndex, uiRightPathIndex, m_vec_state_path, m_graph, uiIter);

		if (0 == uiIter)
		{
			construct_rob_sub_sequences(rob_sub_seq, uiLeftPathIndex, uiRightPathIndex, m_rob_seq, m_vec_state_path, vec_start_end_itr_start_pos, set_comp_HD);
			bFeasible = insert_INP_HOLE_in_rob_seq(c_uiHole, c_uiRobot, pr_hole_pair, rob_sub_seq, m_graph);
			if (false == bFeasible) return false;
			uiIter++;
		}
		else
		{
			construct_rob_sub_sequences_with_iterators(rob_sub_seq, uiLeftPathIndex, uiRightPathIndex, m_vec_state_path, vec_start_end_itr_start_pos, set_comp_HD);
		}

		//modify_to_add_end_depot_sub_seq(vec_end_depot, rob_sub_seq, m_graph);

		check_ifsub_seq_construction_correct(rob_sub_seq, uiLeftPathIndex, uiRightPathIndex, m_vec_state_path);

		set_enabled_verts.clear();
		compute_enabled_holes_for_rob_sub_seq(rob_sub_seq, set_comp_HD, set_enabled_verts);

		//need to adjust start times to take into the fact that some portion of the work for the hole must have been completed already
		for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
		{
			int iRemTime = (int)m_graph.getTime(*std::get<0>(vec_start_end_itr_start_pos[uiRobot])) + (int)m_vec_full_rob_sch[uiRobot][std::get<2>(vec_start_end_itr_start_pos[uiRobot])].m_uiStart - (int)m_vec_state_path[uiLeftPathIndex].m_uiTime;
			//note iRemTime can be negative, this occurs when the robot is waiting at a vertex
			vec_start_times_first_vertex[uiRobot] = 1 + (size_t)std::max(0, iRemTime);		
		}

		std::vector<std::vector<Vertex_Schedule>> new_vec_rob_sub_seq_sch;
		int iRetVal = m_ls_heur_old.compute_greedy_sol(rob_sub_seq, vec_start_times_first_vertex, set_enabled_verts, new_vec_rob_sub_seq_sch);
		
		//remove_pseudo_end_depot_sub_seq(vec_end_depot, rob_sub_seq);

		if (1 == iRetVal)
		{
			//m_ls_heur.minimally_purge_end_depot_info(vec_end_depot);
			bFeasible = true;
		}
		else bFeasible = false;

		if (uiRightPathIndex - uiLeftPathIndex>= std::min(c_uiHoleExchMaxLen, m_vec_state_path.size() - 1)) break;
	}
	return bFeasible;
}

