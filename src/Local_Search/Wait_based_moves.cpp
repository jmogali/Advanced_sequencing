#include "Local_Search.h"
#include "Local_Search_Constants.h"

// < hole index, position >
std::pair<size_t, size_t> get_hole_based_on_time(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, const Layout_LS &graph, size_t uiTime, size_t uiRobot)
{
	size_t uiPos = 0, uiHoleInd = std::numeric_limits<size_t>::max();
	std::string strType;

	for (auto it = full_rob_sch[uiRobot].begin(); it != full_rob_sch[uiRobot].end(); it++)
	{
		strType = graph.getType(it->m_uiInd);
		if ("IV" == strType) continue;

		if (it->m_uiStart > uiTime)
		{
			if ("D" == strType) break;
			return std::make_pair(it->m_uiInd, uiPos);
		}
		uiPos++;
	}
	return std::make_pair(std::numeric_limits<size_t>::max(), std::numeric_limits<size_t>::max());  // returning this way to indicate that at uiTime, 
																									// uiRobot has completed its schedule, so we can 
																									// need to insert through spatial than temporal reasoning.
}

size_t get_Robot_to_Swap_Hole_With(const Node_Partitions &m_node_data, size_t uiHoleIndToMove, size_t uiBottleNeckRobot)
{
	size_t uiSwapRobot;
	const auto &comm_robots = m_node_data.get_common_robots_for_Ind(uiHoleIndToMove);
	
#ifdef WINDOWS	
	assert(comm_robots.set.size() > 1);
#else
	if (comm_robots.set.size() == 1)
	{
		cout << "Not exchangeable vertex \n";
		exit(1);
	}
#endif

	const size_t c_uiRandInd = rand() % (comm_robots.set.size() - 1);
	size_t uiCount = 0;
	for (auto it = comm_robots.set.begin(); it != comm_robots.set.end(); it++)
	{
		if (*it == uiBottleNeckRobot) continue;
		if (c_uiRandInd == uiCount)
		{
			uiSwapRobot = *it;
			break;
		}
		uiCount++;
	}
	return uiSwapRobot;
}

void get_wait_indices_wait_vals_for_robot(const std::vector<Vertex_Schedule> &rob_sch, std::vector<std::pair<size_t, size_t>> &vec_wait_pos_ind_wait_val, const std::list<size_t> &seq, const Layout_LS &graph)
{
	size_t uiPos = 1, uiWait;
	auto it = seq.begin();
	it++;

	for (size_t uiCount = 2; uiCount < rob_sch.size()-1;)
	{
#ifdef ENABLE_FULL_CHECKING
#ifdef WINDOWS
		assert(*it == rob_sch[uiCount].m_uiInd);
		assert("H" == graph.getType(rob_sch[uiCount].m_uiInd));
		assert("IV" == graph.getType(rob_sch[uiCount+1].m_uiInd));
#else
		if (*it != rob_sch[uiCount].m_uiInd) { cout << "mismatch in schedule and robot sequence\n"; exit(1); }
		if ("H" != graph.getType(rob_sch[uiCount].m_uiInd)) { cout << "Not reading a hole vertex\n"; exit(1); }
		if ("IV" != graph.getType(rob_sch[uiCount + 1].m_uiInd)) { cout << "Not an intermediate vertex\n";  exit(1);}
#endif
#endif
		uiWait = rob_sch[uiCount].m_uiWait + rob_sch[uiCount + 1].m_uiWait;
		if (uiWait > 0) vec_wait_pos_ind_wait_val.push_back(std::make_pair(uiPos,uiWait));
		uiCount = uiCount + 2;
		uiPos++;
		it++;
	}
}

//more wait means earlier in the list
void get_sorted_wait_indices_for_robot(const std::vector<Vertex_Schedule> &rob_sch, std::vector<std::pair<size_t, size_t>> &vec_wait_pos_ind_wait_val, const std::list<size_t> &r_seq, const Layout_LS &graph)
{
	get_wait_indices_wait_vals_for_robot(rob_sch, vec_wait_pos_ind_wait_val, r_seq, graph);
	std::sort(vec_wait_pos_ind_wait_val.begin(), vec_wait_pos_ind_wait_val.end(), sort_by_max_second_val());
}

std::tuple<bool, size_t, size_t> Local_Search::wait_based_swap_for_robot(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, std::vector<std::list<size_t>> &rob_seq, size_t uiRobot, std::string strType)
{
	//uiPos in rob_seq[uiRobot], uiInd in full_rob_sch[uiRobot], delay value
	std::vector<std::pair<size_t, size_t>> vec_wait_pos_ind_wait_val;
	get_sorted_wait_indices_for_robot(full_rob_sch[uiRobot], vec_wait_pos_ind_wait_val, rob_seq[uiRobot], m_graph);
	size_t uiPos, uiSchInd;
	bool bValid = false;
	size_t uiMaxMoves = std::min(c_uiMaxWaitEventsPerRobot, vec_wait_pos_ind_wait_val.size());

	for (size_t uiCount = 0; uiCount < uiMaxMoves; uiCount++)
	{
		uiPos = vec_wait_pos_ind_wait_val[uiCount].first;
		uiSchInd = 2 * uiPos;

		std::string strMoveType;
		if ( (uiPos == 0) || (uiPos == 1)) strMoveType = "NEXT_HOLE";
		else strMoveType = ((rand() % 2) == 0) ? "NEXT_HOLE" : "SAME_HOLE";
		
#ifdef PRINT_LOCAL_OPERATOR_MESSAGES
		cout << "Move Type: " << strMoveType << endl;
#endif

		if ("INTER_SEQUENCE" == strType)
		{
			auto res = wait_based_move_inter_sequence(full_rob_sch, rob_seq, uiRobot, full_rob_sch[uiRobot][uiSchInd].m_uiInd, uiPos, full_rob_sch[uiRobot][uiSchInd].m_uiStart, strMoveType);
			if ("SUCCESS" == res.first) return std::make_tuple(true , uiRobot, res.second);
			else if ("NOT_COMMON_NODE" == res.first)
			{
#ifdef PRINT_LOCAL_OPERATOR_MESSAGES			
				cout << "Move Refined: TRUE"<<endl;
#endif

				// do intra sequence
				bValid = wait_based_move_intra_sequence(full_rob_sch, rob_seq, uiRobot, full_rob_sch[uiRobot][uiSchInd].m_uiInd, uiPos, strMoveType);
				if (true == bValid) return std::make_tuple(true, uiRobot, std::numeric_limits<size_t>::max());
			}
		}
		else if ("INTRA_SEQUENCE" == strType)
		{
			bValid = wait_based_move_intra_sequence(full_rob_sch, rob_seq, uiRobot, full_rob_sch[uiRobot][uiSchInd].m_uiInd, uiPos, strMoveType);
			if (true == bValid) return std::make_tuple(true, uiRobot, std::numeric_limits<size_t>::max());
		}
	}
	return std::make_tuple(false, std::numeric_limits<size_t>::max(), std::numeric_limits<size_t>::max());
}

size_t get_best_position_to_insert_in_other_robot(size_t uiLower, size_t uiUpper, size_t uiHoleInd, size_t uiInsertionRobot, const std::list<size_t> &seq, const Layout_LS &graph)
{
	size_t uiDist = std::numeric_limits<size_t>::max(), uiInsertion;
	size_t uiNewPos = uiLower;

	auto it_s = seq.begin();
	std::advance(it_s, uiLower - 1);

	auto it_next = it_s;
	it_next++;

	for (size_t uiCount = uiLower; uiCount <= uiUpper; uiCount++)
	{
//#ifdef ENABLE_FULL_CHECKING
#ifdef WINDOWS
		assert(true == graph.doesEdgeExist(uiInsertionRobot, *it_s, uiHoleInd));
		assert(true == graph.doesEdgeExist(uiInsertionRobot, uiHoleInd, *it_next));
#else
		if ((false == graph.doesEdgeExist(uiInsertionRobot, *it_s, uiHoleInd)) || (false == graph.doesEdgeExist(uiInsertionRobot, uiHoleInd, *it_next)))
		{
			cout << "Edge does not exist for swap";
			exit(1);
		}
#endif
//#endif
		uiInsertion = graph.getEdgeDist(uiInsertionRobot, *it_s, uiHoleInd);
		uiInsertion += graph.getEdgeDist(uiInsertionRobot, uiHoleInd, *it_next);
		if (uiDist > uiInsertion)
		{
			uiNewPos = uiCount;
			uiDist = uiInsertion;
		}
		it_s++;
		it_next++;
	}
	return uiNewPos;
}

size_t get_best_position_to_insert_in_same_robot(size_t uiLower, size_t uiUpper, size_t uiHoleInd, size_t uiCurrHolePos, size_t uiRobot, const std::list<size_t> &seq, const Layout_LS &graph)
{
	size_t uiDist = std::numeric_limits<size_t>::max(), uiInsertion;
	size_t uiNewPos = uiLower;

	auto it_s = seq.begin();
	std::advance(it_s, uiLower - 1);

	auto it_next = it_s;
	it_next++;

	for (size_t uiCount = uiLower; uiCount <= uiUpper; uiCount++, it_s++ , it_next++)
	{
		if ((uiCount == uiCurrHolePos) || (uiCount == uiCurrHolePos + 1)) continue;
		
#ifdef ENABLE_FULL_CHECKING
#ifdef WINDOWS
		assert(true == graph.doesEdgeExist(uiRobot, *it_s, uiHoleInd));
		assert(true == graph.doesEdgeExist(uiRobot, uiHoleInd, *it_next));
#else
		if ((false == graph.doesEdgeExist(uiRobot, *it_s, uiHoleInd)) || (false == graph.doesEdgeExist(uiRobot, uiHoleInd, *it_next)))
		{
			cout << "Edge does not exist for swap";
			exit(1);
		}
#endif
#endif
		uiInsertion = graph.getEdgeDist(uiRobot, *it_s, uiHoleInd);
		uiInsertion += graph.getEdgeDist(uiRobot, uiHoleInd, *it_next);
		if (uiDist > uiInsertion)
		{
			uiNewPos = uiCount;
			uiDist = uiInsertion;
		}		
	}
	return uiNewPos;
}

std::pair<std::string, size_t> Local_Search::wait_based_move_inter_sequence(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, std::vector<std::list<size_t>> &rob_seq, size_t uiRobot, size_t uiWaitHoleInd, size_t uiWaitHolePos, size_t uiTime, std::string strWaitMove)
{	
	size_t uiSwapHolePos;
	size_t uiSwapHoleInd;
	if ("NEXT_HOLE" == strWaitMove)
	{
		auto it = rob_seq[uiRobot].begin();
		uiSwapHolePos = uiWaitHolePos + 1;
		std::advance(it, uiSwapHolePos);
		uiSwapHoleInd  = *it;

#ifdef ENABLE_FULL_CHECKING
#ifdef WINDOWS
		assert("IV" != m_graph.getType(uiSwapHoleInd));
#else
		if ("IV" == m_graph.getType(uiSwapHoleInd))
		{
			cout << "Swapping an intermediate vertex \n";
			exit(1);
		}
#endif
#endif
	}
	else if ("SAME_HOLE" == strWaitMove)
	{
		uiSwapHolePos = uiWaitHolePos;
		uiSwapHoleInd = uiWaitHoleInd;
	}	

#ifdef WINDOWS	
	assert(2 == m_node_data.m_uiNumRobots);
#else
	if (2 != m_node_data.m_uiNumRobots)
	{
		cout << "This heuristic is not designed for more than 2 robots yet";
		exit(1);
	}
#endif
	
	bool bSwapPossible = !(m_node_data.isLocalVertex(uiSwapHoleInd));
	if (false == bSwapPossible) return std::make_pair("NOT_COMMON_NODE", std::numeric_limits<size_t>::max());

	size_t uiOtherRobot = get_Robot_to_Swap_Hole_With(m_node_data, uiSwapHoleInd, uiRobot);
	auto pr = get_hole_based_on_time(full_rob_sch, m_graph, uiTime, uiOtherRobot);
	
	size_t uiLowerBound = (size_t)std::max((int)pr.second - c_iWaitSwapRange, 1);
	if (1 == uiLowerBound)
	{
		if (false == m_graph.doesEdgeExist(uiOtherRobot, m_node_data.m_rob_depo[uiOtherRobot].first, uiSwapHoleInd)) uiLowerBound = 2;
	}

	size_t uiUpperBound = (size_t)std::min((int)pr.second + c_iWaitSwapRange, (int)rob_seq[uiOtherRobot].size()-1);
	
	if (std::abs( (int)uiUpperBound - (int)uiLowerBound ) <= 2) return std::make_pair("BOUND_FAILED", std::numeric_limits<size_t>::max());
	
	//std::uniform_int_distribution<size_t> unif_len(uiLowerBound, uiUpperBound);
	//size_t uiNewPos = unif_len(m_rng);
	
	//cout << "Lower: " << uiLowerBound << " , Upper: " << uiUpperBound << endl;
	
	size_t uiNewPos = get_best_position_to_insert_in_other_robot(uiLowerBound, uiUpperBound, uiSwapHoleInd, uiOtherRobot, rob_seq[uiOtherRobot], m_graph);

	bool bValid = string_relocate(rob_seq[uiRobot], std::make_pair(uiSwapHolePos, 1), uiRobot, rob_seq[uiOtherRobot], uiNewPos, uiOtherRobot, m_graph);
	if (false == bValid) return std::make_pair("RELOCATION_FAILED", std::numeric_limits<size_t>::max());

	return std::make_pair("SUCCESS", uiOtherRobot);
}

bool Local_Search::wait_based_move_intra_sequence(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, std::vector<std::list<size_t>> &rob_seq, size_t uiRobot, size_t uiWaitHoleInd, size_t uiWaitHolePos, std::string strWaitMove)
{
	size_t uiSwapHolePos;
	size_t uiSwapHoleInd;
	
	if ("NEXT_HOLE" == strWaitMove)
	{
		auto it = rob_seq[uiRobot].begin();
		uiSwapHolePos = uiWaitHolePos + 1;
		std::advance(it, uiSwapHolePos);
		uiSwapHoleInd = *it;
		if("H" != m_graph.getType(uiSwapHoleInd)) return false;
	}
	else if ("SAME_HOLE" == strWaitMove)
	{
		uiSwapHolePos = uiWaitHolePos;
		uiSwapHoleInd = uiWaitHoleInd;

#ifdef ENABLE_FULL_CHECKING
		auto it_check = rob_seq[uiRobot].begin();
		std::advance(it_check, uiWaitHolePos);
#ifdef WINDOWS
		assert(uiWaitHoleInd == *it_check);
#else
		if (uiWaitHoleInd != *it_check) 
		{
			cout << "Mismatch in schedule and sequence in wait_based_move_intra_sequence \n";
			exit(1);
		}
#endif
#endif

	}
	else return false;

#ifdef WINDOWS	
	assert(2 == m_node_data.m_uiNumRobots);
#else
	if (2 != m_node_data.m_uiNumRobots)
	{
		cout << "This heuristic is not designed for more than 2 robots yet";
		exit(1);
	}
#endif

	size_t uiLowerBound = (size_t)std::max((int)uiSwapHolePos - c_iWaitSwapRange, 1);
	if (1 == uiLowerBound)
	{
		if (false == m_graph.doesEdgeExist(uiRobot, m_node_data.m_rob_depo[uiRobot].first, uiSwapHoleInd)) uiLowerBound = 2;
	}

	size_t uiUpperBound = (size_t)std::min((int)uiSwapHolePos + c_iWaitSwapRange, (int)rob_seq[uiRobot].size() - 1);
	
	if (uiLowerBound >= uiUpperBound) return false;
	
	//std::uniform_int_distribution<size_t> unif_len(uiLowerBound, uiUpperBound);
	size_t uiNewPos;
	uiNewPos = get_best_position_to_insert_in_same_robot(uiLowerBound, uiUpperBound, uiSwapHoleInd, uiSwapHolePos, uiRobot, rob_seq[uiRobot], m_graph);

	/*while (1)
	{
		uiNewPos = unif_len(m_rng);
		if (uiNewPos != uiSwapHolePos) break;
	}*/

	auto it_remove = rob_seq[uiRobot].begin();
	std::advance(it_remove, uiSwapHolePos);

	auto it_remove_next = it_remove;
	it_remove_next++;

	auto it_insert = rob_seq[uiRobot].begin();
	std::advance(it_insert, uiNewPos);

	rob_seq[uiRobot].splice(it_insert, rob_seq[uiRobot], it_remove, it_remove_next);
	return true;
}