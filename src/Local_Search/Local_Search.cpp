#include "Local_Search.h"
#include <numeric>

void print_schedule(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch)
{
	for (size_t uiRobot = 0; uiRobot < full_rob_sch.size(); uiRobot++)
	{
		cout << "Robot: " << uiRobot << endl;
		for (auto it = full_rob_sch[uiRobot].begin(); it != full_rob_sch[uiRobot].end(); it++)
		{
			it->print_schedule();
			cout << "\n";
		}
		cout << "\n\n";
	}
}

size_t getMakeSpan_From_Schedule(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch)
{
	size_t uiMakeSpan = std::numeric_limits<size_t>::min();
	size_t uiRobot_MakeSpan;

	for (size_t uiRobot = 0; uiRobot < full_rob_sch.size(); uiRobot++)
	{
		uiRobot_MakeSpan = full_rob_sch[uiRobot][full_rob_sch[uiRobot].size() - 1].m_uiEnd;
		if (uiMakeSpan < uiRobot_MakeSpan) uiMakeSpan = uiRobot_MakeSpan;		
	}
	return uiMakeSpan;
}

Local_Search::Local_Search(const Node_Partitions &node_data, const Layout_LS &graph) :m_rng(m_rd()) , m_node_data(node_data) , m_graph(graph)
{
	srand(time(0));
}

std::tuple<bool, size_t, size_t> Local_Search::inter_rand_oper(std::vector<std::list<size_t>> &rob_seq, std::string strType)
{
	size_t uiRobot1, uiRobot2;
	bool bChange = false;
	std::vector<size_t> vec_robots(m_node_data.m_uiNumRobots);
	std::iota(vec_robots.begin(), vec_robots.end(), 0);
	std::shuffle(vec_robots.begin(), vec_robots.end(), m_rng);

	for (size_t uiCount1 = 0; uiCount1 < m_node_data.m_uiNumRobots - 1; uiCount1++)
	{
		uiRobot1 = vec_robots[uiCount1];
		for (size_t uiCount2 = uiCount1 + 1; uiCount2 < m_node_data.m_uiNumRobots; uiCount2++)
		{
			uiRobot2 = vec_robots[uiCount2];
			
			if ("STRING_EXCHANGE" == strType)
			{
				bChange = string_exchange(uiRobot1, uiRobot2, rob_seq);
			}
			else if ("STRING_RELOCATION" == strType)
			{
				bChange = string_relocation(uiRobot1, uiRobot2, rob_seq);
			}
			if (true == bChange) return std::make_tuple(true, uiRobot1 , uiRobot2);
		}		
	}
	return std::make_tuple(false , std::numeric_limits<size_t>::max(), std::numeric_limits<size_t>::max());
}

std::pair<bool, size_t> Local_Search::intra_rand_oper(std::vector<std::list<size_t>> &rob_seq, std::string strType)
{
	size_t uiRobot;
	bool bChange = false;
	std::vector<size_t> vec_robots(m_node_data.m_uiNumRobots);
	std::iota(vec_robots.begin(), vec_robots.end(), 0);
	std::shuffle(vec_robots.begin(), vec_robots.end(), m_rng);

	for (size_t uiCount = 0; uiCount < vec_robots.size(); uiRobot++)
	{
		uiRobot = vec_robots[uiCount];

		if ("SWAP_INTRA_SEQUENCE" == strType)
		{
			bChange = swap_intra_sequence(uiRobot , rob_seq);
		}
		else if ("STRING_CROSS_INTRA_SEQUENCE" == strType)
		{
			bChange = string_cross_intra_sequence(uiRobot, rob_seq , true);
		}
		if (true == bChange)
		{
			if (false == check_validity_of_sequence(rob_seq))
			{
				return std::make_pair(false, -1);
			}
			return std::make_pair(true, uiRobot);
		}
	}
	return std::make_pair(false , std::numeric_limits<size_t>::max());
}

std::tuple<bool, size_t, size_t> Local_Search::wait_based_oper(std::vector<std::list<size_t>> &rob_seq, const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, std::string strType)
{
	//<R_Ind, makespan>
	std::vector<std::pair<size_t, size_t>> vec_rob_ind_mkspan;

	for (size_t uiRobot = 0; uiRobot < m_node_data.m_uiNumRobots; uiRobot++)
	{
		vec_rob_ind_mkspan.emplace_back(std::make_pair(uiRobot, full_rob_sch[uiRobot].rbegin()->m_uiEnd));
	}

	std::sort(vec_rob_ind_mkspan.begin(), vec_rob_ind_mkspan.end(), sort_by_max_second_val());

	//iterate over all robots
	for(size_t uiCount = 0; uiCount < vec_rob_ind_mkspan.size(); uiCount++)
	{
		auto res = wait_based_swap_for_robot(full_rob_sch, rob_seq, vec_rob_ind_mkspan[uiCount].first, strType);
		if (true == std::get<0>(res)) return res;
	}
	return std::make_tuple(false, std::numeric_limits<size_t>::max(), std::numeric_limits<size_t>::max());
}

void Local_Search::perform_local_search(std::string strFolderPath)
{
	std::vector<std::list<size_t>> rob_seq;
	std::vector<std::list<size_t>> old_rob_seq;

	generate_constructive_sequence_VBSS(rob_seq);
	bool bValid = check_validity_of_sequence(rob_seq), bSuccess = true;
	if (false == bValid) { cout << "Initial seq generated is invalid\n"; }
	
	std::string strType;
	size_t uiIter = 0, uiMakeSpan, uiMakeSpan_old, uiBestSol = std::numeric_limits<size_t>::max();
	Power_Set power;
	bool bFirst_Feasible_Sequence = false;

	Greedy_Heuristic heur(m_node_data.m_uiNumRobots, m_graph, power);

#ifdef ENABLE_LEGACY_CODE
	Greedy_Heuristic_old heur_old(m_node_data.m_uiNumRobots, m_graph, power);
#endif

	std::vector<size_t> vec_late_accep(c_uiLate_Acceptace_Length, std::numeric_limits<size_t>::max());
	
	std::clock_t start_time;
	start_time = std::clock();

	while (uiIter < 5000000)
	{
		uiMakeSpan = std::numeric_limits<size_t>::max();
		uiMakeSpan_old = std::numeric_limits<size_t>::max();		
		
		bValid = check_validity_of_sequence(rob_seq);
		assert(true == bValid);

		std::vector<std::vector<Vertex_Schedule>> full_rob_sch;
		std::vector<std::vector<Vertex_Schedule>> full_rob_sch_old;

		int iRetVal = perform_greedy_scheduling(heur, rob_seq, full_rob_sch, strFolderPath);
#ifdef ENABLE_LEGACY_CODE			
		int iRetVal_old = perform_greedy_scheduling_old(heur_old, rob_seq, full_rob_sch_old);
#endif

#ifdef ENABLE_LEGACY_CODE
		if (iRetVal != iRetVal_old)
		{
			print_sequence(rob_seq);
#ifdef WINDOWS	
			assert(iRetVal == iRetVal_old);
#else 
			cout << "assert(iRetVal == iRetVal_old)";
			exit(-1);				
#endif		
		}
#endif

		uiMakeSpan = (iRetVal == 1) ? getMakeSpan_From_Schedule(full_rob_sch) : std::numeric_limits<size_t>::max();
#ifdef ENABLE_LEGACY_CODE			
		uiMakeSpan_old = (iRetVal_old == 1) ? getMakeSpan_From_Schedule(full_rob_sch_old) : std::numeric_limits<size_t>::max();
#endif			
		if (uiBestSol > uiMakeSpan)	uiBestSol = uiMakeSpan;	

#ifdef ENABLE_LEGACY_CODE
		if (uiMakeSpan != uiMakeSpan_old)
		{
			print_sequence(rob_seq);
#ifdef WINDOWS
			assert(uiMakeSpan == uiMakeSpan_old);
#else

			cout << "Makespans not equal";
			exit(1);
#endif		
		}
#endif
			
#ifdef ENABLE_LEGACY_CODE			
		cout << " Iteration: " << uiIter << " , " << (iRetVal == 1 ? "SUCCESS " : "UNSUCCESSFULL ") << " , Makespan: " << uiMakeSpan << " , Old Makespan: " << uiMakeSpan_old << endl;
#else			
		cout << " Iteration: " << uiIter <<" , " << (iRetVal == 1 ? "SUCCESS " : "UNSUCCESSFULL ") <<" , Makespan: " << uiMakeSpan <<" , Best Sol: "<< uiBestSol<< endl;
#endif			
			
		bSuccess = iRetVal == 1 ? true : false;			

		if (false == bFirst_Feasible_Sequence)
		{
			if (false == bSuccess)
			{
				rob_seq.clear();
				generate_constructive_sequence_VBSS(rob_seq);
				uiIter++;
				continue;
			}
			else
			{
				bFirst_Feasible_Sequence = true;
				cout << "Tag: Initial Makespan: " << uiMakeSpan << endl;
				std::fill(vec_late_accep.begin(), vec_late_accep.end(), uiMakeSpan);
				//old_rob_seq = rob_seq; // this is not required because copy is anyway happening below
			}
		}

		if (true == bSuccess)
		{
			if (uiMakeSpan > vec_late_accep[uiIter % c_uiLate_Acceptace_Length]) rob_seq = old_rob_seq;
			else
			{
				vec_late_accep[uiIter % c_uiLate_Acceptace_Length] = uiMakeSpan;
				old_rob_seq = rob_seq;
			}
		}
		else if (false == bSuccess) rob_seq = old_rob_seq;		
		
		generate_new_sequence(full_rob_sch, heur, rob_seq, bSuccess);
		uiIter++;	

		if (((std::clock() - start_time) / (double)CLOCKS_PER_SEC) > LS_SEARCH_TIME) break;
	}
	cout << "Tag: Best Makespan: " << uiBestSol << endl;
	cout<< "Tag: Total Iterations: " << uiIter << endl;
}

void Local_Search::convert_hole_seq_to_full_seq(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::list<size_t>> &full_rob_seq)
{
	full_rob_seq.resize(m_node_data.m_uiNumRobots);
	const auto &vec_rob_iv = m_graph.get_IV_Vec();

	for (size_t uiRobot = 0; uiRobot < m_node_data.m_uiNumRobots; uiRobot++)
	{
		auto it1 = rob_seq[uiRobot].begin();
		full_rob_seq[uiRobot].emplace_back(*it1);
		auto it2 = it1;
		it2++;

		for (; it2 != rob_seq[uiRobot].end(); it2++)
		{
			const auto &vec_iv = vec_rob_iv[uiRobot].map.at(*it1).map.at(*it2).vec;
			for (auto it_iv = vec_iv.begin(); it_iv != vec_iv.end(); it_iv++)
			{
				full_rob_seq[uiRobot].emplace_back(it_iv->getInd());
			}
			full_rob_seq[uiRobot].emplace_back(*it2);
			it1 = it2;
		}
	}
}

size_t Local_Search::get_bottleneck_robot(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch)
{
	size_t uiMaxMakeSpan = std::numeric_limits<size_t>::min();
	size_t uiBottle_Neck_Robot = 0;
	for (size_t uiRobot = 0; uiRobot < m_node_data.m_uiNumRobots; uiRobot++)
	{
		if (full_rob_sch[uiRobot].rbegin()->m_uiEnd >= uiMaxMakeSpan)
		{
			uiMaxMakeSpan = full_rob_sch[uiRobot].rbegin()->m_uiEnd;
			uiBottle_Neck_Robot = uiRobot;
		}
	}
	return uiBottle_Neck_Robot;
}

void Local_Search::get_Wait_Holes_For_Robot(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, size_t uiRobot, std::vector<std::pair<size_t, size_t>> &vec_wait_ind_pos)
{
	size_t uiPos = 0;
	assert(0 == vec_wait_ind_pos.size());

	for (size_t uiInd = 0; uiInd < full_rob_sch[uiRobot].size(); )
	{
		if("IV" == m_graph.getType(full_rob_sch[uiRobot][uiInd].m_uiInd)) continue;
		if (0 < full_rob_sch[uiRobot][uiInd].m_uiWait) vec_wait_ind_pos.emplace_back(std::make_pair(full_rob_sch[uiRobot][uiInd].m_uiInd, uiPos));
		uiPos++;
	}	
}

void Local_Search::generate_new_sequence(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, const Greedy_Heuristic &heur, std::vector<std::list<size_t>> &rob_seq, bool bSuccess)
{
	size_t uiChoice;
	std::string strType;
	bool bChange = false, bWait = false;
	
	if (true == bSuccess) bWait = heur.doRobotsWait();

	do
	{
		std::vector<std::list<size_t>> new_rob_seq = rob_seq;
		
		// if the sequence was feasible and waits were detected, then try to resolve via wait operations, else do random
		if( (true == bSuccess) && (true == bWait)) uiChoice = rand() % 5;
		else uiChoice = rand() % 2;

		if (0 == uiChoice)
		{
			strType = rand() % 2 ? "STRING_EXCHANGE" : "STRING_RELOCATION";
			auto res = inter_rand_oper(new_rob_seq, strType);

			if (true == std::get<0>(res))
			{
				rob_seq[std::get<1>(res)] = new_rob_seq[std::get<1>(res)];
				rob_seq[std::get<2>(res)] = new_rob_seq[std::get<2>(res)];
				bChange = true;
			}
		}
		else if(1 == uiChoice)
		{
			strType = rand() % 2 ? "SWAP_INTRA_SEQUENCE" : "STRING_CROSS_INTRA_SEQUENCE";
			auto res = intra_rand_oper(new_rob_seq, strType);
			if (true == std::get<0>(res))
			{
				rob_seq[std::get<1>(res)] = new_rob_seq[std::get<1>(res)];
				bChange = true;
			}
		}
		else
		{
			strType = rand() % 2 ? "INTER_SEQUENCE" : "INTRA_SEQUENCE";
			auto res = wait_based_oper(new_rob_seq, full_rob_sch, strType);

			if (true == std::get<0>(res))
			{
				rob_seq[std::get<1>(res)] = new_rob_seq[std::get<1>(res)];
				if (std::numeric_limits<size_t>::max() != std::get<2>(res))
				{
					rob_seq[std::get<2>(res)] = new_rob_seq[std::get<2>(res)];
				}
				bChange = true;
			}
		}
	} while (false == bChange);
}

int Local_Search::perform_greedy_scheduling(Greedy_Heuristic &heur, const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, std::string strFolderPath)
{
	std::vector<std::list<size_t>> full_rob_seq;
	convert_hole_seq_to_full_seq(rob_seq, full_rob_seq);
	//full_rob_seq.push_back({ 0,109,31,1626,52,2724,15,747,37,1942,44,2331,55,2889,18,917,45,2353,22,1112,24,1242,46,2391,6,232,8,353,21,1088,54,2827,10,473,33,1699,16,770,5,182,12,558,9,405,19,941,14,679,23,1167,25,1277,27,1376,17,830,11,520,26,1309,4,129,13,610,7,298,20,1008,28,1452,40,3127,1});
	//full_rob_seq.push_back({ 2,3179,86,7140,84,6998,72,6232,87,7171,50,4757,41,4196,66,5829,74,6343,67,5889,69,6015,64,5703,78,6624,89,7340,90,7354,38,4020,85,7039,48,4634,49,4694,43,4336,76,6468,62,5557,61,5482,51,4861,81,6813,83,6934,73,6287,77,6561,91,7463,82,6866,70,6079,63,5603,42,4248,53,4969,59,5340,39,4047,47,4551,30,3494,79,6676,75,6412,71,6146,65,5758,68,5973,88,7243,57,5228,58,5292,56,5165,60,5445,80,6695,29,3382,32,3579,34,3710,35,3776,36,7482,3});
	return heur.compute_greedy_sol(full_rob_seq, full_rob_sch, strFolderPath);
}

int Local_Search::perform_greedy_scheduling_old(Greedy_Heuristic_old &heur_old, const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &full_rob_sch)
{
	std::vector<std::list<size_t>> full_rob_seq;
	convert_hole_seq_to_full_seq(rob_seq, full_rob_seq);
	//full_rob_seq.push_back({ 0,109,31,1626,52,2724,15,747,37,1942,44,2331,55,2889,18,917,45,2353,22,1112,24,1242,46,2391,6,232,8,353,21,1088,54,2827,10,473,33,1699,16,770,5,182,12,558,9,405,19,941,14,679,23,1167,25,1277,27,1376,17,830,11,520,26,1309,4,129,13,610,7,298,20,1008,28,1452,40,3127,1 });
	//full_rob_seq.push_back({ 2,3179,86,7140,84,6998,72,6232,87,7171,50,4757,41,4196,66,5829,74,6343,67,5889,69,6015,64,5703,78,6624,89,7340,90,7354,38,4020,85,7039,48,4634,49,4694,43,4336,76,6468,62,5557,61,5482,51,4861,81,6813,83,6934,73,6287,77,6561,91,7463,82,6866,70,6079,63,5603,42,4248,53,4969,59,5340,39,4047,47,4551,30,3494,79,6676,75,6412,71,6146,65,5758,68,5973,88,7243,57,5228,58,5292,56,5165,60,5445,80,6695,29,3382,32,3579,34,3710,35,3776,36,7482,3 });
	return heur_old.compute_greedy_sol(full_rob_seq, full_rob_sch);
}


