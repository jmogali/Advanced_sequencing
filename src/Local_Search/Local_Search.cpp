#include "Local_Search.h"
#include <numeric>

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

Local_Search::Local_Search(const Node_Partitions &node_data, const Layout_LS &graph, const double dWeightFactor) :m_rng(m_rd()) , m_node_data(node_data) , m_graph(graph), m_dWeight_Factor(dWeightFactor), m_en_graph(graph)
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

	for (size_t uiCount = 0; uiCount < vec_robots.size(); uiCount++)
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
		
		if (true == bChange) return std::make_pair(true, uiRobot);
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

void Local_Search::perform_VBSS_search(std::string strFolderPath)
{
	std::clock_t start_time;
	start_time = std::clock();
	size_t uiIter = 0, uiMakeSpan,uiBestSol = std::numeric_limits<size_t>::max(), uiSuccesFullIter = 0;
	Power_Set power;
	Greedy_Heuristic heur(m_node_data.m_uiNumRobots, m_graph, power);

	while (uiIter < 5000000)
	{
		std::vector<std::list<size_t>> rob_seq;
		std::vector<std::vector<Vertex_Schedule>> full_rob_sch;

		generate_constructive_sequence_VBSS(rob_seq);
		bool bValid = check_validity_of_sequence(rob_seq), bSuccess = true;
		if (false == bValid) 
		{ 
			cout << "Initial seq generated is invalid\n"; 
			uiIter++;
			continue;
		}

		int iRetVal = perform_greedy_scheduling(heur, rob_seq, full_rob_sch, strFolderPath, (size_t)(1.25 * uiBestSol));
		uiMakeSpan = (iRetVal == 1) ? getMakeSpan_From_Schedule(full_rob_sch) : std::numeric_limits<size_t>::max();

		cout << " Iteration: " << uiIter << " , " << (iRetVal == 1 ? "SUCCESS " : "UNSUCCESSFULL ") << " , Makespan: " << uiMakeSpan << " , Best Sol: " << uiBestSol << endl;

		if (uiBestSol > uiMakeSpan)	uiBestSol = uiMakeSpan;
		
		bSuccess = iRetVal == 1 ? true : false;
		if (true == bSuccess) uiSuccesFullIter++;

		uiIter++;
		if (((std::clock() - start_time) / (double)CLOCKS_PER_SEC) > LS_SEARCH_TIME) break;
	}

	cout << "Tag: Best Makespan: " << uiBestSol << endl;
	cout << "Tag: Total Iterations: " << uiIter << endl;
	cout << "Tag: Successfull iterations: " << uiSuccesFullIter << endl;
	double dSuccPercent = (double)(100.0 * uiSuccesFullIter) / ((double)(uiIter * 1.0));
	cout << "Tag: Success %: " << dSuccPercent << endl; 
	cout << "Tag: Accumulated Results: " << uiBestSol << ","<< uiSuccesFullIter << ","<< dSuccPercent <<endl;
}

void Local_Search::perform_local_search(std::string strPlotFolder, std::string strDataDumpFolder, std::string strTSPFolder, size_t ui_KVal)
{
	std::vector<std::list<size_t>> rob_seq;
	std::vector<std::list<size_t>> old_rob_seq;
	bool bConservativeFound, bTSP;
	std::vector<std::vector<Vertex_Schedule>> full_rob_sch_prev;
	
	generate_constructive_sequence_VBSS(rob_seq);
	bool bValid = check_validity_of_sequence(rob_seq), bSuccess = true;
	if (false == bValid) { cout << "Initial seq generated is invalid\n"; }
	
	std::string strType;
	size_t uiIter = 0, uiMakeSpan, uiMakeSpan_legacy, uiBestSol = std::numeric_limits<size_t>::max(), uiConstructiveMakespan;
	size_t uiStaleCounter; //records number of non improving moves in objective
	Power_Set power;
	bool bFirst_Feasible_Sequence = false;

#ifdef DATA_DUMP_ENABLE
	size_t uiDataDumpIndex = 0, uiFeasIter = 0 , uiInfeasIter = 0;
	std::string strDataDump_Feasible = strDataDumpFolder + "FEASIBLE";
	std::string strDataDump_Infeasible = strDataDumpFolder + "INFEASIBLE";
#ifdef WINDOWS
	_mkdir(strDataDumpFolder.c_str());
	_mkdir(strDataDump_Feasible.c_str());
	_mkdir(strDataDump_Infeasible.c_str());
#else
	mkdir(strDataDumpFolder.c_str(), S_IRWXU);
	mkdir(strDataDump_Feasible.c_str(), S_IRWXU);
	mkdir(strDataDump_Infeasible.c_str(), S_IRWXU);
#endif
#endif

	Greedy_Heuristic heur(m_node_data.m_uiNumRobots, m_graph, power);
	//LS_Greedy_Heuristic heur_LS(m_node_data.m_uiNumRobots, m_graph, power);
	Hole_Exchange hole_exchange(m_node_data.m_uiNumRobots, m_graph, power, m_en_graph);

#ifdef ENABLE_LEGACY_CODE
	Greedy_Heuristic_old heur_legacy(m_node_data.m_uiNumRobots, m_graph, power);
#endif

	std::vector<size_t> vec_late_accep(c_uiLate_Acceptace_Length, std::numeric_limits<size_t>::max());
	size_t uiSuccesFullIter = 0;
	
	std::clock_t start_time;
	start_time = std::clock();
		
	while (uiIter < 5000000)
	{
		uiMakeSpan = std::numeric_limits<size_t>::max();
		uiMakeSpan_legacy = std::numeric_limits<size_t>::max();
		bConservativeFound = false;
		bTSP = false;
		
		bValid = check_validity_of_sequence(rob_seq);
#ifdef WINDOWS		
		assert(true == bValid);
#else
		if (false == bValid)
		{
			cout << "Sequence generated is invalid \n";
			print_sequence(rob_seq);
			exit(1);
		}
#endif

		std::vector<std::vector<Vertex_Schedule>> full_rob_sch;
		std::vector<std::vector<Vertex_Schedule>> full_rob_sch_legacy;

		/*rob_seq.clear();
		rob_seq.push_back({ 0,11,4,5,41,20,42,14,15,13,10,23,34,6,7,26,28,39,19,17,18,24,35,8,45,40,25,38,21,31,32,29,22,12,46,30,58,36,43,44,16,9,47,37,27,1 });
		rob_seq.push_back({ 2,86,66,69,89,84,82,90,33,64,73,59,87,71,48,49,74,51,52,70,91,81,68,56,67,77,79,55,83,63,85,50,53,88,57,78,80,65,76,62,61,75,72,60,54,3 });*/

		int iRetVal = perform_greedy_scheduling(heur, rob_seq, full_rob_sch, strPlotFolder);
		
#ifdef ENABLE_LEGACY_CODE			
		int iRetVal_legacy = perform_greedy_scheduling_old(heur_legacy, rob_seq, full_rob_sch_legacy);
#endif

#ifdef ENABLE_LEGACY_CODE
		if (iRetVal != iRetVal_legacy)
		{
			print_sequence(rob_seq);
#ifdef WINDOWS	
			assert(iRetVal == iRetVal_legacy);
#else 
			cout << "assert(iRetVal == iRetVal_legacy)";
			exit(-1);				
#endif		
		}
#endif

		uiMakeSpan = (iRetVal == 1) ? getMakeSpan_From_Schedule(full_rob_sch) : std::numeric_limits<size_t>::max();
#ifdef ENABLE_LEGACY_CODE			
		uiMakeSpan_legacy = (iRetVal_legacy == 1) ? getMakeSpan_From_Schedule(full_rob_sch_legacy) : std::numeric_limits<size_t>::max();
#endif			
		if (uiBestSol > uiMakeSpan)	uiBestSol = uiMakeSpan;	

#ifdef ENABLE_LEGACY_CODE
		if (uiMakeSpan != uiMakeSpan_legacy)
		{
			print_sequence(rob_seq);
#ifdef WINDOWS
			assert(uiMakeSpan == uiMakeSpan_legacy);
#else

			cout << "Makespans not equal";
			exit(1);
#endif		
		}
#endif
			
#ifdef ENABLE_LEGACY_CODE			
		cout << " Iteration: " << uiIter << " , " << (iRetVal == 1 ? "SUCCESS " : "UNSUCCESSFULL ") << " , Makespan: " << uiMakeSpan << " , Old Makespan: " << uiMakeSpan_legacy << endl;
#else			
		cout << " Iteration: " << uiIter <<" , " << (iRetVal == 1 ? "SUCCESS " : "UNSUCCESSFULL ") <<" , Makespan: " << uiMakeSpan <<" , Best Sol: "<< uiBestSol<< endl;
#endif			
			
		bSuccess = iRetVal == 1 ? true : false;		

#ifdef DATA_DUMP_ENABLE
		if (true == bSuccess)
		{
			std::string strFile = "Sequence_" + std::to_string(uiFeasIter) + ".txt";
			dump_data_to_file(rob_seq, full_rob_sch, strDataDump_Feasible, strFile, true);
			uiFeasIter++;
		}
		else
		{
			std::string strFile = "Sequence_" + std::to_string(uiInfeasIter) + ".txt";
			dump_data_to_file(rob_seq, full_rob_sch, strDataDump_Infeasible, strFile, false);
			uiInfeasIter++;
		}
#endif

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
				uiConstructiveMakespan = uiMakeSpan;
				std::fill(vec_late_accep.begin(), vec_late_accep.end(), uiMakeSpan);				
			}
		}

		if (true == bSuccess)
		{
			print_sequence(rob_seq);
			
			if (uiIter % 3 == 0)
			{
				gen_seq_TSP(strTSPFolder, heur, full_rob_sch, rob_seq, uiMakeSpan, ui_KVal);
				bTSP = true;
			}
			else
			{
				bConservativeFound = gen_seq_hole_exchange(hole_exchange, heur, full_rob_sch, rob_seq, uiMakeSpan);
			}
		}

		if (true == bSuccess)
		{
			if (uiMakeSpan > vec_late_accep[uiIter % c_uiLate_Acceptace_Length])
			{
				rob_seq = old_rob_seq;
				full_rob_sch = full_rob_sch_prev;
				uiStaleCounter++;
			}
			else
			{
				vec_late_accep[uiIter % c_uiLate_Acceptace_Length] = uiMakeSpan;
				old_rob_seq = rob_seq;
				full_rob_sch_prev = full_rob_sch;	
				uiStaleCounter = 0;
			}
			uiSuccesFullIter++;
		}
		else if (false == bSuccess)
		{
			rob_seq = old_rob_seq;
			full_rob_sch = full_rob_sch_prev;
		}
		
		if ( ((false == bConservativeFound) && (false == bTSP)) ||(uiStaleCounter > 5))
		{
			rob_seq.clear();
			generate_constructive_sequence_VBSS(rob_seq);
			uiStaleCounter = 0;
		}
		
		uiIter++;	

		if (((std::clock() - start_time) / (double)CLOCKS_PER_SEC) > LS_SEARCH_TIME) break;
	}
	cout << "Tag: Best Makespan: " << uiBestSol << endl;
	cout<< "Tag: Total Iterations: " << uiIter << endl;
	cout << "Tag: Successfull iterations: " << uiSuccesFullIter << endl;
	double dSuccPercent = (double)(100.0 * uiSuccesFullIter)/((double)(uiIter * 1.0));
	cout<< "Tag: Success %: " << dSuccPercent << endl; 
	cout<< "Tag: Accumulated Result: "<< uiConstructiveMakespan << "," << uiBestSol<< "," << uiSuccesFullIter<< "," << dSuccPercent <<endl;
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

void Local_Search::generate_new_sequence(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, const bool c_bWait, std::vector<std::list<size_t>> &rob_seq, bool bSuccess)
{
	size_t uiChoice;
	std::string strType;
	bool bChange = false, bWait = false;
	
	if (true == bSuccess) bWait = c_bWait;

	do
	{
		std::vector<std::list<size_t>> new_rob_seq = rob_seq;
		
		// if the sequence was feasible and waits were detected, then try to resolve via wait operations, else do random
		if( (true == bSuccess) && (true == bWait)) uiChoice = rand() % 5;
		else uiChoice = rand() % 2;
		
#ifdef PRINT_LOCAL_OPERATOR_MESSAGES
		cout << "Local Search Choice: " << uiChoice << endl;
#endif
		
		if (0 == uiChoice)
		{
			strType = rand() % 2 ? "STRING_EXCHANGE" : "STRING_RELOCATION";

#ifdef PRINT_LOCAL_OPERATOR_MESSAGES			
			cout << "String Type: " << strType << endl;
#endif
			
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
			
#ifdef PRINT_LOCAL_OPERATOR_MESSAGES
			cout << "String Type: " << strType << endl;
#endif
			
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
			
#ifdef PRINT_LOCAL_OPERATOR_MESSAGES			
			cout << "String Type: " << strType << endl;
#endif
			
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

int Local_Search::perform_greedy_scheduling(Greedy_Heuristic &heur, std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, std::string strPlotFolder, const size_t c_uiUpperBound)
{
	std::vector<std::list<size_t>> full_rob_seq;
	
	/*rob_seq.clear();
	rob_seq.push_back({ 0,44,41,40,37,53,33,31,39,50,21,35,11,32,20,19,10,16,24,15,26,4,38,5,22,9,29,12,23,42,27,30,36,8,18,7,6,25,17,13,45,47,14,1 });
	rob_seq.push_back({ 2,28,74,48,62,85,70,51,63,73,61,55,43,34,46,56,81,66,58,68,83,65,75,89,60,49,78,86,57,69,79,71,87,84,88,82,90,64,72,52,76,67,80,77,59,91,54,3 });*/
	
	convert_hole_seq_to_full_seq(rob_seq, full_rob_seq);
	return heur.compute_greedy_sol(full_rob_seq, full_rob_sch, strPlotFolder);
}

int Local_Search::perform_greedy_scheduling_old(Greedy_Heuristic_old &heur_old, std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &full_rob_sch)
{
	std::vector<std::list<size_t>> full_rob_seq;

	/*rob_seq.clear();
	rob_seq.push_back({ 0,44,41,40,37,53,33,31,39,50,21,35,11,32,20,19,10,16,24,15,26,4,38,5,22,9,29,12,23,42,27,30,36,8,18,7,6,25,17,13,45,47,14,1 });
	rob_seq.push_back({ 2,28,74,48,62,85,70,51,63,73,61,55,43,34,46,56,81,66,58,68,83,65,75,89,60,49,78,86,57,69,79,71,87,84,88,82,90,64,72,52,76,67,80,77,59,91,54,3 });*/

	convert_hole_seq_to_full_seq(rob_seq, full_rob_seq);
	return heur_old.compute_greedy_sol(full_rob_seq, full_rob_sch);
}



