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
	return std::make_tuple(false , -1, -1);
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

	return std::make_pair(false , -1);
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

		if (uiIter == 0)
		{
			cout << "Tag: Initial Makespan: " << uiMakeSpan << endl;
			std::fill(vec_late_accep.begin(), vec_late_accep.end(), uiMakeSpan);
			old_rob_seq = rob_seq;
		}
		else
		{
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
		}
		
		generate_new_sequence(full_rob_sch, heur, rob_seq);
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

void Local_Search::generate_new_sequence(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, const Greedy_Heuristic &heur, std::vector<std::list<size_t>> &rob_seq)
{
	size_t uiChoice;
	std::string strType;
	bool bChange = false;
	
	do
	{
		std::vector<std::list<size_t>> new_rob_seq = rob_seq;
		uiChoice = rand() % 2;

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
		else
		{
			strType = rand() % 2 ? "SWAP_INTRA_SEQUENCE" : "STRING_CROSS_INTRA_SEQUENCE";
			auto res = intra_rand_oper(rob_seq, strType);
			if (true == std::get<0>(res))
			{
				rob_seq[std::get<1>(res)] = new_rob_seq[std::get<1>(res)];
				bChange = true;
			}
		}
	} while (false == bChange);
}

int Local_Search::perform_greedy_scheduling(Greedy_Heuristic &heur, const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, std::string strFolderPath)
{
	std::vector<std::list<size_t>> full_rob_seq;
	convert_hole_seq_to_full_seq(rob_seq, full_rob_seq);
	//full_rob_seq.push_back({ 0,111,37,1936,38,1992,40,2103,43,2269,47,2484,45,2381,51,2708,54,2838,21,1059,25,1274,23,1166,24,1205,8,338,5,182,12,561,13,616,14,668,11,533,39,2033,26,1331,27,1387,29,1498,32,1664,36,1879,34,1771,35,1819,28,1435,22,1107,18,889,16,774,9,417,31,1615,41,2125,10,455,15,729,19,933,6,241,17,848,30,1562,42,2202,33,1690,7,283,4,136,20,1024,44,3131,1 });
	//full_rob_seq.push_back({ 2,3156,48,4634,49,4697,46,4511,56,5163,58,5293,57,5247,77,6550,80,6745,79,6679,78,6616,81,6812,82,6880,85,7079,89,7341,91,7471,90,7404,88,7260,74,6347,71,6134,53,4972,62,5556,60,5452,87,7173,52,4920,75,6404,63,5627,67,5888,68,5954,69,6010,59,5383,83,6911,50,4787,72,6216,70,6099,84,6990,64,5681,55,5126,86,7129,73,6272,61,5496,66,5821,65,5766,76,7522,3});
	return heur.compute_greedy_sol(full_rob_seq, full_rob_sch, strFolderPath);
}

int Local_Search::perform_greedy_scheduling_old(Greedy_Heuristic_old &heur_old, const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &full_rob_sch)
{
	std::vector<std::list<size_t>> full_rob_seq;
	convert_hole_seq_to_full_seq(rob_seq, full_rob_seq);
	//full_rob_seq.push_back({ 0,111,37,1936,38,1992,40,2103,43,2269,47,2484,45,2381,51,2708,54,2838,21,1059,25,1274,23,1166,24,1205,8,338,5,182,12,561,13,616,14,668,11,533,39,2033,26,1331,27,1387,29,1498,32,1664,36,1879,34,1771,35,1819,28,1435,22,1107,18,889,16,774,9,417,31,1615,41,2125,10,455,15,729,19,933,6,241,17,848,30,1562,42,2202,33,1690,7,283,4,136,20,1024,44,3131,1 });
	//full_rob_seq.push_back({ 2,3156,48,4634,49,4697,46,4511,56,5163,58,5293,57,5247,77,6550,80,6745,79,6679,78,6616,81,6812,82,6880,85,7079,89,7341,91,7471,90,7404,88,7260,74,6347,71,6134,53,4972,62,5556,60,5452,87,7173,52,4920,75,6404,63,5627,67,5888,68,5954,69,6010,59,5383,83,6911,50,4787,72,6216,70,6099,84,6990,64,5681,55,5126,86,7129,73,6272,61,5496,66,5821,65,5766,76,7522,3 });
	return heur_old.compute_greedy_sol(full_rob_seq, full_rob_sch);
}


