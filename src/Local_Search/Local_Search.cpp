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
	generate_constructive_sequence(rob_seq);
	bool bValid = check_validity_of_sequence(rob_seq), bChange, bSuccess = true;
	if (false == bValid) { cout << "Initial seq generated is invalid\n"; }

	std::string strType;
	size_t uiIter = 0, uiChoice, uiMakeSpan, uiMakeSpan_old, uiBestSol = std::numeric_limits<size_t>::max();
	Power_Set power;
	Greedy_Heuristic heur(m_node_data.m_uiNumRobots, m_graph, power);
	Greedy_Heuristic_old heur_old(m_node_data.m_uiNumRobots, m_graph, power);
	std::vector<size_t> vec_late_accep(c_uiLate_Acceptace_Length, std::numeric_limits<size_t>::max());
	
	std::clock_t start_time;
	start_time = std::clock();

	while (uiIter < 5000000)
	{
		std::vector<std::list<size_t>> new_rob_seq = rob_seq;
		std::vector<std::list<size_t>> old_rob_seq = rob_seq;

		uiMakeSpan = std::numeric_limits<size_t>::max();
		uiMakeSpan_old = std::numeric_limits<size_t>::max();
		bChange = uiIter== 0 ? true : false;
		uiChoice = rand() % 2;
		
		if (uiIter > 0)
		{
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
		}

		if (bChange)
		{
			check_validity_of_sequence(rob_seq);
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
				if (true == bSuccess)
					std::fill(vec_late_accep.begin(), vec_late_accep.end(), uiMakeSpan);
			}
			else
			{
				if (true == bSuccess)
				{
					if (uiMakeSpan > vec_late_accep[uiIter % c_uiLate_Acceptace_Length]) rob_seq = old_rob_seq;
					else vec_late_accep[uiIter % c_uiLate_Acceptace_Length] = uiMakeSpan;
				}
				else if (false == bSuccess) rob_seq = old_rob_seq;
			}
		}			
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

int Local_Search::perform_greedy_scheduling(Greedy_Heuristic &heur, const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, std::string strFolderPath)
{
	std::vector<std::list<size_t>> full_rob_seq;
	convert_hole_seq_to_full_seq(rob_seq, full_rob_seq);
	//full_rob_seq.push_back({ 0,115,44,2322,46,2431,47,2482,43,2263,40,2099,38,1997,45,2353,22,1111,23,1167,25,1275,24,1205,8,338,5,182,12,562,14,670,13,620,18,889,16,771,6,266,42,2190,21,1042,7,319,41,2145,30,1577,57,3011,32,1662,34,1772,36,1880,35,1806,15,741,31,1603,28,1451,39,2044,37,1918,19,952,26,1337,33,1694,11,511,17,828,9,391,4,126,10,460,20,3107,1 });
	//full_rob_seq.push_back({ 2,3175,81,6812,82,6858,62,5556,60,5435,70,6086,71,6129,48,4634,49,4701,51,4834,54,5033,58,5292,56,5133,27,3249,29,3438,88,7274,89,7341,91,7471,90,7375,59,5375,75,6419,79,6679,78,6615,80,6727,61,5514,84,7000,74,6329,53,4975,65,5757,67,5889,69,6019,68,5950,64,5697,72,6230,85,7046,55,5091,50,4801,86,7139,83,6927,66,5842,87,7173,52,4918,73,6286,76,6482,77,6534,63,7509,3 });
	return heur.compute_greedy_sol(full_rob_seq, full_rob_sch, strFolderPath);
}

int Local_Search::perform_greedy_scheduling_old(Greedy_Heuristic_old &heur_old, const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &full_rob_sch)
{
	std::vector<std::list<size_t>> full_rob_seq;
	convert_hole_seq_to_full_seq(rob_seq, full_rob_seq);
	//full_rob_seq.push_back({ 0,115,44,2322,46,2431,47,2482,43,2263,40,2099,38,1997,45,2353,22,1111,23,1167,25,1275,24,1205,8,338,5,182,12,562,14,670,13,620,18,889,16,771,6,266,42,2190,21,1042,7,319,41,2145,30,1577,57,3011,32,1662,34,1772,36,1880,35,1806,15,741,31,1603,28,1451,39,2044,37,1918,19,952,26,1337,33,1694,11,511,17,828,9,391,4,126,10,460,20,3107,1 });
	//full_rob_seq.push_back({ 2,3175,81,6812,82,6858,62,5556,60,5435,70,6086,71,6129,48,4634,49,4701,51,4834,54,5033,58,5292,56,5133,27,3249,29,3438,88,7274,89,7341,91,7471,90,7375,59,5375,75,6419,79,6679,78,6615,80,6727,61,5514,84,7000,74,6329,53,4975,65,5757,67,5889,69,6019,68,5950,64,5697,72,6230,85,7046,55,5091,50,4801,86,7139,83,6927,66,5842,87,7173,52,4918,73,6286,76,6482,77,6534,63,7509,3 });
	return heur_old.compute_greedy_sol(full_rob_seq, full_rob_sch);
}


