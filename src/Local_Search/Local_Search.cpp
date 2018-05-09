#include "Local_Search.h"

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
	srand((unsigned)time(0));
}

void Local_Search::perform_VBSS_search(std::string strFolderPath)
{
	std::clock_t start_time;
	start_time = std::clock();
	size_t uiIter = 0, uiMakeSpan,uiBestSol = std::numeric_limits<size_t>::max(), uiUpperBoundFilter = std::numeric_limits<size_t>::max(), uiSuccesFullIter = 0;
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

		int iRetVal = perform_greedy_scheduling(heur, rob_seq, full_rob_sch, strFolderPath, uiUpperBoundFilter);
		uiMakeSpan = (iRetVal == 1) ? getMakeSpan_From_Schedule(full_rob_sch) : std::numeric_limits<size_t>::max();

		cout << " Iteration: " << uiIter << " , " << (iRetVal == 1 ? "SUCCESS " : "UNSUCCESSFULL ") << " , Makespan: " << uiMakeSpan << " , Best Sol: " << uiBestSol << endl;

		if (uiBestSol > uiMakeSpan)
		{
			uiBestSol = uiMakeSpan;
			uiUpperBoundFilter = (size_t)(c_dUpperBoundFilterConstant * uiBestSol);			
		}
		
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
	bool bPrintFirstSol = false, bTSP, bRandGen;
	std::vector<std::vector<Vertex_Schedule>> full_rob_sch_prev;
	std::vector<std::vector<Vertex_Schedule>> full_rob_sch_best;
	std::vector<std::vector<Vertex_Schedule>> full_rob_sch_print_first;
	std::vector<std::pair<size_t, double>> vec_impr_sol;
	
	generate_constructive_sequence_VBSS(rob_seq);
	bool bValid = check_validity_of_sequence(rob_seq), bSuccess = true;
	if (false == bValid) 
	{
		cout << "Initial seq generated is invalid\n"; 
		exit(-1);
	}

	
	std::string strType;
	size_t uiIter = 0, uiMakeSpan, uiMakeSpan_legacy, uiFirstSol = std::numeric_limits<size_t>::max(), uiBestSol = std::numeric_limits<size_t>::max(), uiConstructiveMakespan = std::numeric_limits<size_t>::max(), uiUpperBoundFilter = std::numeric_limits<size_t>::max();
	size_t uiStaleCounter = 0, uiTSPLowerBound , uiTSPMkSpan, uiNumRestart = 0; //records number of non improving moves in objective
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
		if (((std::clock() - start_time) / (double)CLOCKS_PER_SEC) > LS_SEARCH_TIME) break;

		uiMakeSpan = std::numeric_limits<size_t>::max();
		uiMakeSpan_legacy = std::numeric_limits<size_t>::max();
				
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
		rob_seq.push_back({ 0,35,37,36,39,38,41,40,43,42,45,44,47,46,49,48,12,10,11,8,9,6,7,4,5,34,33,32,31,30,29,28,27,14,13,16,15,18,17,20,19,22,21,24,23,26,25,1 });
		rob_seq.push_back({ 2,81,83,82,85,84,87,86,89,88,91,90,93,92,95,94,80,79,78,77,76,75,74,73,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,56,57,54,55,52,53,50,51,3 });*/
		
		int iRetVal = perform_greedy_scheduling(heur, rob_seq, full_rob_sch, strPlotFolder, uiUpperBoundFilter);
		
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
		cout << " Iteration: " << uiIter <<" , " << (iRetVal == 1 ? "SUCCESS " : "UNSUCCESSFULL ") <<" , Makespan: " << uiMakeSpan <<" , Best Sol: "<< uiBestSol <<" , Comparison cost: "<<vec_late_accep[uiIter % c_uiLate_Acceptace_Length] << endl;
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
				if (false == bPrintFirstSol)
				{
					bPrintFirstSol = true;
					full_rob_sch_print_first = full_rob_sch;
					cout << "Tag: Initial Makespan: " << uiMakeSpan << endl;
					uiFirstSol = uiMakeSpan;
				}				
				uiConstructiveMakespan = std::min(uiMakeSpan , uiConstructiveMakespan);
				std::fill(vec_late_accep.begin(), vec_late_accep.end(), uiMakeSpan);				
			}
		}

		if (true == bSuccess)
		{
			//print_sequence(rob_seq);
			
			if (uiIter % 3 == 0)
			{
				const auto rob_seq_before_TSP = rob_seq;
				const auto full_rob_sch_before_TSP = full_rob_sch;
				
				gen_seq_TSP(strTSPFolder, heur, full_rob_sch, rob_seq, uiTSPLowerBound, ui_KVal);				

				full_rob_sch.clear();
				iRetVal = perform_greedy_scheduling(heur, rob_seq, full_rob_sch, strPlotFolder, uiMakeSpan);

				if (iRetVal == 1)
				{
					uiTSPMkSpan = getMakeSpan_From_Schedule(full_rob_sch);
					cout << "TSP MakeSpan: " << uiTSPMkSpan << endl;
					if (uiTSPMkSpan <= uiMakeSpan) uiMakeSpan = uiTSPMkSpan;
					else
					{
						rob_seq = rob_seq_before_TSP;
						full_rob_sch = full_rob_sch_before_TSP;
					}					
				}
				else
				{
					rob_seq = rob_seq_before_TSP;
					full_rob_sch = full_rob_sch_before_TSP;
				}
				bTSP = true;
			}
			else
			{
				gen_seq_hole_exchange(hole_exchange, heur, full_rob_sch, rob_seq, uiMakeSpan);
				bTSP = false;
			}
		}

		if (true == bSuccess)
		{
			if (uiBestSol > uiMakeSpan)
			{
				uiBestSol = uiMakeSpan;
				double dTimeComp = ((std::clock() - start_time) / (double)CLOCKS_PER_SEC);
				double dPercTimeComp = ((double)dTimeComp * 100 / LS_SEARCH_TIME);
				vec_impr_sol.push_back(std::make_pair(uiBestSol, dPercTimeComp));
				uiUpperBoundFilter = (size_t)(c_dUpperBoundFilterConstant * uiBestSol);
				full_rob_sch_best = full_rob_sch;
			}

			if (uiMakeSpan >= vec_late_accep[uiIter % c_uiLate_Acceptace_Length])
			{				
				if (false == old_rob_seq.empty())
				{
					rob_seq = old_rob_seq;
					full_rob_sch = full_rob_sch_prev;
					cout << "Reverting sequence" << endl;
					bRandGen = true;
					uiStaleCounter++;
				}
				else bRandGen = false;
			}
			else
			{
				vec_late_accep[uiIter % c_uiLate_Acceptace_Length] = uiMakeSpan;

				if (rob_seq == old_rob_seq) bRandGen = true;
				else
				{
					old_rob_seq = rob_seq;
					full_rob_sch_prev = full_rob_sch;
					cout << "Accepting sequence" << endl;
					if(bTSP) bRandGen = true; //when TSP is true, we already have the greedy schedule, so we need to perturb for next iteration
					else bRandGen = false;
				}
				uiStaleCounter = 0;
			}
			uiSuccesFullIter++;
		}
		else if (false == bSuccess)
		{
			rob_seq = old_rob_seq;
			full_rob_sch = full_rob_sch_prev;
			uiStaleCounter++;
			bRandGen = true;
		}
		
		if ( uiStaleCounter > c_uiConsecutive_Late_Accep_Failure )
		{
			rob_seq.clear();
			generate_constructive_sequence_VBSS(rob_seq);
			uiStaleCounter = 0;
			bFirst_Feasible_Sequence = false;
			uiNumRestart++;
		}
		else if (true == bRandGen)
		{
#ifdef WINDOWS
			assert(false == rob_seq.empty());
#else
			if (true == rob_seq.empty())
			{
				cout << "Empty sequence fed to random move operator\n";
				exit(-1);
			}
#endif
			generate_new_sequence_rand_moves(rob_seq);
		}
		
		uiIter++;		
	}

	free_VLNS_buffers();
	print_state_transition_path(strPlotFolder +"Best_Sol.txt" , full_rob_sch_best );
	print_state_transition_path(strPlotFolder + "Initial_Sol.txt", full_rob_sch_print_first);
	print_best_solution_progress(strPlotFolder + "Solution_Progress.txt", vec_impr_sol);
	cout << "Tag: Initial Makespan: " << uiFirstSol << endl;
	cout << "Tag: Best Makespan: " << uiBestSol << endl;
	cout<< "Tag: Total Iterations: " << uiIter << endl;
	cout << "Tag: Number Of Restarts: " << uiNumRestart << endl;
	cout << "Tag: Successfull iterations: " << uiSuccesFullIter << endl;
	double dSuccPercent = (double)(100.0 * uiSuccesFullIter)/((double)(uiIter * 1.0));
	cout<< "Tag: Success %: " << dSuccPercent << endl; 
	cout<< "Tag: Accumulated Result: "<< uiFirstSol <<","<<uiConstructiveMakespan << "," << uiBestSol<< "," << uiSuccesFullIter<< "," << dSuccPercent <<endl;
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

int Local_Search::perform_greedy_scheduling(Greedy_Heuristic &heur, std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, std::string strPlotFolder, const size_t c_uiUpperBound)
{
	std::vector<std::list<size_t>> full_rob_seq;
	
	convert_hole_seq_to_full_seq(rob_seq, full_rob_seq);
	return heur.compute_greedy_sol(full_rob_seq, full_rob_sch, strPlotFolder, c_uiUpperBound);
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



