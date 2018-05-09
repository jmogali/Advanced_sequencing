// Boeing-Complete.cpp : Defines the entry point for the console application.
//

#include "Windows_Linux.h"
#ifdef WINDOWS 
#include "stdafx.h"
#endif
#include "Layout_utils.h"
#include "Robot.h"
#include "Boeing_Fuesalage.h"
#include "Data_Generator.h"
#include <sstream> // stringstream
#include "Data_Parser.h"
#include "Layout_Graph.h"
#include "Node_Partitions.h"
#include "Powerset.h"
#include <random>
#include "Local_Search.h"
#include "Kosaraju_Algo.h"
#include "Alternative_Graph.h"
#include "Joris_Sequence_File_Parser.h"
#include "Schedule_Validity_Check.h"
#include "auxgraph.h"

/*
#ifdef __cplusplus
extern "C" {
#endif	
#include "auxgraph.h"
#ifdef __cplusplus
}
#endif
*/

using namespace boost;
using namespace std;
using namespace AUX_GRAPH;

std::string getFolderName(size_t uiFrames, size_t uiRobots, double dWidth, double dHeight, double dHorSpace, double dVertSpace)
{
	stringstream stream_W;
	stream_W << fixed << setprecision(0) << dWidth;
	stringstream stream_H;
	stream_H << fixed << setprecision(0) << dHeight;
	stringstream stream_R;
	stream_R<< uiRobots;
	stringstream stream_F;
	stream_F << uiFrames;
	stringstream stream_HS;
	stream_HS << fixed << setprecision(2) << dHorSpace;
	stringstream stream_VS;
	stream_VS << fixed << setprecision(2) << dVertSpace;

	std::string strFolderName = stream_W.str() + "_" + stream_H.str() + "_" + stream_R.str() + "_";
	strFolderName += stream_F.str() + "_" + stream_HS.str() +"_" + stream_VS.str();
	return strFolderName;
}

std::string generate_data(const Boeing_Fuesalage &boeing, std::string strDataset, std::string strFolder)
{
#ifdef WINDOWS
	_mkdir((strDataset+strFolder).c_str());
#else
	mkdir((strDataset + strFolder).c_str(), S_IRWXU);
#endif

	std::string strFile = "data.txt";
	std::vector<Robot> vec_Robots;
	boeing.help_robot_set_up(vec_Robots);

	Data_Generator obj(boeing.get_num_robots(), boeing.get_num_holes() , vec_Robots);
	obj.populate_data(boeing);
	obj.print_data_files(strDataset + strFolder, strFile);
	
	std::string strFilePath = strDataset + strFolder + "/" + strFile;
	return strFilePath;
}


#ifdef WINDOWS
int main()
#else
int main(int argc, char** argv)
#endif
{	
#ifdef WINDOWS
	size_t uiFrames = 8;
	size_t uiRobots = 2;
	double dWidth = 6;
	double dHeight = 4;
	double dHorSpace = 0.1;
	double dVertSpace = 1.0;
	double dWeightFactor = 1;
	unsigned int uikVal = 13;
	size_t uiSimulNum = 1;
#else
	size_t uiFrames = (size_t) atoi(argv[1]);
	size_t uiRobots = (size_t)atoi(argv[2]);
	double dWidth = atof(argv[3]);
	double dHeight = atof(argv[4]);
	double dHorSpace = atof(argv[5]);
	double dVertSpace = atof(argv[6]);
	double dWeightFactor = atof(argv[7]);
	unsigned int uikVal = (size_t)atoi(argv[8]);
	size_t uiSimulNum = (size_t)atoi(argv[9]);
#endif

	Boeing_Fuesalage obj(uiFrames, uiRobots, dWidth, dHeight, dHorSpace, dVertSpace, "DEFAULT");
#ifdef WINDOWS
	std::string strDatasetFolder = "G:/Visual_Studio_Projects/Boeing-Advanced/Dataset/";
	std::string strPlotFolder = "G:/Visual_Studio_Projects/Boeing-Advanced/Graphical_Plots/";
	std::string strDataDumpFolder = "G:/Visual_Studio_Projects/Boeing-Advanced/Sequence_Info/";
	std::string strTSPFolderPath = "G:/Visual_Studio_Projects/Boeing-Advanced/TSP_Aux/" + to_string(uiSimulNum) + "/";
#else
	std::string strDatasetFolder = "Dataset/";
	std::string strPlotFolder = "Graphical_Plots/";
	std::string strDataDumpFolder = "Sequence_Info/";
	std::string strTSPFolderPath = "TSP_Aux/" + to_string(uiSimulNum) + "/";
#endif	

#ifdef WINDOWS
	_mkdir(strTSPFolderPath.c_str());
#else
	mkdir(strTSPFolderPath.c_str(), S_IRWXU);
#endif
	
	std::string strFolder = getFolderName(uiFrames, uiRobots, dWidth, dHeight, dHorSpace, dVertSpace);
	std::string strFilePath = generate_data(obj, strDatasetFolder, strFolder);
	strPlotFolder += strFolder+"/";
	strDataDumpFolder += strFolder + "/";

	cout << "Tag: File: " << strFolder << endl;

	Data_Parser parser(strFilePath);
	std::pair<size_t, size_t> pr;
	int iOffset = parser.read_header_info(pr);
	Layout_LS graph(pr.second, pr.first);
	parser.populate_info(iOffset, graph);
	graph.finish_construction();
	cout << "Tag: Hole Count: " << graph.get_num_holes() << endl;
	cout << "Tag: Weight Factor: " << dWeightFactor << endl;

	Node_Partitions partition(graph);
	
	//generating files for TSP heuristic
	
	AUX_GRAPH::generate_TSP_files(uikVal, strTSPFolderPath.c_str());
		
	Local_Search obj_ls(partition, graph, dWeightFactor);
	obj_ls.perform_local_search(strPlotFolder, strDataDumpFolder, strTSPFolderPath, uikVal, uiSimulNum);
	//obj_ls.perform_VBSS_search(strPlotFolder);	

	cout << "Tag: \n\n\n";
}


/*
void swap_Intra_Sequence(size_t uiPos1, size_t uiLen1, size_t uiPos2, size_t uiLen2, std::list<size_t> &seq)
{
assert(uiPos1 < uiPos2);
assert(uiPos2 + uiLen2 <= seq.size() - 1);

auto it1 = seq.begin();
std::advance(it1, uiPos1);
auto it2 = it1;
std::advance(it2, uiPos2 - uiPos1);
auto it1_end = it1;
std::advance(it1_end, uiLen1);
auto it2_end = it2;
std::advance(it2_end, uiLen2);

seq.splice(it1, seq, it2, it2_end);
if (uiPos1 + uiLen1 == uiPos2) return;
seq.splice(it2_end, seq, it1, it1_end);
}

std::pair<size_t, size_t> generate_Valid_pos_len_pair(std::mt19937 &rng, const std::list<size_t> &seq)
{
size_t uiPos, uiMaxLen, uiLen;
uiPos = 1 + rand() % (seq.size() - 2);
uiMaxLen = seq.size() - 1 - uiPos;
assert(uiMaxLen != 0);

if (2 > uiMaxLen)
{
std::uniform_int_distribution<size_t> unif_len(1, uiMaxLen);
uiLen = unif_len(rng);
}
else
{
std::uniform_int_distribution<size_t> unif_len(1, 2);
uiLen = unif_len(rng);
}

return std::make_pair(uiPos, uiLen);
}

//<pos, len> pair
inline bool check_validity_for_Intra_swap(const std::pair<size_t, size_t> &pr1, const std::pair<size_t, size_t> &pr2)
{
return pr1.first + pr1.second < pr2.first ? true : false;
}

std::vector<std::pair<size_t, size_t>> get_Valid_pos_length_pair2(std::mt19937 &rng, const std::list<size_t> &seq)
{
std::vector<std::pair<size_t, size_t>> vec_pos_len;
bool bValid = false;

while (1)
{
auto pr1 = generate_Valid_pos_len_pair(rng, seq);
auto pr2 = generate_Valid_pos_len_pair(rng, seq);

if (pr1.first == pr2.first) { continue; }
else if (pr1.first > pr2.first) { std::swap(pr1, pr2); };

bValid = check_validity_for_Intra_swap(pr1, pr2);

if (false == bValid) continue;

vec_pos_len.emplace_back(pr1.first, pr1.second);
vec_pos_len.emplace_back(pr2.first, pr2.second);
break;
}

return vec_pos_len;
}
int main()
{
	std::list<size_t> seq{0 , 17, 4, 6, 18, 7, 19, 22, 13, 11, 8, 5, 9, 1};
	
	size_t uiLength = seq.size();
	for (auto it = seq.begin(); it != seq.end(); it++)
	{
		cout << *it << " ";
	}
	cout << endl;

	std::random_device m_rd;     // only used once to initialise (seed) engine
	std::mt19937 m_rng(m_rd());
	srand(time(0));

	while (1)
	{
		auto res = get_Valid_pos_length_pair2(m_rng, seq);

		//swap_Intra_Sequence(res[0].first, res[0].second, res[1].first, res[1].second, seq);
		auto it1 = seq.begin();
		std::advance(it1, 5);
		auto it2 = seq.begin();
		std::advance(it2, 10);
		std::reverse(it1, it2);

		for (auto it = seq.begin(); it != seq.end(); it++)
		{
			cout << *it << " ";
		}
		assert(seq.size() == uiLength);
		cout << endl;
	}


	size_t uiFrames = 4;
	size_t uiRobots = 2;
	double dWidth = 6;
	double dHeight = 4;
	double dHorSpace = 0.1;
	double dVertSpace = 2;
	
	Boeing_Fuesalage obj(uiFrames , uiRobots, dWidth, dHeight, dHorSpace, dVertSpace, "DEFAULT");
	std::string strDatasetFolder = "G:/Visual_Studio_Projects/Boeing-Advanced/Dataset/";
	std::string strFolder = getFolderName(uiFrames, uiRobots, dWidth, dHeight, dHorSpace, dVertSpace);
	std::string strFilePath = generate_data(obj, strDatasetFolder,strFolder);

	Data_Parser parser(strFilePath);
	std::pair<size_t, size_t> pr;
	int iOffset = parser.read_header_info(pr);
	Layout_LS graph(pr.second , pr.first);
	parser.populate_info(iOffset, graph);
	graph.finish_construction();

	Node_Partitions partition(graph);
	std::vector<std::vector<size_t>> vec_rand_seq;
	
	/*Random_LS_Oper ls_obj;
	ls_obj.allocate_randomly_nodes_to_robots(partition, vec_rand_seq);
}*/


/*
int main()
{
	std::vector<size_t> inp_set{ 0, 1 , 3 , 6 };
	Power_Set power;
	const auto &res1 = power.get_power_set(inp_set);

	for (size_t uiCount1 = 0; uiCount1 < res1.size(); uiCount1++)
	{
		for (size_t uiCount2 = 0; uiCount2 < res1[uiCount1].size(); uiCount2++)
		{
			cout << res1[uiCount1][uiCount2] << " ";
		}
		cout << endl;
	}

	std::vector<size_t> inp_set2{ 0, 1 , 6 , 3 };
	const auto &res2 = power.get_power_set(inp_set2);

	size_t uiIndex = 0;
	std::vector<size_t> vec_check;
	vec_check.push_back(uiIndex++);
	const std::set<size_t> set_frames;

	Robot r(Coordinates(0.0, 0.0 , 0.0) , set_frames);
	//r.getPoseToPoint(Coordinates(5.0 , 6.0 , 0) , pose);
	//r.getPoseEnvelope(Coordinates(5.0, 6.0, 0), Coordinates(0, 6.0, 0), pose);
	std::vector<Coordinates> vec_coord;
	vec_coord.push_back(Coordinates(0, 6.0, 0));
	vec_coord.push_back(Coordinates(3.0, 6.0, 0));
	vec_coord.push_back(Coordinates(5.0, 6.0, 0));
	Polygon pose;
	r.getPoseEnvelope(vec_coord, pose);

    return 0;
}*/

/*
int main()
{
Alternative_Graph graph(8);
graph.add_prec_arc(0, 1, 10);
graph.add_prec_arc(1, 2, 10);
graph.add_prec_arc(2, 3, 10);
graph.add_prec_arc(4, 5, 10);
graph.add_prec_arc(5, 6, 10);
graph.add_prec_arc(6, 7, 10);
graph.add_prec_arc(3 , 6, 0);
graph.add_alt_arc(6, 1, 2, 5);

std::unordered_set<size_t> R = { 0, 4 };
std::unordered_map<size_t, std::pair<std::pair<arc, std::string>, std::pair<arc, std::string>>> map_new_sel_arcs;
graph.get_arcs_to_make_sel_positional(R, map_new_sel_arcs);
graph.make_selection_positional(map_new_sel_arcs);

std::unordered_set<size_t> B_Q = { 1, 5 };
std::unordered_set<size_t> Q = {1, 5, 2 , 3, 6 , 7 };
std::list<std::unordered_set<size_t>> listComp;
graph.get_next_strongly_conn_components(B_Q, Q, listComp);

R = { 0 , 5 };
map_new_sel_arcs.clear();
bool bFeas = graph.get_arcs_to_make_sel_positional(R, map_new_sel_arcs);
graph.make_selection_positional(map_new_sel_arcs);
};*/

/*int main()
{
Alternative_Graph graph(8);
graph.add_prec_arc(0, 1, 10);
graph.add_prec_arc(1, 2, 10);
graph.add_prec_arc(2, 3, 10);
graph.add_prec_arc(4, 5, 10);
graph.add_prec_arc(5, 6, 10);
graph.add_prec_arc(6, 7, 10);
graph.add_alt_arc(6, 0 , 1, 5);
graph.add_alt_arc(2, 4, 5, 1);
graph.add_alt_arc(6, 1, 2, 5);

std::unordered_set<size_t> R = {0, 4};
std::unordered_map<size_t, std::pair<std::pair<arc, std::string>, std::pair<arc, std::string>>> map_new_sel_arcs;
graph.get_arcs_to_make_sel_positional(R , map_new_sel_arcs);
graph.make_selection_positional(map_new_sel_arcs);

std::unordered_set<size_t> B_Q = {1, 5};
std::unordered_set<size_t> Q_B_Q = {2 , 3, 6 , 7};
std::vector<std::unordered_set<size_t>> listComp;
graph.get_next_strongly_conn_components(B_Q, Q_B_Q, listComp);

R = { 1 , 5 };
map_new_sel_arcs.clear();
bool bFeas = graph.get_arcs_to_make_sel_positional(R, map_new_sel_arcs);
};*/

