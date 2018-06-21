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
#include "auxgraph.h"
#include "Special_Parser.h"


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

#ifdef SINGLE_ROBOT_MODE

void print_vertex_mapping(const Special_Parser &obj_parser, const size_t c_uiNumRobots, std::string strFilePath)
{
	ofstream myFile;
	std::string str_hole = strFilePath;
	myFile.open(str_hole.c_str());

	for (size_t uiRobot = 0; uiRobot < c_uiNumRobots; uiRobot++)
	{
		myFile << 2 * uiRobot << "- Robot: " << uiRobot << " , start depot \n";
		myFile << 2 * uiRobot + 1 << "- Robot: " << uiRobot << " , end depot \n";
	}

	const size_t c_uiHoleOffset = 2 * c_uiNumRobots;
	const auto& map_index_hole = obj_parser.get_index_hole_map();

	for (auto it = map_index_hole.begin(); it != map_index_hole.end(); it++)
	{
		myFile << c_uiHoleOffset + it->first << "- Hole: " << it->second << "\n";
	}
	myFile << c_uiHoleOffset + obj_parser.get_num_holes() << "- Dummy Hole";

	myFile.close();
}


int main(int argc, char** argv)
{
	std::string strDataSetFolder = argv[1];
	const double c_dWeightFactor = atof(argv[2]);
	const unsigned int c_uikVal = (size_t)atoi(argv[3]);
	const size_t c_uiSimulNum = (size_t)atoi(argv[4]);

#ifdef WINDOWS
	std::string strTSPFolderPath = "G:/Visual_Studio_Projects/Boeing-Advanced/TSP_Aux/" + to_string(c_uiSimulNum) + "/";
	std::string strOutputFolder = "G:/Visual_Studio_Projects/Boeing-Advanced/Output/" ;	
#else
	std::string strTSPFolderPath = "TSP_Aux/" + to_string(c_uiSimulNum) + "/";
	std::string strOutputFolder = "Output/";	
#endif

#ifdef WINDOWS
	_mkdir(strTSPFolderPath.c_str());
	_mkdir(strOutputFolder.c_str());
#else
	mkdir(strTSPFolderPath.c_str(), S_IRWXU);
	mkdir(strOutputFolder.c_str(), S_IRWXU);
#endif

	std::string strHoleFile = strDataSetFolder + "/S1_Left.csv";
	std::string strDistFile = strDataSetFolder +"/LCFD_S1_left_distances.csv";
	std::string strEnablerFile = strDataSetFolder  + "/LCFD_S1_left_adjacencies.csv";
	std::string strFilledHoles = strDataSetFolder + "/S1_Left_Filled_holes.csv";
	std::string strMapFile = strOutputFolder + "/Hole_Reference.txt";

	Special_Parser obj_parser;
	obj_parser.parse_files(strHoleFile, strDistFile, strEnablerFile, strFilledHoles);

	const size_t c_uiNumRobots = 2;
	const size_t c_uiNumHoles = obj_parser.get_num_holes() + 1; //adding dummy hole
	Data_Generator obj(c_uiNumRobots, c_uiNumHoles);
	obj.parse_single_robot_case(obj_parser);

	std::string strFileName = "data.txt";
	obj.print_data_files(strDataSetFolder , strFileName);
	print_vertex_mapping(obj_parser, c_uiNumRobots, strMapFile);
	std::string strFilePath = strDataSetFolder + "/"+ strFileName;

	Data_Parser parser(strFilePath);
	std::pair<size_t, size_t> pr;
	int iOffset = parser.read_header_info(pr);
	Layout_LS graph(pr.second, pr.first);
	parser.populate_info(iOffset, graph);
	graph.finish_construction();
	cout << "Tag: Hole Count: " << graph.get_num_holes() << endl;
	cout << "Tag: Weight Factor: " << c_dWeightFactor << endl;

	Node_Partitions partition(graph);

	//generating files for TSP heuristic	
	AUX_GRAPH::generate_TSP_files(c_uikVal, strTSPFolderPath.c_str());
	
	Local_Search obj_ls(partition, graph, c_dWeightFactor);
	obj_ls.perform_local_search(strOutputFolder, "", strTSPFolderPath, c_uikVal, c_uiSimulNum);
	
	cout << "Tag: \n\n\n";
	return 1;
}
#else
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
	
	//generating files for TSP heuristic	
	AUX_GRAPH::generate_TSP_files(uikVal, strTSPFolderPath.c_str());

	Local_Search obj_ls(partition, graph, dWeightFactor);
	obj_ls.perform_local_search(strPlotFolder, strDataDumpFolder, strTSPFolderPath, uikVal, uiSimulNum);
	//obj_ls.perform_VBSS_search(strPlotFolder);	

	cout << "Tag: \n\n\n";
	return 1;
}
#endif


