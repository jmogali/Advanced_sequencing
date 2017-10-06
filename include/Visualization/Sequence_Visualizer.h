#ifndef SEQUENCE_VISUALIZATION_H
#define SEQUENCE_VISUALIZATION_H

#include "Windows_Linux.h"
#include "Alternative_Graph.h"
#include <fstream>

class Sequence_Visualization
{
	private:
		void plot_enabling_cons_alt_graph(std::string strFilePath, const Alternative_Graph &alt_graph);
		void plot_coll_cons_alt_graph(std::string strFilePath, const Alternative_Graph &alt_graph);

	public:
		void plot_alternative_graph(std::string strFolderPath, const Alternative_Graph &alt_graph);		
		Sequence_Visualization();
};

#endif