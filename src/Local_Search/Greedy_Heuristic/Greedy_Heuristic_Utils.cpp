#include "Greedy_Heuristic_Utils.h"
#include "Windows_Linux.h"

bool operator== (const State& lhs, const State& rhs)
{
	if (lhs.getRobotNum() != rhs.getRobotNum()) return false;

	for (size_t uiRobot = 0; uiRobot < lhs.getRobotNum(); uiRobot++)
	{
		if (*lhs.m_vec_rob_pos[uiRobot] != *rhs.m_vec_rob_pos[uiRobot])
			return false;
	}
	return true;
}

State::State(const State& state)
{
	m_vec_rob_pos.resize(state.m_vec_rob_pos.size());
	for (size_t uiRobot = 0; uiRobot < state.m_vec_rob_pos.size(); uiRobot++)
	{
		m_vec_rob_pos[uiRobot] = state.m_vec_rob_pos[uiRobot];
	}
}


void State::get_vertices(std::unordered_set<size_t> &set_vert) const
{
	assert(0 == set_vert.size());
	for (size_t uiRobot = 0; uiRobot < m_vec_rob_pos.size(); uiRobot++)
	{
		set_vert.emplace(*m_vec_rob_pos[uiRobot]);
	}
}

void Vertex_Schedule::print_schedule() const
{
	cout << m_uiInd << "-(" << m_uiStart << "-" << m_uiEnd << ") :" << m_uiWait;
}

ST_Time::ST_Time(size_t uiStartTime) : m_uiStartTime(uiStartTime)
{}

Comparison_Object::Comparison_Object(size_t dispatch_time, size_t comp_size, size_t makespan, size_t delay)
{
	uiDispatchTime = dispatch_time;
	uiCompSize = comp_size;
	uiExpMakeSpan = makespan;
	uiMaxDelay = delay;
}

void print_sequence(const std::vector<std::list<size_t>> &rob_seq)
{
	for (size_t uiRobot = 0; uiRobot < rob_seq.size(); uiRobot++)
	{
		for (auto it = rob_seq[uiRobot].begin(); it != rob_seq[uiRobot].end(); it++)
		{
			cout << *it << ",";
		}
		cout << endl;
	}

	cout << "\n\n";
}

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

void dump_data_to_file(const std::vector<std::list<size_t>> &rob_seq, const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, std::string strFolder, std::string strFileName, bool bFeasible)
{
	ofstream myFile;
	std::string strFile = strFolder + "/" + strFileName;
	myFile.open(strFile.c_str());

	myFile << "SEQUENCE_INFO:" << endl;
	for (size_t uiRobot = 0; uiRobot < rob_seq.size(); uiRobot++)
	{
		myFile << "ROBOT: " << uiRobot<<endl;
		for (auto it = rob_seq[uiRobot].begin(); it != rob_seq[uiRobot].end(); it++)
		{
			myFile << *it << ",";
		}	
		myFile << endl;
	}
	
	if (false == bFeasible)
	{
		myFile.close();
		return;
	}

	myFile << endl;
	myFile << "SCHEDULE: " << endl;

	for (size_t uiRobot = 0; uiRobot < full_rob_sch.size(); uiRobot++)
	{
		myFile << "ROBOT: " << uiRobot << endl;
		for (auto it = full_rob_sch[uiRobot].begin(); it != full_rob_sch[uiRobot].end(); it++)
		{
			myFile << it->m_uiInd << "-(" << it->m_uiStart << "-" << it->m_uiEnd << ")," << it->m_uiWait << endl;			
		}		
		myFile << endl;
	}
	myFile.close();
}