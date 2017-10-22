#include "Greedy_Heuristic_Utils.h"


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

