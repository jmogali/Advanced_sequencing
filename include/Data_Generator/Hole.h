#pragma once
#ifndef HOLE_H 
#define HOLE_H

#include "Coordinates.h"
#include "Typedefs.h"
#include <string>

class Hole
{
	private:
		Coordinates m_loc;
		std::string m_strType;
		size_t m_uiFrame;
	
	public:
		Hole(Coordinates loc, std::string strType , size_t uiFrame );
		inline Coordinates getLoc() const { return m_loc; };
		inline std::string getType() const { return m_strType; };
		inline size_t getFrame() const { return m_uiFrame; };
};

#endif