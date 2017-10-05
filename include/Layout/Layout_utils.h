#pragma once
#ifndef LAYOUT_UTILS_H
#define LAYOUT_UTILS_H

#include "Typedefs.h"
#include <set>
#include <unordered_map>
#include <unordered_set>
#include "Coordinates.h"
#include "Hashing_Utils.h"

using namespace std;

class Ind
{
	protected:
		size_t m_uiInd;
	public:
		Ind(size_t uiInd) : m_uiInd(uiInd) {};
		Ind(const Ind &uiInd) { m_uiInd = uiInd.m_uiInd; }
		inline size_t getInd() const { return m_uiInd; };
};

inline bool operator== (const Ind& lhs, const Ind& rhs) { return lhs.getInd() == rhs.getInd(); }
inline bool operator< (const Ind& lhs, const Ind& rhs) { return lhs.getInd() < rhs.getInd(); }

struct IndHasher
{
	std::size_t operator()(const Ind& ind) const
	{
		size_t seed = 0;
		return Hash_It(seed, ind.getInd());
	}
};

class R_Ind : public Ind
{
	public:
		R_Ind(size_t uiInd) : Ind(uiInd) {};
		R_Ind(const R_Ind &ind) : Ind(ind) {};
};

class N_Ind : public Ind
{
	public:
		N_Ind(size_t uiInd) : Ind(uiInd) {};
		N_Ind(const N_Ind &ind) : Ind(ind) {};
};
inline bool operator== (const N_Ind& lhs, const N_Ind& rhs) { return lhs.getInd() == rhs.getInd(); }

class Vx_Ind :public N_Ind
{
	private:
		size_t m_uiTime;
	public:
		Vx_Ind(size_t uiInd, size_t uiTime) : N_Ind(uiInd), m_uiTime(uiTime) {};
		Vx_Ind(const Vx_Ind &ind) : N_Ind(ind) , m_uiTime(ind.m_uiTime) {};
		inline size_t getTime() const { return m_uiTime; };
};

class V_Ind : public Vx_Ind
{
	public:
		V_Ind(size_t uiInd , size_t uiTime) : Vx_Ind(uiInd , uiTime) {};
		V_Ind(const V_Ind &ind) : Vx_Ind(ind) {};			
};

inline bool operator== (const V_Ind& lhs, const V_Ind& rhs) { return lhs.getInd() == rhs.getInd(); }
inline bool operator< (const V_Ind& lhs, const V_Ind& rhs) { return lhs.getInd() < rhs.getInd(); }

class IV_Ind : public Vx_Ind
{
	public:
		IV_Ind(size_t uiInd , size_t uiTime) : Vx_Ind(uiInd , uiTime) {};
		IV_Ind(const IV_Ind &ind) : Vx_Ind(ind) {};
};

struct Neigh
{
	std::unordered_map<N_Ind , size_t, IndHasher> map;
};

struct V_V //Vertex vertex pairs
{
	std::unordered_map<N_Ind, Neigh, IndHasher> map;
};

class Ind_Pair
{
	protected:
		Ind m_uind1, m_uind2;
	public:
		Ind_Pair(size_t uiInd1, size_t uiInd2) : m_uind1(uiInd1), m_uind2(uiInd2) {};
		Ind_Pair(const Ind_Pair &pr) : m_uind1(pr.m_uind1), m_uind2(pr.m_uind2) {};
		inline size_t getInd1() const { return m_uind1.getInd(); };
		inline size_t getInd2() const { return m_uind2.getInd(); };
};

class N_Ind_Pair: public Ind_Pair
{
	public:
		N_Ind_Pair(size_t uiInd1, size_t uiInd2) : Ind_Pair(uiInd1, uiInd2) {};
		N_Ind_Pair(const N_Ind_Pair &pr) : Ind_Pair(pr) {};
};

inline bool operator== (const Ind_Pair& lhs, const Ind_Pair& rhs) { return (lhs.getInd1() == rhs.getInd1()) & (lhs.getInd2() == rhs.getInd2()); }

struct PairHasher
{
	std::size_t operator()(const Ind_Pair& pair) const
	{
		size_t seed = 0;
		::Hash_It(seed, pair.getInd1());
		::Hash_It(seed, pair.getInd2());
		return seed;
	}
};

struct IV_Vec
{
	std::vector<IV_Ind> vec;
};

struct Neigh_IV
{
	std::unordered_map<N_Ind, IV_Vec, IndHasher> map;
};

struct V_I_V
{
	std::unordered_map<N_Ind, Neigh_IV, IndHasher> map;
};

class Coll_Pair
{
	private:
		Ind_Pair m_pr1, m_pr2; // each pair-: Node Index, Robot Index
	public:
		Coll_Pair(N_Ind s1, R_Ind r1, N_Ind s2 , R_Ind r2) :
				  m_pr1(r1.getInd()< r2.getInd() ? s1.getInd() : s2.getInd(), r1.getInd()< r2.getInd() ? r1.getInd() : r2.getInd()), 
			      m_pr2(r1.getInd()< r2.getInd() ? s2.getInd() : s1.getInd() , r1.getInd()< r2.getInd() ? r2.getInd() : r1.getInd()) {};
		inline Ind_Pair getPair1() const { return m_pr1; };
		inline Ind_Pair getPair2() const { return m_pr2; };
};


inline bool operator== (const Coll_Pair& lhs , const Coll_Pair& rhs)
{
	return (lhs.getPair1() == rhs.getPair1()) && (lhs.getPair2() == rhs.getPair2());
}

struct CollHasher
{
	std::size_t operator() (const Coll_Pair &v) const
	{
		size_t seed = 0;
		::Hash_It(seed, v.getPair1().getInd1());
		::Hash_It(seed, v.getPair1().getInd2());
		::Hash_It(seed, v.getPair2().getInd1());
		::Hash_It(seed, v.getPair2().getInd2());
		return seed;
	}
};

struct N_EN
{
	std::unordered_set<N_Ind, IndHasher> set;
};

class Node_Desc
{
	protected:
		size_t m_uiTime;
		Coordinates m_loc;
	public:	
		Node_Desc(size_t uiTime, const Coordinates &loc);
		Node_Desc(const Node_Desc &obj);
		inline size_t getTime() const{ return m_uiTime; };
		inline Coordinates getLoc() const{ return m_loc; };
};

class Depo_Desc : public Node_Desc
{
	private:
		size_t m_uiFromInd , m_uiToInd;
	public:
		Depo_Desc(size_t uiTime, size_t uiFromInd, size_t uiToInd, const Coordinates &loc);
		Depo_Desc(const Depo_Desc &obj);
		inline size_t getFromInd() const { return m_uiFromInd; };
		inline size_t getToInd() const { return m_uiToInd; };
};

class IV_Hole_Pair
{
	private:
		Ind_Pair pr; // hole1, hole2
	public:
		IV_Hole_Pair(size_t uiHole1, size_t uiHole2) : pr(uiHole1,uiHole2) {};
		inline size_t getHole1() { return pr.getInd1(); };
		inline size_t getHole2() { return pr.getInd2(); };
};

#endif

