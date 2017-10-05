#pragma once
#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include <assert.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
typedef boost::tuple<double, double> Point;
typedef boost::geometry::model::polygon<Point, true, true> Polygon;

#endif
