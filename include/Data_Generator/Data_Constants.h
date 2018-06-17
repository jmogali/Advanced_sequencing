#pragma once
#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "Typedefs.h"
#include "Windows_Linux.h"

const size_t TIME_MULT_FAC = 100;
const double ROBOT_SPEED = 0.25;

#ifdef SINGLE_ROBOT_MODE
const size_t TIME_PER_HOLE = 30;
#else
const size_t TIME_PER_HOLE = 40;
#endif

const double ROBOT_FRACTION_COVERAGE = 0.7;
const size_t DEPOT_TIME = 1;

#define SCALE_TIME(x) (size_t) floor(TIME_MULT_FAC * x)

#endif
