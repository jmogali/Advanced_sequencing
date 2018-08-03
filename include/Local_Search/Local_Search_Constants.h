#pragma once
#ifndef LOCAL_SEARCH_CONSTANTS_H
#define LOCAL_SEARCH_CONSTANTS_H

#include "Windows_Linux.h"

const size_t c_uiMax_SE_Length = 2;
const size_t c_uiMaxSwapDiff = 20;
const size_t c_uiMaxRelocateHalfInterval = 5;
const size_t c_uiLate_Acceptace_Length = 25;
const size_t LS_SEARCH_TIME = 600;
const int c_iWaitSwapRange = 5;
const size_t c_uiMaxWaitEventsPerRobot = 3;
const size_t c_uiHoleExchMaxLen = 30;
const size_t c_uiHoleExcStartOffset = 8;
const size_t c_uiHoleExcOffsetJump = 2;
const size_t c_uiMaxNumSwaps = 10;
const size_t c_uiMaxNumInsertTries = 10;
const size_t c_uiMaxCriticalPathCandidates = 15;
const size_t c_uiMaxInfeasibleIters = 10;
const double c_dUpperBoundFilterConstant = 1.5;
const size_t c_uiConsecutive_Late_Accep_Failure = (size_t)(1.5 * c_uiLate_Acceptace_Length);

//#define PRINT_LOCAL_OPERATOR_MESSAGES
//#define ENABLE_LEGACY_CODE
//#define DATA_DUMP_ENABLE

#endif
