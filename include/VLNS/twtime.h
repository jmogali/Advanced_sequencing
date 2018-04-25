#ifndef TWTIME_H
#define TWTIME_H

int optimize_tsp(struct Dyn_Node_Desc *pstAuxNodeInfo, struct Costs_Container *pstCosts, int iNumVts, int kVal, int* new_tour, int bFirstIter, const char* cFolderPath, const int c_uiStartTime, int *iOpt);
void free_buffers();

#endif