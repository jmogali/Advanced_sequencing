/* Example of using dynamic program subroutines for the time
   window problems in the style of E. Baker (Op. Res. 1983) */

#include <stdio.h>
#include <math.h>
#include <time.h>
#include "twtime.h"

#define MAXSIZE 500
long int theMatrix[MAXSIZE][MAXSIZE];   /* n by n matrix of costs */
int tour1[MAXSIZE],tour2[MAXSIZE]; /* space for input/output */
long int wB[MAXSIZE],                   /* release times */
         wE[MAXSIZE],                   /* deadlines */
         sT[MAXSIZE];                   /* service times */

#define _twTime 1
#define theNorm(a,b) (theMatrix[a][b])
#include "solver.h"
#include "Dyn_Node_Desc.h"

void reset_buffers(char k, int maxN, int workN, int maxQ);

int optimize_tsp(struct Dyn_Node_Desc *pstAuxNodeInfo, struct Costs_Container *pstCosts, int iNumVts, int kVal, int* new_tour, int bFirstIter, const char* cFolderPath, const int c_uiStartTime, int *iOpt)
{
	costtype FinalCost;
	nodeXtype c, n;
	
	n = iNumVts;
	long int h = 20 * kVal, k = kVal;
	
	if (n>MAXSIZE)
	{
		fprintf(stderr, "problem size too big.\n");
		exit(1);
	}

	if(1 == bFirstIter) GetAuxgraph(k, MAXSIZE, h, 1, cFolderPath);
	else reset_buffers(k, MAXSIZE, h, 1);

	for (int iPos = 0; iPos < n; iPos++)
	{
		tour1[iPos] = iPos;
	}
	
	tour1[n] = tour1[0];

	FinalCost = DynOpt_NEW(k, n, h, tour1, tour2, pstCosts, pstAuxNodeInfo, c_uiStartTime);
	*iOpt = FinalCost;

	if (FinalCost >= (1 << 30)) // no feasible solution found 
	{
		printf("Could not find a feasible solution.\n");
		printf("Problem may or may not be feasible.\n");
		return -1;
	}
	else
	{
		printf("Final Cost: %d\n", FinalCost);
		if (tour2[0]<n)
		{
			for (c = 0; c<n; c++) 
			{ 
				//printf(" %d", tour2[c]); 
				new_tour[c] = tour2[c];
			}
		}
		else
		{
			printf("  A larger value of h must be used to recover the sequence.");
			for (c = 0; c<n; c++)
			{
				//printf(" %d", tour2[c]); 
				new_tour[c] = tour1[c];
			}
		}
	}

	return 0;
}

void free_buffers()
{
	for (int iCount = 0; iCount < 4; iCount++)
	{
		free(shortMatrix[iCount]);
		shortMatrix[iCount] = NULL;
	}

	free(winBegin);
	winBegin = NULL;

	free(winEnd);
	winEnd = NULL;

	free(servTime);
	servTime = NULL;

	free(costsNow);
	costsNow = NULL;

	free(costsNext);
	costsNext = NULL;

	free(timeNow);
	timeNow = NULL;

	free(timeNext);
	timeNext = NULL;

	free(sublists);
	sublists = NULL;

	free(workarea);
	workarea = NULL;

	free(kval);
	kval = NULL;

	free(depth);
	depth = NULL;

	free(succs);
	succs = NULL;

	free(succInx);
	succInx = NULL;

	free(preds);
	preds = NULL;

	free(predInx);
	predInx = NULL;

	free(levtour);
	levtour = NULL;

	free(j);
	j = NULL;

	free(minK);
	minK = NULL;

	free(predLoc);
	predLoc = NULL;

	free(orientA);
	orientA = NULL;
}

void reset_buffers(char k, int maxN, int workN, int maxQ)
{
	char cantpack = 0;

	if (k >= _KMAX)
	{
		fprintf(stderr, "k value is too high for this program.");
	}
	if ((maxQ>15 || k>15) && (maxQ>32 || k>8) && (maxQ>8))
	{
		cantpack++;
	}

	for (unsigned int uiCount = 0; uiCount < maxQ*bN[k]; uiCount++) costsNow[uiCount] = 0;
	for (unsigned int uiCount = 0; uiCount < maxQ*bN[k]; uiCount++) costsNext[uiCount] = 0;

	for (int uiCount = 0; uiCount < 2 * (workN + 1)*maxN; uiCount++) sublists[uiCount] = 0;
	for (int uiCount = 0; uiCount < (int)_max((1 + cantpack)*maxQ*workN*bN[k] + ((maxN + 7) / 8), maxN * 80); uiCount++) workarea[uiCount] = 0;

	for (int iCount = 0; iCount < maxN; iCount++) tour2[iCount] = 0;
}
