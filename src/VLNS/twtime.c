/* Example of using dynamic program subroutines for the time
   window problems in the style of E. Baker (Op. Res. 1983) */

#include <stdio.h>
#include <math.h>
#include <time.h>

#define MAXSIZE 500
long int theMatrix[MAXSIZE][MAXSIZE];   /* n by n matrix of costs */
long int tour1[MAXSIZE],tour2[MAXSIZE]; /* space for input/output */
long int wB[MAXSIZE],                   /* release times */
         wE[MAXSIZE],                   /* deadlines */
         sT[MAXSIZE];                   /* service times */

#define _twTime 1
#define theNorm(a,b) (theMatrix[a][b])
#include "solver.h"
#include "Dyn_Node_Desc.h"

void free_buffers();
void reset_buffers(char k, int maxN, int workN, int maxQ);

int ReadData(char *fname)
{ 
	FILE *inpf;
	int c1,c2,c3,c4,n;
	//int d;
	// char iStr[50];

	if ((inpf = fopen(fname, "r")) == NULL)
	{ fprintf(stderr,"error with input file %s\n",fname);
	return(0);
	}
	fscanf (inpf,"%d",&n);
	n++;
	for (c1 = 0; c1 < n; c1++)
	{ fscanf (inpf, "%d", wB+c1);
	sT[c1]=0;
	}
	for (c1 = 0; c1 < n; c1++)
	{ fscanf (inpf, "%d", wE+c1);
	}
	for (c1 = 0; c1 < ((n+7)>>3); c1++)
	{ for (c2 = 0; c2 < ((n+7)>>3); c2++)
	{ for (c3 = 0; c3 < _min(8,n-(c1*8)); c3++)
		{ for (c4 = 0; c4 < _min(8,n-(c2*8)); c4++)
		{ fscanf (inpf, "%d", &theMatrix[(c1*8)+c3][(c2*8)+c4]);
	} } } }
	fclose(inpf);
	return(n);
}

int ReadData_NEW(char *fname, struct Costs_Container *pstCosts)
{
	FILE *inpf;
	int c1, c2, c3, c4, n;
	//int d;
	// char iStr[50];

	if ((inpf = fopen(fname, "r")) == NULL)
	{
		fprintf(stderr, "error with input file %s\n", fname);
		return(0);
	}
	fscanf(inpf, "%d", &n);
	n++;

	pstCosts->m_iNumVtx = n;
	pstCosts->m_parTimeWindows = (struct Hole_Overlap_Enabler_Info*) malloc((n+1) * sizeof(struct Hole_Overlap_Enabler_Info));
	pstCosts->m_parProcTime = (int*)malloc((n+1) * sizeof(int));

	for (c1 = 0; c1 < n; c1++)
	{
		//fscanf(inpf, "%d", wB + c1);
		pstCosts->m_parTimeWindows[c1].m_uiNumIntervals = 1;
		pstCosts->m_parTimeWindows[c1].m_piEnablers = NULL;
		pstCosts->m_parTimeWindows[c1].m_uiOtherRobotEnableTime = inFinity;
		int iStart;
		fscanf(inpf, "%d", &iStart);
		pstCosts->m_parTimeWindows[c1].m_parIntervals = (struct Interval*) malloc(sizeof(struct Interval));
		pstCosts->m_parTimeWindows[c1].m_parIntervals[0].m_uiLow = iStart;
		pstCosts->m_parTimeWindows[c1].m_parIntervals[0].m_uiHigh = inFinity;
		pstCosts->m_parProcTime[c1] = 0;
	}

	pstCosts->m_parTimeWindows[n].m_uiNumIntervals = 1;
	pstCosts->m_parTimeWindows[n].m_piEnablers = NULL;
	pstCosts->m_parTimeWindows[n].m_uiOtherRobotEnableTime = inFinity;
	pstCosts->m_parTimeWindows[n].m_parIntervals = (struct Interval*) malloc(sizeof(struct Interval));
	pstCosts->m_parTimeWindows[n].m_parIntervals[0].m_uiLow = 0;
	pstCosts->m_parTimeWindows[n].m_parIntervals[0].m_uiHigh = 0;
	pstCosts->m_parProcTime[n] = 0;

	for (c1 = 0; c1 < n; c1++)
	{
		int iEnd;
		fscanf(inpf, "%d", &iEnd);
		pstCosts->m_parTimeWindows[c1].m_parIntervals[0].m_uiHigh = iEnd;
	}

	/*for (c1 = 0; c1 < ((n + 7) >> 3); c1++)
	{
		for (c2 = 0; c2 < ((n + 7) >> 3); c2++)
		{
			for (c3 = 0; c3 < _min(8, n - (c1 * 8)); c3++)
			{
				for (c4 = 0; c4 < _min(8, n - (c2 * 8)); c4++)
				{
					fscanf(inpf, "%d", &theMatrix[(c1 * 8) + c3][(c2 * 8) + c4]);
				}
			}
		}
	}*/ 


	pstCosts->m_pparTravTime = (int**)malloc(n * sizeof(int*));
	for (int iCount = 0; iCount < n; iCount++)
	{
		pstCosts->m_pparTravTime[iCount] = (int*)malloc((n +1) * sizeof(int));
	}

	for (c1 = 0; c1 < ((n + 7) >> 3); c1++)
	{		
		for (c2 = 0; c2 < ((n + 7) >> 3); c2++)
		{
			for (c3 = 0; c3 < _min(8, n - (c1 * 8)); c3++)
			{
				for (c4 = 0; c4 < _min(8, n - (c2 * 8)); c4++)
				{
					//fscanf(inpf, "%d", &theMatrix[(c1 * 8) + c3][(c2 * 8) + c4]);
					fscanf(inpf, "%d", &pstCosts->m_pparTravTime[(c1 * 8) + c3][(c2 * 8) + c4]);
				}
			}
		}
	}

	for (int iCount = 0; iCount < n; iCount++)
	{
		pstCosts->m_pparTravTime[iCount][n] = pstCosts->m_pparTravTime[iCount][0];
	}
	
	fclose(inpf);
	return(n);
}

/*
#ifndef CUSTOM_CODE
main ()
{ 
  costtype FinalCost;
  nodeXtype c,n;
  char guarantee;
  long int h,k;
 // long int q;

  srand(time(0));

  //if (argc!=4)
 // { fprintf(stderr,"Program needs three arguments (h k filename)\n");
 //   exit(1);
 // }

  //h = 20;
  k = 10;
  char *filename = "Datasets//p25a.dat";
  //sscanf(argv[1],"%d",&h);
  //sscanf(argv[2],"%d",&k);
  //n=ReadData(argv[3]);
  n = ReadData(filename);
  h = n;

  if (n>MAXSIZE)
  { fprintf(stderr,"problem size too big.\n");
    exit(1);
  }

  GetAuxgraph(k,n,h,1);
  tour1[0]=n;
  FinalCost = DynOpt(k,n,h,0,tour1,tour2, wB,wE,sT,&guarantee);

  if (FinalCost == (1<<30)) // no feasible solution found 
  { if (guarantee)
    { printf("There is no feasible solution, guaranteed.\n");
    }
    else
    { printf("Could not find a feasible solution.\n");
      printf("Problem may or may not be feasible.\n");
    }
  }
  else
  { printf("Final Cost: %d\n",FinalCost);
    printf("Sequence:\n");
    if (tour2[0]<n)
    { for (c=0;c<n;c++) {printf(" %d",tour2[c]+1);}
    }
    else
    { printf("  A larger value of h must be used to recover the sequence.");
    }
    if (guarantee)
    { printf("\n**Solution is optimal**\n");
    }
    else
    { printf("\n**Solution may not be optimal**\n");
    }
  } 
}
#endif

*/

void restructure_cost_file(int n, int* temp_tour, struct Costs_Container *pstCosts_old, struct Costs_Container *pstCosts_new)
{
	int iIndex;
	pstCosts_new->m_iNumVtx = pstCosts_old->m_iNumVtx;
	pstCosts_new->m_parTimeWindows = (struct Hole_Overlap_Enabler_Info*) malloc((n+1) * sizeof(struct Hole_Overlap_Enabler_Info));
	pstCosts_new->m_parProcTime = (int*)malloc((n+1) * sizeof(int));

	for (int iPos = 0; iPos < n; iPos++)
	{
		iIndex = temp_tour[iPos];
		pstCosts_new->m_parProcTime[iPos] = pstCosts_old->m_parProcTime[iIndex];
		pstCosts_new->m_parTimeWindows[iPos].m_uiNumIntervals = pstCosts_old->m_parTimeWindows[iPos].m_uiNumIntervals;
		pstCosts_new->m_parTimeWindows[iPos].m_parIntervals = (struct Interval*) malloc(sizeof(struct Interval));
		pstCosts_new->m_parTimeWindows[iPos].m_parIntervals[0].m_uiLow = pstCosts_old->m_parTimeWindows[iIndex].m_parIntervals[0].m_uiLow;
		pstCosts_new->m_parTimeWindows[iPos].m_parIntervals[0].m_uiHigh = pstCosts_old->m_parTimeWindows[iIndex].m_parIntervals[0].m_uiHigh;		
		pstCosts_new->m_parTimeWindows[iPos].m_piEnablers = pstCosts_old->m_parTimeWindows[iIndex].m_piEnablers;
	}

	pstCosts_new->m_parTimeWindows[n].m_uiNumIntervals = 1;
	pstCosts_new->m_parTimeWindows[n].m_piEnablers = NULL;
	pstCosts_new->m_parTimeWindows[n].m_uiOtherRobotEnableTime = inFinity;
	pstCosts_new->m_parTimeWindows[n].m_parIntervals = (struct Interval*) malloc(sizeof(struct Interval));
	pstCosts_new->m_parTimeWindows[n].m_parIntervals[0].m_uiLow = 0;
	pstCosts_new->m_parTimeWindows[n].m_parIntervals[0].m_uiHigh = 0;
	pstCosts_new->m_parProcTime[n] = 0;

	pstCosts_new->m_pparTravTime = (int**)malloc(n * sizeof(int*));
	for (int iCount = 0; iCount < n; iCount++)
	{
		pstCosts_new->m_pparTravTime[iCount] = (int*)malloc(n * sizeof(int));
	}

	int iIndex1, iIndex2;
	for (int iPos1 = 0; iPos1 < n; iPos1++)
	{
		iIndex1 = temp_tour[iPos1];
		for (int iPos2 = 0; iPos2 <= n; iPos2++)
		{
			if(iPos2 < n) iIndex2 = temp_tour[iPos2];
			else iIndex2 = temp_tour[0];

			pstCosts_new->m_pparTravTime[iPos1][iPos2] = pstCosts_old->m_pparTravTime[iIndex1][iIndex2];
		}
	}
}

//int compute_opt(long int* init_path, int N, long int K, struct Costs_Container *pstCosts, struct Dyn_Node_Desc *pstNodeInfo, unsigned int uiStartTime)
/*
int optimize_tsp(struct Dyn_Node_Desc *pstAuxNodeInfo)
{
	costtype FinalCost;
	nodeXtype c, n;
	// long int q;

	srand(time(0));
	char *filename = "Datasets//p25a.dat";

	struct Costs_Container *pstCosts_Temp = (struct Costs_Container *)malloc(sizeof(struct Costs_Container));
	n = ReadData_NEW(filename, pstCosts_Temp);

	int temp_tour[26] = { 0,14,3,17,1,13,12,4,21,10,6,22,23,24,16,2,5,19,20,25,15,7,11,18,9,8 }; //25a
	//int temp_tour[26] = { 0,21,22,1,25,23,5,19,20,14,2,15,13,6,9,3,17,24,16,10,11,7,12,18,4,8 };//25d
	//int temp_tour[26] = { 0,3,25, 24, 23, 1, 21, 13	, 10, 6, 9	, 4	, 19, 17, 14, 12, 11, 18, 22, 2, 16	, 7	, 5	, 8	, 20 , 15 }; //25e
	
	struct Costs_Container *pstCosts = (struct Costs_Container *)malloc(sizeof(struct Costs_Container));
	restructure_cost_file(n, temp_tour, pstCosts_Temp, pstCosts);

	long int h = n, k = 10;

	//n = N;

	if (n>MAXSIZE)
	{
		fprintf(stderr, "problem size too big.\n");
		exit(1);
	}

	GetAuxgraph(k, n, h, 1);
	
	//tour1[0] = n;
	
	for (int iPos = 0; iPos < n; iPos++)
	{
		tour1[iPos] = iPos;
		printf("%d, ", pstCosts->m_parTimeWindows[iPos].m_parIntervals[0].m_uiLow);
	}
	tour1[n] = tour1[0];
	
	FinalCost = DynOpt_NEW(k, n, h, tour1, tour2, pstCosts, pstAuxNodeInfo, 0);

	if (FinalCost >= (1 << 30)) // no feasible solution found 
	{
		printf("Could not find a feasible solution.\n");
		printf("Problem may or may not be feasible.\n");	
		return -1;
	}
	else
	{
		printf("Final Cost: %d\n", FinalCost);
		printf("Sequence:\n");
		if (tour2[0]<n)
		{
			for (c = 0; c<n; c++) { printf(" %d", tour2[c]); }
		}
		else
		{
			printf("  A larger value of h must be used to recover the sequence.");
		}		
	}
	
	free_buffers();
	
	return 0;
}
*/

int optimize_tsp(struct Dyn_Node_Desc *pstAuxNodeInfo, struct Costs_Container *pstCosts, int iNumVts, int kVal, int* new_tour, int bFirstIter, const char* cFolderPath, const int c_uiStartTime)
{
	costtype FinalCost;
	nodeXtype c, n;
	
	n = iNumVts;
	long int h = 10 * kVal, k = kVal;

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

	if (FinalCost >= (1 << 30)) // no feasible solution found 
	{
		printf("Could not find a feasible solution.\n");
		printf("Problem may or may not be feasible.\n");
		return -1;
	}
	else
	{
		printf("Final Cost: %d\n", FinalCost);
		printf("Sequence:\n");
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
	char c, cantpack = 0;

	if (k >= _KMAX)
	{
		fprintf(stderr, "k value is too high for this program.");
	}
	if ((maxQ>15 || k>15) && (maxQ>32 || k>8) && (maxQ>8))
	{
		cantpack++;
	}
	
	for (int uiCount = 0; uiCount < maxN + 1; uiCount++) depth[uiCount] = 0;
	for (unsigned int uiCount = 0; uiCount < bN[k]; uiCount++) j[uiCount] = 0;
	for (unsigned int uiCount = 0; uiCount < bN[k]; uiCount++) minK[uiCount] = 0;
	for (unsigned int uiCount = 0; uiCount < bA[k]; uiCount++) succs[uiCount] = 0;
	for (unsigned int uiCount = 0; uiCount < bN[k]; uiCount++) succInx[uiCount] = 0;
	for (unsigned int uiCount = 0; uiCount < bA[k]; uiCount++) preds[uiCount] = 0;
	for (unsigned int uiCount = 0; uiCount < bN[k]+1; uiCount++) predInx[uiCount] = 0;
	for (unsigned int uiCount = 0; uiCount < bN[k]; uiCount++) predLoc[uiCount] = 0;
	for (int uiCount = 0; uiCount < maxN + 1; uiCount++) kval[uiCount] = 0;

	
	for (c = 0; c<1 + 3 * _enableshrink; c++)
	{
		for (int uiCount = 0; uiCount < maxN*(3 * k - 1); uiCount++)
		{
			shortMatrix[c][uiCount] = 0;
		}		
	}
	if (_enableshrink)
	{
		for (int uiCount = 0; uiCount < (maxN + 7) / 8 + 1; uiCount++) orientA[uiCount] = 0;				
	}
	
	for (int uiCount = 0; uiCount < maxN + 1; uiCount++) levtour[uiCount] = 0;

	if (_timewindow)
	{
		for (int uiCount = 0; uiCount < maxN + 1; uiCount++) winBegin[uiCount] = 0;
		for (int uiCount = 0; uiCount < maxN + 1; uiCount++) winEnd[uiCount] = 0;
		for (int uiCount = 0; uiCount < maxN + 1; uiCount++) servTime[uiCount] = 0;		
	}

	for (unsigned int uiCount = 0; uiCount < maxQ*bN[k]; uiCount++) costsNow[uiCount] = 0;
	for (unsigned int uiCount = 0; uiCount < maxQ*bN[k]; uiCount++) costsNext[uiCount] = 0;

	if (_timewindow == 2)
	{
		for (unsigned int uiCount = 0; uiCount < maxQ*bN[k]; uiCount++) timeNow[uiCount] = 0;
		for (unsigned int uiCount = 0; uiCount < maxQ*bN[k]; uiCount++) timeNext[uiCount] = 0;		
	}

	for (int uiCount = 0; uiCount < 2 * (workN + 1)*maxN; uiCount++) sublists[uiCount] = 0;
	for (int uiCount = 0; uiCount < (int)_max((1 + cantpack)*maxQ*workN*bN[k] + ((maxN + 7) / 8), maxN * 80); uiCount++) workarea[uiCount] = 0;
}
