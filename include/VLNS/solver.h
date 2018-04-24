/* Subroutines for dynamic program for the Traveling Salesman Problem */
/* www.contrib.andrew.cmu.edu/~neils/tsp/index.html                   */
/* Neil Simonetti, May 1998                                           */

#include "VLNS_Constants.h"

#define int32 int /* must be 32 bit integers */
#define costtype int32
#define nodeXtype int32

#define _costonlyd 0  /* 1 -> return the only the cost of the optimal tour */
                    /* 0 -> return the optimal tour and its cost */
char _costonly = _costonlyd;

#ifdef _twTime
#define _timewindow 1
#define _enableshrink 0 /* shrinking has not been implemented with time windows yet */
#elif _twDist
#define _timewindow 2
#define _enableshrink 0 /* shrinking has not been implemented with time windows yet */
#else
#define _timewindow 0
#ifdef _shrink
#define _enableshrink 1
#else
#define _enableshrink 0
#endif
#endif

#ifndef _accuracy
#define _accuracy 1
#endif


/* smINX(a,b,k) determines the index in the small matrix for the cost */
/* of traveling from node a to node b */
#define smINX(a,b,k) ( ((3*(k)-1)*(a)) + ((b)-(a)+(k)-1) )

costtype *shortMatrix[4];
costtype *winBegin=NULL, *winEnd=NULL, *servTime=NULL; /* time windows */
costtype *costsNow, *costsNext, *timeNow=NULL, *timeNext=NULL; /* used in fiddle */
unsigned char *sublists, *workarea;
#define workareaint ((int32 *)workarea)
int *kval, *depth;
unsigned int32 *succs, *succInx, *preds, *predInx, *levtour;
signed char *j, *minK, *predLoc, *orientA;

#include "ag.h"
#ifdef _twTime
#include "agtw.h"
#elif _twDist
#include "agtw.h"
#endif

#include "dynopt.h"
#include "Costs_Container.h"
#include "Dyn_Node_Desc.h"

void GetAuxgraph(char k, int maxN, int workN, int maxQ, const char* cFolderPath)
{ 
	char c, cantpack=0;

	if (k>=_KMAX)
	{ fprintf(stderr,"k value is too high for this program.");}
	if ((maxQ>15 || k>15) && (maxQ>32 || k>8) && (maxQ>8))
	{ cantpack++;}
	fprintf(stderr,"Reading Auxgraph...\n");
	makesure(depth = (int *)calloc(maxN+1,sizeof(int)),allocErr);
	makesure(j = (char *)calloc(bN[k],sizeof(char)),allocErr);
	makesure(minK = (char *)calloc(bN[k],sizeof(char)),allocErr);
	makesure(succs  = (unsigned int32 *)calloc(bA[k], sizeof(int32)),allocErr);
	makesure(succInx= (unsigned int32 *)calloc(bN[k]+1,sizeof(int32)),allocErr);
	makesure(preds  = (unsigned int32 *)calloc(bA[k], sizeof(int32)),allocErr);
	makesure(predInx= (unsigned int32 *)calloc(bN[k]+1,sizeof(int32)),allocErr);
	makesure(predLoc= (char *)calloc(bN[k],sizeof(char)),allocErr);
	succInx[bN[k]]=predInx[bN[k]]=bA[k];
	ReadAuxgraph(k,j,minK,succs,succInx,preds,predInx,predLoc, cFolderPath);
	fprintf(stderr,"Allocating Memory...\n");
	makesure(kval = (int *)calloc(maxN+1,sizeof(int)),allocErr);
	for(c=0; c<1+3*_enableshrink; c++)
	{ makesure(shortMatrix[c] =
				(costtype *)calloc(maxN*(3*k-1),sizeof(costtype)),allocErr);
	}
	if (_enableshrink)
	{ makesure(orientA=(signed char *)calloc((maxN+7)/8+1,sizeof(char)),allocErr);
	}
	makesure(levtour = (nodeXtype *)calloc(maxN+1,sizeof(nodeXtype)),allocErr);
	if (_timewindow)
	{ makesure(winBegin=(costtype *)calloc(maxN+1,sizeof(costtype)),allocErr);
	makesure(winEnd = (costtype *)calloc(maxN+1,sizeof(costtype)),allocErr);
	makesure(servTime=(costtype *)calloc(maxN+1,sizeof(costtype)),allocErr);
	}
	makesure(costsNow = (costtype *)calloc(maxQ*bN[k],sizeof(costtype)),allocErr);
	makesure(costsNext= (costtype *)calloc(maxQ*bN[k],sizeof(costtype)),allocErr);  if (_timewindow==2)
	{ makesure(timeNow = (costtype *)calloc(maxQ*bN[k],sizeof(costtype)),allocErr);
	makesure(timeNext= (costtype *)calloc(maxQ*bN[k],sizeof(costtype)),allocErr);
	}
	makesure(sublists=(unsigned char *)calloc(2*(workN+1)*maxN,sizeof(char)),allocErr);
	makesure(workarea = (char *)calloc(_max((1+cantpack)*maxQ*workN*bN[k]+((maxN+7)/8),maxN*80),
										sizeof(char)),allocErr);
}

#ifdef _twTime

costtype DynOpt (signed char k, nodeXtype n, nodeXtype wn, int targ,
                 nodeXtype *tourIn, nodeXtype *tourOut,
                 costtype *winBeginRaw, costtype *winEndRaw,
                 costtype *servTimeRaw, char *guarantee)

{
  nodeXtype levn, c;
  //nodeXtype n1, n2, n3, n4;;
  signed char localK, noguarantee=0, maketour=0;
  costtype mycost;

  targ *= _enableshrink;
  if (tourIn[0]==n)
  { maketour=1;
    tourIn[0]=0;
  }
  levn = BuildSmallTWmatrix (k, (targ>0), maketour,
                       winBeginRaw, winBegin, winEndRaw, winEnd,
                       servTimeRaw, servTime, shortMatrix[0],
                       tourIn, levtour, targ, n, kval,
                       &localK, workareaint, workareaint+n*4,
                       &noguarantee);
  CalcDepths (kval,depth,levn);
  printf("Optimizing with Dynamic subroutine (k=%d) over %d nodes:\n",localK, levn);
  mycost = Fiddle(localK,levn,shortMatrix[0],j,minK,kval,depth,succs,succInx,
		            preds,predInx,predLoc,tourOut,winBegin,winEnd,
                            servTime,costsNow, costsNext, workarea,
                            sublists, wn);
  if (mycost<inFinity && tourOut[0]<n)
  { if (targ) /* feature not implemented for timewindows yet */
    { 
    }
    else
    { for (c=0; c<n; c++)
      { tourOut[c]=tourIn[tourOut[c]];
      }
    }
  }
  *guarantee = noguarantee?0:1;
  return (mycost);
}

costtype DynOpt_NEW(signed char k, nodeXtype n, nodeXtype wn, 
	nodeXtype *tourIn, nodeXtype *tourOut,
	struct Costs_Container* pstCost, struct Dyn_Node_Desc* pstNodeInfo, costtype uiStartTime)

{
	nodeXtype levn;
	//nodeXtype n1, n2, n3, n4;;
	signed char noguarantee = 0, maketour = 0;
	costtype mycost;

	levn = CalcKval_NEW(k, n, kval);
	CalcDepths(kval, depth, levn);
	printf("Optimizing with Dynamic subroutine (k=%d) over %d nodes:\n", k, levn);

	/*costtype Fiddle(signed char k, nodeXtype n, costtype *shortMatrix,
		signed char *j, signed char *minK, int *kval, int *depth,
		nodeXtype *succs, unsigned int32 *succInx,
		nodeXtype *preds, unsigned int32 *predInx,
		signed char *predLoc, nodeXtype *posttour,
		costtype *winBegin, costtype *winEnd, costtype *servTime,
		costtype *costsNow, costtype *costsNext, unsigned char *prevs,
		unsigned char *sublists, int worknodes)

	mycost = Fiddle(localK, levn, shortMatrix[0], j, minK, kval, depth, succs, succInx,
		preds, predInx, predLoc, tourOut, winBegin, winEnd,
		servTime, costsNow, costsNext, workarea,
		sublists, wn);*/

	mycost = Fiddle_NEW(k, levn, pstCost, pstNodeInfo,j , minK, kval, depth, succs, succInx,
		preds, predInx, predLoc, tourOut, costsNow, costsNext, workarea,
		sublists, wn , uiStartTime);

	/*costtype Fiddle_NEW(signed char k, nodeXtype n, struct Costs_Container* pstCost, struct Dyn_Node_Desc* pstAuxNodeInfo,
		signed char *j, signed char *minK, int *kval, int *depth,
		nodeXtype *succs, unsigned int32 *succInx,
		nodeXtype *preds, unsigned int32 *predInx,
		signed char *predLoc, nodeXtype *posttour,
		costtype *costsNow, costtype *costsNext, unsigned char *prevs,
		unsigned char *sublists, int worknodes, costtype uiStartTime)*/
	
	return (mycost);
}


/*--*/

#elif _twDist

costtype DynOpt (signed char k, nodeXtype n, nodeXtype wn, char q,
                 int targ, nodeXtype *tourIn, nodeXtype *tourOut,
                 costtype *winBeginRaw, costtype *winEndRaw,
                 costtype *servTimeRaw, char *guarantee)

{ nodeXtype levn, c, d, n1, n2, n3, n4;
  signed char localK, noguarantee=0, maketour=0, bitsforpred=0;
  costtype mycost;

  targ *= _enableshrink;
  if (tourIn[0]==n)
  { maketour=1;
    tourIn[0]=0;
  }
  levn = BuildSmallTWmatrix (k, (targ>0), maketour,
                       winBeginRaw, winBegin, winEndRaw, winEnd,
                       servTimeRaw, servTime, shortMatrix[0],
                       tourIn, levtour, targ, n, kval,
                       &localK, workareaint, workareaint+n*4,
                       &noguarantee);
  
  CalcDepths (kval,depth,levn);

  if (q<32 && localK<8) {bitsforpred=3;}
  if (q<8) {bitsforpred=5;}
  if (q<16 && localK<16) {bitsforpred=4;}
  printf("Optimizing with Dynamic subroutine (k=%d) over %d nodes:\n",localK, levn);
  mycost = FiddleDual(localK,levn,shortMatrix[0],j,minK,kval,depth,
                      succs,succInx,preds,predInx,predLoc,tourOut,
                      winBegin,winEnd,servTime,costsNow,costsNext,
                      timeNow,timeNext,workarea,
                      sublists, wn, q, bitsforpred, &noguarantee);
  if (mycost<inFinity && tourOut[0]<n)
  { if (targ) /* feature not implemented for timewindows yet */
    { 
    }
    else
    { for (c=0; c<n; c++)
      { tourOut[c]=tourIn[tourOut[c]];
      }
    }
  }
  *guarantee = (noguarantee)?0:1;
  return (mycost);
}

/*--*/

#else

costtype DynOpt (signed char k, nodeXtype n, nodeXtype wn,
                 char contRule,
                 int targ, nodeXtype *tourIn, nodeXtype *tourOut,
                 int *kvalues)

#define _symm (contRule>10)
{ nodeXtype levn, c, n1, n2, n3, n4, *workTour;
  costtype mycost;

  targ *= _enableshrink;
  contRule *= _enableshrink;
  levn = BuildSmallMatrix (k, contRule, shortMatrix,
               tourIn, levtour, targ, n, workareaint, workareaint+n*4);
  if (levn==n && kvalues != NULL)
  { CalcDepths (kvalues,depth,levn);
  }
  else
  { kvalues = kval;
    depth[0]=kvalues[0]=1;
    for (c=1;c<n+1;c++) {depth[c]=kvalues[c]=k;}
    if (n>2*k)
    { for (c=1;c<k;c++) {kvalues[c]=c;} }
  }
  printf("Optimizing with Dynamic subroutine (k=%d) over %d nodes:\n", k, levn);
#ifdef _shrink
  if (_symm && levn<n)
  { mycost = Fiddlesym(k,levn,shortMatrix,j,minK,kvalues,depth,
                      succs,succInx,preds,predInx,predLoc,tourOut,
                      orientA,costsNow,costsNext,
                      workarea, sublists, wn);
  }
  else
#endif
  { mycost = Fiddle(k,levn,shortMatrix[0],j,minK,kvalues,depth,
                      succs,succInx,preds,predInx,predLoc,tourOut,
                      0L,0L,0L,costsNow,costsNext,
                      workarea, sublists, wn);
  }
  if (mycost<inFinity && tourOut[0]<levn)
  { if (levn<n) 
    { workTour = workareaint;
      for (n2=n4=0; n2<levtour[0]; n2++)
      { workTour[n4++]=tourIn[n2];
      }
      workTour[n4++]=tourIn[levtour[0]];
      for (n1=1; n1<levn; n1++)
      { n3 = levtour[tourOut[n1]] - levtour[tourOut[n1]-1];
        if (_symm && (orientA[(n1)>>3]&(1<<((n1)&7)))>0)
	{ workTour[n4++]=tourIn[levtour[tourOut[n1]]];
          for (n2=n3-1; n2>0; n2--)
	  { workTour[n4++]=tourIn[levtour[tourOut[n1]-1]+n2];
	  }
	}
	else
	{ for (n2=1; n2<n3; n2++)
	  { workTour[n4++]=tourIn[levtour[tourOut[n1]-1]+n2];
	  }
          workTour[n4++]=tourIn[levtour[tourOut[n1]]];
      } }
      for (;n4<n;n4++)
      { workTour[n4]=tourIn[n4];
      }
      for (n1=0;n1<n;n1++)
      { tourOut[n1] = workTour[n1];
      }
    }
    else
    { for (c=0; c<n; c++)
      { tourOut[c]=tourIn[tourOut[c]];
      }
    }
  }
  else
  { tourOut[0]=n;
  }
  return (mycost);
}

#endif
