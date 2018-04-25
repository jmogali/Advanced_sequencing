#ifndef AGTW_H
#define AGTW_H

/* Subroutines for dynamic program for the Traveling Salesman Problem */
/* www.contrib.andrew.cmu.edu/~neils/tsp/index.html                   */
/* Neil Simonetti, May 1998                                           */

/* ------------------------------------------------------------------- */
#include "VLNS_Constants.h"

void SortPretour (costtype *criterion, nodeXtype *tour, nodeXtype n)
/* constructs an initial tour by sorting by the criterion */
/* selection sort should be replaced by a better sorting  */
/* algorithm in future for large problems */
{ costtype earliest;
  nodeXtype earliestInx;
  signed int c1, c2;

  { for (c1 = 1; c1 < n; c1++)
    { earliest = inFinity;
      for (c2 = 1; c2 < n; c2++)
      { if (criterion[c2] < earliest) { earliest = criterion[earliestInx=c2]; }
      }
      tour[c1]=earliestInx;
      criterion[earliestInx]=inFinity;
  } }
}

/* ------------------------------------------------------------------- */

signed char CalcKval (costtype *winBegin, costtype *winEnd,
		      costtype *servTime, int *kval, signed char k,
		      nodeXtype n, costtype *winDepartBound,
                      char *compromise)
/* construct array of k-values based on time windows */
{ signed int c1, c2;
  costtype bound;
  signed char maxK;

  winDepartBound[n]=inFinity;
  for (bound = inFinity, c1 = n-1; c1 > -1; c1--)
  { bound = winDepartBound[c1] = _min(winBegin[c1]+servTime[c1],bound);
  }
  kval[0]=maxK=1;
  for (c1 = c2 = 1; c1 < n; c1++)
  { 
	 /* if (c1 == n - 2)
	  {
		  c1 = n - 2;
	  }*/
	while (winDepartBound[c2] < winEnd[c1]) 
	{
		c2++;
	}
    while (winDepartBound[c2-1]>winEnd[c1]) 
	{
		c2--;
	}
    kval[c1] = _min(k,_max(1,c2-c1));
    maxK = _max(maxK,c2-c1);
  }
  *compromise = (maxK > k);
  printf("Maximum k requested was %d.\n",maxK);
  if (maxK > k) {printf("  Use this value of k to guarantee optimality.\n");}
  maxK=_min(maxK,k);
  kval[n]=1;
  return (maxK);
}

// Set K values s.t. starting and ending vertices of the tour are specified
nodeXtype CalcKval_NEW(signed char k, nodeXtype n, int *kval)
{
	nodeXtype c1, levn;
	// signed int c3;

	//localK[0] = CalcRvKval(winBegin, winEnd, servTime, kval, k, n, winMid);
	//localK[0] = CalcKval(winBegin, winEnd, servTime, kval, k, n, winMid, compromise);

	kval[0] = 1;
	for (c1 = 1; c1 < n - 1; c1++)
	{
		//if (n - 1 - c1 <= k) kval[c1] = n - 1 - c1;
		//else kval[c1] = k;

		if (n - c1 <= k) kval[c1] = n - c1;
		else kval[c1] = k;
	}
	kval[n-1] = 1;
	kval[n] = 1;

	levn = n;
	return (levn);
}


/* ------------------------------------------------------------------- */

signed char FixKval (costtype *winBegin, costtype *winEnd, costtype *servTime,
                      int *kval, signed char k, nodeXtype n, nodeXtype *tour,
                      costtype *winDepartBound)
/* improve array of k-values by shuffling the tour order */
{ signed int c1, c2, swaps, spot, dm1, dm2, bonus, penalty;
  costtype bound, dm3, dm4, dm5;
  signed char maxK;

  swaps = 0;
  winDepartBound[n]=inFinity;
  for (bound = inFinity, c1 = n-1; c1 > -1; c1--)
  { bound = winDepartBound[c1] = _min(winBegin[c1]+servTime[c1],bound);
  }
  kval[0]=maxK=1;
  for (c1 = c2 = 1; c1 < n; c1++)
  { while (winDepartBound[c2] < winEnd[c1]) {c2++;}
    while (winDepartBound[c2-1]>winEnd[c1]) {c2--;}
    kval[c1] = c2-c1;
  }
  for (c1 = 2; c1 < n-1; c1++)
  { if (kval[c1]+1<kval[c1-1]) /* candidate for swap */
    { for (spot = c1-1; kval[c1]+c1-spot<kval[spot] && spot>0; spot--);
      bonus = kval[++spot];
      penalty = 0;
      for (c2 = spot-1; c2 > 0; c2--)
      { if (kval[c2]==c1-c2) {penalty=c2;}
      }
      if (bonus-1 > kval[penalty]) /* do the swap */
      { swaps = 1;
        dm1 = kval[c1]+c1-spot;
        dm2 = tour[c1];
        dm3 = winBegin[c1];
        dm4 = winEnd[c1];
        dm5 = servTime[c1];
        for (c2=c1; c2>spot; c2--)
        { kval[c2]=kval[c2-1]-1;
          tour[c2]=tour[c2-1];
          winBegin[c2]=winBegin[c2-1];
          winEnd[c2]=winEnd[c2-1];
          servTime[c2]=servTime[c2-1];
        }
        kval[spot] = dm1;
        tour[spot] = dm2;
        winBegin[spot] = dm3;
        winEnd[spot] = dm4;
        servTime[spot] = dm5;
        if (penalty) {kval[penalty] += 1;}
      }
    }
  }
  for (bound = inFinity, c1 = n-1; c1 > -1; c1--)
  { bound = winDepartBound[c1] = _min(winBegin[c1]+servTime[c1],bound);
  }
  for (c1 = c2 = 1; c1 < n; c1++)
  { while (winDepartBound[c2] < winEnd[c1]) {c2++;}
    while (winDepartBound[c2-1]>winEnd[c1]) {c2--;}
    kval[c1] = _min(k,c2-c1);
    maxK = _max(maxK,c2-c1);
  }
//  printf("Maximum k requested was %d.\n",maxK);
  maxK=_min(maxK,k);
  kval[n]=1;
  return (swaps);
}

/* ------------------------------------------------------------------- */

signed char FixRvKval (costtype *winBegin, costtype *winEnd, costtype *servTime,
                      int *kval, signed char k, nodeXtype n, nodeXtype *tour,
                      costtype *winEndBound)
/* improve array of "reverse" k-values by shuffling the tour order */
{ signed int c1, c2, bonus, penalty, spot, swaps, dm1, dm2;
  costtype bound, dm3, dm4, dm5;
  signed char maxK;

  swaps = 0;
  for (bound = 0, c1 = 1; c1 < n; c1++)
  { bound = winEndBound[c1] = _max(winEnd[c1],bound);
  }
  kval[0]=maxK=1;
  for (c1 = c2 = 1; c1 < n; c1++)
  { while (winEndBound[c2] < winBegin[c1]+servTime[c1]) {c2++;}
    while (winEndBound[c2-1]>winBegin[c1]+servTime[c1] && c2>1) {c2--;}
    kval[c1] = c1-c2+1;
  }
  for (c1 = n-2; c1 > 0; c1--)
  { if (kval[c1]+1<kval[c1+1]) /* candidate for swap */
    { for (spot = c1+1; kval[c1]+spot-c1<kval[spot] && spot<n; spot++);
      bonus = kval[--spot];
      penalty = 0;
      for (c2 = spot+1; c2 < n; c2++)
      { if (kval[c2]==c2-c1) {penalty=c2;}
      }
      if (bonus-1 > kval[penalty]) /* do the swap */
      { swaps = 1;
        dm1 = kval[c1]+spot-c1;
        dm2 = tour[c1];
        dm3 = winBegin[c1];
        dm4 = winEnd[c1];
        dm5 = servTime[c1];
        for (c2=c1; c2<spot; c2++)
        { kval[c2]=kval[c2+1]-1;
          tour[c2]=tour[c2+1];
          winBegin[c2]=winBegin[c2+1];
          winEnd[c2]=winEnd[c2+1];
          servTime[c2]=servTime[c2+1];
        }
        kval[spot] = dm1;
        tour[spot] = dm2;
        winBegin[spot] = dm3;
        winEnd[spot] = dm4;
        servTime[spot] = dm5;
        if (penalty) {kval[penalty] += 1;}
      }
    }
  }
  for (bound = 0, c1 = 1; c1 < n; c1++)
  { bound = winEndBound[c1] = _max(winEnd[c1],bound);
  }
  for (c1 = c2 = 1; c1 < n; c1++)
  { while (winEndBound[c2] < winBegin[c1]+servTime[c1]) {c2++;}
    while (winEndBound[c2-1]>winBegin[c1]+servTime[c1] && c2>1) {c2--;}
    kval[c1] = _min(k,c1-c2+1);
    maxK = _max(maxK,c1-c2+1);
  }
//  printf("Maximum k requested was %d.\n",maxK);
  maxK=_min(maxK,k);
  kval[n]=1;
  return (swaps);
}

/* ------------------------------------------------------------------- */

nodeXtype BuildSmallTWmatrix (signed char k, char contRule, char maketour,
             costtype *winBeginRaw, costtype *winBegin, costtype *winEndRaw,
             costtype *winEnd, costtype *servTimeRaw, costtype *servTime,
	     costtype *shortMatrix, nodeXtype *tour, nodeXtype *levtour,
             costtype targ, nodeXtype n, int *kval, signed char *localK,
             int *shrinkProbs, costtype *addlCost, char *compromise)

{ nodeXtype c1,c2,levn,z;
 // signed int c3;
  costtype *walkPtr, *winMid, winBtemp, winEtemp;
  int probtotal=0, maxcost=0, h, modified, modifcount;

  winMid = addlCost;
  if (maketour)
  { for (c1=0; c1<n; c1++)
    { winMid[c1] = (winBeginRaw[c1]+winEndRaw[c1])/2;}
    SortPretour(winMid,tour,n);
    tour[n]=tour[0];
    modified = 1;
    /* save time window and service time information relative to tour */
    for (c1=0; c1<n; c1++)
    { winBegin[c1] = winBeginRaw[tour[c1]];
      winEnd[c1] = winEndRaw[tour[c1]];
      servTime[c1] = servTimeRaw[tour[c1]];
    }
    winBegin[n] = winBegin[0];
    winEnd[n]   = winEnd[0];
    servTime[n] = 0;
    /* establish a better order to minimize large k's */
    modifcount=0;
    while (modified && modifcount<8)
    { modified = FixRvKval(winBegin,winEnd,servTime,kval,k,n,tour,winMid);
      modified |= FixKval(winBegin, winEnd,servTime,kval,k,n,tour,winMid);
      modifcount++;
    }
  }
  else
  { for (c1=0; c1<n; c1++)
    { winBegin[c1] = winBeginRaw[tour[c1]];
      winEnd[c1] = winEndRaw[tour[c1]];
      servTime[c1] = servTimeRaw[tour[c1]];
    }
    winBegin[n] = winBegin[0];
    winEnd[n]   = winEnd[0];
    servTime[n] = 0;
  }
//localK[0] = CalcRvKval(winBegin, winEnd, servTime, kval, k, n, winMid);
  localK[0] = CalcKval  (winBegin, winEnd, servTime, kval, k, n, winMid,
    compromise);
  levn = n;
  for (c1=0,walkPtr=addlCost;c1<n;c1++,*(walkPtr++)=0);
  if (contRule)
  { if (targ) /* contracting arcs */
    { switch (contRule)
      { case 1: case 11:
          for (c1=0; c1<n; shrinkProbs[c1++]=1);
          probtotal=n;
          break;
        case 2: case 12:
          printf("Not Available\n");exit(1);
	  break;
        case 3: case 13:
          for (c1=0; c1<n; c1++)
	  { shrinkProbs[c1] = h = theNorm(tour[c1],tour[c1+1]);
	    if (h > maxcost) {maxcost=h;}
	  }
	  for (c1=0; c1<n; c1++)
	  { probtotal+=(shrinkProbs[c1]=maxcost-shrinkProbs[c1]);
	  }
          break;
//      case 4: case 14:
//        if (random()%k)
//	  { for (c1=0; c1<n; c1++)
//	    { for (h=0, c3=_max(0,c1+1-k); c3<_min(n,c1+k-1); c3++)
//	      { for (c2=0; c2<proxlistsize && tour[c3]!=proxlist[c2]; c2++)
//	        { if (c2<proxlistsize) {h++;}}
//	      }
//            probtotal+=(shrinkProbs[c1]=h);
//	    }
//	  }
//        else /* 1/k chance we shrink nothing */
//	  { for (probtotal=1, c1=0; c1<n; shrinkProbs[c1++]=0);
//	  }
//        break;
      }
      for (c1=0; c1<n; c1++)
      { if (shrinkProbs[c1])
        { shrinkProbs[c1] = targ*((shrinkProbs[c1]<<15)/probtotal);
      } }

      winBtemp=winBeginRaw[tour[0]];
      winEtemp=winEndRaw[tour[0]];
      for (levn=0,c1=0;c1<n;c1++)
      { 
		 //if ((random()&32767) >= shrinkProbs[c1])
		  if (( (rand() % 32767) & 32767) >= shrinkProbs[c1])
        { levtour[levn] = c1;
	  winBegin[levn] = winBtemp;
          winEnd[levn] = winEtemp;
          servTime[levn++] = servTime[tour[c1]];
          winBtemp=winBeginRaw[tour[c1+1]];
          winEtemp=winEndRaw[tour[c1+1]];
        }
        else
        { addlCost[levn] += theNorm(tour[c1],tour[c1+1])+servTime[tour[c1]];
          winBtemp = _max(winBtemp,winBeginRaw[tour[c1+1]]-addlCost[levn]);
          winEtemp = _min(winEtemp,winEndRaw[tour[c1+1]]-addlCost[levn]);
	}
      }
      addlCost[0] += addlCost[levn];
    }
    else
    { for (c1=0; c1<n; c1++) {levtour[c1]=c1;addlCost[c1]=0;}
    }
  }

  /* construct short matrix indexed based on order of nodes in tour */
  if (targ && levn < n)
  { for (c1 = 0; c1 < levn; c1++)
    { for (c2 = _max(0,c1-localK[0]+1); c2 < _min(levn+1,c1+2*localK[0]); c2++)
      { shortMatrix[c1*(3*localK[0]-1)+c2-c1+localK[0]-1] = addlCost[c1]+
	  theNorm(tour[levtour[c1]],tour[(c2)?((z=levtour[c2-1]+1)<n)?z:z-n:0]);
    } }
  }
  else
  { for (c1 = 0; c1 < levn; c1++)
    { for (c2 = _max(0,c1-localK[0]+1); c2 < _min(levn+1,c1+2*localK[0]); c2++)
      { shortMatrix[c1*(3*localK[0]-1)+c2-c1+localK[0]-1] =
          theNorm(tour[c1],tour[c2<n?c2:c2-n]);
  } } }
  return (levn);
}


/* ------------------------------------------------------------------- */

#endif