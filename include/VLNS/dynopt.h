#include "Dyn_Node_Desc.h"
#include "Costs_Container.h"
#include "VLNS_Constants.h"

/* Subroutines for dynamic program for the Traveling Salesman Problem */
/* www.contrib.andrew.cmu.edu/~neils/tsp/index.html                   */
/* Neil Simonetti, May 1998                                           */

costtype Fiddle (signed char k, nodeXtype n, costtype *shortMatrix,
                 signed char *j, signed char *minK, int *kval, int *depth,
                 nodeXtype *succs, unsigned int32 *succInx,
                 nodeXtype *preds, unsigned int32 *predInx,
                 signed char *predLoc, nodeXtype *posttour,
                 costtype *winBegin, costtype *winEnd, costtype *servTime,
                 costtype *costsNow, costtype *costsNext, unsigned char *prevs,
                 unsigned char *sublists, int worknodes)
/* find a better tour, return in posttour.  The ordering in posttour */
/* is relative to that in pretour.  The two must be compared to get */
/* the absolute tour. */
/* the distance of the tour is returned in shortMatrix[0] */
{ signed int n1, n2, nn1, nn2;
  unsigned int32 c1, c2;
  costtype *swapper, baseCost, h, h2;
  unsigned int32 backtracker, offset;
//  char *w1;
  signed char offchar;

  costsNow[0]=0;
  for (n1=n2=0;n1<n;n1++,n2++)
  { 
	if (n2>=worknodes) {n2 = 0;}
    for (c1=0;c1<bN[depth[n1+1]];c1++)
    { 
		costsNext[c1]=inFinity;
    }

    for (c1=0;c1<bN[depth[n1]];c1++)
    { 
	  if (kval[j[c1]+n1] >= minK[c1])
      { 
		  baseCost=costsNow[c1];
		  if (_timewindow && baseCost < inFinity) // check against TW boundaries
		  { 
			  if ((26 == j[c1] + n1) && (n1 == 17))
			  {
				  int iTemp = 0;
			  }
			  else if (j[c1] + n1 > n)
			  {
				  printf("%d \n", j[c1] + n1);
				  int iTemp = 0;
			  }

			  if (baseCost < winBegin[j[c1]+n1]) {baseCost=winBegin[j[c1]+n1];}
			  else if (baseCost > winEnd[j[c1]+n1]) {baseCost=inFinity;}
			  baseCost+=servTime[j[c1]+n1];
		  }
		  if (baseCost<inFinity)
		  {
			  for (c2=succInx[c1];succs[c2]<bN[depth[n1+1]];c2++)
			  { 
				  if (j[succs[c2]] + n1 + 1 > 26)
				  {
					  int iCost, itemp;
					  iCost = shortMatrix[smINX(j[c1] + n1, j[succs[c2]] + n1 + 1, k)];
					  printf("%d, %d -: %d\n", j[c1] + n1, j[succs[c2]] + n1 + 1, iCost);
					  itemp = 0;
				  }

				  if ( (h=baseCost+shortMatrix[smINX(j[c1]+n1,j[succs[c2]]+n1+1,k)]) < costsNext[succs[c2]] )
				  { 
					 costsNext[succs[c2]]=h;
					 if (!_costonly) 
					 {
						 prevs[n2*bN[k]+succs[c2]]=predLoc[c1];
					 }
				  }
			  }
			}
       }
    }

    swapper = costsNext;
    costsNext = costsNow;
    costsNow = swapper;
    if (!_costonly && prevs[n2*bN[k]] > 0)
    { 
		backtracker=0;
		for (nn2=n2,nn1=n1,c1=0; (nn1>n1-worknodes && backtracker >= (c1>0)); nn1--,nn2--,c1++)
		{ 
			if (nn2<0) {nn2+=worknodes;}
			offset=prevs[nn2*bN[k]+backtracker];
			backtracker=preds[predInx[backtracker]+offset];
			sublists[n1*(worknodes+1)+c1]=j[backtracker];
		}
      sublists[n1*(worknodes+1)+c1]=127;
      if (backtracker) {_costonly = 1;}
    }
    else {sublists[n1*(worknodes+1)]=127;}
  }

  /* reconstruct tour */
  if (!_costonly)
  { for (n1=n-1;n1>=0;)
    { if (sublists[n1*(worknodes+1)]!=127)
      { for (nn1=n1,c1=0;(offchar=sublists[nn1*(worknodes+1)+c1])!=127;c1++)
        { posttour[n1] = n1+offchar;
          n1--;
        }
      }
      else
      { posttour[n1] = n1;
        n1--;
      }
    }
  }
  else
  { posttour[0]=n;
  }

  if (_timewindow) /* return distance in shortMatrix[0] */
  { h = h2 = 0;
    for (n1=1;n1<n;n1++)
    { h += shortMatrix[smINX(posttour[n1-1],posttour[n1],k)];
      h2 = _max(h2+shortMatrix[smINX(posttour[n1-1],posttour[n1],k)],
               winBegin[posttour[n1]])
         +servTime[posttour[n1]];
    }
    shortMatrix[0] = (h = h + shortMatrix[smINX(posttour[n-1],n,k)]);
    h2 += shortMatrix[smINX(posttour[n-1],n,k)]
              +servTime[n];
    h = costsNow[0]+servTime[0];
  }
  else
  { h = costsNow[0];
  }
  return(h);
}

costtype Fiddle_NEW(signed char k, nodeXtype n, struct Costs_Container* pstCost, struct Dyn_Node_Desc* pstAuxNodeInfo,
	signed char *j, signed char *minK, int *kval, int *depth,
	nodeXtype *succs, unsigned int32 *succInx,
	nodeXtype *preds, unsigned int32 *predInx,
	signed char *predLoc, nodeXtype *posttour,
	costtype *costsNow, costtype *costsNext, unsigned char *prevs,
	unsigned char *sublists, int worknodes, costtype uiStartTime)
	/* find a better tour, return in posttour.  The ordering in posttour */
	/* is relative to that in pretour.  The two must be compared to get */
	/* the absolute tour. */
	/* the distance of the tour is returned in shortMatrix[0] */
{
	signed int n1, n2, nn1, nn2;
	unsigned int32 c1, c2;
	costtype *swapper, baseCost, h;
	unsigned int32 backtracker, offset;
	//  char *w1;
	signed char offchar;
	unsigned int uiEntered;

	//costsNow[0] = 0;
	costsNow[0] = uiStartTime;
	costtype iOptCost = inFinity;

	for (n1 = n2 = 0; n1<n; n1++, n2++)
	{
		if (n2 >= worknodes) { n2 = 0; }
		for (c1 = 0; c1<bN[depth[n1 + 1]]; c1++)
		{
			costsNext[c1] = inFinity;
		}

		uiEntered = 0;
		for (c1 = 0; c1<bN[depth[n1]]; c1++)
		{
			int iVtx1 = j[c1] + n1;
			if (kval[iVtx1] >= minK[c1])
			{
				baseCost = costsNow[c1];
				if (_timewindow && baseCost < inFinity) // check against TW boundaries
				{
					baseCost = getEST(pstCost, iVtx1, baseCost, &pstAuxNodeInfo[c1] , n1, n);
					baseCost += getProcTime(pstCost, iVtx1);

					uiEntered = 1;
					/*if (baseCost < winBegin[j[c1] + n1]) { baseCost = winBegin[j[c1] + n1]; }
					else if (baseCost > winEnd[j[c1] + n1]) { baseCost = inFinity; }
					baseCost += servTime[j[c1] + n1];*/
				}
				if (baseCost<inFinity)
				{
					int iTravTime;
					int iVtx2;

					for (c2 = succInx[c1]; succs[c2]<bN[depth[n1 + 1]]; c2++)
					{
						iVtx2 = j[succs[c2]] + n1 + 1;
						iTravTime = getTravTime(pstCost, iVtx1, iVtx2, n);
						//if ((h = baseCost + shortMatrix[smINX(j[c1] + n1, j[succs[c2]] + n1 + 1, k)]) < costsNext[succs[c2]])
						if ((h = baseCost + iTravTime) < costsNext[succs[c2]])
						{
							costsNext[succs[c2]] = h;
							if (!_costonly)
							{
								prevs[n2*bN[k] + succs[c2]] = predLoc[c1];
							}

							if (n == n1 + 1) 
							{
								iOptCost = (h <= iOptCost) ? h : iOptCost;
							}
						}
					}
				}
			}
		}

		if (0 == uiEntered) 
		{
			return inFinity;
		}

		swapper = costsNext;
		costsNext = costsNow;
		costsNow = swapper;
		if (!_costonly && prevs[n2*bN[k]] > 0)
		{
			backtracker = 0;
			for (nn2 = n2, nn1 = n1, c1 = 0; (nn1>n1 - worknodes && backtracker >= (c1>0)); nn1--, nn2--, c1++)
			{
				if (nn2<0) { nn2 += worknodes; }
				offset = prevs[nn2*bN[k] + backtracker];
				backtracker = preds[predInx[backtracker] + offset];
				sublists[n1*(worknodes + 1) + c1] = j[backtracker];
			}
			sublists[n1*(worknodes + 1) + c1] = 127;
			if (backtracker) { _costonly = 1; }
		}
		else { sublists[n1*(worknodes + 1)] = 127; }
	}

	/* reconstruct tour */
	if (!_costonly)
	{
		for (n1 = n - 1; n1 >= 0;)
		{
			if (sublists[n1*(worknodes + 1)] != 127)
			{
				for (nn1 = n1, c1 = 0; (offchar = sublists[nn1*(worknodes + 1) + c1]) != 127; c1++)
				{
					posttour[n1] = n1 + offchar;
					n1--;
				}
			}
			else
			{
				posttour[n1] = n1;
				n1--;
			}
		}
	}
	else
	{
		posttour[0] = n;
	}

	/* return distance in shortMatrix[0] */
	/*if (_timewindow) 
	{
		h = h2 = 0;
		for (n1 = 1; n1<n; n1++)
		{
			h += shortMatrix[smINX(posttour[n1 - 1], posttour[n1], k)];
			h2 = _max(h2 + shortMatrix[smINX(posttour[n1 - 1], posttour[n1], k)],
				winBegin[posttour[n1]])
				+ servTime[posttour[n1]];
		}
		shortMatrix[0] = (h = h + shortMatrix[smINX(posttour[n - 1], n, k)]);
		h2 += shortMatrix[smINX(posttour[n - 1], n, k)]
			+ servTime[n];
		h = costsNow[0] + servTime[0];
	}
	else
	{
		h = costsNow[0];
	}*/

	//return(h);
	return iOptCost;
}

/* ----------------------------------------------------------------- */

#ifdef _shrink
costtype Fiddlesym (signed char k, nodeXtype n, costtype **shortMatrix,
                  signed char *j, signed char *minK, int *kval, int *depth,
                  unsigned int32 *succs, unsigned int32 *succInx,
                  unsigned int32 *preds, unsigned int32 *predInx,
                  signed char *predLoc, nodeXtype *posttour, signed char *orientA,
                  costtype *costsNow, costtype *costsNext, unsigned char *prevs,
                  unsigned char *sublists, int worknodes)
/* find a better tour, return in posttour.  The ordering in posttour */
/* is relative to that in pretour.  The two must be compared to get */
/* the absolute tour. */
{ signed int n1, n2, nn1, nn2;
  unsigned int32 c1, c2, c3;
  costtype *swapper, baseCost, h, h2;
  unsigned int32 backtracker, offset;
  char *w1;
  signed char offchar;
  unsigned char orient;
  FILE *myfile;

  costsNow[0]=0;
  costsNow[bN[k]]=inFinity;
  for (n1=0; n1<(n+7)/8; orientA[n1++]=0);
  for (n1=n2=0;n1<n;n1++,n2++)
  { /*fprintf(stderr,"%3d ",n1);
    for (c1=0;c1<1;c1++) {fprintf(stderr,"%d,",costsNow[c1]);}
    if (1) {printf("\n");}*/
    if (n2>=worknodes) {n2 = 0;}
    for (c1=0;c1<bN[depth[n1+1]];c1++)
    { costsNext[c1]=costsNext[c1+bN[k]]=inFinity;
    }
    for (c1=0;c1<bN[depth[n1]];c1++)
    { if (kval[j[c1]+n1] >= minK[c1])
      {for (c3=0;c3<2;c3++)
       {baseCost=costsNow[c1+c3*bN[k]];
        if (baseCost<inFinity)
        { for (c2=succInx[c1];succs[c2]<bN[depth[n1+1]];c2++)
          { if ( (h=baseCost+shortMatrix[2*c3][smINX(j[c1]+n1,j[succs[c2]]+n1+1,k)])
                 < costsNext[succs[c2]])
            { costsNext[succs[c2]]=h;
              if (!_costonly) {prevs[n2*2*bN[k]+succs[c2]]=predLoc[c1]+128*c3;}
            }
            if ( (h=baseCost+shortMatrix[1+2*c3][smINX(j[c1]+n1,j[succs[c2]]+n1+1,k)])
                 < costsNext[bN[k]+succs[c2]])
            { costsNext[bN[k]+succs[c2]]=h;
              if (!_costonly) {prevs[n2*2*bN[k]+bN[k]+succs[c2]]=predLoc[c1]+128*c3;}
            }
          }
        }
      }}
    }
    swapper = costsNext;
    costsNext = costsNow;
    costsNow = swapper;
    if (!_costonly && prevs[n2*2*bN[k]] > 0)
    { orient=backtracker=0;
      for (nn2=n2,nn1=n1,c1=0;(nn1>n1-worknodes && backtracker >= (c1>0));nn1--,nn2--,c1++)
      { if (nn2<0) {nn2+=worknodes;}
        offset=prevs[nn2*2*bN[k]+orient*bN[k]+backtracker];
        orient=offset>>7;
        offset&=127;
        backtracker=preds[predInx[backtracker]+offset];
        sublists[n1*(2*worknodes+2)+c1*2]=j[backtracker];
        sublists[n1*(2*worknodes+2)+1+c1*2]=orient;
      }
      sublists[n1*(2*worknodes+2)+c1*2]=127;
      if (backtracker) {_costonly = 1;}
    }
    else {sublists[n1*(2*worknodes+2)]=127;}
  }

  if (!_costonly)
  { for (n1=n-1;n1>=0;)
    { if (sublists[n1*(2*worknodes+2)]!=127)
      { for (nn1=n1,c1=0;(offchar=sublists[nn1*(2*worknodes+2)+c1*2])!=127;c1++)
        { posttour[n1] = n1+offchar;
          if (sublists[nn1*(2*worknodes+2)+1+c1*2])
          { orientA[n1>>3] |= (1<<(n1&7));
          }
          n1--;
        }
      }
      else
      { posttour[n1] = n1;
        n1--;
      }
    }
  }
  else
  { posttour[0]=n;
  }

  h = costsNow[0];
  return(h);
}
#endif

/* ----------------------------------------------------------------- */

#ifdef _twDist
#define _packed (bitsforpred > 0)
costtype FiddleDual (signed char k, nodeXtype n, costtype *shortMatrix,
		  signed char *j, signed char *minK, int *kval,
		  int *depth, unsigned int32 *succs,
		  unsigned int32 *succInx, unsigned int32 *preds,
		  unsigned int32 *predInx, signed char *predLoc,
		  nodeXtype *posttour, costtype *winBegin, costtype *winEnd,
                  costtype *servTime,
                  costtype *costsNow, costtype *costsNext, costtype *timeNow,
                  costtype *timeNext, unsigned char *prevs,
		  unsigned char *sublist, int worknodes,
                  char q, char bitsforpred, char *compromise)
/* find a better tour, return in posttour.  The ordering in posttour */
/* is relative to that in pretour.  The two must be compared to get */
/* the absolute tour. */
{ signed int n1, n2, nn1, nn2;
  unsigned int32 c1, c2, c3, c4, dest;
  costtype *swapper, basecost, basetime, mycost, mytime, h, h2;
  unsigned int32 backtracker, backdepth, offset;
  char *w1;
  signed char offchar;
  FILE *myfile;
  unsigned char *prevThick, myplace, mythick, maxThick, maxThickTour, stillgoing;

  maxThick = maxThickTour = 0;
  if (!_packed) {prevThick = prevs+(q*worknodes*bN[k]);}

  for (n1=0;n1<q;n1++)
  { costsNow[n1]=timeNow[n1]=0;
  }
  for (n1=n2=0;n1<n;n1++,n2++)
  { if (n2>=worknodes) {n2 = 0;}
    for (c1=0;c1<bN[depth[n1+1]]*q;c1++)
    { costsNext[c1]=timeNext[c1]=inFinity;
    }
    for (c1=0;c1<bN[depth[n1]];c1++)
    { if (kval[j[c1]+n1] >= minK[c1]) 
      { for (c3=0;
             (c3<q && (basecost=costsNow[c3+c1*q]+servTime[j[c1]+n1])<inFinity);
             c3++)
        { basetime=timeNow[c3+c1*q]+servTime[j[c1]+n1];
          for (c2=succInx[c1];(dest=succs[c2])<bN[depth[n1+1]];c2++)
	  { mycost=basecost+
              (h=shortMatrix[_max(0,smINX(j[c1]+n1,j[dest]+n1+1,k))]);
            mytime=_max(basetime+h,winBegin[j[dest]+n1+1]);
            myplace=predLoc[c1];
	    if (_packed) {myplace |= (c3<<bitsforpred);}
	    else {mythick=c3;}
            if (mytime <= winEnd[j[dest]+n1+1])
	    { for (c4=0,stillgoing=1;c4<q && stillgoing;)
	      { if (mycost >= costsNext[dest*q+c4] && mytime >= timeNext[dest*q+c4])
		  {stillgoing=0;}
                else if (mycost >= inFinity)
                  {stillgoing=0;}
                else if (mycost < costsNext[dest*q+c4])
		{
                  h = costsNext[dest*q+c4];
                  costsNext[dest*q+c4] = mycost;
                  mycost = h;
                  h = timeNext[dest*q+c4];
                  timeNext[dest*q+c4] = mytime;
                  mytime = h;
                  if (!_costonly)
		  { h = prevs[c4*worknodes*bN[k]+n2*bN[k]+dest];
                    prevs[c4*worknodes*bN[k]+n2*bN[k]+dest] = myplace;
                    myplace = h;
                    if (!_packed)
		    { h = prevThick[c4*worknodes*bN[k]+n2*bN[k]+dest];
                      prevThick[c4*worknodes*bN[k]+n2*bN[k]+dest] = mythick;
                      mythick = h;
		  } }
                }
                else {c4++;}
              }
              maxThick = _max(c4,maxThick);
    } } } } }
    swapper = costsNext;
    costsNext = costsNow;
    costsNow = swapper;
    swapper = timeNext;
    timeNext = timeNow;
    timeNow = swapper;
    if (!_costonly && (prevs[n2*bN[k]]>0 || (!_packed && prevThick[n2*bN[k]]>0)))
    { backtracker=backdepth=0;
      for (nn2=n2,nn1=n1,c1=0;(nn1>n1-worknodes && backtracker+backdepth >= (c1>0));nn1--,nn2--,c1++)
      { 
        if (nn2<0) {nn2+=worknodes;}
        offset=prevs[backdepth*worknodes*bN[k]+nn2*bN[k]+backtracker];
	if (_packed) {backdepth=offset>>(bitsforpred);offset&=((1<<bitsforpred)-1);}
	else {backdepth=prevThick[backdepth*worknodes*bN[k]+nn2*bN[k]+backtracker];}
        backtracker=preds[predInx[backtracker]+offset];
        sublists[n1*(worknodes+1)+c1]=j[backtracker];
        maxThickTour=_max(maxThickTour,backdepth);
      }
      sublists[n1*(worknodes+1)+c1]=127;
      if (backtracker || backdepth) {_costonly = 1;}
    }
    else {sublists[n1*(worknodes+1)]=127;}
  }

  if (maxThick==q)
  { compromise[0]|=1;
    printf("q must be larger to guarantee optimality.\n");
  }
  else
  { printf("Maximum Thickness in graph was %d\n",maxThick+1);
  }

  if (!_costonly)
  { for (n1=n-1;n1>=0;)
    { if (sublists[n1*(worknodes+1)]!=127)
      { for (nn1=n1,c1=0;(offchar=sublists[nn1*(worknodes+1)+c1])!=127;c1++)
        { posttour[n1] = n1+offchar;
          n1--;
        }
      }
      else
      { posttour[n1] = n1;
        n1--;
      }
    }
  }
  else
  { posttour[0]=n;
  }

//  printf("Maximum Thickness in tour was %d\n",maxThickTour+1);

  h = costsNow[0]+servTime[0];
  return(h);
}

#endif
