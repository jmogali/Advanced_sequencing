/* Subroutines for dynamic program for the Traveling Salesman Problem */
/* www.contrib.andrew.cmu.edu/~neils/tsp/index.html                   */
/* Neil Simonetti, May 1998                                           */
#include "VLNS_Constants.h"

#define _KMAX 24
#define makesure(op,msg) {if((op)==0){fprintf(stderr,msg);exit(17);}}
#define allocErr "Allocation Error"
#define _min(a,b) ((a)>(b)?(b):(a))
#define _max(a,b) ((a)>(b)?(a):(b))
#define _round(n) ((int)((n)+0.5))
#include <string.h>
#include <stdlib.h>

#pragma warning( disable : 4996)

unsigned int32
/* bN[k] is the number of nodes in one layer of G(k)** */
    bN[_KMAX]={     0,         1,         3,       4*2,
                  5*4,       6*8,      7*16,      8*32,
                 9*64,    10*128,    11*256,    12*512,
              13*1024,   14*2048,   15*4096,   16*8192,
             17*16384,  18*32768,  19*65536, 20*131072,
            21*262144, 22*524288,23*1048576,24*2097152},
/* bA[k] is the size needed for the arc lists of G(k)** */
    bA[_KMAX]={     0,         2,         5,       6*2,
                  7*4,       8*8,      9*16,     10*32,
                11*64,    12*128,    13*256,    14*512,
              15*1024,   16*2048,   17*4096,   18*8192,
             19*16384,  20*32768,  21*65536, 22*131072,
            23*262144, 24*524288,25*1048576,26*2097152};

/* ------------------------------------------------------------------- */

void ReadAuxgraph (signed char k, char *j, char *minK,
		   nodeXtype *succs, unsigned int32 *succInx,
		   nodeXtype *preds, unsigned int32 *predInx,
		   char *predLoc)
/* reads auxiliary graph for specific k, with the knowledge  */
/* that the auxgraph files were built with equal or larger k */
{ FILE *myfile;
  signed char *w1,h,lastinf,backup,limk;
  char d1[80], d2[100];
  unsigned int32 c, ninfs, *SclassMap;

  if ((myfile = fopen("auxgraph.where", "r")) == NULL)
  { fprintf(stderr,"assuming auxiliary graph is in current directory...\n");
    d1[0] = '.'; d1[1] = 0;
  }
  else 
  { fscanf(myfile,"%s",d1);
    fclose(myfile);
  }
  if ((myfile = fopen(strcat(strcpy(d2,d1),"/auxgraph.lim"), "r")) == NULL)
  { fprintf(stderr,"error with file auxgraph.lim\n");
    exit(1);
  }
  fscanf(myfile,"%d",&c);
  limk=c;
  fclose (myfile);

  if (k>limk)
  { fprintf(stderr,"k value selected is too large for this auxiliary graph\n");
    exit(1);
  }

  makesure(SclassMap =
           (unsigned int32 *)calloc( ( 1<<(limk-1) ) +1,sizeof(int32)),
           allocErr);
  if ((myfile = fopen(strcat(strcpy(d2,d1),"/auxgraph.s.arc"), "rb")) == NULL)
  { fprintf(stderr,"error with file auxgraph.s.arc\n");
    exit(1);
  }
  for (w1=(char *)succs,c=ninfs=lastinf=0;c<bA[k];c++)
  { for (h=0;h<4;h++,w1++) {fscanf(myfile,"%c",w1);}
    if (succs[c]>=bN[k])
    { backup = (lastinf || succs[c]!=inFinity);
      if (succs[c]==inFinity)
      { SclassMap[++ninfs]=c+1-lastinf;
        lastinf=1;
      }
      if (backup) {w1-=4;c--;}
    }
    else {lastinf=0;}
  }
  fclose (myfile);

  if ((myfile = fopen(strcat(strcpy(d2,d1),"/auxgraph.s.inx"), "rb")) == NULL)
  { fprintf(stderr,"error with file auxgraph.s.inx\n");
    exit(1);
  }
  for (w1=(char *)succInx,c=0;c<bN[k];c++)
  { for (h=0;h<4;h++,w1++) {fscanf(myfile,"%c",w1);}
    succInx[c]=SclassMap[succInx[c]];
  }
  fclose (myfile);

  if ((myfile = fopen(strcat(strcpy(d2,d1),"/auxgraph.p.arc"), "rb")) == NULL)
  { fprintf(stderr,"error with file auxgraph.p.arc\n");
    exit(1);
  }
  for (w1=(char *)preds,c=ninfs=lastinf=0;c<bA[k];c++)
  { for (h=0;h<4;h++,w1++) {fscanf(myfile,"%c",w1);}
    if (preds[c]>=bN[k])
    { backup = (lastinf || preds[c]!=inFinity);
      if (preds[c]==inFinity)
      { SclassMap[++ninfs]=c+1-lastinf;
        lastinf=1;
      }
      if (backup) {w1-=4;c--;}
    }
    else {lastinf=0;}
  }
  fclose (myfile);

  if ((myfile = fopen(strcat(strcpy(d2,d1),"/auxgraph.p.inx"), "rb")) == NULL)
  { fprintf(stderr,"error with file auxgraph.p.inx\n");
    exit(1);
  }
  for (w1=(char *)predInx,c=0;c<bN[k];c++)
  { for (h=0;h<4;h++,w1++) {fscanf(myfile,"%c",w1);}
    predInx[c]=SclassMap[predInx[c]];
  }
  fclose (myfile);
   
  if ((myfile = fopen(strcat(strcpy(d2,d1),"/auxgraph.j"), "rb")) == NULL)
  { fprintf(stderr,"error with file auxgraph.j\n");
    exit(1);
  }
  for (w1=j,c=0;c<bN[k];c++,w1++){fscanf(myfile,"%c",w1);}
  fclose (myfile);

  if ((myfile = fopen(strcat(strcpy(d2,d1),"/auxgraph.mk"), "rb")) == NULL)
  { fprintf(stderr,"error with file auxgraph.mk\n");
    exit(1);
  }
  for (w1=minK,c=0;c<bN[k];c++,w1++){fscanf(myfile,"%c",w1);}
  fclose (myfile);

  if ((myfile = fopen(strcat(strcpy(d2,d1),"/auxgraph.loc"), "rb")) == NULL)
  { fprintf(stderr,"error with file auxgraph.loc\n");
    exit(1);
  }
  for (w1=predLoc,c=0;c<bN[k];c++,w1++){fscanf(myfile,"%c",w1);}
  fclose (myfile);
  //cfree(SclassMap);
  free(SclassMap);
}

/* ------------------------------------------------------------------- */

void CalcDepths (int *kval, int *depth, nodeXtype n)
/* calculates node depths from k values */
{ nodeXtype n1, n2;

  for (n1=0; n1<n; n1++)
  {depth[n1]=1;}
  for (n1=0; n1<n; n1++)
  { for (n2=_min(n-1,n1+kval[n1]-1); n2>n1-1; n2--)
    { depth[n2]=_max(depth[n2],kval[n1]);
    }
  }
  depth[n]=depth[0]=1;
}

/* ------------------------------------------------------------------- */

nodeXtype BuildSmallMatrix (signed char k, char contRule,
                            costtype **shortMatrix, nodeXtype *tour,
                            nodeXtype *levtour, costtype targ,
                            nodeXtype n, 
                            int *shrinkProbs, costtype *addlCost)

{ nodeXtype c1,c2,levn,z;
 // signed int c3;
  costtype *walkPtr;
  int probtotal=0, maxcost=0, h;

  levn = n;
  tour[n]=tour[0];
  for (c1=0,walkPtr=addlCost;c1<n;c1++,*(walkPtr++)=0);
  if (contRule)
  { if (targ) /* contracting arcs */
    { switch (contRule)
      { case 1: case 11:
          for (c1=0; c1<n; shrinkProbs[c1++]=1);
          probtotal=n;
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
        case 8: case 18:
          for (c1=0; c1<n; c1++)
          { shrinkProbs[c1] = h = theNorm(tour[c1],tour[c1+1]);
            if (h > maxcost) {maxcost=h;}
          }
          for (c1=0; c1<n; c1++)
          { probtotal+=(shrinkProbs[c1]=
                (maxcost-shrinkProbs[c1])*(maxcost-shrinkProbs[c1]));
          }
          break;
      }
      for (c1=0; c1<n; c1++)
      { if (shrinkProbs[c1])
        { shrinkProbs[c1] = targ*((shrinkProbs[c1]<<15)/probtotal);
      } }

      for (levn=0,c1=0;c1<n;c1++)
      { //if ((random()&32767) >= shrinkProbs[c1])
		  if (( (rand() % 32767 ) & 32767) >= shrinkProbs[c1])
        { levtour[levn++] = c1;}
        else
        { addlCost[levn] += theNorm(tour[c1],tour[c1+1]); }
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
    { for (c2 = _max(0,c1-k+1); c2 < _min(levn+1,c1+2*k); c2++)
      { shortMatrix[0][c1*(3*k-1)+c2-c1+k-1] = addlCost[c1]+
          theNorm(tour[levtour[c1]],
                  tour[(c2)?((z=levtour[c2-1]+1)<n)?z:z-n:0]);
        if (contRule>10)
        {
        shortMatrix[1][c1*(3*k-1)+c2-c1+k-1] = addlCost[c1]+
          theNorm(tour[levtour[c1]],tour[levtour[c2]]);
        shortMatrix[2][c1*(3*k-1)+c2-c1+k-1] = addlCost[c1]+
          theNorm(tour[(c1)?levtour[c1-1]+1:0],
                  tour[(c2)?levtour[c2-1]+1:0]);
        shortMatrix[3][c1*(3*k-1)+c2-c1+k-1] = addlCost[c1]+
          theNorm(tour[(c1)?levtour[c1-1]+1:0],tour[levtour[c2]]);
        }
    } }
  }
  else
  { for (c1 = 0; c1 < levn; c1++)
    { for (c2 = _max(0,c1-k+1); c2 < _min(levn+1,c1+2*k); c2++)
      { shortMatrix[0][c1*(3*k-1)+c2-c1+k-1] =
          theNorm(tour[c1],tour[c2<n?c2:c2-n]);
  } } }


  return (levn);
}
