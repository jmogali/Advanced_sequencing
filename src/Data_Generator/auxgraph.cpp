#include <stdio.h>
#include <stdlib.h>
#include "auxgraph.h"
#include <string.h>

#pragma warning(disable:4996) 
#pragma warning(disable:4018) 

namespace AUX_GRAPH
{
#define tofiles 1 /* 1=write to files, 0=write info to screen (max k=9) */
#define infinity (1<<30) /* 2^30 is close enough */
#define masktype int /* used to declare bit masks */

	//ones(x) returns the number of ones in bit mask x [up to 24]
#define ones(x) ((x&1)+((x>>1)&1)+((x>>2)&1)+((x>>3)&1)+((x>>4)&1)+((x>>5)&1)+((x>>6)&1)+((x>>7)&1)+((x>>8)&1)+((x>>9)&1)+((x>>10)&1)+((x>>11)&1)+((x>>12)&1)+((x>>13)&1)+((x>>14)&1)+((x>>15)&1)+((x>>16)&1)+((x>>17)&1)+((x>>18)&1)+((x>>19)&1)+((x>>20)&1)+((x>>21)&1)+((x>>22)&1)+((x>>23)&1)+((x>>24)&1))

//highbit(x) returns location of the highest one in bit mask x [up to 24]
#define highbit(x) ((x>>15)?(x>>23)?(x>>24)?24:23:(x>>19)?(x>>21)?(x>>22)?22:21:(x>>20)?20:19:(x>>17)?(x>>18)?18:17:(x>>16)?16:15:(x>>7)?(x>>11)?(x>>13)?(x>>14)?14:13:(x>>12)?12:11:(x>>9)?(x>>10)?10:9:(x>>8)?8:7:(x>>3)?(x>>5)?(x>>6)?6:5:(x>>4)?4:3:(x>>1)?(x>>2)?2:1:(x)?0:(-1))

#define max1(a,b) (((a)>b)?((a)>1)?(a):1:((b)>1)?(b):1)
#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)>(b)?(b):(a))

#define numClasses (1<<(k-1)) /* 2^(k-1) */
#define makesure(op,msg) {if((op)==0){fprintf(stderr,msg);exit(17);}}
#define allocErr "Allocation Error"

//printset is used to print bitmask info to the screen
#define printset(x,f) for(c1=0;c1<16;c1++){if(x&(1<<c1)){printf(f,c1);}}

	FILE *myfile;
	signed char
		k,        //value for the precedence constraints
		which2, which,//temporary variables
		*j,       //array to hold j value (as offset from i) for an auxgraph node
		*minK,    //array to hold corresponding minimum k vaule for node j
		*predLoc, //array to hold location into predecessor list for an auxgraph node
		candj,    //used to loop through candidate values for j
		adj,      //flag used to determine if two auxgraph nodes are adjacent
		*w1;      //pointer used to walk through arrays while writing files 
	unsigned int32
		c, c1, c2,    //counter variables
		b,            //number of nodes in an auxiliary graph layer
		*succs,       //array with lists of successor node lists
		*succInx,     //which successor list is used for an auxgraph node
		*preds,       //array with lists of predecessor node lists
		*predInx,     //which predecessor list is used for an auxgraph node
		succCount,    //counter used to build successor lists
		predsCount,   //counter used to build predecessor lists
		predsPoint,   //another counter used to build predecessor lists
		*predCount,   //array of counters used to build predecessor lists
		SclassCount,  //current class of S-minus and S-plus combinations
		*SclassMap,   //mapping of classes to locations within successor lists
		candSrce,     //candidate source for arc in auxgraph
		candDest,     //candidate destination for arc in auxgraph
		candSclass,   //candidate class of nodes for testing auxgraph adjacency
		ktemp;        //counter for progress
	unsigned int32
		//number of nodes in auxgraph for given k.
		bA1[24] = { 0,        1,         3,       4 * 2,
			   5 * 4,      6 * 8,      7 * 16,      8 * 32,
			  9 * 64,   10 * 128,    11 * 256,    12 * 512,
		   13 * 1024,  14 * 2048,   15 * 4096,   16 * 8192,
		  17 * 16384, 18 * 32768,  19 * 65536, 20 * 131072,
			 21 * 262144,22 * 524288,23 * 1048576,24 * 2097152 },
		//number of arcs in auxgraph for given k.
		b2A[24] = { 0,        2,         5,       6 * 2,
				   7 * 4,      8 * 8,      9 * 16,     10 * 32,
			 11 * 64,   12 * 128,    13 * 256,    14 * 512,
		   15 * 1024,  16 * 2048,   17 * 4096,   18 * 8192,
		  19 * 16384, 20 * 32768,  21 * 65536, 22 * 131072,
			 23 * 262144,24 * 524288,23 * 1048576,24 * 2097152 },
		//indices into successor lists while they are built
		counter[24] = { 0,0,      1,         3,       4 * 2,
				   5 * 4,      6 * 8,      7 * 16,      8 * 32,
			  9 * 64,   10 * 128,    11 * 256,    12 * 512,
		   13 * 1024,  14 * 2048,   15 * 4096,   16 * 8192,
		  17 * 16384, 18 * 32768,  19 * 65536, 20 * 131072,
			 21 * 262144,22 * 524288,23 * 1048576 };
	/* bit masks indicating membership */
	masktype //bit arrays indicating membership 
		*Sminus, //uses bits 0 to 23
		*Splus,  //uses bits 1 to 24
		maskmaxplus, maskmaxminus, maskmaxtemp, candSminus, candSplus;
	signed char highPlus, highMinus;

	void generate_new_file_path(const char* fileName, const char* folderPath, char* new_file_path)
	{
		//new_file_path = (char*)malloc(strlen(folderPath) + 1 + strlen(fileName)); /* make space for the new string (should check the return value ...) */
		strcpy(new_file_path, folderPath); /* copy name into the new var */
		strcat(new_file_path, fileName);
	}

	void generate_TSP_files(unsigned int32 uiKVal, const char* cFolderPath)
	{
		return;
		/*if (argc != 2)
		  {fprintf(stderr,"command needs one integer argument (>1)\n"); exit(1); }
		sscanf (argv[1],"%d",&ktemp);*/
		ktemp = uiKVal;
		char *cFilePath = NULL;
		k = ktemp;
		b = bA1[k];
		if (k < 2) { fprintf(stderr, "command needs one integer argument (>1)\n"); exit(1); }
		if (k > 23) { fprintf(stderr, "Surely, you jest\n"); exit(1); }
		maskmaxminus = 1 << (k)-1;
		maskmaxplus = maskmaxminus * 2;

		makesure(Sminus = (masktype *)calloc(b, sizeof(masktype)), allocErr);
		makesure(Splus = (masktype *)calloc(b, sizeof(masktype)), allocErr);
		makesure(j = (signed char *)calloc(b, sizeof(char)), allocErr);
		makesure(minK = (signed char *)calloc(b, sizeof(char)), allocErr);
		makesure(SclassMap = (unsigned int32 *)calloc(numClasses, sizeof(int32)), allocErr);
		makesure(succs = (unsigned int32 *)calloc(numClasses*k, sizeof(int32)), allocErr);
		makesure(succInx = (unsigned int32 *)calloc(b, sizeof(int32)), allocErr);
		makesure(preds = (unsigned int32 *)calloc(numClasses*(k + 1), sizeof(int32)), allocErr);
		makesure(predInx = (unsigned int32 *)calloc(b, sizeof(int32)), allocErr);
		makesure(predLoc = (signed char *)calloc(b, sizeof(char)), allocErr);
		makesure(predCount = (unsigned int32 *)calloc(numClasses, sizeof(int32)), allocErr);
		for (c = 0; c < numClasses*(k + 1); preds[c++] = infinity);

		printf("Generating %d Nodes (%dK) ...\n", b, (b >> 10) + 1);
		SclassCount = succCount = 0;
		for (candSplus = 0; candSplus <= maskmaxplus; candSplus += 2)
		{
			maskmaxtemp = min(maskmaxminus, (1 << (k - (highPlus = highbit(candSplus)))) - 1);
			for (candSminus = 0; candSminus <= maskmaxtemp; candSminus++)
			{
				if (ones(candSplus) == ones(candSminus)) // S-plus & S-minus equal size
				{
					highMinus = highbit(candSminus);
					SclassMap[SclassCount] = succCount;
					for (candj = 1; candj < k; candj++)
					{
						if (candSplus&(1 << candj)) // j is an element of S-plus
						{
							which = max1(highPlus + highMinus + 1, candj + 1);
							j[counter[which]] = -candj;
							Sminus[counter[which]] = candSminus;
							Splus[counter[which]] = candSplus;
							succs[succCount] = counter[which];
							minK[counter[which]] = 1 + candj + highMinus;
							counter[which]++;
							succCount++;
						}
					}
					for (candj = 0; candj < k - max(0, highPlus); candj++)
					{
						if (!(candSminus&(1 << candj))) // j is not an element of S-minus 
						{
							which2 = max1(highPlus + highMinus + 1, candj + highPlus + 1);
							which = max1(which2, candj + 1);
							j[counter[which]] = candj;
							Sminus[counter[which]] = candSminus;
							Splus[counter[which]] = candSplus;
							succs[succCount] = counter[which];
							minK[counter[which]] = 1 + max(highMinus - candj, 0);
							counter[which]++;
							succCount++;
						}
					}
					SclassCount++;
					succs[succCount] = infinity;
					succCount++;
				}
			}
		}

		ktemp = 0;
		printf("Generating Arcs...\n");
		for (candSrce = 0; candSrce < b; candSrce++)
		{
			if ((candSrce & 1023) == 1023)
			{
				fprintf(stderr, "\015%dK out of %dK nodes done", ++ktemp, (b >> 10) + 1);
			}
			for (candSclass = 0; candSclass < numClasses; candSclass++)
			{
				candDest = succs[SclassMap[candSclass]];
				adj = 0;
				if (j[candDest] != j[candSrce] - 1)
				{
					if (j[candSrce] < 0)
					{
						if (Sminus[candSrce] & 1) /* situation (a) */
						{
							adj = (((Sminus[candDest] << 1) == (Sminus[candSrce] - 1)) &&
								((Splus[candDest] >> 1) == (Splus[candSrce] - (1 << (-j[candSrce])))));
						}
						else /* situation (b) */
						{
							adj = (((Sminus[candDest] << 1) == Sminus[candSrce]) &&
								((Splus[candDest] >> 1) == (Splus[candSrce] + 1 - (1 << (-j[candSrce])))));
						}
					}
					else if (j[candSrce] > 0)
					{
						if (Sminus[candSrce] & 1) /* situation (d) */
						{
							adj = (((Splus[candDest] >> 1) == Splus[candSrce]) &&
								((Sminus[candDest] << 1) == (Sminus[candSrce] - 1 + (1 << (j[candSrce])))));
						}
						else /* situation (e) */
						{
							adj = (((Splus[candDest] >> 1) == (Splus[candSrce] + 1)) &&
								((Sminus[candDest] << 1) == (Sminus[candSrce] + (1 << (j[candSrce])))));
						}
					}
					else /* situation (c) */
					{
						if (((Splus[candDest] >> 1) == Splus[candSrce]) &&
							((Sminus[candDest] << 1) == Sminus[candSrce])) {
							adj = 1;
						}
					}
				}
				if (adj) /* there is an arc from candSrce to candDest */
				{
					succInx[candSrce] = candSclass;
					preds[candSclass*(k + 1) + predCount[candSclass]] = candSrce;
					predLoc[candSrce] = predCount[candSclass];
					predCount[candSclass]++;
					candSclass = numClasses;
				}
			}
		}
		for (predsCount = predsPoint = c = 0; c < numClasses; c++)
		{
			for (c1 = SclassMap[c]; succs[c1] < infinity; c1++)
			{
				predInx[succs[c1]] = c;
			}
			do
			{
				preds[predsCount] = preds[predsPoint];
				predsPoint++;
				predsCount++;
			} while (preds[predsPoint - 1] < infinity);
			while (preds[predsPoint] == infinity && c < numClasses - 1)
			{
				predsPoint++;
			}
		}
		fprintf(stderr, "\015All nodes done                    \n");

		if (k < 9 && tofiles == 0)
		{
			for (c = 0; c < b; c++)
			{
				printf("%3d: j=i%+d: minK=%d: S+={", c + 1, j[c], minK[c]);
				printset(Splus[c], ",i-%d");
				printf("}: S-={");
				printset(Sminus[c], ",i+%d");
				printf("}\n");
				printf("    outgoing arcs: (class%3d) ", succInx[c] + 1);
				for (c1 = SclassMap[succInx[c]]; succs[c1] < infinity; c1++)
				{
					printf(",%d", succs[c1] + 1);
				}
				printf("\n");
				printf("    incoming arcs: (class%3d) ", predInx[c] + 1);
				for (c1 = c2 = 0; c2 < predInx[c]; c2 += (preds[c1] == infinity ? 1 : 0), c1++);
				for (; preds[c1] < infinity; c1++)
				{
					printf(",%d", preds[c1] + 1);
				}
				printf("\n");
			}
		}
		else
		{
			printf("Writing to Files...\n");
		}

		/*************   Node Description **************/
		cFilePath = (char*)malloc(strlen(cFolderPath) + 1 + strlen("auxnode_desc.nod"));
		generate_new_file_path("auxnode_desc.nod", cFolderPath, cFilePath);

		//if ((myfile = fopen("auxnode_desc.nod", "w")) == NULL)
		if ((myfile = fopen(cFilePath, "w")) == NULL)
		{
			fprintf(stderr, "error with file auxgraph.lim\n");
			exit(1);
		}

		fprintf(myfile, "NODE_COUNT: %d\n", b);
		for (c = 0; c < b; c++)
		{
			fprintf(myfile, "Node: %d Offset: %d\n", c, j[c]);
			fprintf(myfile, "S+:");

			const unsigned int cuiEnd = 16;

			for (c1 = 0; c1 < cuiEnd; c1++)
			{
				if (Splus[c] & (1 << c1)) fprintf(myfile, " -%d", c1);
			}
			fprintf(myfile, "\n");

			fprintf(myfile, "S-:");
			for (c1 = 0; c1 < cuiEnd; c1++)
			{
				if (Sminus[c] & (1 << c1)) fprintf(myfile, " %d", c1);
			}
			fprintf(myfile, "\n");
		}
		fclose(myfile);
		free(Sminus);
		free(Splus);
		free(cFilePath);
		cFilePath = NULL;

		/*************   Lim Description **************/
		cFilePath = (char*)malloc(strlen(cFolderPath) + 1 + strlen("auxgraph.lim"));
		generate_new_file_path("auxgraph.lim", cFolderPath, cFilePath);
		//if ((myfile = fopen("auxgraph.lim", "w")) == NULL)
		if ((myfile = fopen(cFilePath, "w")) == NULL)
		{
			fprintf(stderr, "error with file auxgraph.lim\n");
			exit(1);
		}
		fprintf(myfile, "%d", k);
		fclose(myfile);
		free(cFilePath);
		cFilePath = NULL;

		/*************   INX.S Description **************/
		cFilePath = (char*)malloc(strlen(cFolderPath) + 1 + strlen("auxgraph.s.inx"));
		generate_new_file_path("auxgraph.s.inx", cFolderPath, cFilePath);
		//if ((myfile = fopen("auxgraph.s.inx", "wb")) == NULL)
		if ((myfile = fopen(cFilePath, "wb")) == NULL)
		{
			fprintf(stderr, "error with file auxgraph.s.inx\n");
			exit(1);
		}
		for (w1 = (signed char *)succInx, c = 0; c < b * 4; c++, w1++) { fprintf(myfile, "%c", *w1); }
		fclose(myfile);
		free(cFilePath);
		cFilePath = NULL;

		/*************   INX.P Description **************/
		cFilePath = (char*)malloc(strlen(cFolderPath) + 1 + strlen("auxgraph.p.inx"));
		generate_new_file_path("auxgraph.p.inx", cFolderPath, cFilePath);
		//if ((myfile = fopen("auxgraph.p.inx", "wb")) == NULL)
		if ((myfile = fopen(cFilePath, "wb")) == NULL)
		{
			fprintf(stderr, "error with file auxgraph.p.inx\n");
			exit(1);
		}
		for (w1 = (signed char *)predInx, c = 0; c < b * 4; c++, w1++) { fprintf(myfile, "%c", *w1); }
		fclose(myfile);
		free(cFilePath);
		cFilePath = NULL;

		/*************   ARC.S Description **************/
		cFilePath = (char*)malloc(strlen(cFolderPath) + 1 + strlen("auxgraph.s.arc"));
		generate_new_file_path("auxgraph.s.arc", cFolderPath, cFilePath);
		//if ((myfile = fopen("auxgraph.s.arc", "wb")) == NULL)
		if ((myfile = fopen(cFilePath, "wb")) == NULL)
		{
			fprintf(stderr, "error with file auxgraph.s.arc\n");
			exit(1);
		}
		for (w1 = (signed char *)succs, c = 0; c < b2A[k] * 4; c++, w1++)
		{
			fprintf(myfile, "%c", *w1);
		}
		fclose(myfile);
		free(cFilePath);
		cFilePath = NULL;

		/*************   ARC.P Description **************/
		cFilePath = (char*)malloc(strlen(cFolderPath) + 1 + strlen("auxgraph.p.arc"));
		generate_new_file_path("auxgraph.p.arc", cFolderPath, cFilePath);
		//if ((myfile = fopen("auxgraph.p.arc", "wb")) == NULL)
		if ((myfile = fopen(cFilePath, "wb")) == NULL)
		{
			fprintf(stderr, "error with file auxgraph.p.arc\n");
			exit(1);
		}
		for (w1 = (signed char *)preds, c = 0; c < b2A[k] * 4; c++, w1++)
		{
			fprintf(myfile, "%c", *w1);
		}
		fclose(myfile);
		free(cFilePath);
		cFilePath = NULL;

		/*************   AUXGRAPH.J Description **************/
		cFilePath = (char*)malloc(strlen(cFolderPath) + 1 + strlen("auxgraph.j"));
		generate_new_file_path("auxgraph.j", cFolderPath, cFilePath);
		//if ((myfile = fopen("auxgraph.j", "wb")) == NULL)
		if ((myfile = fopen(cFilePath, "wb")) == NULL)
		{
			fprintf(stderr, "error with file auxgraph.j\n");
			exit(1);
		}
		for (w1 = j, c = 0; c < b; c++, w1++) { fprintf(myfile, "%c", *w1); }
		fclose(myfile);
		free(cFilePath);
		cFilePath = NULL;

		/*************   AUXGRAPH.MK Description **************/
		cFilePath = (char*)malloc(strlen(cFolderPath) + 1 + strlen("auxgraph.mk"));
		generate_new_file_path("auxgraph.mk", cFolderPath, cFilePath);
		//if ((myfile = fopen("auxgraph.mk", "wb")) == NULL)
		if ((myfile = fopen(cFilePath, "wb")) == NULL)
		{
			fprintf(stderr, "error with file auxgraph.mk\n");
			exit(1);
		}
		for (w1 = minK, c = 0; c < b; c++, w1++) { fprintf(myfile, "%c", *w1); }
		fclose(myfile);
		free(cFilePath);
		cFilePath = NULL;

		/*************   AUXGRAPH.LOC Description **************/
		cFilePath = (char*)malloc(strlen(cFolderPath) + 1 + strlen("auxgraph.loc"));
		generate_new_file_path("auxgraph.loc", cFolderPath, cFilePath);
		//if ((myfile = fopen("auxgraph.loc", "wb")) == NULL)
		if ((myfile = fopen(cFilePath, "wb")) == NULL)
		{
			fprintf(stderr, "error with file auxgraph.loc\n");
			exit(1);
		}
		for (w1 = predLoc, c = 0; c < b; c++, w1++) { fprintf(myfile, "%c", *w1); }
		fclose(myfile);
		free(cFilePath);
		cFilePath = NULL;

		/*************   EOD   **************/

		free(SclassMap);
		free(succs);
		free(succInx);
		free(preds);
		free(predInx);
		free(predLoc);
		free(predCount);
		free(j);
		free(minK);
		for (c = 1; c < k + 1; c++) { printf("[%d-%d]", c, counter[c]); } printf("\nDONE\n");
	}
}