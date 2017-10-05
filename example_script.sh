#!/bin/bash
# Invoke MIP on the tspLib/tsp instance files mentioned below in paralel.
outputFile=Results.txt

#message="BAP, 1 threads, 30 min timelimit, asynchronous model, lemon"
#header="name	nrNodes	edges	lowerBound	objectiveValue	rootNode	optimal	feasible	iterations	nrGeneratedColumns	processedNodes	nrConstrPenaltyUpdates	totalSolveTime	masterSolveTime	pricingSolveTime"

prefix="Tag:"

printf "Read Files"
touch ${outputFile}
touch ${outputFile}_2
echo "${prefix}${message}" > ${outputFile}
echo "${prefix}${header}" >> ${outputFile}

bTSPInstances=bash cat Batch_Files_hard.list | parallel -P 1 -k --eta >> ${outputFile}

#printf " " "${bTSPInstances[@]}" | parallel -P 1 -k --eta --colsep ' ' ' 0 1 0 2 512' >> ${outputFile}

mv ${outputFile} "${outputFile}_tmp"
grep ${prefix} "${outputFile}_tmp" | sed -e "s/$prefix//g" > ${outputFile}
sed 's/Results.txt_tmp://g' ${outputFile} > ${outputFile}_2
mv ${outputFile}_2 "${outputFile}"
rm "${outputFile}_tmp"
rm "${outputFile}_2"
