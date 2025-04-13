#!/bin/bash

set -e
echo "Running Experiment 1 and the balanced matching algorithm"

for n in 0 1 2 3 4 5 6 7 8 9 10 11
do
	for i in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49
	do
		echo "Running Bal-Matching plot_${n}_$i"
		../build/find-assignment "Experiments1/plot_${n}_$i.txt" 3 1 Experiments1/ $n >> Experiments1/output.txt&
		wait
	done
done


