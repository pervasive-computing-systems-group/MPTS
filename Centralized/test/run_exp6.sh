#!/bin/bash

set -e
echo "Running Experiment 1"


# for n in 0 1 2 3 4 5 6 7 8 9 10 11 12
# do
# 	for i in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49
# 	do
# 		echo "Running Match-Auction plot_${n}_$i"
# 		../build/find-assignment "Experiments6/plot_${n}_$i.txt" 4 1 Experiments6/ $n >> Experiments6/output.txt&
# 		wait
# 	done
# done

for n in 0 1 2 3 4 5 6 7 8 9 10 11 12
do
	for i in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49
	do
		echo "Running Bal-Matching plot_${n}_$i"
		../build/find-assignment "Experiments6/plot_${n}_$i.txt" 3 1 Experiments6/ $n >> Experiments6/output.txt&
		wait
	done
done

for n in 0 1 2 3 4 5 6 7 8 9 10 11 12
do
	for i in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49
	do
		echo "Running Min-Dist plot_${n}_$i"
		../build/find-assignment "Experiments6/plot_${n}_$i.txt" 8 1 Experiments6/ $n >> Experiments6/output.txt&
		wait
	done
done

for n in 0 1 2 3 4 5 6 7 8 9 10 11 12
do
	for i in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49
	do
		echo "Running Gradient Search plot_${n}_$i"
		../build/find-assignment "Experiments6/plot_${n}_$i.txt" 6 1 Experiments6/ $n >> Experiments6/output.txt&
		wait
	done
done

for n in 0 1 2 3 4 5 6 7 8 9 10 11 12
do
	for i in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49
	do
		echo "Running Matching + Gradient Search plot_${n}_$i"
		../build/find-assignment "Experiments6/plot_${n}_$i.txt" 7 1 Experiments6/ $n >> Experiments6/output.txt&
		wait
	done
done

for n in 0 1 2 3 4 5 6 7 8 9 10 11 12
do
	for i in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49
	do
		echo "Running Match and Swap plot_${n}_$i"
		../build/find-assignment "Experiments6/plot_${n}_$i.txt" 10 1 Experiments6/ $n >> Experiments6/output.txt&
		wait
	done
done

for n in 0 1 2 3 4 5 6 7 8 9 10 11 12
do
	for i in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49
	do
		echo "Running Complete plot_${n}_$i"
		../build/find-assignment "Experiments6/plot_${n}_$i.txt" 1 1 Experiments6/ $n >> Experiments6/output.txt&
		wait
	done
done



