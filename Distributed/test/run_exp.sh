#!/bin/bash

# Function to launch a process
launch_process() {
    local cmd=$1
    eval "$cmd &"
}


for n in 4 8 12 16 20 24 28 32 36 40
do
	for i in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49
	do
		echo "Running Dist-GD on plot_${n}_$i"
		# Launch n-1 processes
		for ((j=1; j<=(n-1); j++))
		do
			launch_process "../build/assignTasks $j 1 rpi/plot_${n}_$i.txt > rpi/output_$j.txt"
		done

		# Wait for 1 second
		sleep 0.5

		# Launch the fourth process
		launch_process "../build/assignTasks 0 1 rpi/plot_${n}_$i.txt > rpi/output_0.txt"
		
		wait
	done
done

for n in 4 8 12 16 20 24 28 32 36 40
do
	for i in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49
	do
		echo "Running Dist-Match+GD on plot_${n}_$i"
		# Launch n-1 processes
		for ((j=1; j<=(n-1); j++))
		do
			launch_process "../build/assignTasks $j 2 rpi/plot_${n}_$i.txt > rpi/output_$j.txt"
		done

		# Wait for 1 second
		sleep 0.5

		# Launch the fourth process
		launch_process "../build/assignTasks 0 2 rpi/plot_${n}_$i.txt > rpi/output_0.txt"
		
		wait
	done
done

# Print "Done" when all processes are finished
echo "Done"
