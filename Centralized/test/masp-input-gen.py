import random
from statistics import NormalDist
# Import math Library
import math 

# Problem Setup:
#  Sets:
#  i in I, set of agents, |I| = N
#  j in J, set of tasks, |J| = M
#  k in K, set of capabilities, |K| = E
# 
#  Parameters
#  c_ik[N][E] - capabilities per agent
#  r_jk[M][E] - capabilities requirement per task
#  p_ij[N][M] - probability agent i can complete task j
#  d_j[M]     - Minimum number of agents required to complete task j
# 
# Expected file structure:
#  N M E
# 	c_ik - N x E
# 	...
# 	r_jk - M x E
# 	...
# 	p_ij - N x M
# 	...
# 	d_j  - M
# 	...
# 
# Types of sensors: (1) EO/IR Camera, (2) Thermal Camera, (3) HD Camera, (4) 4K Camera, (5) Lidar
# Types of drones:
#           Matrice 600 PRO, Mavic PRO, FireFLY6 PRO/S, 3DR Solo Quad, Pulse Vapor 55TM, Parrot Anafi
# v_m (kph):        65          72              110            89              40               -
# v_m (m/s):        18          20              30.5          25.5             11              15
# d-max (m):       19500*      18000             -            800              -               -
# d-max (v_m):    13300*      19800*           73000*         1000*           20000*         10800*
# s_m (min):       12.3*       16.5*            40             4.5*            30*             12*
# s_h (min):        18          29              20             18*             45              18*
# s_m (s):         740*        990*            2400            270*           1800*            720*
# s_h (s):         1080        1740            1200            1080*          2700             1080*
# Sensor:          2,3,5       1,4             2,4              3             1,3,5             4
# 
#  * indicates estimated entries
#  Data found at https://www.doi.gov/aviation/uas/fleet and from manufacture specs
#  Drop in battery performance: https://apps.dtic.mil/sti/tr/pdf/ADA572554.pdf
#   [0, 0, 0, 4, 4, 1, 1, 1, 1, 1, 1, 5, 5, 5, 5]

# Based on sensors (note that any drone with a 4K camera can do a task requiring an HD camera)
Agent_types_C = ["0 1 1 0 1", "1 0 1 1 0", "0 1 1 1 0", "0 0 1 0 0", "1 0 1 0 1", "0 0 1 1 0"]
Drone_C = [[2,3,5], [1,3,4], [2,3,4], [3], [1,3,5], [3,4]]
Task_Req_C = ["1 0 0 0 0", "0 1 0 0 0", "0 0 1 0 0", "0 0 0 1 0", "0 0 0 0 1", "1 0 1 0 0"]
Capabilities = [1, 2, 3, 4, 5]

# Est_FltTm = [18, 18, 40, 25, 60, 25]
# s_m = [12.3, 16.5, 40, 4.5, 30, 12]
s_m = [740.0, 990.0, 2400.0, 270.0, 1800.0, 720.0]
# s_h = [18, 29, 20, 18, 45, 18]
s_h = [1080.0, 1740.0, 1200.0, 1080.0, 2700.0, 1080.0]
v_max = [18, 20, 30.5, 25.5, 11, 15]
dist_vmax = [13300, 19800, 73000, 1000, 20000, 10800]


# Problem input parameters
N = 15
E = 5
AGENTS_TO_TASKS = 0.25
M = 4
PRCNT_FLOATERS = 0.25
BASE_TO_DRONE = 0.1

# Type of input to make
INC_AGENTS = 1
INC_TASK = 2
INC_FLOATING = 3
CASE_STUDY = 4
LARGE_INPUT = 5

# Set Scenario
SCENARIO_TYPE = LARGE_INPUT
FILE_PATH = "Experiments8/"

# File parameters
NUM_PLOTS = 50
START_COUNT = 50
END_COUNT = 150
Agent_Increment = 10
MIN_Task_Count = 2
MAX_Task_Count = int(N*(1-PRCNT_FLOATERS))
Task_Increment = 2
NUM_PCNT_INC = 20

# Error parameters
DIST_ERROR = 0.3
FltTm_Error = 0.3
Budget_Error = 0.25
MAX_CYCLES = 400
# Task duration
MAX_DUR = 900	# 15 min
MIN_DUR = 300	# 5 min
# Task location
MAX_X = 1500
# Distance to base
MAX_BASE_X = 50

# Used to determine average d_j
d_j_total = 0
d_j_count = 0


def createD(n,m,f_a):
	lst = []
	assigned = 0
	for i in range(m):
		lst.append(1)
		assigned += 1
	np = n - f_a
	while assigned < np:
		index = random.randint(0, len(lst) - 1)
		lst[index] += 1
		assigned += 1
	# print(lst)
	return lst

#  Picks a random agent i for capability e
def pickIforJ(e):
	while True:
		# Pick a random drone
		index = random.randint(0, len(Drone_C) - 1)
		# Check if this drone has capability e
		for cap in Drone_C[index]:
			if cap is e:
				return index

# Takes agent i, assumes i is assigned to the expanded list j, then picks a random agent type that meets the requirements of j
def whatR(i,D,req_lst):
	j = 0
	j_track = 0
	for jj in range(len(D)):
		if i >= j_track:
			if i < (j_track + D[j]):
				# This is type j
				# print(f"{i} is task {j}, capability {req_lst[j]}")
				return pickIforJ(req_lst[j])
		j_track += D[j]
		j +=1
	# Must be floating... pick randomly
	# print(f"{i} is a floating agent")
	index = random.randint(0, len(D) - 1)
	return pickIforJ(req_lst[index])

# Returns the expected percent degradation of the battery 
def batDeg(cycles):
	return -0.1/200*cycles + 1

print("Printing to:", FILE_PATH)
print("Scenario:", SCENARIO_TYPE)
dummyText = input("Press Enter To Continue")

# What increases?
if SCENARIO_TYPE is INC_AGENTS: # Agents
	# Loop over team sizes
	for n in range(START_COUNT, (END_COUNT + Agent_Increment), Agent_Increment):
		# Determine how many floaters to make
		f_a = int(n*PRCNT_FLOATERS)+1
		m = int(n*AGENTS_TO_TASKS)+1
		print("Making n,m,f_a =",n,m,f_a)
		# Generate NUM_PLOTS plots
		for l in range(NUM_PLOTS):
			# Open the file
			file_name = f"{FILE_PATH}plot_{n}_{l}.txt"
			with open(file_name, 'w') as file:
				## Problem setup
				# Determine minimum number of agents per task 
				D = createD(n, m, f_a)
				# Determine task requirements and assign agents
				task_req = []
				task_time = []
				task_loc = []
				for j in range(m):
					# Pick a random requirement  Capabilities
					index = random.randint(0, len(Capabilities) - 1)
					task_req.append(Capabilities[index])
					# Pick a random time
					time = (MAX_DUR - MIN_DUR) * random.random() + MIN_DUR
					task_time.append(time)
					# Pick a random location
					x = (2*MAX_X) * random.random() - MAX_X
					y = (2*MAX_X) * random.random() - MAX_X
					task_loc.append([x,y])
				# Determine agents
				agent_list = []
				agent_loc = []
				for i in range(n):
					# Pick a drone
					r = whatR(i,D,task_req)
					agent_list.append(r)
					# Pick a random drone location
					x = (2*MAX_X) * random.random() - MAX_X
					y = (2*MAX_X) * random.random() - MAX_X
					agent_loc.append([x,y])
				## Start printing info
				# Input info
				file.write(f"# {n} agents, {m} tasks, {E} capabilities\n")
				file.write(f"{n} {m} {E}\n")
				# Create robot capability matrix
				file.write(f"# Agent Capabilities: c_ik - N x E\n")
				for i in range(n):
					file.write(Agent_types_C[agent_list[i]])
					file.write(f"\n")
				# Create role requirement matrix
				file.write(f"# Task Requirements: r_jk - M x E\n")
				for j in range(m):
					# print(Task_Req_C[task_req[j]-1])
					file.write(f"{Task_Req_C[task_req[j]-1]}")
					file.write(f"\n")
				# Probability of success matrix: p_ij - N x M
				file.write(f"# Probability agent i can complete task j: p_ij - N x M\n")
				for i in range(n):
					battery_cycles = MAX_CYCLES * random.random()
					batDegrade = batDeg(battery_cycles)
					budget = s_m[agent_list[i]]*batDegrade
					# print("Cycles:", battery_cycles, "Capacity:", batDegrade, "Budget", budget)
					for j in range(m):
						# Determine how much budget i will spend doing j
						dist_to_j = math.sqrt( pow(task_loc[j][0] - agent_loc[i][0], 2) + pow(task_loc[j][1] - agent_loc[i][1], 2) )
						dist_j_to_bs = math.sqrt( pow(task_loc[j][0], 2) + pow(task_loc[j][1], 2) )
						s1 = dist_to_j/v_max[agent_list[i]]
						s2 = dist_j_to_bs/v_max[agent_list[i]]
						s_tot = s1+s2+(task_time[j]*s_m[agent_list[i]]/s_h[agent_list[i]])
						prob_ij = 1 - NormalDist(mu=budget, sigma=(budget * Budget_Error)).cdf(s_tot)
						file.write("{:.20f} ".format(prob_ij))
					file.write(f"\n")
				# Set min agents required for each j: d_j  - M
				file.write(f"# Minimum number of agents for each task: d_j\n")
				for j in range(m):
					file.write(f"{D[j]}\n")
					d_j_total += D[j]
					d_j_count += 1
				# Record the location of each task
				file.write(f"# Location of each task: M x 2\n")
				for j in range(m):
					file.write(f"{task_loc[j][0]} {task_loc[j][1]}\n")
				file.write(f"# Location of each drone: N x 2\n")
				for i in range(n):
					file.write(f"{agent_loc[i][0]} {agent_loc[i][1]}\n")

elif SCENARIO_TYPE is INC_TASK: # Number of tasks
	# Loop over team sizes
	pass

elif SCENARIO_TYPE is INC_FLOATING: # % of floating agents
	# Loop over team sizes
	for f_a in range(START_COUNT, (END_COUNT + Agent_Increment), Agent_Increment):
		print("Making n,m,f_a =",N,M,f_a)
		# Generate NUM_PLOTS plots
		for l in range(NUM_PLOTS):
			# Open the file
			file_name = f"{FILE_PATH}plot_{f_a}_{l}.txt"
			with open(file_name, 'w') as file:
				# Determine minimum number of agents per task 
				D = createD(N, M, f_a)
				# Determine task requirements and assign agents
				task_req = []
				task_time = []
				task_loc = []
				for j in range(M):
					# Pick a random requirement  Capabilities
					index = random.randint(0, len(Capabilities) - 1)
					task_req.append(Capabilities[index])
					# Pick a random time
					time = (MAX_DUR - MIN_DUR) * random.random() + MIN_DUR
					task_time.append(time)
					# Pick a random location
					x = (2*MAX_X) * random.random() - MAX_X
					y = (2*MAX_X) * random.random() - MAX_X
					task_loc.append([x,y])
				# Determine agents
				agent_list = []
				agent_loc = []
				for i in range(N):
					# Pick a drone
					r = whatR(i,D,task_req)
					agent_list.append(r)
					# Pick a random drone location
					x = (2*MAX_X) * random.random() - MAX_X
					y = (2*MAX_X) * random.random() - MAX_X
					agent_loc.append([x,y])
				## Start printing info
				# Input info
				file.write(f"# {N} agents, {M} tasks, {E} capabilities\n")
				file.write(f"{N} {M} {E}\n")
				# Create robot capability matrix
				file.write(f"# Agent Capabilities: c_ik - N x E\n")
				for i in range(N):
					file.write(Agent_types_C[agent_list[i]])
					file.write(f"\n")
				# Create role requirement matrix
				file.write(f"# Task Requirements: r_jk - M x E\n")
				for j in range(M):
					# print(Task_Req_C[task_req[j]-1])
					file.write(f"{Task_Req_C[task_req[j]-1]}")
					file.write(f"\n")
				# Probability of success matrix: p_ij - N x M
				file.write(f"# Probability agent i can complete task j: p_ij - N x M\n")
				for i in range(N):
					battery_cycles = MAX_CYCLES * random.random()
					batDegrade = batDeg(battery_cycles)
					budget = s_m[agent_list[i]]*batDegrade
					# print("Cycles:", battery_cycles, "Capacity:", batDegrade, "Budget", budget)
					for j in range(M):
						# Determine how much budget i will spend doing j
						dist_to_j = math.sqrt( pow(task_loc[j][0] - agent_loc[i][0], 2) + pow(task_loc[j][1] - agent_loc[i][1], 2) )
						dist_j_to_bs = math.sqrt( pow(task_loc[j][0], 2) + pow(task_loc[j][1], 2) )
						s1 = dist_to_j/v_max[agent_list[i]]
						s2 = dist_j_to_bs/v_max[agent_list[i]]
						s_tot = s1+s2+(task_time[j]*s_m[agent_list[i]]/s_h[agent_list[i]])
						prob_ij = 1 - NormalDist(mu=budget, sigma=(budget * Budget_Error)).cdf(s_tot)
						file.write("{:.20f} ".format(prob_ij))
					file.write(f"\n")
				# Set min agents required for each j: d_j  - M
				file.write(f"# Minimum number of agents for each task: d_j\n")
				for j in range(M):
					file.write(f"{D[j]}\n")
					d_j_total += D[j]
					d_j_count += 1
				# Record the location of each task
				file.write(f"# Location of each task: M x 2\n")
				for j in range(M):
					file.write(f"{task_loc[j][0]} {task_loc[j][1]}\n")
				file.write(f"# Location of each drone: N x 2\n")
				for i in range(N):
					file.write(f"{agent_loc[i][0]} {agent_loc[i][1]}\n")

elif SCENARIO_TYPE is CASE_STUDY: # Creates input for case study
	# Open the file
	file_name = f"{FILE_PATH}case_study.txt"
	with open(file_name, 'w') as file:
		# Minimum number of agents per task
		D = [2, 4, 2, 2]
		# Task requirements: HD camera, IR+HD camera, HD camera, thermal
		task_req = [3, 5, 3, 2]
		# Task time reqs: 10, 14, 10, 8 mins
		task_time = [600,840,600,480]
		# Task locations
		task_loc = [[1600,4160],[2200,3760],[2590,2960],[3910,4350]]
		print("Agents per task:", D)
		print("Task reqs:", task_req)
		print("Task times:", task_time)
		print("Task locations", task_loc)
		# Drone types: (4) Pulse Vapor 55TM, (0) Matrice 600 PRO, (1) Mavic PRO, (2) FireFLY6 PRO, (5) Parrot Anafi
		agent_list = [4, 4, 4, 4, 4, 0, 0, 0, 0, 1, 1, 2, 2, 5, 5]
		# Launch/depot locations
		launch_a = [1480, 4890]
		launch_b = [1850, 2990]
		depot = [840, 2550]
		# depot = [740, 1020]
		# depot = [0, 0]
		agent_loc = [launch_a,launch_a,launch_a,launch_a,launch_a,launch_b,launch_b,launch_b,launch_b,launch_b,launch_b,launch_b,launch_b,launch_b,launch_b]
		print("Drones:", agent_list)
		print("Drone loc:", agent_loc)
		
		## Start printing info
		# Input info
		file.write(f"# {N} agents, {M} tasks, {E} capabilities\n")
		file.write(f"{N} {M} {E}\n")
		# Create robot capability matrix
		file.write(f"# Agent Capabilities: c_ik - N x E\n")
		for i in range(N):
			file.write(Agent_types_C[agent_list[i]])
			file.write(f"\n")
		# Create role requirement matrix
		file.write(f"# Task Requirements: r_jk - M x E\n")
		for j in range(M):
			# print(Task_Req_C[task_req[j]-1])
			file.write(f"{Task_Req_C[task_req[j]-1]}")
			file.write(f"\n")
		# Probability of success matrix: p_ij - N x M
		file.write(f"# Probability agent i can complete task j: p_ij - N x M\n")
		bat_cycles = [350, 350, 350, 350, 350, 10, 10, 10, 10, 100, 100, 150, 150, 250, 250]
		for i in range(N):
			# battery_cycles = MAX_CYCLES * random.random()
			batDegrade = batDeg(bat_cycles[i])
			budget = s_m[agent_list[i]]*batDegrade
			# print("Cycles:", battery_cycles, "Capacity:", batDegrade, "Budget", budget)
			for j in range(M):
				# Determine how much budget i will spend doing j
				dist_to_j = math.sqrt( pow(task_loc[j][0] - agent_loc[i][0], 2) + pow(task_loc[j][1] - agent_loc[i][1], 2) )
				dist_j_to_bs = math.sqrt( pow(task_loc[j][0] - depot[0], 2) + pow(task_loc[j][1] - depot[1], 2) )
				s1 = dist_to_j/v_max[agent_list[i]]
				s2 = dist_j_to_bs/v_max[agent_list[i]]
				s_tot = s1+s2+(task_time[j]*s_m[agent_list[i]]/s_h[agent_list[i]])
				prob_ij = 1 - NormalDist(mu=budget, sigma=(budget * Budget_Error)).cdf(s_tot)
				file.write("{:.20f} ".format(prob_ij))
			file.write(f"\n")
		# Set min agents required for each j: d_j  - M
		file.write(f"# Minimum number of agents for each task: d_j\n")
		for j in range(M):
			file.write(f"{D[j]}\n")
			d_j_total += D[j]
			d_j_count += 1
		# Record the location of each task
		file.write(f"# Location of each task: M x 2\n")
		for j in range(M):
			file.write(f"{task_loc[j][0]} {task_loc[j][1]}\n")
		file.write(f"# Location of each drone: N x 2\n")
		for i in range(N):
			file.write(f"{agent_loc[i][0]} {agent_loc[i][1]}\n")
			
elif SCENARIO_TYPE is LARGE_INPUT: # Increase agents to large scale
	# Loop over team sizes
	for n in range(START_COUNT, (END_COUNT + Agent_Increment), Agent_Increment):
		# Determine how many floaters to make
		f_a = int(n*PRCNT_FLOATERS)+1
		m = int(n*AGENTS_TO_TASKS)+1
		num_base = int(n*BASE_TO_DRONE)+1
		# max_x = MAX_X*(n/START_COUNT)
		print("Making n,m,f_a =",n,m,f_a)
		print("Num bases =",num_base)
		# Generate NUM_PLOTS plots
		for l in range(NUM_PLOTS):
			# Open the file
			file_name = f"{FILE_PATH}plot_{n}_{l}.txt"
			with open(file_name, 'w') as file:
				## Problem setup
				# Determine minimum number of agents per task
				D = createD(n, m, f_a)
				# Determine task requirements and assign agents
				task_req = []
				task_time = []
				task_loc = []
				base_loc = []
				for j in range(m):
					# Pick a random requirement  Capabilities
					index = random.randint(0, len(Capabilities) - 1)
					task_req.append(Capabilities[index])
					# Pick a random time
					time = (MAX_DUR - MIN_DUR) * random.random() + MIN_DUR
					task_time.append(time)
					# Pick a random location
					x = (2*MAX_X) * random.random() - MAX_X
					y = (2*MAX_X) * random.random() - MAX_X
					task_loc.append([x,y])
				for b in range(num_base):
					# Pick a random location
					x = (2*MAX_X) * random.random() - MAX_X
					y = (2*MAX_X) * random.random() - MAX_X
					base_loc.append([x,y])
				# print("Bases:", base_loc)
				# Determine agents
				agent_list = []
				agent_loc = []
				agent_base = []
				for i in range(n):
					# Pick a drone
					r = whatR(i,D,task_req)
					agent_list.append(r)
					# Pick a base
					base_i = random.randint(0, num_base-1)
					# print("Agent goes to base:", base_i)
					agent_base.append(base_i)
					# Pick a random drone location
					x = (2*MAX_BASE_X) * random.random() - MAX_BASE_X
					y = (2*MAX_BASE_X) * random.random() - MAX_BASE_X
					# Put the drone close-ish to the base
					agent_loc.append([base_loc[base_i][0]+x,base_loc[base_i][1]+y])
				## Start printing info
				# Input info
				file.write(f"# {n} agents, {m} tasks, {E} capabilities\n")
				file.write(f"{n} {m} {E}\n")
				# Create robot capability matrix
				file.write(f"# Agent Capabilities: c_ik - N x E\n")
				for i in range(n):
					file.write(Agent_types_C[agent_list[i]])
					file.write(f"\n")
				# Create role requirement matrix
				file.write(f"# Task Requirements: r_jk - M x E\n")
				for j in range(m):
					# print(Task_Req_C[task_req[j]-1])
					file.write(f"{Task_Req_C[task_req[j]-1]}")
					file.write(f"\n")
				# Probability of success matrix: p_ij - N x M
				file.write(f"# Probability agent i can complete task j: p_ij - N x M\n")
				for i in range(n):
					battery_cycles = MAX_CYCLES * random.random()
					batDegrade = batDeg(battery_cycles)
					budget = s_m[agent_list[i]]*batDegrade
					base_i = agent_base[i]
					# print("Cycles:", battery_cycles, "Capacity:", batDegrade, "Budget", budget)
					for j in range(m):
						# Determine how much budget i will spend doing j
						dist_to_j = math.sqrt( pow(task_loc[j][0] - agent_loc[i][0], 2) + pow(task_loc[j][1] - agent_loc[i][1], 2) )
						dist_j_to_bs = math.sqrt( pow(task_loc[j][0] - base_loc[base_i][0], 2) + pow(task_loc[j][1] - base_loc[base_i][1], 2) )
						s1 = dist_to_j/v_max[agent_list[i]]
						s2 = dist_j_to_bs/v_max[agent_list[i]]
						s_tot = s1+s2+(task_time[j]*s_m[agent_list[i]]/s_h[agent_list[i]])
						prob_ij = 1 - NormalDist(mu=budget, sigma=(budget * Budget_Error)).cdf(s_tot)
						file.write("{:.20f} ".format(prob_ij))
					file.write(f"\n")
				# Set min agents required for each j: d_j  - M
				file.write(f"# Minimum number of agents for each task: d_j\n")
				for j in range(m):
					file.write(f"{D[j]}\n")
					d_j_total += D[j]
					d_j_count += 1
				# Record the location of each task
				file.write(f"# Location of each task: M x 2\n")
				for j in range(m):
					file.write(f"{task_loc[j][0]} {task_loc[j][1]}\n")
				file.write(f"# Location of each drone: N x 2\n")
				for i in range(n):
					file.write(f"{agent_loc[i][0]} {agent_loc[i][1]}\n")
print("Done!")
print("Average d_j:", d_j_total/d_j_count)
