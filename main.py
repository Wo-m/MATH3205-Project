import numpy as np

import data_read

depot_data, jobs_data, travel_data, general_data, vehicle_data = data_read.get_data(1, 10, 3, 1)

# reus is people, non reus is equipment
DEPOT_JOB = 0

# periods = int(general_data[0])
# jobs = int(general_data[1])  # also num of turbines
# vehicles = int(general_data[2])
# people_types = int(general_data[3])

periods = 1
jobs = 3
vehicles = 1
people_types = 1

# Create the nodes size = 2n+2 with final two nodes as depot pickup and delivery, n = num jobs
N = 2*jobs + 2
R = 0

routes = {}  # route for vehicle v in period t on route r (dimensions will be expanded)
cost = {}  # cost for vehicle v in period t on route r (dimensions will be expanded)
service = {}  # is vehicle v servicing job j in period t on route r
techs = {}  # number of reus of type p on vehicle v in period t on route r

def get_routes():
	pass


def generate_routes():
	for t in range(periods):
		for v in range(vehicles):
			recursive_generation(v, t, [DEPOT_JOB], 1)


def recursive_generation(v, t, J, j_start):
	for j in range(j_start, jobs + 1):
		Jdash = J.copy()
		Jdash.append(j)
		solve_route(v, t, Jdash)
		recursive_generation(v, t, Jdash, j + 1)


def solve_route(v, t, J):
	global R
	routes[v, t, R] = J
	R += 1  # update index if feasible



generate_routes()
print(routes)

