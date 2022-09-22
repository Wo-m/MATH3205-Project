import numpy as np

import data_read

depot_data, jobs_data, travel_data, general_data, vehicle_data = data_read.get_data(1, 10, 3, 1)

# reus is people, non reus is equipment

# periods = int(general_data[0])
# jobs = int(general_data[1])  # also num of turbines
# vehicles = int(general_data[2])
# people_types = int(general_data[3])

periods = 1
jobs = 2
vehicles = 1
people_types = 1

# Create the nodes size = 2n+2 with final two nodes as depot pickup and delivery, n = num jobs
N = 2*jobs + 2

routes = {}  # route for vehicle v in period t on route r (dimensions will be expanded)
cost = {}  # cost for vehicle v in period t on route r (dimensions will be expanded)
service = {}  # is vehicle v servicing job j in period t on route r
techs = {}  # number of reus of type p on vehicle v in period t on route r

def get_routes():
	pass


def generate_routes():
	for t in range(periods):
		for v in range(vehicles):
			r_index = 0  # index of the route being created
			j_start = 0
			recursive_generation(v, t, [N + 1], r_index, j_start)


def recursive_generation(v, t, J, r_index, j_start):
	r = r_index
	job_set = J.copy()
	for j in range(j_start, jobs):
		job_set.append(j)
		r = solve_route(v, t, job_set, r)  # increment route index if feasible route
		recursive_generation(v, t, job_set, r, j_start + 1)  # increment both r and j indexes


def solve_route(v, t, J, r_index):
	routes[v, t, r_index] = J
	r = r_index + 1  # update index if feasible
	return r



generate_routes()
print(routes)

