import milp
import data_read
from gurobipy import *

depot_data, jobs_data, travel_data, general_data, vehicle_data = data_read.get_data(1, 10, 3, 1)


def main():
	ordered_routes, route_cost, service, people, routes_vt, num_routes = milp.milp()
	service_cost, people_max = generate_data()
	print(service_cost, people_max)
	# Sets
	R = [r for r in range(num_routes)]
	V = [v for v in range(int(general_data[2]))]
	T = [t for t in range(int(general_data[0]))]
	P = [p for p in range(int(general_data[3]))]
	J = [j for j in range(int(general_data[1]))]

	# Data ---------------------
	# routes_vt: v, t
	# ordered_routes: v, t, r
	# service_cost: t, j
	# route_cost: v, t, r
	# service: v, t, r, j
	# people_max: p
	# people: v, t, r, p

	model = Model()

	# Variables
	U = {(v, t, r): model.addVar(vtype=GRB.BINARY) for v in V for t in T for r in R}

	# Objective
	model.setObjective(quicksum(U[v, t, r] *
								(route_cost[v, t, r] + quicksum(service[v, t, r, j] * service_cost[t, j] for j in J))
								for v in V for t in T for r in routes_vt[v, t]))

	twenty = {(v, t): model.addConstr(quicksum(U[v, t, r] for r in routes_vt[v, t]) == 1) for v in V for t in T}

	twenty_one = {j: model.addConstr(quicksum(U[v, t, r]*service[v, t, r, j] for v in V for t in T for r in routes_vt[v, t]) == 1)
				for j in J}

	twenty_two = {(p, t): model.addConstr(quicksum(U[v, t, r] * people[v, t, r, p] for v in V for r in routes_vt[v, t]))
				for p in P for t in T}

	model.optimize()

def generate_data():
	service_cost = {}
	people_max = {}
	for t in range(int(general_data[0])):
		for j in range(int(general_data[1])):
			service_cost[t, j] = jobs_data[j, t + 8]
	for p in range(int(general_data[3])):
		people_max[p] = depot_data[0][p]
	return service_cost, people_max


main()