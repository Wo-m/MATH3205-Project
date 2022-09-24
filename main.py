import numpy as np
from gurobipy import *

import data_read

depot_data, jobs_data, travel_data, general_data, vehicle_data = data_read.get_data(1, 10, 3, 1)

periods = int(general_data[0])
jobs = int(general_data[1])  # also num of turbines
vehicles = int(general_data[2])
people_types = int(general_data[3])
DEPOT_DROP = 2*jobs
DEPOT_PICK = 2*jobs+1

R = 0

routes = {}  # route for vehicle v in period t on route r (dimensions will be expanded)
cost = {}  # cost for vehicle v in period t on route r (dimensions will be expanded)
service = {}  # is vehicle v servicing job j in period t on route r
techs = {}  # number of reus of type p on vehicle v in period t on route r

# SETS
N_d = [n for n in range(0, jobs)]  # drop nodes
N_p = [n for n in range(jobs, 2*jobs)]  # pick nodes
N = [n for n in range(0, 2*jobs + 2)]  # drop + pick + depot nodes
P = [p for p in range(people_types)]  # people types

# DATA
ttv = None  # travel time from node i to j for vehicle v
tcv = None  # travel cost from node i to j for vehicle v
st = None  # service time for job i
mt = None  # max travel time for vehicle v in period t
c = None  # max capacity for vehicle v
jt = None  # job techs of type p required for job i
nt = None  # num techs of type p in period t
tc = None  # cost of tech type p in period t


def generate_routes():
    for t in range(periods):
        for v in range(vehicles):
            recursive_generation(v, t, [], 1)


def recursive_generation(v, t, J, j_start):
    for j in range(j_start, jobs):
        Jdash = J.copy()
        Jdash.append(j)
        solve_route(v, t, Jdash)
        recursive_generation(v, t, Jdash, j + 1)


def solve_route(v, t, J):
    global R
    # pass data to MILP model in terms of vehicle v and period t
    solve_MILP(tcv[v], ttv[v], tc[t], st, mt[v][t], c[v], jt, nt[t])
    routes[v, t, R] = J
    R += 1  # update index if feasible


def solve_MILP(travel_costs, travel_times, tech_costs, service_time, max_travel, capacity, job_techs, num_techs):
    """
    All of these parameters are sourced from data with respect to vehicle, time period
    :param travel_costs: from node i to j
    :param travel_times: from node i to j
    :param service_time: for node i
    :param max_travel: constant value, max travel time
    :param capacity: constant value, max capacity
    :param job_techs: techs of type p required for job at node i
    :param num_techs: number of techs of each type p available
    :return:
    """
    global N, N_d, N_p, P   # Sets

    milp = Model()
    X = {(i, j): milp.addVar(vtype=GRB.BINARY) for i in N for j in N}  # binary travel from node i to j
    Y = {i: milp.addVar() for i in N}  # time vessel visits node i
    Z = {(p, i): milp.addVar() for p in P for i in N}  # techs of type p on ship after leaving node i
    Techs = {p: milp.addVar() for p in P}  # techs of type p required


    milp.setObjective(
        quicksum(Techs[p] * tech_costs[p] for p in P)
        + quicksum(X[i, j]*travel_costs[i][j] for i in N for j in N),
        GRB.MINIMIZE
    )






"""
HELPER METHODS
"""
def generate_data():
    global ttv, tcv, tc, st, mt, c, jt, nt
    ttv = np.zeros([vehicles, jobs*2+2, jobs*2+2])
    tcv = np.zeros([vehicles, jobs*2+2, jobs*2+2])
    st = np.zeros([len(jobs_data)])
    mt = np.zeros([periods, vehicles])
    c = np.zeros([vehicles])
    jt = np.zeros([jobs, people_types])
    nt = np.zeros([periods, people_types])
    tc = np.zeros([periods, people_types])

    for i, i_job in enumerate(jobs_data):
        st[i] = i_job[7]

        for n in range(people_types):
            jt[i][n] = i_job[3 + n]

        ix, iy = i_job[1], i_job[2]
        for j, j_job in enumerate(jobs_data):
            jx, jy = j_job[1], j_job[2]
            distance = np.sqrt((ix - jx) ** 2 + (iy - jy) ** 2)
            for v, vehicle in enumerate(vehicle_data):
                c_rate, t_rate = vehicle[3], vehicle[4]
                tcv[v, i, j] = c_rate * distance
                ttv[v, i, j] = t_rate * distance

                # somewhat redundant but need drop and pick nodes
                tcv[v, i + jobs, j + jobs] = c_rate * distance
                ttv[v, i + jobs, j + jobs] = t_rate * distance
                tcv[v, i, j + jobs] = c_rate * distance
                ttv[v, i, j + jobs] = t_rate * distance
                tcv[v, i + jobs, j] = c_rate * distance
                ttv[v, i + jobs, j] = t_rate * distance

    for i, row in enumerate(travel_data):
        for j in range(vehicles):
            mt[i][j] = row[j]

    for i, vehicle in enumerate(vehicle_data):
        c[i] = vehicle[1]

    for i, i_depot in enumerate(depot_data):
        for n in range(people_types):
            nt[i][n] = i_depot[n]
            tc[i][n] = i_depot[n+3]

    # calc distances to route nodes
    for v, vehicle in enumerate(vehicle_data):
        c_rate, t_rate = vehicle[3], vehicle[4]
        for i, i_job in enumerate(jobs_data):
            ix, iy = i_job[1], i_job[2]
            distance = np.sqrt((ix)** 2 + (iy - 30) ** 2)

            for j in (0, jobs):
                tcv[v, i + j, DEPOT_DROP] = c_rate * distance
                ttv[v, i + j, DEPOT_DROP] = t_rate * distance

                tcv[v, DEPOT_DROP, i + j] = c_rate * distance
                ttv[v, DEPOT_DROP, i + j] = t_rate * distance

                tcv[v, i + j, DEPOT_PICK] = c_rate * distance
                ttv[v, i + j, DEPOT_PICK] = t_rate * distance

                tcv[v, DEPOT_PICK, i + j] = c_rate * distance
                ttv[v, DEPOT_PICK, i + j] = t_rate * distance


generate_data()
solve_route(0, 0, {DEPOT_DROP, 0, 1, 2, 3, DEPOT_PICK})
