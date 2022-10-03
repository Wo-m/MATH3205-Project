import numpy as np
import data_read
from milp import *

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
    print(routes, cost, service, techs)


def recursive_generation(v, t, J, j_start):
    for j in range(j_start, jobs):
        Jdash = J.copy()
        Jdash.append(j)
        solve_route(v, t, Jdash)
        recursive_generation(v, t, Jdash, j + 1)


def solve_route(v, t, J):
    global R, routes, cost, service, techs
    # pass data to MILP model in terms of vehicle v and period t
    milp, X, Y, Z, Q, N = solve_MILP(J, tcv[v], ttv[v], tc[t], st, mt[v][t],
                                        c[v], jt, nt[t], DEPOT_DROP, DEPOT_PICK, jobs, P)

    # infeasible model
    if milp.Status != 2:
        return
    print(v, t, J)
    cost[v, t, R] = milp.objVal
    for p in P:
        techs[v, t, R, p] = Q[p].x
    for j in N:
        service[v, t, R, j] = 1 if j in J else 0
    routes[v, t, R] = ordered_route(milp, X, N)


    routes[v, t, R] = J
    R += 1  # update index if feasible

"""
HELPER METHODS
"""
def ordered_route(milp, X, N):
    node = DEPOT_DROP
    ordered_route = [node]
    while node != DEPOT_PICK:
        for n in N:
            if X[node, n].x == 1:
                node = n
                ordered_route.append(node)
    return ordered_route

def generate_data():
    global ttv, tcv, tc, st, mt, c, jt, nt
    ttv = np.zeros([vehicles, jobs*2+2, jobs*2+2])
    tcv = np.zeros([vehicles, jobs*2+2, jobs*2+2])
    st = np.zeros([len(jobs_data)])
    mt = np.zeros([vehicles, periods])
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
                tcv[v, i, j] = distance / c_rate
                ttv[v, i, j] = distance / t_rate

                # somewhat redundant but need drop and pick nodes
                tcv[v, i + jobs, j + jobs] = distance / c_rate
                ttv[v, i + jobs, j + jobs] = distance / t_rate
                tcv[v, i, j + jobs] = distance / c_rate
                ttv[v, i, j + jobs] = distance / t_rate
                tcv[v, i + jobs, j] = distance / c_rate
                ttv[v, i + jobs, j] = distance / t_rate

    for i, row in enumerate(travel_data):
        for j in range(vehicles):
            mt[j][i] = row[j]

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
                tcv[v, i + j, DEPOT_DROP] = distance / c_rate
                ttv[v, i + j, DEPOT_DROP] = distance / t_rate

                tcv[v, DEPOT_DROP, i + j] = distance / c_rate
                ttv[v, DEPOT_DROP, i + j] = distance / t_rate

                tcv[v, i + j, DEPOT_PICK] = distance / c_rate
                ttv[v, i + j, DEPOT_PICK] = distance / t_rate

                tcv[v, DEPOT_PICK, i + j] = distance / c_rate
                ttv[v, DEPOT_PICK, i + j] = distance / t_rate


generate_data()

generate_routes()

# solve_route(0, 0, {DEPOT_DROP, 1, 0, DEPOT_PICK})

