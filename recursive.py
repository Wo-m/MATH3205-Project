import numpy as np
import data_read
import itertools
from milp_model import *
import time

jobs_data = None
vehicle_data = None

periods = None
jobs = None
vehicles = None
people_types = None
DEPOT_DROP = None
DEPOT_PICK = None

R = 0

# Data passed to master solve
routes = {}  # route for vehicle v in period t on route r
cost = {}  # cost for vehicle v in period t on route r
service = {}  # is vehicle v servicing job j in period t on route r
techs = {}  # number of reus of type p on vehicle v in period t on route r
route_indexes = {}

# feasibility dicts
window = {}
route_parts = {}

N_d = None # drop nodes
P = None # people types

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
    """
    Generates all feasible ordered routes
    """

    for t in range(periods):
        for v in range(vehicles):
            start = time.time()
            recursive_generation(v, t, [], 0)
            end = time.time()
            print(t,v,R, end-start)


def recursive_generation(v, t, J, j_start):
    """
    Recursive generation of the unique job sets
    (calls solve_ordered_routes to get feasbile NODE permutations of the JOB sets)
    Params:
    v (int): vehicle used
    t (int): time period
    J (set(int)): unique set of routes
    j_start (int): j index to generate from
    """

    for j in range(j_start, jobs):
        Jdash = J.copy()
        Jdash.append(j)
        feasible = solve_ordered_routes(v, t, Jdash)  # solve all permutations of a route set
        if feasible:  # supersets can only be feasible if subset is
            recursive_generation(v, t, Jdash, j + 1)


def solve_ordered_routes(v, t, J):
    """
    Solve all NODE permutations of the JOB set J
    generates half routes
    Params:
    v (int): vehicle used
    t (int): time period
    J (set(int)): unique set of routes
    Returns:
    bool: feasibility
    """
    global R, routes, cost, service, techs, window, route_parts


    # Parts Constraint (Feasibility is independant of order)
    parts = route_parts.get(frozenset(J), 0)
    if parts == 0:
        for j in J:
            parts += jobs_data[j][6]
        route_parts[frozenset(J)] = parts

    # Check our ship can handle parts
    if parts > vehicle_data[v][2]:
        return False  # infeasible set

    # infeasible for smaller window, therefore infeasible set
    time = window.get((frozenset(J), v), -1)  # (period, window, feasibility, solved_routes)
    if time != -1 and (not time[2] and time[1] >= mt[v][t]):  # infeasible for same or smaller window
        return False
    elif time != -1 and (time[2] and time[1] == mt[v][t]):  # solved for same time window)
        for route, r_c, personnel in time[3]:
            routes[v,t,R] = route
            cost[v,t,R] = r_c
            for j in N_d:
                service[v, t, R, j] = 1 if j in J else 0
            for p in P:
                techs[v,t,R,p] = personnel[p]
            route_indexes[v,t].append(R)


    nodes = []
    for j in J:
        nodes.append(j)
        nodes.append(j+jobs)


    # Handles drop before pick constraint
    route_perms = get_feasible_permutations(nodes)

    # route_perms = [[1,11,7,5,15,17]]

    solved_routes = []  # solved ordered routes
    # print('nodes', len(nodes))
    # print('perms', len(route_perms))
    for route in route_perms:
        feasible = True

        # keep track of time, cost, personnel, time dropped
        dropped = {i: -1 for i in J}
        personnel = {i: 0 for i in P}
        max_personnel = {i: 0 for i in P}
        time = 0.25
        route_cost = 0


        past_node = DEPOT_DROP
        node_index = 0
        while node_index < len(route):
            current_node = route[node_index]

            # Travel to current node
            time += ttv[v][past_node][current_node] + 0.25
            route_cost += tcv[v][past_node][current_node]

            if current_node - jobs < 0:  # drop node
                # update drop time for this job
                dropped[current_node] = time

                # Add personnel
                for p in P:
                    personnel[p] += jt[current_node][p]
                    # update max_personnel
                    if personnel[p] > max_personnel[p]:
                        max_personnel[p] = personnel[p]
            else:  # pick node
                diff = time - dropped[current_node-jobs]
                if diff < st[current_node-jobs]:
                    # have to wait for job to finish
                    time += st[current_node-jobs]-diff

                # Subtract personnel
                for p in P:
                    personnel[p] -= jt[current_node-jobs][p]

            # determine feasibility ----------------
            # Time Window
            if time > mt[v][t]:
                feasible = False
                break

            # Personnel Available
            for p in P:
                if personnel[p] > nt[t][p]:
                    feasible = False
                    break

            # Ship Capacity
            if sum(max_personnel.values()) > c[v]:
                feasible = False
                break
            # --------------------------------------

            past_node = current_node
            node_index += 1

        # travel to depot
        time += ttv[v][current_node][DEPOT_PICK] + 0.25
        route_cost += tcv[v][current_node][DEPOT_PICK]

        for p in P:
            route_cost += tc[t][p]*max_personnel[p]

        # recheck time constraint
        if time > mt[v][t]:
            feasible = False

        if feasible:
            solved_routes.append((route, route_cost, max_personnel))
            # print(time)
            # print(route_cost)
            # print(route)

    # Get best route
    if len(solved_routes) > 0:
        for route, r_c, personnel in solved_routes:
            routes[v,t,R] = route
            cost[v,t,R] = r_c
            for j in N_d:
                service[v, t, R, j] = 1 if j in J else 0
            for p in P:
                techs[v,t,R,p] = personnel[p]
            route_indexes[v,t].append(R)
            R += 1
        window[frozenset(J), v] = (t, mt[v][t], True, solved_routes)
        return True
    else:
        window[frozenset(J), v] = (t, mt[v][t], False, -1)
        return False

"""
    HELPER METHODS
    """

def get_feasible_permutations(nodes):
    """
    sourced from itertools.permutations
    adjusted to include constraints
    """
    perms = []
    permut(perms, [], nodes)
    return perms

def permut(perms, perm, left):
    if len(left) == 0:
        perms.append(perm)
        return
    else:
        for i in left:
            new_perm = perm.copy()
            new_perm.append(i)
            new_left = left.copy()
            new_left.remove(i)
            val = i - jobs
            if val >= 0:  # pick node so must check if drop node is in list
                if val not in perm:
                    continue
            permut(perms, new_perm, new_left)

def get_best_route(routes_data):
    """
    get best route for set of routes
    Params:
    routes_data ( list( list(int), int, dict() ) ): route data including route, cost and personnel
    Returns:
    list(int): best route
    """
    best_cost = math.inf
    best_personnel = {p: math.inf for p in P}
    best_route = []
    for data in routes_data:
        if data[1] < best_cost:
            better_p = True
            for p in P:
                if data[2][p] > best_personnel[p]:
                    better_p = False
            if better_p:
                best_route = data[0]
                best_cost = data[1]
                best_personnel = data[2]

    return best_route

def generate_data(depot_data, jobs_data, travel_data, general_data, vehicle_data):
    global ttv, tcv, tc, st, mt, c, jt, nt, periods, jobs, vehicles, people_types, DEPOT_DROP, DEPOT_PICK, route_indexes, N_d, P

    periods = int(general_data[0])
    jobs = int(general_data[1])  # also num of turbines
    vehicles = int(general_data[2])
    people_types = int(general_data[3])
    DEPOT_DROP = 2*jobs
    DEPOT_PICK = 2*jobs+1

    route_indexes = {(v, t): [] for v in range(vehicles) for t in range(periods)}

    N_d = [n for n in range(0, jobs)]
    P = [p for p in range(people_types)]

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
                tcv[v, i, j] = distance * c_rate
                ttv[v, i, j] = distance / t_rate

                # somewhat redundant but need drop and pick nodes
                tcv[v, i + jobs, j + jobs] = distance * c_rate
                ttv[v, i + jobs, j + jobs] = distance / t_rate
                tcv[v, i, j + jobs] = distance * c_rate
                ttv[v, i, j + jobs] = distance / t_rate
                tcv[v, i + jobs, j] = distance * c_rate
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
            distance = np.sqrt(ix** 2 + (iy - 30) ** 2)

            for d in (0, jobs):
                tcv[v, i + d, DEPOT_DROP] = distance * c_rate
                ttv[v, i + d, DEPOT_DROP] = distance / t_rate

                tcv[v, DEPOT_DROP, i + d] = distance * c_rate
                ttv[v, DEPOT_DROP, i + d] = distance / t_rate

                tcv[v, i + d, DEPOT_PICK] = distance * c_rate
                ttv[v, i + d, DEPOT_PICK] = distance / t_rate

                tcv[v, DEPOT_PICK, i + d] = distance * c_rate
                ttv[v, DEPOT_PICK, i + d] = distance / t_rate

def main(depot_data, jobs_d, travel_data, general_data, vehicle_d):
    global jobs_data, vehicle_data
    jobs_data, vehicle_data = jobs_d, vehicle_d
    generate_data(depot_data, jobs_data, travel_data, general_data, vehicle_data)
    generate_routes()
    return routes, cost, service, techs, route_indexes, R