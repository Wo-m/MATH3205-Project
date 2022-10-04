from gurobipy import *

def solve_MILP(J, travel_costs, travel_times, tech_costs, service_time, max_travel, capacity, job_techs, num_techs,
                DEPOT_DROP, DEPOT_PICK, jobs, P):
    """
    All of these parameters are sourced from data with respect to vehicle, time period
    :param travel_costs: from node i to j
    :param travel_times: from node i to j
    :param service_time: for node i
    :param max_travel: constant value, max travel time
    :param capacity: constant value, max capacity
    :param job_techs: techs of type p required for job at node i
    :param num_techs: number of techs of each type p available
    :return: model
    """
    # global N, N_d, N_p, P   # Sets

    # redefine sets in terms of J
    N_d = [j for j in J][:-2]
    N_p = [j+jobs for j in J][:-2]
    N_star = N_d + N_p
    N = N_star + [DEPOT_DROP, DEPOT_PICK]

    milp = Model()
    X = {(i, j): milp.addVar(vtype=GRB.BINARY) for i in N for j in N}  # binary travel from node i to j
    Y = {i: milp.addVar() for i in N}  # time vessel visits node i
    Z = {(p, i): milp.addVar(vtype=GRB.INTEGER) for p in P for i in N}  # techs of type p on ship after leaving node i
    Q = {p: milp.addVar(vtype=GRB.INTEGER) for p in P}  # techs of type p required


    milp.setObjective(
        quicksum(Q[p] * tech_costs[p] for p in P)
        + quicksum(X[i, j]*travel_costs[i][j] for i in N for j in N),
        GRB.MINIMIZE
    )

    # ------------ Constraints -------------

    # Each node visited and left exactly once
    visited_once = {i: milp.addConstr(quicksum(X[j, i] for j in N) == 1) for i in N_star}  # 1
    leave_once = {i: milp.addConstr(quicksum(X[i, j] for j in N) == 1) for i in N_star}  # 2

    # Leave and return to depot
    leaves = milp.addConstr(quicksum(X[DEPOT_DROP, i] for i in N_d) == 1)  # 3
    returns = milp.addConstr(quicksum(X[i, DEPOT_PICK] for i in N_p) == 1)  # 4

    # cannot go from i->j and j->i
    same_trip = None

    # depot cant go to pick node
    leaves = milp.addConstr(quicksum(X[DEPOT_DROP, i] for i in N_p) == 0)  # 3

    # drop node cant go to depot
    returns = milp.addConstr(quicksum(X[i, DEPOT_PICK] for i in N_d) == 0)  # 4

    # ensure time between drop off and pick up is greater than service time
    job_time = {i: milp.addConstr(Y[jobs+i] - Y[i] >= service_time[i]) for i in N_d}  # 5

    # vessel stays with travel window
    start_time = milp.addConstr(Y[DEPOT_DROP] == 0)  # 6
    end_time = milp.addConstr(Y[DEPOT_PICK] <= max_travel)  # 7

    # ensure number of people on vessel does not exceed people available
    num_people_max = {i: milp.addConstr(quicksum(Z[p, i] for p in P) <= capacity) for i in N}  # 8
    num_people_zero = {i: milp.addConstr(quicksum(Z[p, i] for p in P) >= 0) for i in N}  # 9

    # ensure number of techs required is <= techs available for each type
    p_type = {(p, i): milp.addConstr(Z[p, i] <= num_techs[p]) for p in P for i in N}  # 10

    # find number of techs required on vessl
    p_num = {(p, i): milp.addConstr(Q[p] >= Z[p, i]) for p in P for i in N}  # 11

    # Pick up and drop off flow constraints
    drop_off_flow_lower = {(i, j, p): milp.addConstr(Z[p, i] - job_techs[j][p] - Z[p, j] >=
                                                     -1*(1 - X[i, j]) * 2 * num_techs[p])
                                                    for i in N_star for j in N_d for p in P}  # 12

    drop_off_flow_upper = {(i, j, p): milp.addConstr(Z[p, i] - job_techs[j][p] - Z[p, j] <=
                                                     (1 - X[i, j]) * 2 * num_techs[p])
                                                    for i in N_star for j in N_d for p in P}  # 13

    pick_up_flow_lower = {(i, j, p): milp.addConstr(Z[p, i] + job_techs[j-jobs][p] - Z[p, j] >=
                                                    -1 * (1 - X[i, j]) * 2 * num_techs[p])
                                                    for i in N_star for j in N_p for p in P}  # 15

    pick_up_flow_upper = {(i, j, p): milp.addConstr(Z[p, i] + job_techs[j-jobs][p] - Z[p, j] <=
                                                    (1 - X[i, j]) * 2 * num_techs[p])
                                                    for i in N_star for j in N_p for p in P}  # 14

    # Time Flow
    time_flow ={(i, j): milp.addConstr(Y[i] + travel_times[i, j] - Y[j] <= (1-X[i, j]) * (max_travel * 2))
                for i in N for j in N}

    milp.setParam("OutputFlag", 0)
    milp.optimize()

    return milp, X, Y, Z, Q, N
