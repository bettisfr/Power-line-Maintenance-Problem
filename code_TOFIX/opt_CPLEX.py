from docplex.mp.model import Model
import numpy as np


def opt_ilp_cplex(input, budget, n_drones, debug):

    input = input[1:]
    input = np.array(input)

    B = budget
    m = n_drones

    # p = [10, 13, 18, 31, 7, 15]
    # w = [11, 15, 20, 35, 10, 33]
    #
    # y = [
    #     [0, 1, 0, 0, 0, 0],
    #     [0, 0, 0, 0, 0, 0],
    #     [0, 0, 0, 1, 0, 0],
    #     [0, 0, 0, 0, 1, 0],
    #     [0, 0, 0, 0, 0, 0],
    # ]
    #
    # B = 80
    # m = 1

    p = input[:, 3]
    w = input[:, 2]


    y = np.ones((len(p), len(w)))
    for j in range(0, len(w)):
        for k in range(j + 1, len(w)):
            dep1 = input[j][0]
            arr1 = input[j][1]

            dep2 = input[k][0]
            arr2 = input[k][1]

            if (arr1 <= dep2) or (dep1 >= arr2):
                y[j][k] = 0
                y[k][j] = 0

    N = range(len(w))
    M = range(m)

    model = Model()
    model.verbose = 0

    A = [(i, j) for i in M for j in N]
    x = model.binary_var_dict(A, name='x')
    # x = [[model.add_var(var_type=BINARY) for j in N] for i in M]

    # model.objective = maximize(xsum(p[j] * x[i][j] for j in N for i in M))
    model.maximize(model.sum(model.sum(p[j] * x[i, j] for j in N for i in M)))

    # for i in M:
    # mdl.add_constraints(mdl.sum(x[i, 0, k, l] * (end[i - 1] + f[i][0]) for i in N) <= L for k in drones for l in trips)
    model.add_constraints(model.sum(w[j] * x[i, j] for j in N) <= B for i in M)
        # model += xsum(w[j] * x[i][j] for j in N) <= B

    model.add_constraints(model.sum(x[i, j] for i in M) <= 1 for j in N)
    # for j in N:
    #     model += model.sum(x[i][j] for i in M) <= 1
        # model += xsum(x[i][j] for i in M) <= 1

    # x[i][j] + x[i][k] <= 1
    for i in M:
        for j in range(0, len(w)):
            for k in range(j + 1, len(w)):
                if (y[j][k] == 1) and (y[j][k] == 1):
                    model.add_constraint(x[i, j] + x[i, k] <= 1)
                    model.add_constraint(x[i, k] + x[i, j] <= 1)
                    # model += (x[i][j] + x[i][k] <= 1)
                    # model += (x[i][k] + x[i][j] <= 1)

    sol = model.solve(log_output=False)

    total_profit = 0

    if debug:
        print()
    for i in M:
        weight = 0
        profit = 0
        selected = [j for j in N if sol.get_value(x[i,j]) >= 0.99]
        if debug:
            print("Items: {}".format(selected))
        for j in N:
            #print("[%d,%d] = %d" % (i, j, sol.get_value(x[i,j])))
            #print("[",i,",",j,"]=",sol.get_value(x[i,j])," w: ",w[j]," p: ",p[j])
            weight = weight + (sol.get_value(x[i,j]) * w[j])
            profit = profit + (sol.get_value(x[i,j]) * p[j])
        #print("DRONE ",i," weight: ",weight," reward: ",profit)
        if debug:
            print("  W: %d" % weight)
            print("  P: %d" % profit)
        total_profit = total_profit + profit
        if debug:
            print()

    if debug:
        print("TOT P: %.10f" % total_profit)

    # Add other shit if you want other fields to be returned
    output = total_profit

    return output