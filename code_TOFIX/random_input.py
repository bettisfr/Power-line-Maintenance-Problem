import numpy as np
from knapsack2 import greedy_weight_selection


def random_generator():

    seed = 10
    np.random.seed(seed)

    num_delivery = 10
    max_len_road = 1000
    max_interval_len = 100
    max_cost = 10
    max_reward = 15

    input = []

    for i in range(0, num_delivery):
        departure = int(np.random.rand() * max_len_road)
        arrival = departure + int(np.random.rand() * max_interval_len)
        if arrival > max_len_road:
            arrival = max_len_road

        cost = int(np.random.rand() * max_cost)
        reward = int(np.random.rand() * max_reward)

        delivery = [departure, arrival, cost, reward]
        input.append(delivery)

    for departure, arrival, cost, reward in input:
        print("departure: %d, arrival: %d, cost: %d, reward: %d" % (departure, arrival, cost, reward))

    out = greedy_weight_selection(input)
    print(out)



def greedy_weight_selection(intervals, drone=1, budget=5000):
    total_reward = 0
    intervals_copy = intervals.copy()  ## intervals è l'input generato dalla random
    solution = []  ##lista di soluzioni; sarà poui lista di liste
    batteries = []
    cost = 0
    for i in range(drone):#drone=1=> itera 1 volta=>1 solo budget
        #una sola lista dentro solution
        batteries.append(budget)
        solution.append([])#[[]]
    intervals_copy = sorted(intervals_copy, key=lambda x: (x[2]))
    while len(intervals_copy) != 0 and sum(batteries) != 0:
        b = batteries.index(max(batteries))#index=0
        task = intervals_copy.pop(0)#il primo degli input
        if (len(solution[b]) == 0):#si, ho solo 1 elemento
            #che sta ad indice 0
            if (task[2] <= batteries[b]):
                #se task[2](=costo)<=durata batteria
                solution[b].append(task)
                #all'indice corrispondente a quella batteria
                #associo il task intero, cioè drone e sue caratteristiche
                total_reward = total_reward + task[3]
                #al total reward sommo quello del task
                batteries[b] = batteries[b] - task[2]
                #alla batteria sottraggo il costo
                cost = cost + task[2]
                #al costo(totale) aggiungo il costo
        else:
            #se costo <= batteria
            if task[2] <= batteries[b] and check_correct_interval(solution[b], task):
                solution[b].append(task)
                total_reward = total_reward + task[3]
                batteries[b] = batteries[b] - task[2]
                cost = cost + task[2]
    return [solution, total_reward, cost]


#def getOverlap(a, b, interval=None):
#   i1 = interval.interval[a[0], a[1]]
 #   i2 = interval.interval[b[0], b[1]]
#    return len(i1 & i2)


def getOverlap(a, b):
    test = max(0, min(a[1], b[1]) - max(a[0], b[0]))
    if test == 0:
        return True
    else:
        return False


def check_correct_interval(list_of_intervals, interval):
    list_of_intervals = sorted(list_of_intervals, key=lambda x: (x[1]))
    for i in list_of_intervals:
        if (getOverlap(i, interval) != 0):
            return False
    return True


random_generator()

