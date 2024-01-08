# HGS Solver for  VRPTW with OOH locations
import subprocess
import os
import platform
import tools
from collections import OrderedDict

def solve_static_vrptw(instance, time_limit=10, tmp_dir="tmp", seed=1, fractionSREX = 0.5):

    # Prevent passing empty instances to the static solver, e.g. when
    # strategy decides to not dispatch any requests for the current epoch
    if instance['coords'].shape[0] <= 1:
        yield [], 0
        return

    if instance['coords'].shape[0] <= 2:
        solution = [[1]]
        cost = tools.validate_static_solution(instance, solution)
        yield solution, cost
        return

    os.makedirs(tmp_dir, exist_ok=True)
    instance_filename = os.path.join(tmp_dir, "problem.vrptw")
    tools.write_vrplib(instance_filename, instance, is_vrptw=True)
    
    executable = os.path.join('source', 'genvrp')
    # On windows, we may have genvrp.exe
    if platform.system() == 'Windows' and os.path.isfile(executable + '.exe'):
        executable = executable + '.exe'
    assert os.path.isfile(executable), f"HGS executable {executable} does not exist!"
    with subprocess.Popen([
                executable, instance_filename, str(max(time_limit, 2)), 
                '-seed', str(seed), '-veh', '-1', '-useWallClockTime', '1', '-minSweepFillPercentage', '20', '-fractionSREX', str(fractionSREX)
            ], stdout=subprocess.PIPE, text=True) as p:
        routesCustomer = []
        routesServedBy = []
        routesDiscount = []
        for line in p.stdout:
            if not line.startswith('Route') and not line.startswith('Discount'):
                print(line)
            line = line.strip()
            # Parse only lines which contain a route
            if line.startswith('Route'):
                label, route = line.split(": ")
                route_nr = int(label.split("#")[-1])
                assert route_nr == len(routesCustomer) + 1, "Route number should be strictly increasing"
                routesCustomer.append([int(node) for node in route.split(" ")])
            elif line.startswith('ServedBy Route'):
                label, route = line.split(": ")
                route_nr = int(label.split("#")[-1])
                assert route_nr == len(routesServedBy) + 1, "Route number should be strictly increasing"
                routesServedBy.append([int(node) for node in route.split(" ")])
            elif line.startswith('Discount Route'):
                label, route = line.split(": ")
                route_nr = int(label.split("#")[-1])
                assert route_nr == len(routesDiscount) + 1, "Route number should be strictly increasing"
                routesDiscount.append([float(node) for node in route.split(" ")])
            elif line.startswith('Cost'):
                # End of solution
                solutionCustomer = routesCustomer
                solutionServedBy = routesServedBy
                solutionsDiscount = routesDiscount
                cost = float(line.split(" ")[-1].strip())
                #check_cost = tools.validate_static_solution(instance, solution)
                #assert cost == check_cost, "Cost of HGS VRPTW solution could not be validated"
                yield solutionServedBy, solutionCustomer, solutionsDiscount, cost
                routesCustomer, routesServedBy, routesDiscount = [], [], []
            elif "EXCEPTION" in line:
                raise Exception("HGS failed with exception: " + line)
        assert len(routesCustomer) == 0, "HGS has terminated with imcomplete solution (is the line with Cost missing?)"

def solutionChecker(solution, instance):
    feasible = True
    totalCost = 0
    for vehicle in solution:
        for index, customer in enumerate(vehicle):
            if customer > instance["nCustomers"]:
                totalCost += instance["pricenSensitivity"]
    
    visitList = []
    for vehicle in solution:
        visitList.append(list(OrderedDict.fromkeys(vehicle)))

    for vehicle in visitList:
        vehicle = [0] + vehicle + [0]
        totalCost += instance["costPerVehicle"]
        arrivalCurrentStop = 0
        for index in range(len(vehicle)-1):
            currentStop = vehicle[index]
            nextStop = vehicle[index+1]
            totalCost += instance["duration_matrix"][currentStop, nextStop]
            if(arrivalCurrentStop > instance["time_windows"][currentStop, 1]):
                feasible = False
            
            arrivalNextStop = max(instance["time_windows"][currentStop,0] + instance["duration_matrix"][currentStop, nextStop] + instance["service_times"][nextStop], arrivalCurrentStop + instance["duration_matrix"][currentStop, nextStop] + instance["service_times"][nextStop])
            arrivalCurrentStop = arrivalNextStop


    print("Solution value: ", round(totalCost/1000,2))
    print("Solution feasible: ", feasible)

    return round(totalCost/1000,2), feasible


if __name__ == "__main__":
    t = 10
    nbClients = [25, 50, 75]
    seeds = [0.5,0.75,0.8]
    for i in nbClients:
        if i ==25:
            t = 10
        elif i == 50:
            t = 30
        else:
            t = 180
        for j in range(10):
            for k in seeds:
                instanceName = str(i) + " requests/r" + str(i) + "_5_" + str(j+1) + ".txt"
                print(instanceName, t)
                instance = tools.read_instance(instanceName)
                tmp_dir = os.path.join("tmp", instanceName)
                cleanup_tmp_dir = False
                try:
                    solutions = list(solve_static_vrptw(instance, time_limit=t, tmp_dir=tmp_dir, seed=42, fractionSREX=k))
                    #print("Final solution: ", solutions[len(solutions)-1][0])
                    solVal = solutionChecker(solutions[len(solutions)-1][0],instance)[0]
                    file1 = open("solutions.txt", "a") 
                    file1.write(str(solVal)+"\n")
                finally:
                    if cleanup_tmp_dir:
                        tools.cleanup_tmp_dir(tmp_dir)
    file1.close()