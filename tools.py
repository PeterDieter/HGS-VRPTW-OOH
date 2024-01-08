import numpy as np
import os

def readlines(filename):
    try:
        with open(filename, 'r') as f:
            return f.readlines()
    except:
        with open(filename, 'rb') as f:
            return [line.decode('utf-8', errors='ignore').strip() for line in f.readlines()]



def read_vrptw_solution(filename, return_extra=False):
    """Reads a VRPTW solution in VRPLib format (one route per row)"""
    solution = []
    extra = {}
    
    for line in readlines(filename):
        if line.startswith('Route'):
            solution.append(np.array([int(node) for node in line.split(":")[-1].strip().split(" ")]))
        else:
            if len(line.strip().split(" ")) == 2:
                key, val = line.strip().split(" ")
                extra[key] = val
    
    if return_extra:
        return solution, extra
    return solution   

def cleanup_tmp_dir(tmp_dir):
    if not os.path.isdir(tmp_dir):
        return
    # Don't use shutil.rmtree for safety :)
    for filename in os.listdir(tmp_dir):
        filepath = os.path.join(tmp_dir, filename)
        if 'problem.vrptw' in filename and os.path.isfile(filepath):
            os.remove(filepath)
    assert len(os.listdir(tmp_dir)) == 0, "Unexpected files in tmp_dir"    
    os.rmdir(tmp_dir)

def read_instance(name):

    with open("instances/"+name) as file:
        lines = [line.rstrip().split("\t") for line in file]
    
    nCustomers = int(lines[0][0][2:])
    nParcelPoints = int(lines[1][0][2:])
    nVehicles = int(lines[3][0][2:])

    coords,serviceTimes, tws, cellsPerParcelPoint = [], [], [], []
    for line in lines[6:]:
        coords.append([float(line[1]),float(line[2])])
        tws.append([max(0,int(line[3])*1000),int(line[4])*1000])
        serviceTimes.append(int(line[5])*1000)
        if float(line[6])>0:
            cellsPerParcelPoint.append(int(line[6]))
    

    demandVector = np.array([0] + [1]*nCustomers +  [0]*nParcelPoints) 
    is_depot = np.array([True] + [False]*(nCustomers+nParcelPoints)) 
    is_parcelPoint = np.array([False]*(1+nCustomers) + [True]*nParcelPoints) 
    is_customer = np.array([False] + [True]*nCustomers+[False]*nParcelPoints) 
    
    tws = np.array(tws)
    tws[0][1] = 720*1000
    tws[is_parcelPoint,1] = np.ones((nParcelPoints))* tws[0][1]*2

    distanceMatrix = np.zeros((len(coords), len(coords)))
    for i in range(len(coords)):
        for j in range(len(coords)):
            distanceMatrix[i, j] = np.linalg.norm(np.array(coords[i])- np.array(coords[j]))*3000
    instance = {'coords': np.array(coords),
                'demands': demandVector,
                'is_depot': np.array(is_depot),
                'duration_matrix': distanceMatrix,
                'capacityVehicle': 1000,
                'capacityParcelPointCell': 1,
                'cellsPerParcelPoint': cellsPerParcelPoint,
                'nVehicles': nVehicles,
                'nCustomers': nCustomers,
                'pricenSensitivity': int(lines[4][0][6:])*1000,
                'costPerVehicle': int(lines[5][0][6:])*1000,
                'nParcelPoints': nParcelPoints,
                'maxWalkingMeter': 15*1000,
                'service_times': np.array(serviceTimes),
                'time_windows': np.array(tws),
                'is_parcelPoint': np.array(is_parcelPoint),
                'is_customer': np.array(is_customer)
                }

    return instance


def write_vrplib(filename, instance, name="problem", euclidean=False, is_vrptw=True, is_vrppp=True):
    # LKH/VRP does not take floats (HGS seems to do)
    coords = instance['coords'].astype(int)
    demands = instance['demands'].astype(int)
    is_depot = instance['is_depot']
    duration_matrix = np.round(instance['duration_matrix']).astype(int)
    capacity = instance['capacityVehicle']
    nVehicles = instance['nVehicles']
    assert (np.diag(duration_matrix) == 0).all()
    assert (demands[instance['is_customer']] > 0).all()
    
    with open(filename, 'w') as f:
        f.write("\n".join([
            "{} : {}".format(k, v)
            for k, v in [
                ("NAME", name),
                ("COMMENT", "MANCINI&GANSTERER"),  # For HGS we need an extra row...
                ("TYPE", "CVRP"),
                ("NUMBER_PARCEL_POINTS", instance['nParcelPoints']),
                ("DIMENSION", len(coords)),
                ("EDGE_WEIGHT_TYPE", "EUC_2D" if euclidean else "EXPLICIT"),
            ] + ([] if euclidean else [
                ("EDGE_WEIGHT_FORMAT", "FULL_MATRIX")
            ]) + [("VEHICLE_CAPACITY", capacity)]
            + [("VEHICLES", nVehicles)]
            + [("PARCELPOINT_CAPACITY_PER_CELL", instance['capacityParcelPointCell'])]
            + [("MAXIMAL_WALKING_DISTANCE", instance['maxWalkingMeter'])]
            + [("COST_PER_VEHICLE", instance['costPerVehicle'])]
            + [("PRICE_SENSITIVITY", instance['pricenSensitivity'])]
        ]))
        f.write("\n")
        
        if not euclidean:
            f.write("EDGE_WEIGHT_SECTION\n")
            for row in duration_matrix:
                f.write("\t".join(map(str, row)))
                f.write("\n")
        
        f.write("NODE_COORD_SECTION\n")
        f.write("\n".join([
            "{}\t{}\t{}".format(i + 1, x, y)
            for i, (x, y) in enumerate(coords)
        ]))
        f.write("\n")
        
        f.write("DEMAND_SECTION\n")
        f.write("\n".join([
            "{}\t{}".format(i + 1, d)
            for i, d in enumerate(demands)
        ]))
        f.write("\n")

        f.write("PARCELPOINT_CAPACITY_NUMBER_CELLS\n")
        f.write("\n".join([
            "{}\t{}".format(i+1,d)
            for i,d in enumerate(instance['cellsPerParcelPoint'])
        ]))
        f.write("\n")
        
        f.write("DEPOT_SECTION\n")
        for i in np.flatnonzero(is_depot):
            f.write(f"{i+1}\n")
        f.write("-1\n")
        
        if is_vrptw:
            
            service_t = instance['service_times'].astype(int)
            timewi = instance['time_windows'].astype(int)
            
            # Following LKH convention
            f.write("SERVICE_TIME_SECTION\n")
            f.write("\n".join([
                "{}\t{}".format(i + 1, s)
                for i, s in enumerate(service_t)
            ]))
            f.write("\n")
            
            f.write("TIME_WINDOW_SECTION\n")
            f.write("\n".join([
                "{}\t{}\t{}".format(i + 1, l, u)
                for i, (l, u) in enumerate(timewi)
            ]))
            f.write("\n")

            if 'release_times' in instance:
                release_times = instance['release_times']

                f.write("RELEASE_TIME_SECTION\n")
                f.write("\n".join([
                    "{}\t{}".format(i + 1, s)
                    for i, s in enumerate(release_times)
                ]))
                f.write("\n")
        f.write("EOF\n")