import json
import numpy as np
import FindPath

def loadFile(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data

def convertGraphFileToCSR(graphfile):

    num_nodes = len(graphfile["graph"])

    offsets = []
    edges = []

    cur_offset = 0
    for node in range(num_nodes):
        offsets.append(cur_offset)
        cur_offset += len(graphfile["graph"][str(node)]["edges"])
        edges = edges + graphfile["graph"][str(node)]["edges"]

    offsets.append(cur_offset)
    return np.array(offsets), np.array(edges)

def generateMatrix(offsets,edges,weights):
    graph=FindPath.convertFromCSRToDijGraph(offsets,edges,weights)

    costmatrix,pathmatrix=FindPath.findAllShortestPath(graph)
    return costmatrix,pathmatrix


def calculateDistance(location1,location2):
    return((location1[0]-location2[0])**2+(location1[1]-location2[1])**2+(location1[2]-location2[2])**2)**0.5

def calculateGraphWeightByFile(graphfile):
    weights=[]
    for startnode in graphfile["graph"]:
        for endnode in graphfile["graph"][startnode]["edges"]:
            startlocation=graphfile["node_locations"][int(startnode)]
            endlocation=graphfile["node_locations"][int(endnode)]
            weights.append(calculateDistance(startlocation,endlocation))
    return weights

def caluclateGraphWeightByValue(waypoint_graph_locations,waypoint_graph_edges,waypoint_graph_offsets):
    nodeLen=len(waypoint_graph_locations)/2
    weights=[]
    for startnode in range(nodeLen):
        offset=waypoint_graph_offsets[startnode]
        if(startnode!=nodeLen-1):
            nextoffset=waypoint_graph_offsets[startnode+1]
        else:
            nextoffset=len(waypoint_graph_edges)
        endNodes=waypoint_graph_edges[offset:nextoffset]

        for endnode in endNodes:
            startlocation=[waypoint_graph_locations[startnode*2],waypoint_graph_locations[startnode*2+1]]
            endlocation=[waypoint_graph_locations[endnode*2],waypoint_graph_locations[endnode*2+1]]
            weights.append(calculateDistance(startlocation,endlocation))
    return weights

    

def findOrderRelativeToNodeIndexByFile(orderlocation,graphFile):
    for index,nodeLocation in enumerate(graphFile["node_locations"]):
        if(abs(nodeLocation[0]-orderlocation[0])<1 and abs(nodeLocation[1]-orderlocation[1])<1 and abs(nodeLocation[2]-orderlocation[2])<1):
            return index
        
def findOrderRelativeToNodeIndexByValue(orderlocation,waypoint_graph_locations):
    nodeLen=len(waypoint_graph_locations)/2
    for index in range(nodeLen+1):
        nodeLocation=[waypoint_graph_locations[index],waypoint_graph_locations[index+1]]
        if(abs(nodeLocation[0]-orderlocation[0])<1 and abs(nodeLocation[1]-orderlocation[1])<1 and abs(nodeLocation[2]-orderlocation[2])<1):
            return index

    return -1


def convertOrdersDataByFile(ordersdata,graphFile):
    orders=[]
    for orderlocation in ordersdata["task_locations"]:
        index=findOrderRelativeToNodeIndexByFile(orderlocation,graphFile)
        orders.append(index)
    return orders

def convertOrdersDataByValue(ordersLocation,waypoint_graph_locations):
    orderLen=len(ordersLocation)/2
    orders=[]
    for index in range(orderLen+1):
        orderlocation=[ordersLocation[index],ordersLocation[index+1]]
        index=findOrderRelativeToNodeIndexByValue(orderlocation,waypoint_graph_locations)
        orders.append(index)
    return orders


def convertGraphData(graphfile):
    print(type(graphfile))
    offsets, edges = convertGraphFileToCSR(graphfile)
    weights=calculateGraphWeightByFile(graphfile)
    return offsets, edges, weights


def preprocess(graphfilelocation,ordersfilelocation):
    graphFile = loadFile(graphfilelocation)
    offsets, edges, weights=convertGraphData(graphFile)


    ordersLocation = loadFile(ordersfilelocation)
    orders = convertOrdersDataByFile(ordersLocation,graphFile)
    ## start from zero
    orders.insert(0,0)

    return offsets, edges, weights,orders


def preprocessROS(waypoint_graph_locations,waypoint_graph_edges,waypoint_graph_offsets,task_locations,vehicle_start_location):
    weights=caluclateGraphWeightByValue(waypoint_graph_locations,waypoint_graph_edges,waypoint_graph_offsets)
    orders=convertOrdersDataByValue(task_locations,vehicle_start_location)
    vehicleStartNodeIndex=findOrderRelativeToNodeIndexByValue(vehicle_start_location,waypoint_graph_locations)
    if(vehicleStartNodeIndex==-1):
        orders.insert(0,0)
    else:
        orders.insert(vehicleStartNodeIndex,0)
    return weights,orders


def convertSolutionPathToLocation(solutionpath,waypoint_graph_locations):
    solutionPathWithLocations=[]
    for node in solutionpath:
        solutionPathWithLocations.push(waypoint_graph_locations[node*2])
        solutionPathWithLocations.push(waypoint_graph_locations[node*2+1])
    return solutionPathWithLocations


def convertFromFileToROSServiceFormat(waypoint_graph_file_location,orders_file_location,vehicle_data_location):
    waypointGraphFile=loadFile(waypoint_graph_file_location)
    orderFile=loadFile(orders_file_location)
    vehicleFile=loadFile(vehicle_data_location)
    waypoint_graph_locations=waypointGraphFile["node_locations"]
    waypoint_graph_edges,waypoint_graph_offsets=convertGraphFileToCSR(waypointGraphFile)
    waypoint_graph_edges=waypoint_graph_edges.tolist()
    waypoint_graph_offsets=waypoint_graph_offsets.tolist()
    task_locations=orderFile["task_locations"]
    vehicle_start_location=vehicleFile["vehicle_locations"][0]
    
    return waypoint_graph_locations,waypoint_graph_edges,waypoint_graph_offsets,task_locations,vehicle_start_location