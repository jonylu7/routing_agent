import json
import numpy as np
import FindPath


def loadJSONFile(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data

def loadJSONIntoStr(file_path):
    data=loadJSONFile(file_path)
    data=json.dumps(data)
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
    return  np.array(edges),np.array(offsets),

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
    nodeLen=int(len(waypoint_graph_locations)/3)
    weights=[]
    for startnode in range(nodeLen):
        offset=waypoint_graph_offsets[startnode]
        if(startnode!=nodeLen-1):
            nextoffset=waypoint_graph_offsets[startnode+1]
        else:
            nextoffset=len(waypoint_graph_edges)

        endNodes=waypoint_graph_edges[offset:nextoffset]
        startlocation=waypoint_graph_locations[startnode*3:startnode*3+3]
        if(len(endNodes)==0):
            continue
        for endnode in endNodes:
            endlocation=waypoint_graph_locations[endnode*3:endnode*3+3]
            if(len(endlocation)==0):
                continue
            weights.append(calculateDistance(startlocation,endlocation))
    return weights

    

def findOrderRelativeToNodeIndexByFile(orderlocation,graphFile):
    for index,nodeLocation in enumerate(graphFile["node_locations"]):
        if(abs(nodeLocation[0]-orderlocation[0])<1.0 and abs(nodeLocation[1]-orderlocation[1])<1.0 and abs(nodeLocation[2]-orderlocation[2])<1.0):
            return index
        
def findOrderRelativeToNodeIndexByValue(orderlocation,waypoint_graph_locations):
    nodeLen=int(len(waypoint_graph_locations)/3)
    for index in range(nodeLen):
        nodeLocation=waypoint_graph_locations[index*3:index*3+3]
        if(abs(nodeLocation[0]-orderlocation[0])<1.0 and abs(nodeLocation[1]-orderlocation[1])<1.0 and abs(nodeLocation[2]-orderlocation[2])<1.0):
            return index

    raise RuntimeError("FailedToFindCompatiableOrders")


def convertOrdersDataByFile(ordersdata,graphFile):
    orders=[]
    for orderlocation in ordersdata["task_locations"]:
        index=findOrderRelativeToNodeIndexByFile(orderlocation,graphFile)
        orders.append(index)
    return orders

def convertOrdersDataByValue(ordersLocation,waypoint_graph_locations):
    orderLen=int(len(ordersLocation)/3)
    orders=[]
    for index in range(orderLen):
        orderlocation=ordersLocation[index*3:index*3+3]
        index=findOrderRelativeToNodeIndexByValue(orderlocation,waypoint_graph_locations)
        orders.append(index)
    return orders


def convertGraphData(graphfile):
    offsets, edges = convertGraphFileToCSR(graphfile)
    weights=calculateGraphWeightByFile(graphfile)
    return offsets, edges, weights


def preprocess(graphfilelocation,ordersfilelocation):
    graphFile = loadJSONFile(graphfilelocation)
    offsets, edges, weights=convertGraphData(graphFile)


    ordersLocation = loadJSONFile(ordersfilelocation)
    orders = convertOrdersDataByFile(ordersLocation,graphFile)
    ## start from zero
    orders.insert(0,0)

    return offsets, edges, weights,orders


def preprocessROS(waypoint_graph_locations,waypoint_graph_edges,waypoint_graph_offsets,task_locations,vehicle_start_location):
    weights=caluclateGraphWeightByValue(waypoint_graph_locations,waypoint_graph_edges,waypoint_graph_offsets)
    orders=convertOrdersDataByValue(task_locations,waypoint_graph_locations)

    vehicleStartNodeIndex=findOrderRelativeToNodeIndexByValue(vehicle_start_location,waypoint_graph_locations)
    if(vehicleStartNodeIndex==-1):
        orders.insert(0,0)
    else:
        orders.insert(vehicleStartNodeIndex,0)
    return weights,orders


def convertSolutionPathToLocation(solutionpath,waypoint_graph_locations):
    solutionPathWithLocations=[]
    for node in solutionpath:
        solutionPathWithLocations+=waypoint_graph_locations[node*3:node*3+3]
    return solutionPathWithLocations

def convertFrom2DListTo1D(list):
    plainList=[]
    for item in list:
        plainList+=item
    return plainList

def convertFromFileToROSServiceFormat(waypoint_graph_file_location,orders_file_location,vehicle_data_location):
    waypointGraphFile=loadJSONFile(waypoint_graph_file_location)
    orderFile=loadJSONFile(orders_file_location)
    vehicleFile=loadJSONFile(vehicle_data_location)
    waypoint_graph_locations=waypointGraphFile["node_locations"]
    waypoint_graph_locations=list(map(float,convertFrom2DListTo1D(waypoint_graph_locations)))

    waypoint_graph_edges,waypoint_graph_offsets=convertGraphFileToCSR(waypointGraphFile)
    waypoint_graph_edges=waypoint_graph_edges.tolist()
    waypoint_graph_offsets=waypoint_graph_offsets.tolist()

    task_locations=orderFile["task_locations"]
    task_locations=list(map(float,convertFrom2DListTo1D(task_locations)))

    vehicle_start_location=list(map(float,vehicleFile["vehicle_locations"][0]))
    
    return waypoint_graph_locations,waypoint_graph_edges,waypoint_graph_offsets,task_locations,vehicle_start_location

def readRequestFiles(waypoint_graph_file_location,orders_file_location,vehicle_data_location):
    waypointGraphData=loadJSONIntoStr(waypoint_graph_file_location)
    orderData=loadJSONIntoStr(orders_file_location)
    vehicleData=loadJSONIntoStr(vehicle_data_location)
    return waypointGraphData,orderData,vehicleData

def mergeWaypointGraph(waypoint_graph_file_data):
    

    # shift map_1:[0,50]
    mapDict={"map_1":[1,23]}
    
    return 


def isWithInRange(index,mapRange)->bool:
    if index>=mapRange[0] and index<=mapRange[1]:
        return True
    else:
        return False

def splitWaypointGraph():
    isWithInRange(3,[1,2])
    return
    