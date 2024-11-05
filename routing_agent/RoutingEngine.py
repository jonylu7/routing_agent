import WaypointGraph
import elkai
import numpy as np
import Node
import FindPath
from ConvertDataFormat import loadJSONFile
class Task:
    def __init__(self,locationnode,demand=1):
        self.locationNode=locationnode
        self.demand=demand

class Vehicle:
    def __init__(self,locationnode,capcity=1):
        self.locationNode=locationnode
        self.capacity=capcity


def convertNodeListToNodeIndex(nodeList:list,nodeIndex):
    newList=[]
    for node in nodeList:
        if(type(node)==Vehicle or type(node)==Task):
            newList.append(nodeIndex.index(node.locationNode.id))
        elif(type(node)==Node.Node):
             newList.append(nodeIndex.index(node.id))
    return newList


def findIdInNodeList(nodeList:list,nodeId):
    found=-1
    for index,node in enumerate(nodeList):
        if(type(node)==Vehicle or type(node)==Task):
            if(node.locationNode.id==nodeId):
                found=index
        elif(type(node)==Node.Node):
             if(node.id==nodeId):
                 found=index
    return found


def loadVehiclesData(waypointGraph:WaypointGraph,vehicledata):
    vehicleSize=len(vehicledata["vehicle_locations"])
    fleet=[]
    for i in range(vehicleSize):
        node=waypointGraph.getNodeById(vehicledata["vehicle_locations"][i])
        capacity=vehicledata["vehicle_locations"][i]
        fleet.append(Vehicle(node,capacity))
    return fleet



def loadTasksData(waypointGraph:WaypointGraph,taskdata):
    vehicleSize=len(taskdata["task_sequence"])
    tasks=[]
    for i in range(vehicleSize):
        try:
            node=waypointGraph.getNodeById(taskdata["task_sequence"][i])
        except:
            raise KeyError("Failed to load task data, can't match task node with waypointgraph")
        demand=taskdata["task_sequence"][i]
        tasks.append(Vehicle(node,demand))
    return tasks




class RoutingEngine:
    occupiedEdgeMatrix:np.array
    latestSolutionPath:list=[]
    routeAtNextUpdate=False
    def __init__(self,waypointgraph:WaypointGraph,tasklist:list,initfleet:list[Vehicle]):
        self.nodeIndex,self.distanceMatrix=waypointgraph.convertToDistanceMatrix()

        _,self.ogDijGraph=waypointgraph.convertToDijGraph()
        self.occupiedDijGraph=self.ogDijGraph

        self.occupiedEdgeMatrix=np.zeros(self.distanceMatrix.shape)
        self.taskList=tasklist
        self.fleet=initfleet

    def __solveTSP(self):
        goThoughNodes=self.taskList
        goThoughNodes.insert(0,self.fleet[0].locationNode)
        goThoughNodeIndexList=convertNodeListToNodeIndex(goThoughNodes,self.nodeIndex)
        
        costMatrix=self.occupiedEdgeMatrix+self.distanceMatrix
        prunnedCostMatrix=self.__prunCostMatrix(costMatrix,goThoughNodeIndexList)
        costMatrixSol=self.__solvePrunnedCostMatrix(prunnedCostMatrix,goThoughNodeIndexList)

        totalCost=0
        #print(self.distanceMatrix)
        for i in range(len(costMatrixSol)-1):
            totalCost+=costMatrix[costMatrixSol[i],costMatrixSol[i+1]]

        return costMatrixSol,totalCost


    def solve(self):
        costMatrixSol,totalCost=self.__solveTSP()
        solutionId=[]
        for index in costMatrixSol:
            solutionId.append(self.nodeIndex[index])
        self.routeAtNextUpdate=True
        return solutionId
    

    def findPathBetweenTwoPoints(self,fromnodeid,tonodeid):
        fromIndex=self.nodeIndex.index(fromnodeid)
        toIndex=self.nodeIndex.index(tonodeid)
        distance,dijpathToAll=FindPath.dijkstra(self.occupiedDijGraph,fromIndex)
        path=FindPath.getPathToDest(dijpathToAll,toIndex)

        idPath=[fromnodeid]
        distanceEstimate=[]
        for p in path:
           idPath.append(self.nodeIndex[p])
           distanceEstimate.append(distance[p])
        return idPath,distanceEstimate

    def __prunCostMatrix(self,costMatrix,orders):
        prunnedCostMatrix=[]
        for index,orderx in enumerate(orders):
            row=[]
            for ordery in orders:
                row.append(costMatrix[orderx-1][ordery-1])
            prunnedCostMatrix.append(row)
        return np.array(prunnedCostMatrix)
    
    def __solvePrunnedCostMatrix(self,costMatrix,orders):
        tempSol=elkai.DistanceMatrix(costMatrix.tolist()).solve_tsp()
        sol=[]
        for t in tempSol:
            sol.append(orders[t])
        return sol

    def update(self,currentnodeid):
        # route
        if(self.routeAtNextUpdate or len(self.latestSolutionPath)==1):
            found=findIdInNodeList(self.taskList,currentnodeid)
            if(found!=-1):
                self.taskList.pop(found)
            if(len(self.taskList)>1):
                nextTaskId=self.taskList[1].locationNode.id
            elif(len(self.taskList)==1):
               nextTaskId=self.taskList[0].locationNode.id
            elif(len(self.taskList)==0):
                print("Finished All Tasks")
                return []
            idPath,distanceEstimates=self.findPathBetweenTwoPoints(currentnodeid,nextTaskId)
            self.latestSolutionPath=idPath
            self.routeAtNextUpdate=False

        #update solutionpath
        startindex=self.latestSolutionPath.index(currentnodeid)
        self.latestSolutionPath=self.latestSolutionPath[startindex+1::]
        return self.latestSolutionPath

    def response(self):
        #for 
        jsondata={}



    def setOccupiedEdge(self,occupiedEdge:list[str]):
        self.__setDijGraphValue(occupiedEdge,float("inf"))
        self.routeAtNextUpdate=True
    def removeOccupiedEdge(self,occupiedEdge:list[str]):
        self.__setDijGraphValue(occupiedEdge,0)
        self.routeAtNextUpdate=True

    def __setMatrixEdgeValue(self,matrix:np.array,edge:list[str],value):
        node1,node2=edge[0],edge[1]

        node1Index=self.nodeIndex.index(node1)
        node2Index=self.nodeIndex.index(node2)

        matrix[node1Index,node2Index]=value
        matrix[node2Index,node1Index]=value

    def __setDijGraphValue(self,occupiedEdge,value):
        node1,node2=occupiedEdge[0],occupiedEdge[1]

        node1Index=self.nodeIndex.index(node1)
        node2Index=self.nodeIndex.index(node2)
        
        if(value==0):
            self.occupiedDijGraph[node1Index][node2Index]=self.ogDijGraph[node1Index][node2Index]
            self.occupiedDijGraph[node2Index][node1Index]=self.ogDijGraph[node2Index][node1Index]
        elif(value==float('inf')):
            self.occupiedDijGraph[node1Index][node2Index]=float('inf')
            self.occupiedDijGraph[node2Index][node1Index]=float('inf')

def testRoutingEngine():
    graph=WaypointGraph.testMergeMaps()
    vehdata=loadJSONFile("routing_agent/vehicle_data.json")
    vehicles=loadVehiclesData(graph,vehdata)
    taskdata=loadJSONFile("routing_agent/task_data.json")
    tasks=loadTasksData(graph,taskdata)
    re=RoutingEngine(graph,tasks,vehicles)
    re.solve()
    re.update("000_000")
    re.update("000_002")
    re.update("001_000")
    re.update("001_001")
    re.update("001_002")
    re.update("002_000")
    re.update("002_001")


if __name__=="__main__":
    testRoutingEngine()
    



    

        







        




