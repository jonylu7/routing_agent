import WaypointGraph
import elkai
import numpy as np
from Node import Node
import FindPath
from ConvertDataFormat import loadJSONFile
from Task import Task,loadTasksData
from Vehicle import Vehicle,loadVehiclesData

def convertNodeListToNodeIndex(nodeList:list,nodeIndex):
    newList=[]
    for node in nodeList:
        if(type(node)==Vehicle or type(node)==Task):
            newList.append(nodeIndex.index(node.locationNode.id))
        elif(type(node)==Node):
             newList.append(nodeIndex.index(node.id))
    return newList

def findIdInNodeList(nodeList:list,nodeId):
    found=-1
    for index,node in enumerate(nodeList):
        if(type(node)==Vehicle or type(node)==Task):
            if(node.locationNode.id==nodeId):
                found=index
        elif(type(node)==Node):
             if(node.id==nodeId):
                 found=index
    return found


class RoutingEngine:
    occupiedEdgeMatrix:np.array
    taskSequence:list=[]
    latestSolutionPath:list=[]
    routeAtNextUpdate=False
    def __init__(self,waypointgraph:WaypointGraph,tasklist:list,initfleet:list[Vehicle]):
        self.nodeIndex,self.distanceMatrix=waypointgraph.convertToDistanceMatrix()
        self.waypointGraph=waypointgraph
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
        self.taskSequence=solutionId
        return solutionId
    

    def findPathBetweenTwoPoints(self,fromnodeid,tonodeid):
        if(fromnodeid==tonodeid):
            return [],0
        print(self.occupiedDijGraph)
        fromIndex=self.nodeIndex.index(fromnodeid)
        toIndex=self.nodeIndex.index(tonodeid)
        distance,dijpathToAll=FindPath.dijkstra(self.occupiedDijGraph,fromIndex)
        path=FindPath.getPathToDest(dijpathToAll,toIndex)

        idPath=[fromnodeid]
        distanceEstimate=[]
        for p in path:
           idPath.append(self.nodeIndex[p])
           distanceEstimate.append(distance[p])
        print(idPath)
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
            found=self.taskSequence.index(currentnodeid)
            if(found!=-1):
                self.taskSequence.pop(found)

            if(len(self.taskSequence)==0):
                self.latestSolutionPath=[]
                print("Finished All Tasks")
                return []
            
            nextTaskId=self.taskSequence[0]
            
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
        nodeSequence=[]
        graph={}
        for nodeid in self.latestSolutionPath:
            nodeSequence.append(nodeid)
            graph[nodeid]={"local_location":self.waypointGraph.getNodeById(nodeid).localLocation.toList(),"action":"None"}
        jsondata["node_sequence:"]=nodeSequence
        jsondata["graph"]=graph
        return jsondata
            

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
    print(re.update("000_000"))
    print(re.update("000_002"))
    print(re.update("001_000"))
    print(re.update("001_001"))
    print(re.update("001_002"))
    print(re.update("002_000"))
    print(re.update("002_001"))
    print(re.update("002_002"))
    print(re.update("000_003"))
    print(re.update("000_000"))


if __name__=="__main__":
    testRoutingEngine()
    



    

        







        




