import WaypointGraph
import elkai
import numpy as np
import Node
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
    return index



class RoutingEngine:
    occupiedEdgeMatrix:np.array
    latestSolutionPath:list
    def __init__(self,waypointgraph:WaypointGraph,tasklist:list,initfleet:list[Vehicle]):
        self.nodeIndex,self.distanceMatrix=waypointgraph.convertToDistanceMatrix()
        self.occupiedEdgeMatrix=np.zeros(self.distanceMatrix.shape)
        self.taskList=tasklist
        self.fleet=initfleet

    def solveTSP(self):
        goThoughNodes=self.taskList.insert(0,self.fleet[0].locationNode.id)
        goThoughNodeIndexList=convertNodeListToNodeIndex(goThoughNodes,self.nodeIndex)
        
        costMatrix=self.occupiedEdgeMatrix+self.distanceMatrix

        prunnedCostMatrix=self.__prunCostMatrix(costMatrix,goThoughNodeIndexList)
        costMatrixSol=self.__solvePrunnedCostMatrix(prunnedCostMatrix,goThoughNodeIndexList)

        totalCost=0
        for i in range(len(costMatrixSol)-1):
            totalCost+=costMatrix[costMatrixSol[i],costMatrixSol[i+1]]

        ##convert to int 64
        val = np.int64(0)
        totalCost=val.item()
        return costMatrixSol,totalCost


    def generateSolutionsAndConvertToNodeId(self):
        costMatrixSol,totalCost=self.solveTSP()
        solutionId=[]
        for index in costMatrixSol:
            solutionId.append(self.nodeIndex[index])
        self.latestSolutionPath=solutionId
        return solutionId
        

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
        startindex=self.latestSolutionPath.index(currentnodeid)
        self.fleet[0].locationNode=currentnodeid
        self.latestSolutionPath=self.latestSolutionPath[startindex::]

        found=findIdInNodeList(self.taskList,currentnodeid)
        if(found!=-1):
            self.taskList.pop(found)

    def setOccupiedEdge(self,occupiedEdge:list[str]):
        self.__setMatrixEdgeValue(self.occupiedEdgeMatrix,occupiedEdge,float("inf"))
        
    def removeOccupiedEdge(self,occupiedEdge:list[str]):
        self.__setMatrixEdgeValue(self.occupiedEdgeMatrix,occupiedEdge,0)

    def __setMatrixEdgeValue(self,matrix:np.array,edge:list[str],value):
        node1,node2=edge[0],edge[1]

        node1Index=self.nodeIndex.index(node1)
        node2Index=self.nodeIndex.index(node2)

        matrix[node1Index,node2Index]=value
        matrix[node2Index,node1Index]=value


    

        







        




