import elkai
import numpy as np
import json
import FindPath

def prunCostMatrix(costMatrix,orders):
    prunnedCostMatrix=[]
    for index,orderx in enumerate(orders):
        row=[]
        for ordery in orders:
            row.append(costMatrix[orderx-1][ordery-1])
        prunnedCostMatrix.append(row)
    return np.array(prunnedCostMatrix)

def solvePrunnedCostMatrix(costMatrix,orders):
    tempSol=elkai.DistanceMatrix(costMatrix.tolist()).solve_tsp()
    sol=[]
    for t in tempSol:
        sol.append(orders[t])
    return sol

def solveTSP(costMatrix,orders):
    prunnedCostMatrix=prunCostMatrix(costMatrix,orders)
    costMatrixSol=solvePrunnedCostMatrix(prunnedCostMatrix,orders)

    totalCost=0
    for i in range(len(costMatrixSol)-1):
        totalCost+=costMatrix[costMatrixSol[i],costMatrixSol[i+1]]

    ##convert to int 64
    val = np.int64(0)
    totalCost=val.item()
    return costMatrixSol,totalCost

def generateSolution(pathMatrix, costMatrixSol):
    solutionPath=[]
    for i in range(len(costMatrixSol)-1):
        solutionPath+=(pathMatrix[costMatrixSol[i]][costMatrixSol[i+1]])[:-1]
    return solutionPath

def convertWaypointIndexsToLocations(waypointIndexs,nodeLocations):
    nodeLocations=[]
    for i in waypointIndexs:
        nodeLocations.append(nodeLocations[i])
    return nodeLocations

def generateMatrix(offsets,edges,weights):
    graph=FindPath.convertFromCSRToDijGraph(offsets,edges,weights)
    costmatrix,pathmatrix=FindPath.findAllShortestPath(graph)
    return costmatrix,pathmatrix



