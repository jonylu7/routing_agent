import json
import ConvertDataFormat
import OR
import ORExport
import ORAgentClientInit
graph_file_location= "./og_data/waypoint_graph.json"
orders_file_location= "./og_data/orders_data.json"
vehicle_file_location="./og_data/vehicle_data.json"
export_file_location="./og_data/OR_response.json"

def runORAgentByFile(graph_file_location,orders_file_location,vehicle_file_location,export_file_location):
    ORAgentClientInit.initClient()
    ##preprocess
    offsets,edges,weights,orders=ConvertDataFormat.preprocess(graph_file_location,orders_file_location)

    costmatrix,pathmatrix=ConvertDataFormat.generateMatrix(offsets,edges,weights)


    solution,totalCost=OR.solveTSP(costmatrix,orders)
    solutionPath=OR.generateSolution(pathmatrix, solution)
    ORExport.exportFormat(solutionPath,totalCost,export_file_location)
    ##costmatrix as argument

def runORAgentByROS(waypoint_graph_locations,waypoint_graph_edges,waypoint_graph_offsets,task_locations,vehicle_start_location):
    ORAgentClientInit.initClient()
    weights,orders=ConvertDataFormat.preprocessROS(waypoint_graph_locations,waypoint_graph_edges,waypoint_graph_offsets,task_locations,vehicle_start_location)

    costmatrix,pathmatrix=ConvertDataFormat.generateMatrix(waypoint_graph_offsets,waypoint_graph_edges,weights)


    solution,totalCost=OR.solveTSP(costmatrix,orders)
    solutionPath=OR.generateSolution(pathmatrix, solution)
    solPathLocations=ConvertDataFormat.convertSolutionPathToLocation(solutionPath)

    return solutionPath,solPathLocations

if __name__=="__main__":
    runORAgentByFile(graph_file_location,orders_file_location,vehicle_file_location,export_file_location)