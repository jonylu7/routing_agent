from WaypointGraph import WaypointGraph
from Vehicle import Vehicle
class Task:
    def __init__(self,locationnode,demand=1):
        self.locationNode=locationnode
        self.demand=demand
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
