from Vector import Vector3
from Node import Node,convertToNodeId,convertJSONnodeToNode
from ConvertDataFormat import loadJSONFile
class WaypointGraph:
    graph={}
    def __init__(self,nodeid:list=[],nodes:list=[]):
        #load files and convert it into 
        for index,id in enumerate(nodeid):
            self.addNode(id,nodes[index])

    def getNodeLocalLocationById(self,id):
        return self.graph[id].localLocation

    def getNodeById(self,id)->Node:
        return self.graph[id]

    def calculateDistanceMatrix(self):
        return

    def addNode(self,id,node):
        self.graph[id]=node

    def findPath(self,source,destination):
        return

    def setEntryPoints(self,fromId,toId):
        fromNode=self.getNodeById(fromId)
        toNode=self.getNodeById(toId)
        if(fromNode.entryToNodeId!=toId):
            fromNode.setEntryPointById(toId)
        if(toNode.entryToNodeId!=fromId):
            toId.setEntryPoint(fromId)
    


def mergeWaypointGraph(allmaps)->WaypointGraph:
    meredGraph=WaypointGraph()
    for mapid,value in allmaps.items():
        map=value["file_data"]
        graph=map["graph"]
        #setup all nodes
        for nodeindex,nodeValue in graph.items():
            nodeid=convertToNodeId(mapid,int(nodeindex))
            newNode=Node(nodeid,nodeValue["local_location"],nodeValue["map_id"],nodeValue["edges"])
            meredGraph.append(newNode)
        #setup all entry points
        entrypoints=value["entry_points"]
        for nodeindex,value in entrypoints.items():
            fromNodeId=convertToNodeId(mapid,nodeindex)
            toNodeId=convertToNodeId(value["map_id"],value["node_index"])
            meredGraph.setEntryPoints(fromNodeId,toNodeId)
    return meredGraph


def convertWaypointGraphToJSON(graph:WaypointGraph):
    jsondata={}
    for nodeid,node in graph.graph.items():
        jsondata[nodeid]={
            "edges":node.edges,
            "local_locations":node.localLocation.toList()
        }
    return jsondata

def loadWaypointGraphData(waypointgraphdata)->WaypointGraph:
    idlist=[]
    nodeList=[]
    for nodeid,value in waypointgraphdata.items():
        idlist.append(nodeid)
        nodeList.append(convertJSONnodeToNode(nodeid,value))

    return WaypointGraph(idlist,nodeList)


if __name__=="__main__":
    data=loadJSONFile("/home/csl/ros2_ws/src/routing_agent/routing_agent/waypoint_graph_global.json")
    graph=loadWaypointGraphData(data)
    print(convertWaypointGraphToJSON(graph))

        






