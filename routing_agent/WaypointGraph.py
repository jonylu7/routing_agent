from Vector import Vector3
from Node import Node,convertToNodeId,convertJSONnodeToNode,calculateDistanceBetweenNodes
import ConvertDataFormat
from ConvertDataFormat import loadJSONFile
from pathlib import Path
import FindPath
class WaypointGraph:
    graph={}
    def __init__(self,nodeid:list[str]=[],nodes:list[Node]=[]):
        #load files and convert it into 
        for index,id in enumerate(nodeid):
            self.addNode(id,nodes[index])

    def getNodeLocalLocationById(self,id):
        return self.graph[id].localLocation

    def getNodeById(self,id)->Node:
        return self.graph[id]

    def addNode(self,node):
        self.graph[node.id]=node

    def setEntryPoints(self,fromId,toId):
        fromNode=self.getNodeById(fromId)
        toNode=self.getNodeById(toId)
        if(fromNode.entryToNodeId!=toId):
            fromNode.setEntryPointById(toId)
        if(toNode.entryToNodeId!=fromId):
            toNode.setEntryPointById(fromId)

    
    def convertToDistanceMatrix(self):
        nodeIdIndex,offsets,edges,weights=self.__convertToCSR()
        graph=FindPath.convertFromCSRToDijGraph(offsets,edges,weights)
        costmatrix,pathmatrix=FindPath.findAllShortestPath(graph)
        return nodeIdIndex,costmatrix

    def __calculateDistanceOfEdges(self,nodeIdIndex,fromNode,edges):
        weights=[float('inf')]*len(edges)
        for toNode in edges:
            insertAt=nodeIdIndex.index(toNode)
            weights[insertAt]=calculateDistanceBetweenNodes(fromNode,toNode)
        #connect entrypoints
        if fromNode.isEntryPoint:
            weights[nodeIdIndex.index(fromNode.entryToNodeId)]=0
        return weights

    def __convertToCSR(self):
        nodeIdIndex=list(self.graph.keys())
        offsets=[]
        edges=[]
        weights=[]
        for _,thisNode in self.graph.items():
            offsets.append(len(edges))
            edges.append(thisNode.edges)
            weights+=self.__calculateDistanceOfEdges(nodeIdIndex,thisNode,self.edges)
        return nodeIdIndex,offsets,edges,weights
    

def mergeWaypointGraph(allmaps)->WaypointGraph:
    meredGraph=WaypointGraph()
    for mapid,value in allmaps.items():
        map=value["file_data"]
        graph=map["graph"]
        nodeLocations=map["node_locations"]
        #setup all nodes
        for nodeindex,nodeValue in graph.items():
            nodeid=convertToNodeId(mapid,int(nodeindex))
            x=float(nodeLocations[int(nodeindex)][0])
            y=float(nodeLocations[int(nodeindex)][1])
            z=float(nodeLocations[int(nodeindex)][2])
            location=Vector3(x,y,z)
            edgesNewId=[]
            for edgesNodeIndex in nodeValue["edges"]:
                edgesNodeId=convertToNodeId(mapid,edgesNodeIndex)
                edgesNewId.append(edgesNodeId)
            newNode=Node(nodeid,location,mapid,edgesNewId)
            meredGraph.addNode(newNode)
    #setup all entry points
    for mapid,value in allmaps.items():
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




def testLoadMap():
    data=loadJSONFile("/home/csl/ros2_ws/src/routing_agent/routing_agent/waypoint_graph_global.json")
    graph=loadWaypointGraphData(data)
    print(convertWaypointGraphToJSON(graph))

def testMergeMaps():
    filepath = Path("routing_agent/preprocess_maps/maps.config.json")

    configData=ConvertDataFormat.loadJSONFile(filepath)
    for mapid in configData.keys():
        mapPath=filepath.parent+configData[mapid]["file_path"]
        mapData=ConvertDataFormat.loadJSONFile(mapPath)
        configData[mapid]["file_data"]=mapData
    print(configData)
    graph=mergeWaypointGraph(configData)
    print(convertWaypointGraphToJSON(graph))


if __name__=="__main__":
   testMergeMaps()

        






