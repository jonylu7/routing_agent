from Vector import Vector3
class Node:
    shift=Vector3(0,0,0)
    def __init__(self,id:str="DEFAULT_NODE",locallocation:Vector3=Vector3(),mapid:str="DEFAULT_MAP",edges:list=[],isentrypoint:bool=False,entrytonodeid:str="DEFAULT_ENTRY_TO_NODE"):
        self.id=id
        self.edges=edges
        self.localLocation=locallocation
        self.mapId=mapid
        self.isEntryPoint=isentrypoint
        self.entryToNodeId=entrytonodeid
    
    def setLocalToGlobalShift(self,shift):
        self.shift=shift

    def getGlobalLocation(self)->Vector3:
        return self.localLocation+self.shift
    def setEntryPointById(self,nodeid:str):
        self.isEntryPoint=True
        self.entryToNodeId


def convertToNodeId(mapid,nodeindex)->str:
    return 

def getMapIdByNodeId(nodeid):
    mapid,_=nodeid.split("_")
    return mapid

def convertJSONnodeToNode(id,nodedata):
    x=float(nodedata["local_location"][0])
    y=float(nodedata["local_location"][1])
    z=float(nodedata["local_location"][2])
    location=Vector3(x,y,z)
    mapId=getMapIdByNodeId(id)
    return Node(id,location,mapId,nodedata["edges"])