from routing_agent.Vector import Vector3
class Node:
    shift=Vector3(0,0,0)
    def __init__(self,id:str,locallocation:Vector3,mapid:str,edges:list):
        self.id=id
        self.edges=edges
        self.localLocation=locallocation
        self.mapId=mapid
    
    def setLocalToGlobalShift(self,shift):
        self.shift=shift

    def getGlobalLocation(self)->Vector3:
        return self.localLocation+self.shift


def convertToNodeId(mapid,nodeindex)->str:
    return ""