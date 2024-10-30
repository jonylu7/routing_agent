from routing_agent.Vector import Vector3

class WaypointGraph:
    nodes={}
    def __init__(self,nodeid:list=[],nodes:list=[]):
        #load files and convert it into 
        for index,id in enumerate(nodeid):
            self.addNode(id,nodes[index])

    def getNodeLocalLocationById(self,id):
        return self.nodes[id].localLocation

    def getNodeGlobalLocationById(self,id):
        return self.nodes[id].getGlobalLocation()

    def calculateDistanceMatrix(self):
        return

    def saveAsJSON(self,filepath):
        jsondata=self.getJSONStr()
        return 
    
    def getJSONStr(self):

        return ""

    def addNode(self,id,node):
        self.nodes[id]=node

    def findPath(self,source,destination):
        return

    




        






