class Vector3:
    def __init__(self,x=0,y=0,z=0):
        self.x=x
        self.y=y
        self.z=z
        return

    def __add__(self,other):
        return Vector3(self.x+other.x,self.y+other.y,self.z+other.z)

    def toList(self):
        return [self.x,self.y,self.z]
def calculateDistance(source:Vector3,destination:Vector3):
    return((source.x-destination.x)**2+(source.y-destination.y)**2+(source.z-destination.z)**2)**0.5
