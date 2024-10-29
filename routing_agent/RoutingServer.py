from routing_agent_interfaces.srv import RoutingServiceMsg,MergeWaypointGraphServiceMsg,LoadWaypointGraphServiceMsg,NavServiceMsg  # CHANGE
import rclpy
from rclpy.node import Node
import routing_agent.RoutingAgent as RoutingAgent


class RoutingServer(Node):
    waypointGraphData=""

    def __init__(self):
        super().__init__('routing_service')
        self.routingService = self.create_service(RoutingServiceMsg, 'RoutingService', self.RoutingServiceMsg)
        #self.ORService=self.create_service()
        #self.updateWaypointGraphService=self.create_service()
        self.navService=self.create_service()
        self.loadWaypointGraphService=self.create_service()  
        self.routingService=self.create_service()     # CHANGE

    def RoutingServiceCallBack(self, request, response):
        return response


    def MergeWaypointGraphCallBack(self,request,response):
        #send maps.config files
        #success or not and stores at ? location
        return response
    def LoadWaypointGraphCallBack(self,request,response):
        #load 
        return 

    def NavCallBack(self,request,response):
        #send "sucess" current map id local locations/ 
        # "failed" current map id local locations 
        # set occupied map and reroute again
        # path to next task locations
        return response

        
    

    

def main(args=None):
    rclpy.init(args=args)

    routing_server = RoutingServer()

    rclpy.spin(routing_server)

    rclpy.shutdown()

if __name__ == '__main__':
    main()