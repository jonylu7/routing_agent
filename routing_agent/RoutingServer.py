from routing_agent_interfaces.srv import RoutingServiceMsg,MergeWaypointGraphServiceMsg,LoadWaypointGraphServiceMsg,NavServiceMsg  # CHANGE
import rclpy
from rclpy.node import Node
import routing_agent.RoutingAgent as RoutingAgent
import json
import ConvertDataFormat


class RoutingServer(Node):
    waypointGraphData=""
    hasLoadedWaypointGraph=False

    def __init__(self):
        super().__init__('routing_service')
        self.loadWaypointGraphService=self.create_service(LoadWaypointGraphServiceMsg,"LoadWaypointGraphService",self.LoadWaypointGraphServiceCallBack)
        self.mergeWaypointGraphService=self.create_service(MergeWaypointGraphServiceMsg,"MergeWaypointGraphService",self.MergeWaypointGraphServiceCallBack)
        self.routingService = self.create_service(RoutingServiceMsg, 'RoutingService', self.RoutingServiceCallBack)
        self.navService=self.create_service(NavServiceMsg,'NavService',self.NavServiceCallBack)


    def MergeWaypointGraphServiceCallBack(self,request,response):
        mapsConfigData=json.loads(request.maps_config_data)
        response.can_merge,response.global_waypoint_graph_file_data=ConvertDataFormat.mergeWaypointGraph(mapsConfigData)
        #stores at 
        #self.response.global_waypoint_graph_file_location
        return response
    
    def LoadWaypointGraphServiceCallBack(self,request,response):
        #load 
        
        return response
    
    def RoutingServiceCallBack(self, request, response):

        return response
    def NavServiceCallBack(self,request,response):
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