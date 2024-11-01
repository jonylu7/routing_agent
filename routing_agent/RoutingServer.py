from routing_agent_interfaces.srv import RoutingServiceMsg,MergeWaypointGraphServiceMsg,LoadWaypointGraphServiceMsg,NavServiceMsg  # CHANGE
import rclpy
from rclpy.node import Node
import routing_agent.RoutingAgent as RoutingAgent
import json
import ConvertDataFormat
from routing_agent.WaypointGraph import WaypointGraph


class RoutingServer(Node):
    isWaypointGraphLoaded=False
    
    unfinishedOrderData=json.loads("{}")
    currentVehicleData=json.loads("{}")
    unfinishedPath=json.loads("{}")

    def __init__(self):
        super().__init__('routing_service')
        self.loadWaypointGraphService=self.create_service(LoadWaypointGraphServiceMsg,"LoadWaypointGraphService",self.LoadWaypointGraphServiceCallBack)
        self.mergeWaypointGraphService=self.create_service(MergeWaypointGraphServiceMsg,"MergeWaypointGraphService",self.MergeWaypointGraphServiceCallBack)
        self.routingService = self.create_service(RoutingServiceMsg, 'RoutingService', self.RoutingServiceCallBack)
        self.navService=self.create_service(NavServiceMsg,'NavService',self.NavServiceCallBack)


    def MergeWaypointGraphServiceCallBack(self,request,response):
        mapsConfigData=json.loads(request.maps_config_data)
        response.can_merge,response.global_waypoint_graph_file_data=ConvertDataFormat.mergeWaypointGraph(mapsConfigData)
        if(response.can_merge):
            self.waypointGraphData=response.global_waypoint_graph_file_data
            self.isWaypointGraphLoaded=True
        return response
    
    def LoadWaypointGraphServiceCallBack(self,request,response):
        #load 
        self.waypointGraphData=request.waypoint_graph_data
        response.can_load=True
        self.isWaypointGraphLoaded=True
        return response
    
    def RoutingServiceCallBack(self, request, response):
        if(self.isWaypointGraphLoaded):
            orderData=json.loads(request.order_node_data)
            vehicleData=json.loads(request.vehicle_initial_data)
            response.response_data=RoutingAgent.runRoutingAgent(self.waypointGraphData,orderData,vehicleData)
            self.unfinishedOrderData=orderData
            self.currentVehicleData=vehicleData
            self.unfinishedPath=response.response_data
        else:
            #change it to log... something
            print("Please Load WaypointGraph First, use command loadGraph <waypoint_graph_file_location>")
        return response
    def NavServiceCallBack(self,request,response):
        #if(request.can_arrive):
            #response.path_to_next_task=
        #request.can_arrive true
        #request.can_go_to false 
        #1. solve TSP
        #2. reroute

        return response

        

def main(args=None):
    rclpy.init(args=args)

    routing_server = RoutingServer()

    rclpy.spin(routing_server)

    rclpy.shutdown()

if __name__ == '__main__':
    main()