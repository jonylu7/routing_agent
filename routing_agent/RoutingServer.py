from routing_agent_interfaces.srv import RoutingServiceMsg,MergeWaypointGraphServiceMsg,LoadWaypointGraphServiceMsg,NavServiceMsg,GoalPathMsg  # CHANGE
import rclpy

from rclpy.node import Node
import routing_agent.RoutingAgent as RoutingAgent
import json
from ConvertDataFormat import convertJSONToStr,convertStrToJSON,saveJSONAt
from WaypointGraph import WaypointGraph,mergeWaypointGraph,loadWaypointGraphData
from RoutingEngine import RoutingEngine
from Task import loadTasksData
from Vehicle import loadVehiclesData
import sys

class RoutingServer(Node):
    isWaypointGraphLoaded=False
    waypointGraph:WaypointGraph
    routingEngine:RoutingEngine

    def __init__(self):
        super().__init__('routing_server')
        self.loadWaypointGraphService=self.create_service(LoadWaypointGraphServiceMsg,"LoadWaypointGraphService",self.LoadWaypointGraphServiceCallBack)
        self.mergeWaypointGraphService=self.create_service(MergeWaypointGraphServiceMsg,"MergeWaypointGraphService",self.MergeWaypointGraphServiceCallBack)
        self.routingService = self.create_service(RoutingServiceMsg, 'RoutingService', self.RoutingServiceCallBack)
        self.navService=self.create_service(NavServiceMsg,'NavService',self.NavServiceCallBack)

        self.cli = self.create_client(GoalPathMsg, 'goal_path')  #goal_path     # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GoalPathMsg.Request()


    def MergeWaypointGraphServiceCallBack(self,request,response):
        mapsConfigData=json.loads(request.maps_config_data)
        mapSaveLocation=mapsConfigData["merged_file_location"]
        try:
            self.waypointGraph=mergeWaypointGraph(mapsConfigData)
            response.global_waypoint_graph_file_data=convertJSONToStr(self.waypointGraph.convertToJSON())
            saveJSONAt(self.waypointGraph.convertToJSON(),mapSaveLocation)
            response.global_waypoint_graph_file_location=mapSaveLocation
            response.can_merge=True
        except:
            response.can_merge=False

        self.isWaypointGraphLoaded=response.can_merge
           
        return response
    
    def LoadWaypointGraphServiceCallBack(self,request,response):
        #load 
        print(request.waypoint_graph_data)
        try:
            self.waypointGraph=loadWaypointGraphData(convertStrToJSON(request.waypoint_graph_data))
            response.can_load=True
        except:
            response.can_load=False
        self.isWaypointGraphLoaded=response.can_load
        return response
    
    def RoutingServiceCallBack(self, request, response):
        if(self.isWaypointGraphLoaded):
            try:
                tasks=loadTasksData(self.waypointGraph,convertStrToJSON(request.task_data))
            except:
                raise KeyError("Failed to load tasks into routing Engine")
            try:
                vehicles=loadVehiclesData(self.waypointGraph,convertStrToJSON(request.vehicle_data))
            except:
                raise KeyError("Failed to load vehicles into routing Engnine")
            try:
                self.routingEngine=RoutingEngine(self.waypointGraph,tasks,vehicles)
                response.response_data=str(self.routingEngine.taskSequence)
            except:
                raise KeyError("Failed to findpath")

        else:
            #change it to log... something
            print("Please Load WaypointGraph First, use command loadGraph <waypoint_graph_file_location>")
        return response
    
    def NavServiceCallBack(self,request,response):
        if(request.can_arrive):
            self.routingEngine.update(request.i_am_at)
            response.path_to_next_task=convertJSONToStr(self.routingEngine.response())
        else:
            #set occupied
            self.routingEngine.update(request.i_am_at)
            response.path_to_next_task="Failed"


        return response

    def send_request(self):
        self.req.path_to_next_task=convertJSONToStr(self.routingEngine.response())
        self.future = self.cli.call_async(self.req)


        

def main(args=None):
    rclpy.init(args=args)

    routing_server = RoutingServer()

    rclpy.spin(routing_server)

    
    routing_server.send_request()

    while rclpy.ok():
        rclpy.spin_once(routing_server)
        if routing_server.future.done():
            try:
                response = routing_server.future.result()
            except Exception as e:
                routing_server.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                if(response.can_arrive=="T"):
                    routing_server.routingEngine.update(response.i_am_at)
                routing_server.get_logger().info(
                    'PathToNextTask: '+response.i_am_at+"\n")  # CHANGE
            break

    routing_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()