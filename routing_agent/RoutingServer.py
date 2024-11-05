from routing_agent_interfaces.srv import RoutingServiceMsg,MergeWaypointGraphServiceMsg,LoadWaypointGraphServiceMsg,NavServiceMsg  # CHANGE
import rclpy
from rclpy.node import Node
import routing_agent.RoutingAgent as RoutingAgent
import json
from ConvertDataFormat import convertJSONToStr,convertStrToJSON
from WaypointGraph import WaypointGraph,mergeWaypointGraph,loadWaypointGraphData
from RoutingEngine import RoutingEngine
from Task import loadTasksData
from Vehicle import loadVehiclesData

class RoutingServer(Node):
    isWaypointGraphLoaded=False
    waypointGraph:WaypointGraph
    routingEngine:RoutingEngine

    def __init__(self):
        super().__init__('routing_service')
        self.loadWaypointGraphService=self.create_service(LoadWaypointGraphServiceMsg,"LoadWaypointGraphService",self.LoadWaypointGraphServiceCallBack)
        self.mergeWaypointGraphService=self.create_service(MergeWaypointGraphServiceMsg,"MergeWaypointGraphService",self.MergeWaypointGraphServiceCallBack)
        self.routingService = self.create_service(RoutingServiceMsg, 'RoutingService', self.RoutingServiceCallBack)
        self.navService=self.create_service(NavServiceMsg,'NavService',self.NavServiceCallBack)


    def MergeWaypointGraphServiceCallBack(self,request,response):
        mapsConfigData=json.loads(request.maps_config_data)
        try:
            self.waypointGraph=mergeWaypointGraph(mapsConfigData)
            response.global_waypoint_graph_file_data=convertJSONToStr(self.waypointGraph.convertToJSON())
            response.can_merge=True
        except:
            response.can_merge=False

        self.isWaypointGraphLoaded=response.can_merge
           
        return response
    
    def LoadWaypointGraphServiceCallBack(self,request,response):
        #load 
        try:
            self.waypointGraph=loadWaypointGraphData(request.waypoint_graph_data)
            response.can_load=True
        except:
            response.can_load=False
        self.isWaypointGraphLoaded=response.can_load
        return response
    
    def RoutingServiceCallBack(self, request, response):
        if(self.isWaypointGraphLoaded):
            try:
                tasks=loadTasksData(convertStrToJSON(request.order_node_data))
                vehicles=loadVehiclesData(convertStrToJSON(request.vehicle_data))
                self.routingEngine=RoutingEngine(self.waypointGraph,tasks,vehicles)
            except:
                print("Failed to findpath")
            response.response_data=self.routingEngine.taskSequence

        else:
            #change it to log... something
            print("Please Load WaypointGraph First, use command loadGraph <waypoint_graph_file_location>")
        return response
    def NavServiceCallBack(self,request,response):
        try:
            self.routingEngine.solve()
        except:
            print("Routing Engine faild to solve, please give another set of tasks and vehicle locations")

        if(request.can_go_to):
            self.routingEngine.update(request.I_am_at_nodeid)
            response.path_to_next_task=convertJSONToStr(self.routingEngine.response())
        else:
            #set occupied
            self.routingEngine.update(request.I_am_at_nodeid)


        return response

        

def main(args=None):
    rclpy.init(args=args)

    routing_server = RoutingServer()

    rclpy.spin(routing_server)

    rclpy.shutdown()

if __name__ == '__main__':
    main()