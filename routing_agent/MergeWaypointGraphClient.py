from routing_agent_interfaces.srv import NavServiceMsg               # CHANGE
import sys
import rclpy
from rclpy.node import Node
import ConvertDataFormat
import json


class MergeWaypointGraphClient(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(NavServiceMsg, 'RoutingAgent')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = NavServiceMsg.Request()
        
                                           # CHANG

    def send_request(self):
        self.req.can_go_to,self.req.task_locations_data,self.req.vehicle_start_locations_data="","",""                                     # CHANGE
        self.future = self.cli.call_async(self.req)
        

def main(args=None):
    rclpy.init(args=args)
    mergeClient = MergeWaypointGraphClient()
    mergeClient.send_request()

    while rclpy.ok():
        rclpy.spin_once(mergeClient)
        if mergeClient.future.done():
            try:
                response = mergeClient.future.result()
            except Exception as e:
                mergeClient.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                response_file_data=json.loads(response.response_file_data)
                nl=response_file_data["node_locations"]
                mergeClient.get_logger().info(
                    'Routes: '+str(nl)+"\nRoutesWithLocations"
                    )  # CHANGE
            break

    mergeClient.destroy_node()
    rclpy.shutdown()
    
                
    

