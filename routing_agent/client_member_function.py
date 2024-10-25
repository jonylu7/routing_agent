from routing_agent_interfaces.srv import RoutingAgentMsg               # CHANGE
import sys
import rclpy
from rclpy.node import Node
import ConvertDataFormat
import json


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(RoutingAgentMsg, 'RoutingAgent')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = RoutingAgentMsg.Request()
        
                                           # CHANGE

    def user_send_request(self):
        waypoint_graph_file_location=sys.argv[1]
        orders_file_location=sys.argv[2]
        vehicle_data_location=sys.argv[3]
        self.req.waypoint_graph_data,self.req.task_locations_data,self.req.vehicle_start_locations_data=ConvertDataFormat.readRequestFiles(waypoint_graph_file_location,orders_file_location,vehicle_data_location)
        self.future = self.cli.call_async(self.req)

    def nav_send_request(self):
        self.req.waypoint_graph_data,self.req.task_locations_data,self.req.vehicle_start_locations_data="","",""                                     # CHANGE
        self.future = self.cli.call_async(self.req)
        




def userRunClient(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    minimal_client.user_send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                response_file_data=json.loads(response.response_file_data)
                minimal_client.get_logger().info(
                    'Routes: '+str(response_file_data)+"\nRoutesWithLocations"
                    )  # CHANGE
            break

    rclpy.shutdown()

def navRunClient(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    minimal_client.nav_send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                response_file_data=json.loads(response.response_file_data)
                nl=response_file_data["node_locations"]
                minimal_client.get_logger().info(
                    'Routes: '+str(nl)+"\nRoutesWithLocations"
                    )  # CHANGE
            break

    minimal_client.destroy_node()
    rclpy.shutdown()
    
                
    

