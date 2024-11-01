from routing_agent_interfaces.srv import RoutingServiceMsg               # CHANGE
import sys
import rclpy
from rclpy.node import Node
import ConvertDataFormat
import json


class RoutingClient(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(RoutingServiceMsg, 'RoutingAgent')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = RoutingServiceMsg.Request()
        

    def send_request(self):
        orders_file_location=sys.argv[1]
        vehicle_data_location=sys.argv[2]
        self.req.task_locations_data,self.req.vehicle_start_locations_data=self.readFiles(orders_file_location,vehicle_data_location)
        self.future = self.cli.call_async(self.req)

    def readFiles(self,orders_file_location,vehicle_data_location):
        orderData=ConvertDataFormat.loadJSONIntoStr(orders_file_location)
        vehicleData=ConvertDataFormat.loadJSONIntoStr(vehicle_data_location)
        return orderData,vehicleData


def main(args=None):
    rclpy.init(args=args)
    routingClient = RoutingClient()
    routingClient.send_request()

    while rclpy.ok():
        rclpy.spin_once(routingClient)
        if routingClient.future.done():
            try:
                response = routingClient.future.result()
            except Exception as e:
                routingClient.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                response_file_data=json.loads(response.response_file_data)
                routingClient.get_logger().info(
                    'Routes: '+str(response_file_data)+"\nRoutesWithLocations"
                    )  # CHANGE
            break

    rclpy.shutdown()

                
    

