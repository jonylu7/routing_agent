from simple_interfaces.srv import ORAgentMsg               # CHANGE
import sys
import rclpy
from rclpy.node import Node
import ConvertDataFormat


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(ORAgentMsg, 'ORAgent')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ORAgentMsg.Request()                                   # CHANGE

    def send_request(self):
        waypoint_graph_file_location=sys.argv[1]
        orders_file_location=sys.argv[2]
        vehicle_data_location=sys.argv[3]
        self.req.waypoint_graph_locations,self.req.waypoint_graph_edges,self.req.waypoint_graph_offsets,self.req.task_locations,self.req.vehicle_start_location=ConvertDataFormat.convertFromFileToROSServiceFormat(waypoint_graph_file_location,orders_file_location,vehicle_data_location)                                      # CHANGE
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Routes: '+response.routes_node_index+"\nRoutesWithLocations"
                    +response.routes_location
                    )  # CHANGE
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()