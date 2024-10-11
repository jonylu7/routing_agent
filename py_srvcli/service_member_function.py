from simple_interfaces.srv import ORAgentMsg  # CHANGE
import rclpy
from rclpy.node import Node
import ORAgentClient


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(ORAgentMsg, 'ORAgent', self.ORAgentMsgCallback)       # CHANGE

    def ORAgentMsgCallback(self, request, response):
        response.routes_node_index,response.routes_location=ORAgentClient.runORAgentByROS(request.waypoint_graph_locations,request.waypoint_graph_edges,request.waypoint_graph_offsets,request.task_locations,request.vehicle_start_location)                                 # CHANGE
        #self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c))  # CHANGE
        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()