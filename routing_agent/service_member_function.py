from routing_agent_interfaces.srv import RoutingAgentMsg  # CHANGE
import rclpy
from rclpy.node import Node
import routing_agent.RoutingAgent as RoutingAgent


class MinimalService(Node):
    storedResponse=""

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(RoutingAgentMsg, 'RoutingAgent', self.RoutingAgentMsgCallback)       # CHANGE

    def RoutingAgentMsgCallback(self, request, response):
        if not (request.waypoint_graph_data=="" and request.task_locations_data=="" and request.vehicle_start_locations_data==""):
            response.response_file_data=RoutingAgent.runRoutingAgent(request.waypoint_graph_data,request.task_locations_data,request.vehicle_start_locations_data)
            self.storedResponse=response
            return response
        else:
            #response.response_file_data=
            return self.storedResponse
                                # CHANGE
        #self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c))  # CHANGE

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()