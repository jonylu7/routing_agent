from routing_agent_interfaces.srv import NavServiceMsg               # CHANGE
import sys
import rclpy
from rclpy.node import Node
import ConvertDataFormat
import json


class NavClient(Node):

    def __init__(self):
        super().__init__('NavClient')
        self.cli = self.create_client(NavServiceMsg, 'NavService')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = NavServiceMsg.Request()
        
                                           # CHANG
    def send_request(self):
        self.req.can_arrive=sys.argv[1]
        self.req.i_am_at=sys.argv[2]
        self.future = self.cli.call_async(self.req)
        

def main(args=None):
    rclpy.init(args=args)
    navClient = NavClient()
    navClient.send_request()

    while rclpy.ok():
        rclpy.spin_once(navClient)
        if navClient.future.done():
            try:
                response = navClient.future.result()
            except Exception as e:
                navClient.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                navClient.get_logger().info(
                    'PathToNextTask: '+response.path_to_next_task+"\n")  # CHANGE
            break

    navClient.destroy_node()
    rclpy.shutdown()
    
                
    

