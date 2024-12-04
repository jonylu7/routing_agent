from routing_agent_interfaces.srv import GoalPathMsg # CHANGE
import rclpy
from rclpy.node import Node
import routing_agent.RoutingAgent as RoutingAgent

class DummyService(Node):

    def __init__(self):
        super().__init__('routing_server')
        self.DummyService=self.create_service(GoalPathMsg,"dummy",self.DummyServiceCallBack)

    
    def DummyServiceCallBack(self,request,response):
        path=request.path_to_next_task
        #do some process
        response.can_arrive=True
        response.i_am_at="000_000"

        return response

        

def main(args=None):
    rclpy.init(args=args)

    dummy_service = DummyService()

    rclpy.spin(dummy_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()