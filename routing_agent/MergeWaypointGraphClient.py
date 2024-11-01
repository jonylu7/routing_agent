from routing_agent_interfaces.srv import NavServiceMsg               # CHANGE
import sys
import rclpy
from rclpy.node import Node
import ConvertDataFormat
import json
from pathlib import Path


class MergeWaypointGraphClient(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(NavServiceMsg, 'RoutingAgent')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = NavServiceMsg.Request()
        
                                           # CHANG

    def send_request(self):
        self.req.maps_config_data=self.loadMapsConfig(sys.argv[1])
        self.future = self.cli.call_async(self.req)

    def loadMapsConfig(self,filepath):
        filepath = Path(filepath)
        configData=ConvertDataFormat.loadJSONFile(filepath)
        for mapid in configData.keys():
            mapPath=filepath.parent+configData[mapid]["file_path"]
            mapData=ConvertDataFormat.loadJSONFile(mapPath)
            configData[mapid]["file_data"]=mapData
        return configData



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
                if(response.can_merge):
                    mergeClient.get_logger().info(
                    'Merge WaypointGraph Successfully')
                else:
                    mergeClient.get_logger().info(
                    'Merge WaypointGraph Failed')
            break

    mergeClient.destroy_node()
    rclpy.shutdown()
    
                
    

