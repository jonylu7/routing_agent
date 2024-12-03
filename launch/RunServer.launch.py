from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    #routing_service=LaunchConfiguration('server')
    loadGraph=LaunchConfiguration('graph')
    loadAndRoute=LaunchConfiguration("loadAndRoute")

    arg_waypointgraph = DeclareLaunchArgument(
        'graph', default_value='test_run/sample_data/waypointgraph.json', description='Load Waypointgraph'
    )
    arg_tasksAndVehicles = DeclareLaunchArgument(
        'loadAndRoute', default_value='test_run/sample_data/task_data.json', description='Load task and vehicles and then route'
    )

    runServer=ExecuteProcess(cmd=[['ros2 run routing_agent server']],shell=True)
    loadGraphArg=ExecuteProcess(cmd=[['ros2 run routing_agent loadGraph test_run/sample_data/waypointgraph.json']],shell=True)
    loadTaskAndVehicleArg=ExecuteProcess(cmd=[['ros2 run routing_agent routingClient test_run/sample_data/task_data.json test_run/sample_data/vehicle_data.json']],shell=True)

    
    return LaunchDescription([
        #arg_waypointgraph,
        #arg_tasksAndVehicles,
        runServer,
        loadGraphArg,
        loadTaskAndVehicleArg,

    ])