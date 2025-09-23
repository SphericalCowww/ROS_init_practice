from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    ld = LaunchDescription()
    
    node_name = 'Number_publisher'
    lifecycle_node = LifecycleNode(package='lifecycle_scripts',\
                                   executable='Number_publisher',\
                                   name=node_name, namespace="")
    manager_node = Node(package='lifecycle_scripts',\
                        executable='Number_manager',\
                        parameters=[{'managed_node_name': node_name}])
    ld.add_action(lifecycle_node)
    ld.add_action(manager_node)

    return ld





