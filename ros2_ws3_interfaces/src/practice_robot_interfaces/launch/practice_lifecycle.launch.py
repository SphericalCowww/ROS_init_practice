from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode

def generate_launch_description():
    node_name = "Number_publisher"

    launchDesc = LaunchDescription()
    Number_publisherNode = LifecycleNode(
        package="lifecycle_scripts",
        executable=node_name,
        name="Number_publisher",
        namespace="",
    )

    Number_clientNode = Node(
        package="lifecycle_scripts",
        executable="Number_client",
        parameters = [
            {"managed_node_name": node_name}
        ]
    )

    launchDesc.add_action(Number_publisherNode)
    launchDesc.add_action(Number_clientNode)
    return launchDesc





