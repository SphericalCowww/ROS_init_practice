from launch import LaunchDescription
from laucn_ros.actions import Node LifecycleNode

def generate_launch_description():
    node_name = "Number_publisher"

    launchDesc = LaunchDescription()
    Number_publisherNode = LifecycleNode(
        package="lifecycle_scripts",
        executable=node_name,
        name="Number_publisher",
        namespace="",
    )

    lifecycleNode_manager = Node(
        package="lifecycle_script",
        executable="lifecycleNode_manager",
        parameters = [
            {"lifecycleNode_manager": node_name}
        ]
    )

    launchDesc.add_acction(Number_publisherNode)
    launchDesc.add_acction(lifecycleNode_manager)
    return ld





