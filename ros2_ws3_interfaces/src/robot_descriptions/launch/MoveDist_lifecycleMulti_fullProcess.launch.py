import time
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import TimerAction

def generate_launch_description():
    ld = LaunchDescription()
    
    lifecycle_node_name1 = 'LC_MoveDist1'
    lifecycle_node_name2 = 'LC_MoveDist2'
    lifecycle_node1 = LifecycleNode(package='lifecycle_scripts',\
                                    executable='MoveDist_lifecycleMulti',\
                                    name=lifecycle_node_name1, namespace="")
    lifecycle_node2 = LifecycleNode(package='lifecycle_scripts',\
                                    executable='MoveDist_lifecycleMulti',\
                                    name=lifecycle_node_name2, namespace="")
    manager_node = Node(package='lifecycle_scripts',\
                        executable='MoveDist_manager',\
                        parameters=[{'managed_node_names': [lifecycle_node_name1,\
                                                            lifecycle_node_name2]}])

    client_node_name1 = 'move_dist_client1'
    client_node_name2 = 'move_dist_client2'
    move_dist_client_node1 = Node(package='action_scripts',\
                                  executable='MoveDist_client',\
                                  name=client_node_name1,\
                                  output='screen',\
                                  parameters=[{'actionName': 'LC_MoveDist1_ACleg3',\
                                               'target_position': 68,\
                                               'target_speed': 1}])
    move_dist_client_node2 = Node(package='action_scripts',\
                                  executable='MoveDist_client',\
                                  name=client_node_name2,\
                                  output='screen',\
                                  parameters=[{'actionName': 'LC_MoveDist2_ACleg0',\
                                               'target_position': 12,\
                                               'target_speed': 4}])
    delayed_node = TimerAction(period=5.0,\
                               actions=[move_dist_client_node1, move_dist_client_node2])
 
    ld.add_action(lifecycle_node1)
    ld.add_action(lifecycle_node2)
    ld.add_action(manager_node)
    ld.add_action(delayed_node)
    
    return ld





