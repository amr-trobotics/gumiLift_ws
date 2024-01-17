from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='tc_ezio', executable='tc_ezio_node', output='screen'),
        Node(package='tc_pio', executable='tc_pio_node', output='screen'),
        Node(package='tc_pio_hybrid', executable='tc_pio_hybrid_node', output='screen'),
        Node(package='tc_communication', executable='acs_communication_node', output='screen'),
        Node(package='tc_communication', executable='gui_communication_node', output='screen'),
        Node(package='tc_scenario', executable='scenario_node', output='screen'),
        Node(package='tc_scenario', executable='data_aggregator_node', output='screen'),
        Node(package='tc_scenario', executable='report_node', output='screen'),
        Node(package='tc_scenario', executable='sound_node', output='screen'),
        Node(package='tc_scenario', executable='light_node', output='screen'),
        Node(package='tc_scenario', executable='ezio_io_node', output='screen'),
        Node(package='tc_scenario', executable='manual_node', output='screen'),
        Node(package='tc_scenario', executable='alarm_node', output='screen'),
        Node(package='tc_tdriver_manager', executable='tdriver_manager_node', output='screen'),
        Node(package='tc_camera', executable='tc_camera_node', output='screen'),
        #Node(package='tc_pad', executable='tc_pad_node', output='screen')



        
        #scenario_node
        #data_aggregator_node
        #report_node
        #sound_node
        #light_node ezio_io_node
    ])