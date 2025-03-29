from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    ld = LaunchDescription()
    
    # Flight schedule node - provides access to drone schedules
    flight_schedule_node = Node(
        package='uav_deconfliction',
        executable='flight_schedule_node',
        name='flight_schedule_node',
        output='screen'
    )
    
    # Spatial conflict checker node
    spatial_conflict_checker_node = Node(
        package='uav_deconfliction',
        executable='spatial_conflict_checker_node',
        name='spatial_conflict_checker_node',
        output='screen'
    )
    
    # Temporal conflict checker node
    temporal_conflict_checker_node = Node(
        package='uav_deconfliction',
        executable='temporal_conflict_checker_node',
        name='temporal_conflict_checker_node',
        output='screen'
    )
    
    # Deconfliction service node - central coordinator
    deconfliction_service_node = Node(
        package='uav_deconfliction',
        executable='deconfliction_service_node',
        name='deconfliction_service_node',
        output='screen'
    )
    
    # Add all nodes to the launch description
    ld.add_action(flight_schedule_node)
    ld.add_action(spatial_conflict_checker_node)
    ld.add_action(temporal_conflict_checker_node)
    ld.add_action(deconfliction_service_node)
    
    return ld