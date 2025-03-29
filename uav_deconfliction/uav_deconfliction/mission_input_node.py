#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import numpy as np
import json
from datetime import datetime

from uav_interfaces.srv import GetMissionClearance

class MissionInputNode(Node):

    def __init__(self):
        super().__init__('mission_input_node')
        
        self.mission_clearance_client = self.create_client(
            GetMissionClearance, 
            'get_mission_clearance'
        )
        
        self.get_logger().info('Waiting for mission clearance service...')
        while not self.mission_clearance_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Mission clearance service not available, waiting...')
        
        self.get_logger().info('Mission input node initialized')
        
        if len(sys.argv) > 1:
            self.process_mission_file(sys.argv[1])
        else:
            self.get_logger().info('No mission file provided. Use: ros2 run uav_deconfliction mission_input_node.py <mission_file.json>')
            self.process_sample_mission()
    
    def process_mission_file(self, file_path):
        """
        Process mission parameters from a JSON file
        """
        try:
            with open(file_path, 'r') as f:
                mission_data = json.load(f)
                
            self.get_logger().info(f'Processing mission from file: {file_path}')
            self.check_mission_clearance(mission_data)
            
        except Exception as e:
            self.get_logger().error(f'Error processing mission file: {e}')
            self.process_sample_mission()
    
    def process_sample_mission(self):
        """
        Process a sample mission for demonstration purposes
        """
        self.get_logger().info('Using sample mission data')
        # S1
        sample_mission1 = {
            'waypoints': [10.0, 10.0, 0.0, 20.0, 15.0, 0.0, 30.0, 25.0, 0.0, 40.0, 30.0, 0.0],
            'start_time': '10:00:00',
            'end_time': '10:15:00'
        }
        # S2
        sample_mission2 = {
            'waypoints': [10.0, 10.0, 0.0, 20.0, 15.0, 0.0, 30.0, 25.0, 0.0, 40.0, 30.0, 0.0],
            'start_time': '10:00:00',
            'end_time': '10:15:00'
        }
        
        self.get_logger().info('Checking Sample Mission 1 (Expected: No conflicts)')
        self.check_mission_clearance(sample_mission1)
        
        self.get_logger().info('\n\nChecking Sample Mission 2 (Expected: Conflicts)')
        self.check_mission_clearance(sample_mission2)
    
    def check_mission_clearance(self, mission_data):
        """
        Send mission data to deconfliction service and handle the response
        """
        request = GetMissionClearance.Request()
        
        request.waypoints = mission_data['waypoints']
        
        start_time = datetime.strptime(mission_data['start_time'], '%H:%M:%S')
        end_time = datetime.strptime(mission_data['end_time'], '%H:%M:%S')
        
        start_seconds = start_time.hour * 3600 + start_time.minute * 60 + start_time.second
        end_seconds = end_time.hour * 3600 + end_time.minute * 60 + end_time.second
        
        request.start_time = start_seconds
        request.end_time = end_seconds
        
        future = self.mission_clearance_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        waypoints = mission_data.get('waypoints', [])
        if not isinstance(waypoints, list) or len(waypoints) % 3 != 0:
            self.get_logger().error(f"Invalid waypoints format. Expected list with length divisible by 3, got {len(waypoints)}")
            return
            
        for i, val in enumerate(waypoints):
            if not isinstance(val, (int, float)) or not np.isfinite(val):
                self.get_logger().error(f"Corrupted waypoint value at index {i}: {val}")
                return
        try:
            request.waypoints = [float(wp) for wp in waypoints]
        except ValueError as e:
            self.get_logger().error(f"Invalid waypoint values: {e}")
            return
        

        
        if future.result() is not None:
            response = future.result()
            
            self.get_logger().info('\n' + '=' * 50)
            if response.is_clear:
                self.get_logger().info('MISSION CLEAR FOR EXECUTION')
            else:
                self.get_logger().info('MISSION CONFLICTS DETECTED')
            
            self.get_logger().info('=' * 50)
            self.get_logger().info('\nDetails:')
            self.get_logger().info(response.details_json)
            self.get_logger().info('=' * 50)
        else:
            self.get_logger().error('Failed to call mission clearance service')


def main(args=None):
    rclpy.init(args=args)
    mission_input = MissionInputNode()
    rclpy.spin(mission_input)
    mission_input.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()