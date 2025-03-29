#!/usr/bin/env python3

import json
import os
import rclpy
from rclpy.node import Node
from datetime import datetime, timedelta
from uav_interfaces.msg import FlightSchedule
from uav_interfaces.srv import GetFlightSchedule

class FlightScheduleNode(Node):

    def __init__(self):
        super().__init__('flight_schedule_node')
        
        self.get_schedule_srv = self.create_service(
            GetFlightSchedule, 
            'get_flight_schedule', 
            self.get_flight_schedule_callback
        )
        
        self.schedules = self._load_flight_schedules()
        self.get_logger().info('Flight schedule node initialized with {} drone schedules'.format(len(self.schedules)))
        
    def _load_flight_schedules(self):
        """
        Load sample flight schedules from JSON file
        """
        try:
            from ament_index_python.packages import get_package_share_directory

            package_name = 'uav_deconfliction'
            package_path = get_package_share_directory(package_name)
                    
            schedules_path = os.path.join(
                package_path,
                'resource',
                'flight_schedules.json'
            )
            self.get_logger().info(f'Loading schedules from: {schedules_path}')

            with open(schedules_path, 'r') as f:
                schedules_data = json.load(f)
                
            schedules = {}
            for drone_id, schedule_data in schedules_data.items():
                waypoints = []
                for wp in schedule_data['waypoints']:
                    waypoints.append((wp['x'], wp['y'], wp.get('z', 0.0)))
                
                start_time = datetime.strptime(schedule_data['start_time'], '%H:%M:%S')
                end_time = datetime.strptime(schedule_data['end_time'], '%H:%M:%S')
                
                schedules[drone_id] = {
                    'waypoints': waypoints,
                    'start_time': start_time,
                    'end_time': end_time
                }
                
            return schedules
            
        except Exception as e:
            self.get_logger().error(f'Error loading flight schedules: {e}')
            return self._generate_sample_schedules()
    
    def _generate_sample_schedules(self):
        """
        Generate sample flight schedules for testing
        """
        base_time = datetime.now().replace(hour=10, minute=0, second=0, microsecond=0)
        
        return {
            'Drone_A': {
                'waypoints': [(50.0, 50.0, 0.0), (60.0, 60.0, 0.0)],
                'start_time': base_time + timedelta(minutes=20),
                'end_time': base_time + timedelta(minutes=30)
            },
            'Drone_B': {
                'waypoints': [(70.0, 20.0, 0.0), (80.0, 25.0, 0.0)],
                'start_time': base_time - timedelta(minutes=30),
                'end_time': base_time - timedelta(minutes=15)
            },
            'Drone_X': {
                'waypoints': [(15.0, 12.0, 0.0), (25.0, 18.0, 0.0)],
                'start_time': base_time + timedelta(minutes=5),
                'end_time': base_time + timedelta(minutes=10)
            },
            'Drone_Y': {
                'waypoints': [(35.0, 28.0, 0.0), (45.0, 33.0, 0.0)],
                'start_time': base_time + timedelta(minutes=10),
                'end_time': base_time + timedelta(minutes=20)
            }
        }
    
    def get_flight_schedule_callback(self, request, response):
        """
        Service callback to retrieve flight schedules
        """
        schedule_msg = FlightSchedule()
        all_schedules = []
        
        for drone_id, schedule in self.schedules.items():
            flight_schedule = FlightSchedule()
            flight_schedule.drone_id = drone_id
            
            waypoints_flat = []
            for wp in schedule['waypoints']:
                waypoints_flat.extend(wp)
            flight_schedule.waypoints = waypoints_flat
            
            start_seconds = schedule['start_time'].hour * 3600 + schedule['start_time'].minute * 60 + schedule['start_time'].second
            end_seconds = schedule['end_time'].hour * 3600 + schedule['end_time'].minute * 60 + schedule['end_time'].second
            
            flight_schedule.start_time = start_seconds
            flight_schedule.end_time = end_seconds
            
            all_schedules.append(flight_schedule)
        
        response.schedules = all_schedules
        return response


def main(args=None):
    rclpy.init(args=args)
    flight_schedule_node = FlightScheduleNode()
    rclpy.spin(flight_schedule_node)
    flight_schedule_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()