#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node
from datetime import datetime, timedelta
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from uav_interfaces.srv import (
    GetFlightSchedule, 
    CheckSpatialConflict, 
    CheckTemporalConflict,
    ValidateMission, 
    GetMissionClearance
)
from uav_interfaces.msg import FlightSchedule, ConflictStatus

class DeconflictionServiceNode(Node):
    
    def __init__(self):
        super().__init__('deconfliction_service_node')
        
        self.callback_group = ReentrantCallbackGroup()
        
        self.get_flight_schedules_client = self.create_client(
            GetFlightSchedule, 
            'get_flight_schedule', 
            callback_group=self.callback_group
        )
        
        self.check_spatial_conflict_client = self.create_client(
            CheckSpatialConflict, 
            'check_spatial_conflict', 
            callback_group=self.callback_group
        )
        
        self.check_temporal_conflict_client = self.create_client(
            CheckTemporalConflict, 
            'check_temporal_conflict', 
            callback_group=self.callback_group
        )
        
        self.validate_mission_srv = self.create_service(
            ValidateMission,
            'validate_mission',
            self.validate_mission_callback,
            callback_group=self.callback_group
        )
        
        self.get_mission_clearance_srv = self.create_service(
            GetMissionClearance,
            'get_mission_clearance',
            self.get_mission_clearance_callback,
            callback_group=self.callback_group
        )
        
        self._wait_for_services()
        
        self.get_logger().info('Deconfliction service node initialized')
    
    def _wait_for_services(self):
        """
        Wait for all required services to be available
        """
        self.get_logger().info('Waiting for flight schedule service...')
        while not self.get_flight_schedules_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Flight schedule service not available, waiting...')
            
        self.get_logger().info('Waiting for spatial conflict checker service...')
        while not self.check_spatial_conflict_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spatial conflict checker service not available, waiting...')
            
        self.get_logger().info('Waiting for temporal conflict checker service...')
        while not self.check_temporal_conflict_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Temporal conflict checker service not available, waiting...')
            
        self.get_logger().info('All required services are available')
    
    def validate_mission_callback(self, request, response):
        """
        Service callback to check if a mission has conflicts
        """
        
        flight_schedules = self._get_flight_schedules()
        
        if flight_schedules is None:
            response.status.has_conflicts = True
            response.status.message = "Failed to retrieve flight schedules"
            return response
        
        spatial_conflicts = self._check_spatial_conflicts(
            request.waypoints, flight_schedules
        )
        
        temporal_conflicts = self._check_temporal_conflicts(
            request.start_time, request.end_time, flight_schedules
        )
        
        has_conflicts = spatial_conflicts['has_conflicts'] or temporal_conflicts['has_conflicts']
        
        response.status.has_conflicts = has_conflicts
        
        if has_conflicts:
            if spatial_conflicts['has_conflicts'] and temporal_conflicts['has_conflicts']:
                response.status.message = "Both spatial and temporal conflicts detected"
            elif spatial_conflicts['has_conflicts']:
                response.status.message = "Spatial conflicts detected"
            else:
                response.status.message = "Temporal conflicts detected"
                
            response.status.conflict_details = self._generate_conflict_details(
                spatial_conflicts, temporal_conflicts
            )
        else:
            response.status.message = "No conflicts detected"
            
        return response
    
    def get_mission_clearance_callback(self, request, response):
        """
        Service callback for getting mission clearance with conflict details
        """
        
        flight_schedules = self._get_flight_schedules()
        
        if flight_schedules is None:
            response.is_clear = False
            response.message = "Failed to retrieve flight schedules"
            response.details_json = json.dumps({
                "status": "error",
                "message": "Failed to retrieve flight schedules",
                "details": {}
            })
            return response
        
        spatial_conflicts = self._check_spatial_conflicts(
            request.waypoints, flight_schedules
        )
        
        temporal_conflicts = self._check_temporal_conflicts(
            request.start_time, request.end_time, flight_schedules
        )
        
        has_conflicts = spatial_conflicts['has_conflicts'] or temporal_conflicts['has_conflicts']
        
        if has_conflicts:
            conflict_list = []
            
            if spatial_conflicts['has_conflicts']:
                for i in range(len(spatial_conflicts['conflict_drone_ids'])):
                    drone_id = spatial_conflicts['conflict_drone_ids'][i]
                    loc_x = spatial_conflicts['conflict_locations_x'][i]
                    loc_y = spatial_conflicts['conflict_locations_y'][i]
                    
                    time_range = "Unknown"
                    if temporal_conflicts['has_conflicts']:
                        for j in range(len(temporal_conflicts['conflict_drone_ids'])):
                            if temporal_conflicts['conflict_drone_ids'][j] == drone_id:
                                start_seconds = temporal_conflicts['conflict_start_times'][j]
                                end_seconds = temporal_conflicts['conflict_end_times'][j]
                                start_str = self._seconds_to_time_str(start_seconds)
                                end_str = self._seconds_to_time_str(end_seconds)
                                time_range = f"{start_str} - {end_str}"
                                break
                    
                    conflict_list.append({
                        "location": f"({loc_x:.1f}, {loc_y:.1f})",
                        "time": time_range,
                        "simulated_drone": drone_id,
                        "explanation": f"Primary mission trajectory comes within the safety buffer of {drone_id}."
                    })
            
            if temporal_conflicts['has_conflicts']:
                for i in range(len(temporal_conflicts['conflict_drone_ids'])):
                    drone_id = temporal_conflicts['conflict_drone_ids'][i]
                    
                    if drone_id in spatial_conflicts.get('conflict_drone_ids', []):
                        continue
                    
                    start_seconds = temporal_conflicts['conflict_start_times'][i]
                    end_seconds = temporal_conflicts['conflict_end_times'][i]
                    start_str = self._seconds_to_time_str(start_seconds)
                    end_str = self._seconds_to_time_str(end_seconds)
                    
                    conflict_list.append({
                        "location": "Undetermined",
                        "time": f"{start_str} - {end_str}",
                        "simulated_drone": drone_id,
                        "explanation": f"Mission time window overlaps with {drone_id}'s schedule."
                    })
            
            details = {
                "status": "conflict detected",
                "message": "Conflicts detected at multiple points.",
                "details": {
                    "conflicts": conflict_list
                }
            }
            
            response.is_clear = False
            response.message = "Conflicts detected"
            response.details_json = json.dumps(details, indent=2)
            
        else:
            details = {
                "status": "clear",
                "message": "No spatial or temporal conflicts detected.",
                "details": {}
            }
            
            response.is_clear = True
            response.message = "No conflicts detected"
            response.details_json = json.dumps(details, indent=2)
        
        return response
    
    def _get_flight_schedules(self):
        """
        Get all flight schedules from the flight schedule service
        """
        request = GetFlightSchedule.Request()
        
        future = self.get_flight_schedules_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            return future.result().schedules
        else:
            self.get_logger().error('Failed to call flight schedule service')
            return None
    
    def _check_spatial_conflicts(self, waypoints, flight_schedules):
        """
        Check for spatial conflicts using the spatial conflict checker service
        """
        request = CheckSpatialConflict.Request()
        request.primary_waypoints = waypoints
        request.other_schedules = flight_schedules
        
        future = self.check_spatial_conflict_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            result = future.result()
            
            return {
                'has_conflicts': result.has_conflicts,
                'conflict_drone_ids': result.conflict_drone_ids if result.has_conflicts else [],
                'conflict_locations_x': result.conflict_locations_x if result.has_conflicts else [],
                'conflict_locations_y': result.conflict_locations_y if result.has_conflicts else [],
                'conflict_distances': result.conflict_distances if result.has_conflicts else []
            }
        else:
            self.get_logger().error('Failed to call spatial conflict checker service')
            return {'has_conflicts': True}  # Assume conflict if service call fails
    
    def _check_temporal_conflicts(self, start_time, end_time, flight_schedules):
        """
        Check for temporal conflicts using the temporal conflict checker service
        """
        request = CheckTemporalConflict.Request()
        request.primary_start_time = start_time
        request.primary_end_time = end_time
        request.other_schedules = flight_schedules
        
        future = self.check_temporal_conflict_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            result = future.result()
            
            return {
                'has_conflicts': result.has_conflicts,
                'conflict_drone_ids': result.conflict_drone_ids if result.has_conflicts else [],
                'conflict_start_times': result.conflict_start_times if result.has_conflicts else [],
                'conflict_end_times': result.conflict_end_times if result.has_conflicts else []
            }
        else:
            self.get_logger().error('Failed to call temporal conflict checker service')
            return {'has_conflicts': True}  
    
    def _generate_conflict_details(self, spatial_conflicts, temporal_conflicts):
        """
        Generate detailed conflict information for the response
        """
        details = []
        
        if spatial_conflicts['has_conflicts']:
            for i in range(len(spatial_conflicts['conflict_drone_ids'])):
                detail = (
                    f"Spatial conflict with {spatial_conflicts['conflict_drone_ids'][i]} "
                    f"at location ({spatial_conflicts['conflict_locations_x'][i]:.2f}, "
                    f"{spatial_conflicts['conflict_locations_y'][i]:.2f})"
                )
                details.append(detail)
        
        if temporal_conflicts['has_conflicts']:
            for i in range(len(temporal_conflicts['conflict_drone_ids'])):
                start_time = self._seconds_to_time_str(temporal_conflicts['conflict_start_times'][i])
                end_time = self._seconds_to_time_str(temporal_conflicts['conflict_end_times'][i])
                
                detail = (
                    f"Temporal conflict with {temporal_conflicts['conflict_drone_ids'][i]} "
                    f"during time window {start_time} - {end_time}"
                )
                details.append(detail)
        
        return '; '.join(details)
    
    def _seconds_to_time_str(self, seconds):
        """
        Convert seconds since midnight to time string
        """
        hours = seconds // 3600
        minutes = (seconds % 3600) // 60
        secs = seconds % 60
        return f"{hours:02d}:{minutes:02d}:{secs:02d}"


def main(args=None):
    rclpy.init(args=args)
    
    executor = MultiThreadedExecutor()
    deconfliction_service = DeconflictionServiceNode()
    
    executor.add_node(deconfliction_service)
    executor.spin()
    
    deconfliction_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()