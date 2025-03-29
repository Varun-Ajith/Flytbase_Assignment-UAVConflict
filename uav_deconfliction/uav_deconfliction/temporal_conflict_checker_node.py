#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from datetime import datetime, timedelta
from uav_interfaces.srv import CheckTemporalConflict

class TemporalConflictCheckerNode(Node):
    
    def __init__(self):
        super().__init__('temporal_conflict_checker_node')
        
        self.temporal_check_srv = self.create_service(
            CheckTemporalConflict,
            'check_temporal_conflict',
            self.check_temporal_conflict_callback
        )
        
        self.get_logger().info('Temporal conflict checker node initialized')
        
    def check_temporal_conflict_callback(self, request, response):
        """
        Service callback to check for temporal conflicts between missions
        """
        
        primary_start_time = request.primary_start_time
        primary_end_time = request.primary_end_time
        
        conflicts = []
        for schedule in request.other_schedules:
            if self._has_temporal_overlap(
                primary_start_time, primary_end_time,
                schedule.start_time, schedule.end_time
            ):
                conflicts.append({
                    'drone_id': schedule.drone_id,
                    'start_overlap': max(primary_start_time, schedule.start_time),
                    'end_overlap': min(primary_end_time, schedule.end_time)
                })
        
        response.has_conflicts = len(conflicts) > 0
        
        if response.has_conflicts:
            response.conflict_drone_ids = []
            response.conflict_start_times = []
            response.conflict_end_times = []
            
            for conflict in conflicts:
                response.conflict_drone_ids.append(conflict['drone_id'])
                response.conflict_start_times.append(conflict['start_overlap'])
                response.conflict_end_times.append(conflict['end_overlap'])
        
        return response
    
    def _has_temporal_overlap(self, start1, end1, start2, end2):
        """
        Check if two time windows overlap
        """
        return max(start1, start2) < min(end1, end2)
    
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
    temporal_checker_node = TemporalConflictCheckerNode()
    rclpy.spin(temporal_checker_node)
    temporal_checker_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()