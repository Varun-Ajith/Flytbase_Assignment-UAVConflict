#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from shapely.geometry import LineString, Point
from shapely.errors import TopologicalError
from uav_interfaces.srv import CheckSpatialConflict

class SpatialConflictCheckerNode(Node):

    def __init__(self):
        super().__init__('spatial_conflict_checker_node')
        
        self.safety_buffer = 5.0  # meters
        
        self.spatial_check_srv = self.create_service(
            CheckSpatialConflict,
            'check_spatial_conflict',
            self.check_spatial_conflict_callback
        )
        
        self.get_logger().info('Spatial conflict checker node initialized')
    
    def flatten_waypoints(self, waypoints):
        """
        Flattens the waypoints list. 
        """
        if len(waypoints) > 0 and isinstance(waypoints[0], dict):
            flat_list = []
            for wp in waypoints:
                try:
                    flat_list.extend([float(wp["x"]), float(wp["y"]), float(wp["z"])])
                except (KeyError, ValueError) as e:
                    self.get_logger().error(f"Error flattening waypoint {wp}: {e}")
            return flat_list
        else:
            return list(waypoints)
    
    def ensure_three_coordinates(self, flat_list):
        """
        If the flat list length is a multiple of 2 ,convert it to a list with z=0.0 added for each waypoint.
        """
        if len(flat_list) % 3 == 0:
            return flat_list
        elif len(flat_list) % 2 == 0:
            new_list = []
            for i in range(0, len(flat_list), 2):
                new_list.extend([flat_list[i], flat_list[i+1], 0.0])
            return new_list
        else:
            return flat_list  # Return as is; it will be caught by validation later
    
    def check_spatial_conflict_callback(self, request, response):
        """
        Service callback to check for spatial conflicts between missions
        """
        
        response.has_conflicts = False
        response.conflict_locations_x = []
        response.conflict_locations_y = []
        response.conflict_drone_ids = []
        response.conflict_distances = []
        
        try:
            primary_waypoints_flat = self.flatten_waypoints(request.primary_waypoints)
            primary_waypoints_flat = self.ensure_three_coordinates(primary_waypoints_flat)
            
            if len(primary_waypoints_flat) % 3 != 0:
                self.get_logger().error(
                    f"Invalid waypoints format. Got {len(primary_waypoints_flat)} values, "
                    f"expected multiple of 3. First 10 values: {primary_waypoints_flat[:10]}"
                )
                response.has_conflicts = True  
                return response

            primary_waypoints = []
            for i in range(0, len(primary_waypoints_flat), 3):
                try:
                    x = primary_waypoints_flat[i]
                    y = primary_waypoints_flat[i+1]
                    z = primary_waypoints_flat[i+2]
                    primary_waypoints.append((x, y, z))
                except (IndexError, ValueError) as e:
                    self.get_logger().error(f"Invalid waypoint data at index {i}: {e}")
                    return response

            primary_segments = []
            for i in range(len(primary_waypoints) - 1):
                p1 = primary_waypoints[i]
                p2 = primary_waypoints[i+1]
                
                if np.allclose([p1[0], p2[0]], atol=1e-6) and np.allclose([p1[1], p2[1]], atol=1e-6):
                    continue
                
                try:
                    primary_segments.append(LineString([(p1[0], p1[1]), (p2[0], p2[1])]))
                except Exception as e:
                    self.get_logger().warn(f"Failed to create primary segment {i}: {e}")
                    continue

            for schedule in request.other_schedules:
                schedule_waypoints_flat = self.flatten_waypoints(schedule.waypoints)
                schedule_waypoints_flat = self.ensure_three_coordinates(schedule_waypoints_flat)
                
                drone_waypoints = []
                if len(schedule_waypoints_flat) % 3 != 0:
                    self.get_logger().warn(f"Invalid waypoints for drone {schedule.drone_id}")
                    continue

                for i in range(0, len(schedule_waypoints_flat), 3):
                    try:
                        x = schedule_waypoints_flat[i]
                        y = schedule_waypoints_flat[i+1]
                        z = schedule_waypoints_flat[i+2]
                        drone_waypoints.append((x, y, z))
                    except (IndexError, ValueError) as e:
                        self.get_logger().warn(f"Invalid waypoint for drone {schedule.drone_id} at index {i}: {e}")
                        continue

                drone_segments = []
                for i in range(len(drone_waypoints) - 1):
                    p1 = drone_waypoints[i]
                    p2 = drone_waypoints[i+1]
                    
                    if np.allclose([p1[0], p2[0]], atol=1e-6) and np.allclose([p1[1], p2[1]], atol=1e-6):
                        continue
                        
                    try:
                        drone_segments.append(LineString([(p1[0], p1[1]), (p2[0], p2[1])]))
                    except Exception as e:
                        self.get_logger().warn(f"Failed to create drone segment {i} for {schedule.drone_id}: {e}")
                        continue

                for primary_seg in primary_segments:
                    for drone_seg in drone_segments:
                        try:
                            if primary_seg.intersects(drone_seg):
                                try:
                                    conflict_point = primary_seg.intersection(drone_seg)
                                    if not conflict_point.is_empty:
                                        response.has_conflicts = True
                                        response.conflict_locations_x.append(float(conflict_point.x))
                                        response.conflict_locations_y.append(float(conflict_point.y))
                                        response.conflict_drone_ids.append(schedule.drone_id)
                                        response.conflict_distances.append(0.0)
                                except TopologicalError:
                                    continue

                            distance = primary_seg.distance(drone_seg)
                            if not np.isfinite(distance):
                                continue
                                
                            if distance < (self.safety_buffer - 1e-6):  # Account for floating point precision
                                response.has_conflicts = True
                                p1, p2 = self._closest_points(primary_seg, drone_seg)
                                conflict_x = (p1[0] + p2[0]) / 2.0
                                conflict_y = (p1[1] + p2[1]) / 2.0
                                response.conflict_locations_x.append(float(conflict_x))
                                response.conflict_locations_y.append(float(conflict_y))
                                response.conflict_drone_ids.append(schedule.drone_id)
                                response.conflict_distances.append(float(distance))
                        except Exception as e:
                            self.get_logger().warn(f"Conflict check failed between segments: {e}")
                            continue

        except Exception as e:
            self.get_logger().error(f"Critical error in spatial conflict check: {e}")
            response.has_conflicts = True  # Fail-safe: assume conflict if error occurs

        return response
    
    def _closest_points(self, line1, line2):
        """
        Find closest points between two line segments
        """
        coords1 = np.array(list(line1.coords))
        coords2 = np.array(list(line2.coords))
        
        v1 = coords1[1] - coords1[0]
        v2 = coords2[1] - coords2[0]
        
        w0 = coords1[0] - coords2[0]
        
        a = np.dot(v1, v1)
        b = np.dot(v1, v2)
        c = np.dot(v2, v2)
        d = np.dot(v1, w0)
        e = np.dot(v2, w0)
        
        denom = a * c - b * b
        
        if denom == 0:  
            t0 = 0
            s0 = e / c if c != 0 else 0
        else:
            t0 = (b * e - c * d) / denom
            s0 = (a * e - b * d) / denom
            
        t0 = max(0, min(1, t0))
        s0 = max(0, min(1, s0))
        
        p1 = coords1[0] + t0 * v1
        p2 = coords2[0] + s0 * v2
        
        return p1, p2

def main(args=None):
    rclpy.init(args=args)
    spatial_checker_node = SpatialConflictCheckerNode()
    rclpy.spin(spatial_checker_node)
    spatial_checker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
