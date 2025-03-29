#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
import json
import os
from datetime import datetime, timedelta

from uav_interfaces.srv import GetFlightSchedule
from uav_interfaces.msg import FlightSchedule

class VisualizationNode(Node):

    def __init__(self):
        super().__init__('visualization_node')
        
        self.get_flight_schedules_client = self.create_client(
            GetFlightSchedule, 
            'get_flight_schedule'
        )
        
        self.get_logger().info('Waiting for flight schedule service...')
        while not self.get_flight_schedules_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Flight schedule service not available, waiting...')
        
        self.safety_buffer = 5.0  # meters
        
        self.flight_schedules = self._get_flight_schedules()
        
        self.output_dir = os.path.join(os.path.expanduser('~'), 'uav_deconfliction_viz')
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.get_logger().info('Visualization node initialized')
        self.get_logger().info(f'Visualizations will be saved to: {self.output_dir}')
        
        self.generate_sample_visualizations()
    
    def _get_flight_schedules(self):
        """Get all flight schedules from the flight schedule service"""
        request = GetFlightSchedule.Request()
        
        future = self.get_flight_schedules_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            return future.result().schedules
        else:
            self.get_logger().error('Failed to call flight schedule service')
            return []
    
    def generate_sample_visualizations(self):
        """
        Generate sample visualizations for two scenarios
        """
        
        primary_mission1 = {
            'waypoints': [(10.0, 10.0, 0.0), (20.0, 15.0, 0.0), (30.0, 25.0, 0.0), (40.0, 30.0, 0.0)],
            'start_time': 36000,  # 10:00:00
            'end_time': 36900     # 10:15:00
        }
        
        primary_mission2 = {
            'waypoints': [(10.0, 10.0, 0.0), (20.0, 15.0, 0.0), (30.0, 25.0, 0.0), (40.0, 30.0, 0.0)],
            'start_time': 36000,  # 10:00:00
            'end_time': 36900     # 10:15:00
        }
        
        conflicts = [
            {
                'location': (20.0, 15.0),
                'time': "10:05:00 - 10:10:00",
                'simulated_drone': "Drone_X"
            },
            {
                'location': (30.0, 25.0),
                'time': "10:10:00 - 10:15:00",
                'simulated_drone': "Drone_Y"
            }
        ]
        
        self.generate_static_plot(primary_mission1, "conflict_free", [])
        self.generate_static_plot(primary_mission2, "with_conflicts", conflicts)
        
        self.generate_animation(primary_mission1, "conflict_free", [])
        self.generate_animation(primary_mission2, "with_conflicts", conflicts)
        
        self.get_logger().info('Sample visualizations generated')
    
    def generate_static_plot(self, primary_mission, scenario_name, conflicts=[]):
        """
        Generate a static plot showing all flight paths and conflicts
        """
        fig, ax = plt.subplots(figsize=(10, 8))
        
        waypoints = primary_mission['waypoints']
        x_coords = [wp[0] for wp in waypoints]
        y_coords = [wp[1] for wp in waypoints]
        
        ax.plot(x_coords, y_coords, 'b-', linewidth=2, label='Primary Mission')
        ax.scatter(x_coords, y_coords, c='blue', marker='o')
        
        for i, (x, y) in enumerate(zip(x_coords, y_coords)):
            ax.text(x, y + 1, f"WP{i}", fontsize=8)
        
        colors = ['red', 'green', 'purple', 'orange', 'brown', 'pink']
        color_idx = 0
        
        for schedule in self.flight_schedules:
            drone_id = schedule.drone_id
            
            if conflicts and drone_id not in [c['simulated_drone'] for c in conflicts]:
                continue
                
            waypoints = []
            for i in range(0, len(schedule.waypoints), 3):
                x = schedule.waypoints[i]
                y = schedule.waypoints[i+1]
                waypoints.append((x, y))
            
            x_coords = [wp[0] for wp in waypoints]
            y_coords = [wp[1] for wp in waypoints]
            
            color = colors[color_idx % len(colors)]
            ax.plot(x_coords, y_coords, color=color, linewidth=1.5, linestyle='--', 
                   label=f'{drone_id} ({self._seconds_to_time_str(schedule.start_time)} - {self._seconds_to_time_str(schedule.end_time)})')
            ax.scatter(x_coords, y_coords, c=color, marker='s')
            
            color_idx += 1
        
        for conflict in conflicts:
            loc = conflict['location']
            circle = Circle(loc, self.safety_buffer, fill=True, alpha=0.3, color='red')
            ax.add_patch(circle)
            ax.text(loc[0], loc[1] + self.safety_buffer + 1, 
                   f"Conflict with {conflict['simulated_drone']}\n{conflict['time']}", 
                   fontsize=8, ha='center')
        
        ax.set_xlabel('X Coordinate (m)')
        ax.set_ylabel('Y Coordinate (m)')
        ax.set_title(f'UAV Mission Deconfliction - {scenario_name.replace("_", " ").title()}')
        ax.grid(True)
        ax.legend(loc='upper left', bbox_to_anchor=(1, 1))
        
        plt.tight_layout()
        filename = os.path.join(self.output_dir, f'static_plot_{scenario_name}.png')
        plt.savefig(filename, dpi=200, bbox_inches='tight')
        plt.close()
        
        self.get_logger().info(f'Static plot saved to: {filename}')
    
    def generate_animation(self, primary_mission, scenario_name, conflicts=[]):
        """
        Generate an animation showing the flight paths over time
        """
        fig, ax = plt.subplots(figsize=(10, 8))
        
        all_x = []
        all_y = []
        
        primary_waypoints = primary_mission['waypoints']
        primary_x = [wp[0] for wp in primary_waypoints]
        primary_y = [wp[1] for wp in primary_waypoints]
        all_x.extend(primary_x)
        all_y.extend(primary_y)
        
        for schedule in self.flight_schedules:
            for i in range(0, len(schedule.waypoints), 3):
                all_x.append(schedule.waypoints[i])
                all_y.append(schedule.waypoints[i+1])
        
        primary_start = primary_mission['start_time']
        primary_end = primary_mission['end_time']
        
        time_buffer = 300 
        anim_start = primary_start - time_buffer
        anim_end = primary_end + time_buffer
        
        time_resolution = 60  
        frames = int((anim_end - anim_start) / time_resolution)
        
        def init():
            ax.clear()
            
            min_x, max_x = min(all_x) - 10, max(all_x) + 10
            min_y, max_y = min(all_y) - 10, max(all_y) + 10
            ax.set_xlim(min_x, max_x)
            ax.set_ylim(min_y, max_y)
            
            ax.plot(primary_x, primary_y, 'b--', alpha=0.3)
            
            for schedule in self.flight_schedules:
                if conflicts and schedule.drone_id not in [c['simulated_drone'] for c in conflicts]:
                    continue
                    
                waypoints = []
                for i in range(0, len(schedule.waypoints), 3):
                    waypoints.append((schedule.waypoints[i], schedule.waypoints[i+1]))
                
                drone_x = [wp[0] for wp in waypoints]
                drone_y = [wp[1] for wp in waypoints]
                ax.plot(drone_x, drone_y, '--', alpha=0.3)
            
            ax.set_title('UAV Trajectories - Time: 00:00:00')
            ax.grid(True)
            ax.set_xlabel('X Coordinate (m)')
            ax.set_ylabel('Y Coordinate (m)')
            
            return []
        
        def animate(frame):
            current_time = anim_start + frame * time_resolution
            time_str = self._seconds_to_time_str(current_time)
            
            ax.set_title(f'UAV Trajectories - Time: {time_str}')
            
            for artist in ax.get_children():
                if isinstance(artist, plt.Line2D) and artist.get_label().startswith('drone_pos_'):
                    artist.remove()
            
            plot_elements = []
            
            if primary_start <= current_time <= primary_end:
                progress = (current_time - primary_start) / (primary_end - primary_start)
                position = self._interpolate_position(primary_waypoints, progress)
                
                primary_dot = ax.plot(position[0], position[1], 'bo', markersize=8, 
                                     label='drone_pos_primary')[0]
                plot_elements.append(primary_dot)
                
                primary_label = ax.text(position[0] + 1, position[1] + 1, 'Primary', fontsize=8)
                plot_elements.append(primary_label)
            
            for schedule in self.flight_schedules:
                if conflicts and schedule.drone_id not in [c['simulated_drone'] for c in conflicts]:
                    continue
                    
                drone_start = schedule.start_time
                drone_end = schedule.end_time
                
                if drone_start <= current_time <= drone_end:
                    waypoints = []
                    for i in range(0, len(schedule.waypoints), 3):
                        waypoints.append((schedule.waypoints[i], schedule.waypoints[i+1]))
                    
                    progress = (current_time - drone_start) / (drone_end - drone_start)
                    position = self._interpolate_position(waypoints, progress)
                    
                    if schedule.drone_id == 'Drone_X':
                        color = 'r'
                    elif schedule.drone_id == 'Drone_Y':
                        color = 'g'
                    else:
                        color = 'purple'
                    
                    drone_dot = ax.plot(position[0], position[1], color + 'o', markersize=8,
                                       label=f'drone_pos_{schedule.drone_id}')[0]
                    plot_elements.append(drone_dot)
                    
                    # Add label
                    drone_label = ax.text(position[0] + 1, position[1] + 1, schedule.drone_id, fontsize=8)
                    plot_elements.append(drone_label)
                    
                    if primary_start <= current_time <= primary_end:
                        primary_pos = self._interpolate_position(primary_waypoints, 
                                                               (current_time - primary_start) / (primary_end - primary_start))
                        
                        distance = np.sqrt((position[0] - primary_pos[0])**2 + (position[1] - primary_pos[1])**2)
                        
                        if distance < self.safety_buffer:
                            conflict_circle = plt.Circle(primary_pos, self.safety_buffer, color='red', fill=False, 
                                                        linestyle='--', linewidth=2, alpha=0.7)
                            ax.add_patch(conflict_circle)
                            plot_elements.append(conflict_circle)
                            
                            conflict_text = ax.text(primary_pos[0], primary_pos[1] + self.safety_buffer + 1, 
                                                  'CONFLICT!', color='red', fontweight='bold', ha='center')
                            plot_elements.append(conflict_text)
            
            handles, labels = ax.get_legend_handles_labels()
            filtered_handles = [h for h, l in zip(handles, labels) if not l.startswith('drone_pos_')]
            filtered_labels = [l for l in labels if not l.startswith('drone_pos_')]
            legend = ax.legend(filtered_handles, filtered_labels, loc='upper left', bbox_to_anchor=(1, 1))
            plot_elements.append(legend)
            
            return plot_elements
        
        anim = animation.FuncAnimation(fig, animate, init_func=init, frames=frames, 
                                     interval=500, blit=False)
        
        filename = os.path.join(self.output_dir, f'animation_{scenario_name}.mp4')
        writer = animation.FFMpegWriter(fps=5, metadata=dict(artist='UAV Deconfliction System'))
        anim.save(filename, writer=writer)
        plt.close()
        
        self.get_logger().info(f'Animation saved to: {filename}')
    
    def _interpolate_position(self, waypoints, progress):
        """
        Interpolate position along a path based on progress (0.0 to 1.0)
        """
        if progress <= 0.0:
            return waypoints[0]
        
        if progress >= 1.0:
            return waypoints[-1]
        
        total_length = 0.0
        segment_lengths = []
        
        for i in range(len(waypoints) - 1):
            p1 = waypoints[i]
            p2 = waypoints[i + 1]
            length = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
            segment_lengths.append(length)
            total_length += length
        
        target_length = progress * total_length
        current_length = 0.0
        
        for i, length in enumerate(segment_lengths):
            if current_length + length >= target_length:
                segment_progress = (target_length - current_length) / length
                p1 = waypoints[i]
                p2 = waypoints[i + 1]
                
                x = p1[0] + segment_progress * (p2[0] - p1[0])
                y = p1[1] + segment_progress * (p2[1] - p1[1])
                
                return (x, y)
            
            current_length += length
        
        return waypoints[-1]
    
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
    visualization_node = VisualizationNode()
    rclpy.spin(visualization_node)
    visualization_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()