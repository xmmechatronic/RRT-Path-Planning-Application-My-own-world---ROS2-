#!/usr/bin/env python3
"""
RRT Path Planner for Maze Navigation
Implements Rapidly-exploring Random Tree algorithm for path planning
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import numpy as np
import math
import time
import csv
import os
from collections import deque
import matplotlib.pyplot as plt


class RRTNode:
    """Node in RRT tree"""
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0


class RRTPlanner(Node):
    def __init__(self):
        super().__init__('rrt_maze_nav')
        
        # Publishers & Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Reset service client
        self.reset_world_client = self.create_client(Empty, '/reset_world')
        
        # Maze boundaries (from labirent.world)
        self.maze_bounds = {
            'x_min': -5.0,
            'x_max': 5.0,
            'y_min': -10.0,
            'y_max': 10.0
        }
        
        # Start and goal positions
        self.start_pos = np.array([-3.73, -8.73])
        self.goal_pos = np.array([3.64, 7.39])
        self.goal_threshold = 0.5  # Success threshold
        
        # Current robot state
        self.current_pos = np.array([0.0, 0.0])
        self.current_yaw = 0.0
        self.scan_data = None
        
        # RRT parameters
        self.max_iterations = 2000
        # Path following parameters (OPTIMIZED for safety)
        self.linear_speed = 0.4
        self.angular_speed = 1.5
        self.position_tolerance = 0.40  # Stricter tolerance to prevent corner cutting
        self.angle_tolerance = 0.15  # Stricter angle tolerance
        
        # Episode tracking
        self.episode_num = 0
        self.max_episodes = 20
        self.step_count = 0
        self.max_steps = 3000
        self.episode_timer = None
        
        # RRT parameters
        self.max_iterations = 3000
        self.step_size = 0.4  # Smaller steps for better precision
        self.goal_sample_rate = 0.2
        self.collision_check_resolution = 0.05
        self.obstacle_clearance = 0.45  # Increased safety margin (from 0.3)
        
        # Episode tracking
        self.episode_num = 0
        self.max_episodes = 20
        self.step_count = 0
        self.max_steps = 3000  # Increased for longer navigation time
        self.episode_timer = None  # Timer for episode scheduling
        
        # State machine for path following
        self.state = 'IDLE'  # States: IDLE, PLANNING, FOLLOWING, COMPLETED
        self.waypoints = []  # List of waypoints to follow
        self.current_waypoint_idx = 0  # Index of current target waypoint
        self.control_timer = None  # Timer for control loop
        self.planning_start_time = 0
        self.episode_start_time = 0
        
        # Performance metrics
        self.episode_data = []
        self.log_file = '/home/ceri/Desktop/rl_data_rrt/training_log.csv'
        os.makedirs(os.path.dirname(self.log_file), exist_ok=True)
        
        # Initialize CSV
        with open(self.log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Episode', 'Steps', 'Success', 'PlanningTime', 'PathLength', 
                           'ExecutionTime', 'TotalTime', 'NodesExplored'])
        
        self.get_logger().info('🌳 RRT Planner Initialized')
        self.get_logger().info(f'📝 Logging to: {self.log_file}')
        
        # Create control timer (runs continuously)
        self.control_timer = self.create_timer(0.2, self.control_callback)
        
        # Wait for sensors to initialize
        self.episode_timer = self.create_timer(2.0, self.start_episode_wrapper)
    
    def scan_callback(self, msg):
        """Process LiDAR scan data"""
        self.scan_data = np.array(msg.ranges)
        # Replace inf values with max range
        self.scan_data = np.nan_to_num(self.scan_data, nan=3.5, posinf=3.5, neginf=0.0)
    
    def odom_callback(self, msg):
        """Process odometry data"""
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def reset_simulation(self):
        """Reset Gazebo simulation and wait for sensors"""
        if self.reset_world_client.wait_for_service(timeout_sec=1.0):
            future = self.reset_world_client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
        # Wait longer for simulation and sensors to stabilize
        time.sleep(2.5)
        
        # Verify sensors are ready
        max_wait = 5.0
        start_wait = time.time()
        while (time.time() - start_wait) < max_wait:
            if self.scan_data is not None and len(self.scan_data) > 0:
                self.get_logger().info('✓ Sensors ready')
                return True
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.0)
        
        self.get_logger().warn('⚠️ Sensors not ready after reset!')
        return False
    
    
    def is_collision_free(self, pos):
        """Check if position is collision-free using static maze map"""
        x, y = pos[0], pos[1]
        
        # Check if position is within maze bounds
        if (x < self.maze_bounds['x_min'] or x > self.maze_bounds['x_max'] or
            y < self.maze_bounds['y_min'] or y > self.maze_bounds['y_max']):
            return False
        
        # Static obstacle map from labirent.world
        # Format: (center_x, center_y, half_width, half_height)
        obstacles = [
            # Boundary walls
            (-5.0, 0.0, 0.075, 10.0),      # Wall 1 - Left
            (5.0, 0.0, 0.075, 10.0),       # Wall 2 - Right
            (0.0, 10.0, 5.0, 0.075),       # Wall 3 - Top
            (0.0, -10.0, 5.0, 0.075),      # Wall 4 - Bottom
            # Internal obstacles
            (2.5, 5.0, 2.5, 0.075),        # Obstacle 1
            (-2.5, 0.0, 2.5, 0.075),       # Obstacle 2
            (2.5, -4.0, 2.0, 0.075),       # Obstacle 3
        ]
        
        # Check collision with each obstacle (with clearance)
        clearance = self.obstacle_clearance
        for obs_x, obs_y, half_w, half_h in obstacles:
            # Check if point is inside expanded obstacle box
            if (abs(x - obs_x) < (half_w + clearance) and 
                abs(y - obs_y) < (half_h + clearance)):
                return False
        
        return True
    
    def get_random_point(self):
        """Sample random point in maze space"""
        if np.random.random() < self.goal_sample_rate:
            return self.goal_pos.copy()
        
        x = np.random.uniform(self.maze_bounds['x_min'], self.maze_bounds['x_max'])
        y = np.random.uniform(self.maze_bounds['y_min'], self.maze_bounds['y_max'])
        return np.array([x, y])
    
    def get_nearest_node(self, tree, point):
        """Find nearest node in tree to given point"""
        min_dist = float('inf')
        nearest = None
        
        for node in tree:
            dist = np.linalg.norm([node.x - point[0], node.y - point[1]])
            if dist < min_dist:
                min_dist = dist
                nearest = node
        
        return nearest
    
    def steer(self, from_node, to_point):
        """Create new node by steering from from_node toward to_point"""
        dx = to_point[0] - from_node.x
        dy = to_point[1] - from_node.y
        distance = np.sqrt(dx**2 + dy**2)
        
        if distance <= self.step_size:
            return RRTNode(to_point[0], to_point[1])
        
        # Limit to step size
        ratio = self.step_size / distance
        new_x = from_node.x + dx * ratio
        new_y = from_node.y + dy * ratio
        
        return RRTNode(new_x, new_y)
    
    def extract_path(self, node):
        """Extract path from start to node by following parent pointers"""
        path = []
        current = node
        
        while current is not None:
            path.append([current.x, current.y])
            current = current.parent
        
        path.reverse()
        
        # Aggressive simplification: keep only every 5th waypoint for faster execution
        simplified = [path[0]]  # Always keep start
        for i in range(5, len(path), 5):
            simplified.append(path[i])
        if len(path) > 1:
            simplified.append(path[-1])  # Always keep goal
        
        return np.array(simplified)
    
    def plan_path_rrt(self):
        """Plan path using RRT algorithm"""
        start_time = time.time()
        
        # Initialize tree with start node
        start_node = RRTNode(self.start_pos[0], self.start_pos[1])
        tree = [start_node]
        
        for i in range(self.max_iterations):
            # Sample random point
            rand_point = self.get_random_point()
            
            # Find nearest node
            nearest_node = self.get_nearest_node(tree, rand_point)
            
            # Steer toward random point
            new_node = self.steer(nearest_node, rand_point)
            
            # Check if collision-free
            if self.is_collision_free(np.array([new_node.x, new_node.y])):
                new_node.parent = nearest_node
                new_node.cost = nearest_node.cost + np.linalg.norm(
                    [new_node.x - nearest_node.x, new_node.y - nearest_node.y]
                )
                tree.append(new_node)
                
                # Check if reached goal
                goal_dist = np.linalg.norm([new_node.x - self.goal_pos[0], 
                                          new_node.y - self.goal_pos[1]])
                if goal_dist <= self.goal_threshold:
                    planning_time = time.time() - start_time
                    path = self.extract_path(new_node)
                    self.get_logger().info(
                        f'✅ Path found! Nodes: {len(tree)}, Time: {planning_time:.2f}s, '
                        f'Path length: {new_node.cost:.2f}m'
                    )
                    return path, planning_time, len(tree), new_node.cost
        
        planning_time = time.time() - start_time
        self.get_logger().warn(f'❌ No path found after {self.max_iterations} iterations')
        return None, planning_time, len(tree), 0.0
    
    def calculate_heading_error(self, target_pos):
        """Calculate angle error to target position"""
        dx = target_pos[0] - self.current_pos[0]
        dy = target_pos[1] - self.current_pos[1]
        target_yaw = math.atan2(dy, dx)
        error = target_yaw - self.current_yaw
        # Normalize to [-pi, pi]
        return math.atan2(math.sin(error), math.cos(error))
    
    
    def start_path_following(self, path):
        """Initialize path following (non-blocking)"""
        self.waypoints = path[1:]  # Skip first point (current position)
        self.current_waypoint_idx = 0
        self.state = 'FOLLOWING'
        self.step_count = 0
        
        if len(self.waypoints) > 0:
            self.get_logger().info(f'Starting path following with {len(self.waypoints)} waypoints')
        else:
            self.get_logger().warn('No waypoints to follow!')
            self.state = 'COMPLETED'
    
    def control_callback(self):
        """Control timer callback - executes path following logic"""
        
        # Only run control when in FOLLOWING state
        if self.state != 'FOLLOWING':
            return
        
        # Check if we have waypoints
        if self.current_waypoint_idx >= len(self.waypoints):
            # All waypoints visited
            goal_dist = np.linalg.norm(self.goal_pos - self.current_pos)
            if goal_dist < self.goal_threshold:
                self.end_episode(success=True)
            else:
                self.end_episode(success=False)
            return
        
        # Increment step counter
        self.step_count += 1
        
        # Check max steps limit
        if self.step_count >= self.max_steps:
            self.get_logger().warn('⏱️ Max steps reached')
            self.stop_robot()
            self.end_episode(success=False)
            return
        
        # Get current target waypoint
        waypoint = self.waypoints[self.current_waypoint_idx]
        
        # Calculate distance and angle to waypoint
        dx = waypoint[0] - self.current_pos[0]
        dy = waypoint[1] - self.current_pos[1]
        distance = np.sqrt(dx**2 + dy**2)
        
        # Log progress every 100 steps
        if self.step_count % 100 == 0:
            self.get_logger().info(
                f'WP {self.current_waypoint_idx+1}/{len(self.waypoints)}: '
                f'dist={distance:.2f}m, pos=({self.current_pos[0]:.2f},{self.current_pos[1]:.2f})'
            )
        
        # Check if waypoint reached
        if distance < self.position_tolerance:
            self.get_logger().info(f'✓ Waypoint {self.current_waypoint_idx+1}/{len(self.waypoints)} reached!')
            self.current_waypoint_idx += 1
            
            # Check if reached goal early
            goal_dist = np.linalg.norm(self.goal_pos - self.current_pos)
            if goal_dist < self.goal_threshold:
                self.get_logger().info('🎯 Goal reached!')
                self.stop_robot()
                self.end_episode(success=True)
            return
        
        # Calculate target angle
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.current_yaw
        # Normalize to [-pi, pi]
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        # Control logic
        cmd = Twist()
        
        if abs(angle_diff) > 0.3:  # ~17 degrees
            cmd.angular.z = np.clip(angle_diff * 1.5, -1.0, 1.0)
            cmd.linear.x = 0.0  # Strict rotation in place (safer)
        else:
            cmd.linear.x = self.linear_speed
            cmd.angular.z = angle_diff * 0.8
        
        self.cmd_vel_pub.publish(cmd)
    
    def end_episode(self, success):
        """End current episode and log results"""
        self.state = 'COMPLETED'
        self.stop_robot()
        
        # Calculate times
        total_time = time.time() - self.episode_start_time
        execution_time = total_time - self.episode_planning_time
        
        # Log episode data
        with open(self.log_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                self.episode_num,
                self.step_count,
                1 if success else 0,
                f'{self.episode_planning_time:.3f}',
                f'{self.episode_path_length:.3f}',
                f'{execution_time:.3f}',
                f'{total_time:.3f}',
                self.episode_nodes_explored
            ])
        
        result = '✅ SUCCESS' if success else '❌ FAILED'
        self.get_logger().info(
            f'Episode {self.episode_num} {result} | Steps: {self.step_count} | '
            f'Planning: {self.episode_planning_time:.2f}s | Total: {total_time:.2f}s'
        )
        
        self.episode_num += 1
        
        # Schedule next episode
        if self.episode_num < self.max_episodes:
            self.episode_timer = self.create_timer(2.0, self.start_episode_wrapper)
        else:
            self.get_logger().info('🏁 All episodes completed!')
            self.get_logger().info(f'Results saved to: {self.log_file}')
            self.plot_results()
    
    def force_save_incomplete_episode(self):
        """Save current incomplete episode metrics on interrupt"""
        if self.state == 'FOLLOWING' or self.state == 'PLANNING':
            self.stop_robot()
            
            # Calculate times so far
            if self.episode_start_time > 0:
                total_time = time.time() - self.episode_start_time
            else:
                total_time = 0
            
            execution_time = total_time - self.episode_planning_time if self.episode_planning_time > 0 else 0
            
            # Log incomplete data
            with open(self.log_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    self.episode_num,
                    self.step_count,
                    0,  # Failed/Incomplete
                    f'{self.episode_planning_time:.3f}',
                    f'{self.episode_path_length if hasattr(self, "episode_path_length") else 0:.3f}',
                    f'{execution_time:.3f}',
                    f'{total_time:.3f}',
                    self.episode_nodes_explored if hasattr(self, "episode_nodes_explored") else 0
                ])
            
            self.get_logger().info('💾 Saved incomplete episode data.')
    
    def old_follow_path(self, path):
        """Execute path following with improved control logic"""
        
        for i, waypoint in enumerate(path[1:], 1):  # Skip first point (current position)
            self.get_logger().info(f'Target waypoint {i}/{len(path)-1}: ({waypoint[0]:.2f}, {waypoint[1]:.2f})')
            
            max_attempts = 300  # Increased attempts per waypoint
            attempt = 0
            
            while attempt < max_attempts:
                # Update step counters
                self.step_count += 1
                attempt += 1
                
                # Check global step limit
                if self.step_count >= self.max_steps:
                    self.get_logger().warn('⏱️ Max steps reached')
                    self.stop_robot()
                    return False
                
                # Get current distance and angle to waypoint
                dx = waypoint[0] - self.current_pos[0]
                dy = waypoint[1] - self.current_pos[1]
                distance = np.sqrt(dx**2 + dy**2)
                
                # Log progress every 50 steps
                if attempt % 50 == 0:
                    self.get_logger().info(
                        f'Progress: dist={distance:.2f}m, pos=({self.current_pos[0]:.2f},{self.current_pos[1]:.2f}), '
                        f'yaw={math.degrees(self.current_yaw):.1f}°'
                    )
                
                # Check if reached waypoint
                if distance < self.position_tolerance:
                    self.get_logger().info(f'✓ Waypoint {i} reached!')
                    break
                
                # Calculate target angle
                target_angle = math.atan2(dy, dx)
                angle_diff = target_angle - self.current_yaw
                # Normalize to [-pi, pi]
                angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
                
                # Improved control logic with slower, more deliberate movements
                cmd = Twist()
                
                if abs(angle_diff) > 0.2:  # ~11 degrees - even more sensitive
                    # Rotate towards target with minimal forward motion
                    cmd.angular.z = np.clip(angle_diff * 1.0, -0.8, 0.8)  # Reduced rotation speed
                    cmd.linear.x = 0.05  # Very small forward motion while rotating
                else:
                    # Move forward with angular correction
                    cmd.linear.x = 0.3  # Reduced forward speed for better control
                    cmd.angular.z = angle_diff * 0.5  # Gentler correction
                
                # Send command
                self.cmd_vel_pub.publish(cmd)
                
                # Sleep to allow robot to move and odometry to update
                time.sleep(0.2)  # Increased from 0.1 to 0.2 for better stability
                
                # Check if reached final goal early
                goal_dist = np.linalg.norm(self.goal_pos - self.current_pos)
                if goal_dist < self.goal_threshold:
                    self.get_logger().info('🎯 Goal reached!')
                    self.stop_robot()
                    return True
            
            # If couldn't reach this waypoint, continue to next
            if attempt >= max_attempts:
                self.get_logger().warn(f'Could not reach waypoint {i}, moving to next')
        
        # Final goal check
        goal_dist = np.linalg.norm(self.goal_pos - self.current_pos)
        if goal_dist < self.goal_threshold:
            self.stop_robot()
            return True
        
        self.stop_robot()
        return False
    
    def stop_robot(self):
        """Stop robot movement"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
    
    def start_episode_wrapper(self):
        """Wrapper to cancel timer and call start_episode"""
        if self.episode_timer is not None:
            self.episode_timer.cancel()
            self.episode_timer = None
        self.start_episode()
    
    def start_episode(self):
        """Start new planning and execution episode"""
        if self.episode_num >= self.max_episodes:
            self.get_logger().info('🏁 All episodes completed!')
            self.get_logger().info(f'Results saved to: {self.log_file}')
            return
        
        self.get_logger().info(f'\n🎬 EPISODE {self.episode_num} STARTED (RRT)')
        self.get_logger().info(f'Robot current position: ({self.current_pos[0]:.2f}, {self.current_pos[1]:.2f})')
        self.get_logger().info(f'Start position: ({self.start_pos[0]:.2f}, {self.start_pos[1]:.2f})')
        self.get_logger().info(f'Goal position: ({self.goal_pos[0]:.2f}, {self.goal_pos[1]:.2f})')
        episode_start = time.time()
        self.step_count = 0
        
        # Reset simulation and wait for sensors
        sensors_ready = self.reset_simulation()
        if not sensors_ready:
            self.get_logger().error('Sensors not ready, skipping episode')
            self.episode_num += 1
            self.episode_timer = self.create_timer(1.0, self.start_episode_wrapper)
            return
        
        # Plan path using RRT
        self.planning_start_time = time.time()
        path, planning_time, nodes_explored, path_length = self.plan_path_rrt()
        
        # Store episode start time and metrics
        self.episode_start_time = episode_start
        self.episode_planning_time = planning_time
        self.episode_nodes_explored = nodes_explored
        self.episode_path_length = path_length
        
        if path is not None and len(path) > 1:
            # Start non-blocking path following
            self.start_path_following(path)
        else:
            self.get_logger().warn('No path found, ending episode')
            self.end_episode(success=False)
    
    def plot_results(self):
        """Generate performance plots from training log"""
        try:
            # Load data from CSV
            episodes = []
            steps = []
            successes = []
            planning_times = []
            path_lengths = []
            total_times = []
            nodes_explored = []
            
            with open(self.log_file, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    episodes.append(int(row['Episode']))
                    steps.append(int(row['Steps']))
                    successes.append(int(row['Success']))
                    planning_times.append(float(row['PlanningTime']))
                    path_lengths.append(float(row['PathLength']))
                    total_times.append(float(row['TotalTime']))
                    nodes_explored.append(int(row['NodesExplored']))
            
            if len(episodes) == 0:
                self.get_logger().warn('No data to plot')
                return
            
            # Convert to numpy arrays
            episodes = np.array(episodes)
            steps = np.array(steps)
            successes = np.array(successes)
            planning_times = np.array(planning_times)
            path_lengths = np.array(path_lengths)
            total_times = np.array(total_times)
            nodes_explored = np.array(nodes_explored)
            
            # Calculate rolling average for success rate
            window = min(5, len(successes))
            success_rate = np.convolve(successes, np.ones(window)/window, mode='same') * 100
            
            # Create figure with 2x3 subplots
            fig, axes = plt.subplots(2, 3, figsize=(18, 10))
            fig.suptitle('RRT Path Planning Performance', fontsize=16, fontweight='bold')
            
            # Plot 1: Success Rate (rolling average)
            ax1 = axes[0, 0]
            ax1.plot(episodes, success_rate, color='#2ecc71', linewidth=2.5, marker='o')
            ax1.fill_between(episodes, 0, success_rate, alpha=0.3, color='#2ecc71')
            ax1.set_xlabel('Episode', fontweight='bold')
            ax1.set_ylabel('Success Rate (%)', fontweight='bold')
            ax1.set_title(f'Success Rate (Rolling Avg, window={window})', fontweight='bold')
            ax1.grid(True, alpha=0.3)
            ax1.set_ylim([0, 105])
            
            # Plot 2: Steps per Episode
            ax2 = axes[0, 1]
            ax2.plot(episodes, steps, color='#3498db', linewidth=2, marker='s')
            ax2.axhline(y=self.max_steps, color='red', linestyle='--', label='Max Steps')
            ax2.set_xlabel('Episode', fontweight='bold')
            ax2.set_ylabel('Steps', fontweight='bold')
            ax2.set_title('Steps per Episode', fontweight='bold')
            ax2.legend()
            ax2.grid(True, alpha=0.3)
            
            # Plot 3: Planning Time
            ax3 = axes[0, 2]
            ax3.bar(episodes, planning_times, color='#9b59b6', alpha=0.7)
            avg_planning = planning_times.mean()
            ax3.axhline(y=avg_planning, color='#e74c3c', linestyle='--', 
                       label=f'Avg: {avg_planning:.2f}s', linewidth=2)
            ax3.set_xlabel('Episode', fontweight='bold')
            ax3.set_ylabel('Time (seconds)', fontweight='bold')
            ax3.set_title('RRT Planning Time', fontweight='bold')
            ax3.legend()
            ax3.grid(True, alpha=0.3)
            
            # Plot 4: Path Length
            ax4 = axes[1, 0]
            ax4.plot(episodes, path_lengths, color='#e67e22', linewidth=2.5, marker='D')
            avg_length = path_lengths.mean()
            ax4.axhline(y=avg_length, color='#c0392b', linestyle='--', 
                       label=f'Avg: {avg_length:.2f}m', linewidth=2)
            ax4.set_xlabel('Episode', fontweight='bold')
            ax4.set_ylabel('Path Length (m)', fontweight='bold')
            ax4.set_title('Planned Path Length', fontweight='bold')
            ax4.legend()
            ax4.grid(True, alpha=0.3)
            
            # Plot 5: Total Episode Time
            ax5 = axes[1, 1]
            ax5.plot(episodes, total_times, color='#16a085', linewidth=2, marker='^')
            ax5.set_xlabel('Episode', fontweight='bold')
            ax5.set_ylabel('Time (seconds)', fontweight='bold')
            ax5.set_title('Total Episode Time', fontweight='bold')
            ax5.grid(True, alpha=0.3)
            
            # Plot 6: Nodes Explored
            ax6 = axes[1, 2]
            colors = ['red' if s == 0 else 'green' for s in successes]
            ax6.scatter(episodes, nodes_explored, c=colors, s=100, alpha=0.7, edgecolors='black')
            avg_nodes = nodes_explored.mean()
            ax6.axhline(y=avg_nodes, color='#34495e', linestyle='--', 
                       label=f'Avg: {avg_nodes:.0f} nodes', linewidth=2)
            ax6.set_xlabel('Episode', fontweight='bold')
            ax6.set_ylabel('Nodes', fontweight='bold')
            ax6.set_title('RRT Tree Nodes Explored', fontweight='bold')
            ax6.legend()
            ax6.grid(True, alpha=0.3)
            
            plt.tight_layout()
            
            # Save figure
            plot_dir = os.path.dirname(self.log_file)
            plot_path = os.path.join(plot_dir, 'rrt_performance.png')
            plt.savefig(plot_path, dpi=300, bbox_inches='tight')
            self.get_logger().info(f'📊 Performance plot saved to: {plot_path}')
            
            # Print statistics
            self.get_logger().info('\n' + '='*60)
            self.get_logger().info('PERFORMANCE SUMMARY')
            self.get_logger().info('='*60)
            self.get_logger().info(f'Total Episodes: {len(episodes)}')
            self.get_logger().info(f'Success Rate: {successes.mean()*100:.1f}%')
            self.get_logger().info(f'Avg Planning Time: {planning_times.mean():.2f}s')
            self.get_logger().info(f'Avg Path Length: {path_lengths.mean():.2f}m')
            self.get_logger().info(f'Avg Nodes Explored: {nodes_explored.mean():.0f}')
            if successes.sum() > 0:
                success_steps = steps[successes == 1]
                self.get_logger().info(f'Avg Steps (Success): {success_steps.mean():.1f}')
            self.get_logger().info('='*60)
            
            # Show plot
            plt.show()
            
        except Exception as e:
            self.get_logger().error(f'Error plotting results: {e}')


def main(args=None):
    rclpy.init(args=args)
    planner = RRTPlanner()
    
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        planner.get_logger().info('⚠️ Keyboard interrupt, generating plots...')
        planner.force_save_incomplete_episode() # Save current progress
        planner.plot_results()  # EXPLICITLY call plot on interrupt
    finally:
        planner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
