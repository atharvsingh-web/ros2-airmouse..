#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path
from tf2_ros import TransformBroadcaster
import math

class ArenaSimulator(Node):
    def __init__(self):
        super().__init__('arena_simulator')
        self.tf_broadcaster = TransformBroadcaster(self)
        
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.path_pub = self.create_publisher(Path, '/drone_trail', 10)

        self.x = -3.0
        self.y = -3.0
        self.yaw = 0.0

        self.waypoints = [
            (0.0, -3.2),
            (3.2, -3.2),
            (3.2, 0.0),
            (3.2, 3.2),
            (0.0, 3.2),
            (-3.2, 3.2),
            (-3.2, 0.0),
            (-3.2, -3.2)
        ]
        self.wp_idx = 0
        self.trail = Path()
        self.trail.header.frame_id = 'map'

        self.grid_size = 100 # 10m x 10m at 0.1m resolution
        self.res = 0.1
        
        # True hidden arena layout (0 = free, 100 = wall)
        self.true_grid = [0] * (self.grid_size * self.grid_size)
        # Progressive 2D SLAM map (-1 = unexplored/unknown, 0 = free, 100 = wall)
        self.explored_grid = [-1] * (self.grid_size * self.grid_size)
        self.build_true_maze()

        self.timer = self.create_timer(0.05, self.update_physics)
        self.map_timer = self.create_timer(0.2, self.publish_2d_map)
        self.get_logger().info('Real-Time Progressive 2D SLAM Simulator Started!')

    def build_true_maze(self):
        for r in range(self.grid_size):
            for c in range(self.grid_size):
                if r == 0 or r == self.grid_size - 1 or c == 0 or c == self.grid_size - 1:
                    self.true_grid[r * self.grid_size + c] = 100
                elif (30 <= r <= 70 and 30 <= c <= 70) and (r in (30, 70) or c in (30, 70)):
                    if not (45 <= r <= 55 or 45 <= c <= 55):
                        self.true_grid[r * self.grid_size + c] = 100

    def update_physics(self):
        dt = 0.05
        tx, ty = self.waypoints[self.wp_idx]
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        if dist < 0.35:
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
        else:
            target_yaw = math.atan2(dy, dx)
            angle_diff = (target_yaw - self.yaw + math.pi) % (2.0 * math.pi) - math.pi
            self.yaw += max(min(angle_diff * 4.0, 2.0), -2.0) * dt
            speed = 0.9 if abs(angle_diff) < 0.4 else 0.15
            self.x += speed * math.cos(self.yaw) * dt
            self.y += speed * math.sin(self.yaw) * dt

        now = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.05
        self.trail.header.stamp = now
        self.trail.poses.append(pose)
        if len(self.trail.poses) > 500:
            self.trail.poses.pop(0)
        self.path_pub.publish(self.trail)

        self.publish_lidar_and_update_2d_slam(now)

    def publish_lidar_and_update_2d_slam(self, stamp):
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.radians(2.0)
        scan.range_min = 0.15
        scan.range_max = 5.5 # Indoor LiDAR discovery range

        ranges = []
        for i in range(180):
            angle = scan.angle_min + i * scan.angle_increment + self.yaw
            d = 0.15
            hit_wall = False
            while d < scan.range_max:
                px = int(((self.x + 5.0) + d * math.cos(angle)) / self.res)
                py = int(((self.y + 5.0) + d * math.sin(angle)) / self.res)
                if 0 <= px < self.grid_size and 0 <= py < self.grid_size:
                    cell_idx = py * self.grid_size + px
                    if self.true_grid[cell_idx] == 100:
                        # Mark wall cell (100 = Black on 2D map)
                        self.explored_grid[cell_idx] = 100
                        hit_wall = True
                        break
                    else:
                        # Mark free corridor space (0 = White on 2D map)
                        if self.explored_grid[cell_idx] == -1:
                            self.explored_grid[cell_idx] = 0
                else:
                    break
                d += 0.05
            ranges.append(d if hit_wall else scan.range_max)
        scan.ranges = ranges
        self.scan_pub.publish(scan)

    def publish_2d_map(self):
        m = OccupancyGrid()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.info.resolution = self.res
        m.info.width = self.grid_size
        m.info.height = self.grid_size
        m.info.origin.position.x = -5.0
        m.info.origin.position.y = -5.0
        m.data = self.explored_grid
        self.map_pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = ArenaSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()