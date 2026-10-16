#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import tf2_ros
import math

class SurvivorDetector(Node):
    def __init__(self):
        super().__init__('survivor_detector')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.marker_pub = self.create_publisher(MarkerArray, '/survivor_markers', 10)

        # 6 Dummy Humans placed across the indoor maze rooms (NIDAR Rulebook: up to 6 survivors)
        self.dummy_humans = [
            (4.5, 1.5),
            (8.2, 2.0),
            (8.5, 5.5),
            (8.0, 8.5),
            (4.5, 8.2),
            (1.8, 5.5)
        ]
        self.found = set()
        self.timer = self.create_timer(0.2, self.update_scene)
        self.get_logger().info('6 Dummy Humans Spawned! Vision Survivor Detector Online.')

    def update_scene(self):
        dx, dy, dyaw = 1.5, 1.5, 0.0
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            dx = t.transform.translation.x
            dy = t.transform.translation.y
            # Extract yaw from quaternion
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            dyaw = 2.0 * math.atan2(qz, qw)

            # Check if any dummy human is within camera detection range (3.2m)
            for idx, (sx, sy) in enumerate(self.dummy_humans):
                if idx not in self.found and math.hypot(dx - sx, dy - sy) < 3.2:
                    self.found.add(idx)
                    gx, gy = math.floor(sx), math.floor(sy)
                    self.get_logger().info(
                        f'>>> [YOLO VISION] SURVIVOR #{idx+1} SPOTTED! Tagged at 1m Grid Cell: [{gx}, {gy}] <<<'
                    )
        except Exception:
            pass

        self.publish_all_markers(dx, dy, dyaw)

    def publish_all_markers(self, dx, dy, dyaw):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, (sx, sy) in enumerate(self.dummy_humans):
            is_spotted = i in self.found
            gx, gy = math.floor(sx), math.floor(sy)

            # 1. Dummy Human Torso
            torso = Marker()
            torso.header.frame_id = 'map'
            torso.header.stamp = now
            torso.ns = 'dummy_torso'
            torso.id = i
            torso.type = Marker.CYLINDER
            torso.action = Marker.ADD
            torso.pose.position.x = sx
            torso.pose.position.y = sy
            torso.pose.position.z = 0.45
            torso.scale.x, torso.scale.y, torso.scale.z = 0.35, 0.25, 0.9
            # Unspotted = Yellow/Orange Dummy | Spotted = Bright Rescue Green
            torso.color.r = 0.1 if is_spotted else 1.0
            torso.color.g = 0.95 if is_spotted else 0.65
            torso.color.b = 0.2 if is_spotted else 0.0
            torso.color.a = 1.0
            arr.markers.append(torso)

            # 2. Dummy Human Head (Sphere)
            head = Marker()
            head.header.frame_id = 'map'
            head.header.stamp = now
            head.ns = 'dummy_head'
            head.id = i + 100
            head.type = Marker.SPHERE
            head.action = Marker.ADD
            head.pose.position.x = sx
            head.pose.position.y = sy
            head.pose.position.z = 1.05
            head.scale.x, head.scale.y, head.scale.z = 0.26, 0.26, 0.26
            head.color.r, head.color.g, head.color.b, head.color.a = 0.95, 0.8, 0.65, 1.0
            arr.markers.append(head)

            # 3. Floating Status Text Label above Dummy
            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = now
            label.ns = 'dummy_label'
            label.id = i + 200
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = sx
            label.pose.position.y = sy
            label.pose.position.z = 1.4
            label.scale.z = 0.25
            if is_spotted:
                label.text = f"SURVIVOR #{i+1} TAGGED\nGrid [{gx}, {gy}]"
                label.color.r, label.color.g, label.color.b, label.color.a = 0.2, 1.0, 0.2, 1.0
            else:
                label.text = f"Dummy #{i+1} (Unspotted)"
                label.color.r, label.color.g, label.color.b, label.color.a = 1.0, 1.0, 1.0, 0.85
            arr.markers.append(label)

            # 4. Red 1m x 1m Grid Cell Highlight Pad on Map once Spotted
            if is_spotted:
                pad = Marker()
                pad.header.frame_id = 'map'
                pad.header.stamp = now
                pad.ns = 'grid_tag'
                pad.id = i + 300
                pad.type = Marker.CUBE
                pad.action = Marker.ADD
                pad.pose.position.x = gx + 0.5
                pad.pose.position.y = gy + 0.5
                pad.pose.position.z = 0.02
                pad.scale.x, pad.scale.y, pad.scale.z = 0.95, 0.95, 0.04
                pad.color.r, pad.color.g, pad.color.b, pad.color.a = 1.0, 0.0, 0.0, 0.55
                arr.markers.append(pad)

        # 5. Pi Camera Module 3 Field-of-View (FOV) Cone in front of Drone
        fov = Marker()
        fov.header.frame_id = 'map'
        fov.header.stamp = now
        fov.ns = 'camera_fov'
        fov.id = 999
        fov.type = Marker.LINE_STRIP
        fov.action = Marker.ADD
        fov.scale.x = 0.03
        fov.color.r, fov.color.g, fov.color.b, fov.color.a = 0.0, 1.0, 0.5, 0.8

        p0 = Point(x=dx, y=dy, z=0.9)
        p1 = Point(x=dx + 2.5 * math.cos(dyaw + 0.55), y=dy + 2.5 * math.sin(dyaw + 0.55), z=0.4)
        p2 = Point(x=dx + 2.5 * math.cos(dyaw - 0.55), y=dy + 2.5 * math.sin(dyaw - 0.55), z=0.4)
        fov.points = [p0, p1, p2, p0]
        arr.markers.append(fov)

        self.marker_pub.publish(arr)

def main(args=None):
    rclpy.init(args=args)
    node = SurvivorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()