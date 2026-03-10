#!/usr/bin/env python3
import math

import carla
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class CarlaPosePublisher(Node):
    def __init__(self):
        super().__init__('carla_pose_publisher')
        self.pose_publishers = {
            i: self.create_publisher(PoseStamped, f'/truck{i}/pose_from_carla', 10)
            for i in range(3)
        }
        self.client = None
        self.world = None
        self.actors = {}

        self._connect_carla()
        self.create_timer(0.05, self._tick)

    def _connect_carla(self):
        try:
            self.client = carla.Client('localhost', 2000)
            self.client.set_timeout(2.0)
            self.world = self.client.get_world()
            self.get_logger().info('CARLA 연결 성공')
        except Exception as exc:
            self.world = None
            self.get_logger().warn(f'CARLA 연결 실패: {exc}')

    def _refresh_actors(self):
        if self.world is None:
            return
        try:
            found = 0
            for actor in self.world.get_actors().filter('vehicle.*'):
                role_name = actor.attributes.get('role_name', '')
                if not role_name.startswith('truck'):
                    continue
                try:
                    truck_id = int(role_name.replace('truck', ''))
                except ValueError:
                    continue
                if truck_id in self.pose_publishers:
                    self.actors[truck_id] = actor
                    found += 1
            if found:
                self.get_logger().info(f'CARLA truck actor 갱신: {found}대')
        except Exception as exc:
            self.get_logger().warn(f'actor 탐색 실패: {exc}')

    @staticmethod
    def _yaw_to_quaternion(yaw_deg):
        yaw = math.radians(yaw_deg)
        half = yaw * 0.5
        return 0.0, 0.0, math.sin(half), math.cos(half)

    def _tick(self):
        if self.world is None:
            self._connect_carla()
            return

        if len(self.actors) < len(self.pose_publishers):
            self._refresh_actors()

        for truck_id, pub in self.pose_publishers.items():
            actor = self.actors.get(truck_id)
            if actor is None:
                continue
            try:
                tf = actor.get_transform()
            except RuntimeError:
                self.actors.pop(truck_id, None)
                continue
            except Exception:
                continue

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.position.x = float(tf.location.x)
            msg.pose.position.y = float(tf.location.y)
            msg.pose.position.z = float(tf.location.z)
            qx, qy, qz, qw = self._yaw_to_quaternion(float(tf.rotation.yaw))
            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw
            pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CarlaPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
