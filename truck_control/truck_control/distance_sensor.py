import rclpy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np

class DistanceSensor:
    def __init__(self, node, namespace):
        self.node = node
        self.namespace = namespace
        self.lidar_distance = None

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.node.create_subscription(
            PointCloud2,
            f'/{namespace}/front_lidar',
            self.lidar_callback,
            qos_profile
        )

    def update_lane_bounds(self, y_min, y_max):
        """차선 기반 y 필터 범위 업데이트"""
        self.y_filter_min = y_min
        self.y_filter_max = y_max


    def lidar_callback(self, msg):
        try:
            points = np.array(list(pc2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True)))
            points[:, 1] *= -1  # 좌표계 보정

            # 🚗 차선 기반 동적 y 필터 적용 (기본 fallback: ±2.0)
            y_min = getattr(self, 'y_filter_min', -1.2)
            y_max = getattr(self, 'y_filter_max', 1.2)

            front_points = points[
                (points[:, 0] > 0) &
                (points[:, 1] > y_min) &
                (points[:, 1] < y_max) &
                (points[:, 2] > -2.0)
            ]

            if len(front_points) == 0:
                self.lidar_distance = None
                return

            angles = np.arctan2(front_points[:, 1], front_points[:, 0])
            valid_points = front_points[np.abs(angles) < np.deg2rad(45)]

            if len(valid_points) > 0:
                self.lidar_distance = np.min(valid_points[:, 0])
            else:
                self.lidar_distance = None

        except Exception as e:
            self.node.get_logger().error(f"[{self.namespace}] LiDAR 처리 오류: {e}")
            self.lidar_distance = None

    def get_distance(self):
        return self.lidar_distance

if __name__ == "__main__":
    pass
