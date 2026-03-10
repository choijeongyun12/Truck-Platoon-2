import os
from dataclasses import dataclass
from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


@dataclass
class MonitorState:
    speed_ms: float = 0.0
    soc: float = 0.0
    power_w: float = 0.0
    wh_per_km: float = 0.0


class EnergyMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("energy_monitor")

        self.declare_parameter("truck_count", 3)
        self.declare_parameter("refresh_hz", 1.0)
        self.truck_count = int(self.get_parameter("truck_count").value)
        refresh_hz = float(self.get_parameter("refresh_hz").value)

        self.states: Dict[int, MonitorState] = {i: MonitorState() for i in range(self.truck_count)}

        for truck_id in range(self.truck_count):
            self.create_subscription(
                Float32, f"/truck{truck_id}/velocity", lambda msg, i=truck_id: self._speed_cb(msg, i), 10
            )
            self.create_subscription(
                Float32, f"/truck{truck_id}/battery_soc", lambda msg, i=truck_id: self._soc_cb(msg, i), 10
            )
            self.create_subscription(
                Float32, f"/truck{truck_id}/battery_power_w", lambda msg, i=truck_id: self._power_cb(msg, i), 10
            )
            self.create_subscription(
                Float32, f"/truck{truck_id}/energy_wh_per_km", lambda msg, i=truck_id: self._whkm_cb(msg, i), 10
            )

        self.create_timer(1.0 / max(0.2, refresh_hz), self._render)
        self.get_logger().info(f"energy_monitor started (truck_count={self.truck_count})")

    def _speed_cb(self, msg: Float32, truck_id: int) -> None:
        self.states[truck_id].speed_ms = float(msg.data)

    def _soc_cb(self, msg: Float32, truck_id: int) -> None:
        self.states[truck_id].soc = float(msg.data)

    def _power_cb(self, msg: Float32, truck_id: int) -> None:
        self.states[truck_id].power_w = float(msg.data)

    def _whkm_cb(self, msg: Float32, truck_id: int) -> None:
        self.states[truck_id].wh_per_km = float(msg.data)

    def _render(self) -> None:
        os.system("clear")
        print("=== Truck Energy Monitor (ROS2) ===")
        print("truck | speed(km/h) | power(kW) | soc(%) | wh/km")
        print("------|-------------|-----------|--------|------")
        fleet_avg_whkm = 0.0
        for truck_id in range(self.truck_count):
            st = self.states[truck_id]
            speed_kmh = st.speed_ms * 3.6
            power_kw = st.power_w / 1000.0
            fleet_avg_whkm += st.wh_per_km
            print(
                f"{truck_id:>5} | {speed_kmh:>11.2f} | {power_kw:>9.2f} | "
                f"{st.soc:>6.2f} | {st.wh_per_km:>5.2f}"
            )
        if self.truck_count > 0:
            fleet_avg_whkm /= self.truck_count
        print("")
        print(f"fleet avg wh/km: {fleet_avg_whkm:.2f}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EnergyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
