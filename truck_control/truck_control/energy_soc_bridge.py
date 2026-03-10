import csv
import math
import os
from dataclasses import dataclass
from typing import Any, Dict, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

try:
    import matlab.engine  # type: ignore
except Exception:  # pragma: no cover
    matlab = None


@dataclass
class TruckState:
    velocity_ms: float = 0.0
    accel_ms2: float = 0.0
    pitch_deg: float = 0.0
    last_z: Optional[float] = None
    soc: float = 80.0
    energy_wh: float = 0.0
    distance_m: float = 0.0
    wh_per_km: float = 0.0
    battery_power_w: float = 0.0


class EnergySocBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("energy_soc_bridge")

        self.declare_parameter("truck_count", 3)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("log_dir", "/home/tmo/ros2_ws/log/energy")
        self.declare_parameter("use_matlab_engine", True)
        self.declare_parameter("matlab_model_dir", "")
        self.declare_parameter("mass_kg", 44000.0)
        self.declare_parameter("rolling_resistance", 0.006)
        self.declare_parameter("cd_a", 5.9)
        self.declare_parameter("air_density", 1.225)
        self.declare_parameter("drive_efficiency", 0.92)
        self.declare_parameter("regen_efficiency", 0.70)
        self.declare_parameter("aux_power_w", 3000.0)
        self.declare_parameter("battery_capacity_kwh", 450.0)
        self.declare_parameter("initial_soc", 80.0)

        self.truck_count = int(self.get_parameter("truck_count").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.log_dir = str(self.get_parameter("log_dir").value)
        self.use_matlab_engine = bool(self.get_parameter("use_matlab_engine").value)
        self.mass_kg = float(self.get_parameter("mass_kg").value)
        self.rolling_resistance = float(self.get_parameter("rolling_resistance").value)
        self.cd_a = float(self.get_parameter("cd_a").value)
        self.air_density = float(self.get_parameter("air_density").value)
        self.drive_efficiency = float(self.get_parameter("drive_efficiency").value)
        self.regen_efficiency = float(self.get_parameter("regen_efficiency").value)
        self.aux_power_w = float(self.get_parameter("aux_power_w").value)
        self.battery_capacity_kwh = float(self.get_parameter("battery_capacity_kwh").value)
        initial_soc = float(self.get_parameter("initial_soc").value)

        self.states: Dict[int, TruckState] = {i: TruckState(soc=initial_soc) for i in range(self.truck_count)}
        self.last_update_time = self.get_clock().now()

        self.soc_publishers: Dict[int, Any] = {}
        self.power_publishers: Dict[int, Any] = {}
        self.whkm_publishers: Dict[int, Any] = {}

        self._csv_files: Dict[int, Any] = {}
        self._csv_writers: Dict[int, csv.writer] = {}

        self._matlab_engine = None
        self._matlab_available = False
        self._configure_matlab_engine()
        self._configure_loggers()
        self._configure_io()

        period = 1.0 / max(1.0, self.publish_hz)
        self.timer = self.create_timer(period, self._on_timer)
        self.get_logger().info(
            f"energy_soc_bridge started: trucks={self.truck_count}, hz={self.publish_hz:.1f}, "
            f"matlab_mode={self._matlab_available}"
        )

    def _configure_matlab_engine(self) -> None:
        if not self.use_matlab_engine or matlab is None:
            self.get_logger().warn("MATLAB engine unavailable or disabled. Using Python fallback model.")
            return

        try:
            self._matlab_engine = matlab.engine.start_matlab()
            model_dir = str(self.get_parameter("matlab_model_dir").value).strip()
            if model_dir:
                self._matlab_engine.addpath(model_dir, nargout=0)
            else:
                share_dir = os.path.join(get_package_share_directory("truck_control"), "matlab")
                source_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "matlab"))
                if os.path.isdir(share_dir):
                    self._matlab_engine.addpath(share_dir, nargout=0)
                if os.path.isdir(source_dir):
                    self._matlab_engine.addpath(source_dir, nargout=0)

            # Check that helper exists on MATLAB path.
            exists_flag = self._matlab_engine.exist("eplatoon_power_step", "file")
            if int(exists_flag) == 2:
                self._matlab_available = True
            else:
                self.get_logger().warn("MATLAB function eplatoon_power_step.m not found. Falling back to Python model.")
        except Exception as exc:
            self.get_logger().warn(f"Failed to start MATLAB engine ({exc}). Using Python fallback model.")
            self._matlab_available = False

    def _configure_loggers(self) -> None:
        os.makedirs(self.log_dir, exist_ok=True)
        for truck_id in range(self.truck_count):
            path = os.path.join(self.log_dir, f"truck{truck_id}_energy.csv")
            f = open(path, "w", newline="", encoding="utf-8")
            w = csv.writer(f)
            w.writerow(["time_sec", "velocity_ms", "accel_ms2", "pitch_deg", "battery_power_w", "soc", "wh_per_km"])
            self._csv_files[truck_id] = f
            self._csv_writers[truck_id] = w

    def _configure_io(self) -> None:
        for truck_id in range(self.truck_count):
            self.create_subscription(
                Float32, f"/truck{truck_id}/velocity", lambda msg, i=truck_id: self._velocity_cb(msg, i), 10
            )
            self.create_subscription(
                Float32MultiArray, f"/truck{truck_id}/accel", lambda msg, i=truck_id: self._accel_cb(msg, i), 10
            )
            self.create_subscription(
                Float32, f"/truck{truck_id}/pitch_deg", lambda msg, i=truck_id: self._pitch_cb(msg, i), 10
            )
            self.create_subscription(
                PoseStamped, f"/truck{truck_id}/pose3d", lambda msg, i=truck_id: self._pose_cb(msg, i), 10
            )

            self.soc_publishers[truck_id] = self.create_publisher(Float32, f"/truck{truck_id}/battery_soc", 10)
            self.power_publishers[truck_id] = self.create_publisher(Float32, f"/truck{truck_id}/battery_power_w", 10)
            self.whkm_publishers[truck_id] = self.create_publisher(Float32, f"/truck{truck_id}/energy_wh_per_km", 10)

    def _velocity_cb(self, msg: Float32, truck_id: int) -> None:
        self.states[truck_id].velocity_ms = float(msg.data)

    def _accel_cb(self, msg: Float32MultiArray, truck_id: int) -> None:
        if msg.data:
            self.states[truck_id].accel_ms2 = float(msg.data[0])

    def _pitch_cb(self, msg: Float32, truck_id: int) -> None:
        self.states[truck_id].pitch_deg = float(msg.data)

    def _pose_cb(self, msg: PoseStamped, truck_id: int) -> None:
        # Keep z only for diagnostics/future grade fallback when pitch topic is absent.
        self.states[truck_id].last_z = float(msg.pose.position.z)

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_update_time = now

        stamp_sec = now.nanoseconds / 1e9
        for truck_id in range(self.truck_count):
            st = self.states[truck_id]
            power_w = self._estimate_power_w(st)
            st.battery_power_w = power_w

            delta_wh = max(0.0, power_w) * (dt / 3600.0)
            st.energy_wh += delta_wh
            st.distance_m += max(0.0, st.velocity_ms) * dt

            if self.battery_capacity_kwh > 0.0:
                delta_soc = (delta_wh / 1000.0) / self.battery_capacity_kwh * 100.0
                st.soc = max(0.0, min(100.0, st.soc - delta_soc))

            if st.distance_m > 1.0:
                st.wh_per_km = st.energy_wh / (st.distance_m / 1000.0)
            else:
                st.wh_per_km = 0.0

            self._publish_one(truck_id, st)
            self._csv_writers[truck_id].writerow(
                [f"{stamp_sec:.3f}", f"{st.velocity_ms:.4f}", f"{st.accel_ms2:.4f}", f"{st.pitch_deg:.4f}",
                 f"{st.battery_power_w:.2f}", f"{st.soc:.4f}", f"{st.wh_per_km:.4f}"]
            )

    def _estimate_power_w(self, st: TruckState) -> float:
        v = max(0.0, st.velocity_ms)
        a = st.accel_ms2
        pitch_deg = st.pitch_deg

        if self._matlab_available and self._matlab_engine is not None:
            try:
                result = self._matlab_engine.eplatoon_power_step(
                    float(v),
                    float(a),
                    float(pitch_deg),
                    float(self.mass_kg),
                    float(self.rolling_resistance),
                    float(self.cd_a),
                    float(self.air_density),
                    float(self.drive_efficiency),
                    float(self.regen_efficiency),
                    float(self.aux_power_w),
                    nargout=1,
                )
                return float(result)
            except Exception as exc:
                self.get_logger().warn(f"MATLAB call failed, switching to Python model: {exc}")
                self._matlab_available = False

        return self._python_power_step(v, a, pitch_deg)

    def _python_power_step(self, v: float, a: float, pitch_deg: float) -> float:
        g = 9.81
        pitch_rad = math.radians(pitch_deg)
        grade_force = self.mass_kg * g * math.sin(pitch_rad)
        rolling_force = self.mass_kg * g * self.rolling_resistance
        aero_force = 0.5 * self.air_density * self.cd_a * (v ** 2)
        inertial_force = self.mass_kg * a
        total_force = grade_force + rolling_force + aero_force + inertial_force
        wheel_power = total_force * v

        if wheel_power >= 0.0:
            batt_power = wheel_power / max(0.05, self.drive_efficiency) + self.aux_power_w
        else:
            batt_power = wheel_power * max(0.0, min(1.0, self.regen_efficiency)) + self.aux_power_w
        return batt_power

    def _publish_one(self, truck_id: int, st: TruckState) -> None:
        soc_msg = Float32()
        soc_msg.data = float(st.soc)
        self.soc_publishers[truck_id].publish(soc_msg)

        power_msg = Float32()
        power_msg.data = float(st.battery_power_w)
        self.power_publishers[truck_id].publish(power_msg)

        whkm_msg = Float32()
        whkm_msg.data = float(st.wh_per_km)
        self.whkm_publishers[truck_id].publish(whkm_msg)

    def destroy_node(self) -> bool:
        for f in self._csv_files.values():
            try:
                f.close()
            except Exception:
                pass
        if self._matlab_engine is not None:
            try:
                self._matlab_engine.quit()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EnergySocBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
