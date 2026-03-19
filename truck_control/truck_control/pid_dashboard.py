import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class PIDDashboard(Node):
    def __init__(self):
        super().__init__('lateral_pid_dashboard')
        self._closing = False
        self._after_id = None
        self.publisher = self.create_publisher(Float32MultiArray, '/lateral_pid_tuning', 10)

        self.root = tk.Tk()
        self.root.title("Lateral PID Dashboard")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.resizable(False, False)

        self.selected_truck = tk.StringVar(value='1')
        self.kp_var = tk.DoubleVar(value=0.9)
        self.ki_var = tk.DoubleVar(value=0.0)
        self.kd_var = tk.DoubleVar(value=0.2)
        self.status_var = tk.StringVar(value="준비됨")

        self._build_ui()
        self._after_id = self.root.after(50, self.spin_ros_once)

    def _build_ui(self):
        container = ttk.Frame(self.root, padding=16)
        container.grid(row=0, column=0, sticky="nsew")

        ttk.Label(container, text="대상 차량").grid(row=0, column=0, sticky="w")
        truck_select = ttk.Combobox(
            container,
            textvariable=self.selected_truck,
            state="readonly",
            values=("all", "0", "1", "2"),
            width=10
        )
        truck_select.grid(row=0, column=1, sticky="ew", pady=(0, 12))
        truck_select.bind("<<ComboboxSelected>>", lambda _event: self.publish_pid_values(reset_state=False))

        self._create_slider(container, "Kp", self.kp_var, 1, 0.0, 3.0, 0.01)
        self._create_slider(container, "Ki", self.ki_var, 2, 0.0, 1.0, 0.001)
        self._create_slider(container, "Kd", self.kd_var, 3, 0.0, 5.0, 0.01)

        button_row = ttk.Frame(container)
        button_row.grid(row=4, column=0, columnspan=3, sticky="ew", pady=(12, 8))

        ttk.Button(
            button_row,
            text="현재 값 적용",
            command=lambda: self.publish_pid_values(reset_state=False)
        ).grid(row=0, column=0, padx=(0, 8))
        ttk.Button(
            button_row,
            text="적분 초기화 후 적용",
            command=lambda: self.publish_pid_values(reset_state=True)
        ).grid(row=0, column=1)

        ttk.Label(
            container,
            text="횡방향 조향 PID를 실시간으로 조정합니다."
        ).grid(row=5, column=0, columnspan=3, sticky="w", pady=(4, 4))

        ttk.Label(container, textvariable=self.status_var).grid(
            row=6, column=0, columnspan=3, sticky="w"
        )

    def _create_slider(self, parent, label, variable, row, start, end, resolution):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w")

        scale = tk.Scale(
            parent,
            from_=start,
            to=end,
            orient=tk.HORIZONTAL,
            resolution=resolution,
            variable=variable,
            length=280,
            command=lambda _value: self.publish_pid_values(reset_state=False)
        )
        scale.grid(row=row, column=1, sticky="ew")

        value_label = ttk.Label(parent, width=8, text=f"{variable.get():.3f}")
        value_label.grid(row=row, column=2, padx=(8, 0))

        def refresh_label(*_args):
            value_label.config(text=f"{variable.get():.3f}")

        variable.trace_add("write", refresh_label)

    def publish_pid_values(self, reset_state=False):
        target = self.selected_truck.get()
        truck_id = -1 if target == 'all' else int(target)

        msg = Float32MultiArray()
        msg.data = [
            float(truck_id),
            float(self.kp_var.get()),
            float(self.ki_var.get()),
            float(self.kd_var.get()),
            1.0 if reset_state else 0.0,
        ]
        self.publisher.publish(msg)

        label = "all" if truck_id == -1 else f"truck{truck_id}"
        self.status_var.set(
            f"{label} 적용: Kp={self.kp_var.get():.3f}, Ki={self.ki_var.get():.3f}, Kd={self.kd_var.get():.3f}"
        )

    def spin_ros_once(self):
        if self._closing:
            return
        if rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self._after_id = self.root.after(50, self.spin_ros_once)

    def on_close(self):
        if self._closing:
            return
        self._closing = True
        if self._after_id is not None:
            try:
                self.root.after_cancel(self._after_id)
            except Exception:
                pass
            self._after_id = None
        self.destroy_node()
        try:
            self.root.quit()
            self.root.destroy()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    dashboard = PIDDashboard()
    try:
        dashboard.run()
    except KeyboardInterrupt:
        dashboard.on_close()


if __name__ == '__main__':
    main()
