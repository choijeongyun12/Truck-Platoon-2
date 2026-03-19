import argparse
import csv
import os
from typing import List, Tuple

import matplotlib.pyplot as plt


def _read_soc_csv(path: str) -> Tuple[List[float], List[float]]:
    times: List[float] = []
    socs: List[float] = []
    with open(path, "r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                times.append(float(row["time_sec"]))
                socs.append(float(row["soc"]))
            except (KeyError, TypeError, ValueError):
                continue
    return times, socs


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Plot SOC over time for truck0/1/2 from energy CSV logs."
    )
    parser.add_argument(
        "--log-dir",
        default="/home/tmo/ros2_ws/log/energy",
        help="Directory containing truck*_energy.csv logs.",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Output image path. Defaults to <log-dir>/soc_vs_time.png",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display the plot window after saving.",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    output_path = args.output or os.path.join(args.log_dir, "soc_vs_time.png")

    truck_ids = [0, 1, 2]
    truck_data = {}
    global_start = None

    for truck_id in truck_ids:
        path = os.path.join(args.log_dir, f"truck{truck_id}_energy.csv")
        if not os.path.exists(path):
            raise FileNotFoundError(f"Missing log file: {path}")

        times, socs = _read_soc_csv(path)
        if not times:
            raise ValueError(f"No valid rows found in: {path}")

        truck_data[truck_id] = (times, socs)
        local_start = min(times)
        global_start = local_start if global_start is None else min(global_start, local_start)

    plt.figure(figsize=(10, 6))
    for truck_id in truck_ids:
        times, socs = truck_data[truck_id]
        rel_times_min = [(t - global_start) / 60.0 for t in times]
        plt.plot(rel_times_min, socs, linewidth=2, label=f"truck {truck_id}")

    plt.title("SOC Over Time")
    plt.xlabel("Elapsed Time (min)")
    plt.ylabel("SOC (%)")
    plt.grid(True, linestyle="--", alpha=0.4)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=180)
    print(f"Saved SOC plot: {output_path}")

    if args.show:
        plt.show()
    else:
        plt.close()


if __name__ == "__main__":
    main()
