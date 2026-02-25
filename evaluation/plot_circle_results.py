import csv
import math
import os
import sys
from typing import List, Tuple

import matplotlib.pyplot as plt


def load_csv(path: str):
    times: List[float] = []
    xs: List[float] = []
    ys: List[float] = []

    cx = cy = R = omega = t0 = None

    with open(path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get("time_sec", "") == "":
                continue

            t = float(row["time_sec"])
            x = float(row["x"])
            y = float(row["y"])

            times.append(t)
            xs.append(x)
            ys.append(y)

            # latch meta once it appears
            if cx is None and row.get("cx", "") != "":
                cx = float(row["cx"])
                cy = float(row["cy"])
                R = float(row["R"])
                omega = float(row["omega"])
                t0 = float(row["t0_sec"])

    if cx is None:
        raise RuntimeError("ref meta (cx,cy,R,omega,t0_sec) not found in CSV.")

    if len(times) < 2:
        raise RuntimeError("not enough samples in CSV.")

    return times, xs, ys, cx, cy, R, omega, t0


def align_time(times: List[float], t0: float) -> Tuple[List[float], str]:
    t_min = min(times)
    t_max = max(times)

    # If t0 lies within the time range, treat time_sec as absolute sim time and align by t0.
    # Otherwise, treat time_sec as elapsed and align by the first sample.
    if (t_min - 1.0) <= t0 <= (t_max + 1.0):
        tref = [t - t0 for t in times]
        mode = "ABS(time_sec - t0)"
    else:
        t0_local = times[0]
        tref = [t - t0_local for t in times]
        mode = "ELAPSED(time_sec - time_sec[0])"

    return tref, mode


def ref_circle(cx: float, cy: float, R: float, omega: float, tref: List[float]):
    xref = [cx + R * math.cos(omega * t) for t in tref]
    yref = [cy + R * math.sin(omega * t) for t in tref]
    return xref, yref


def compute_error(xs, ys, xref, yref):
    e = [math.hypot(rx - x, ry - y) for x, y, rx, ry in zip(xs, ys, xref, yref)]
    return e


def rmse(values: List[float]) -> float:
    if not values:
        return float("nan")
    return math.sqrt(sum(v * v for v in values) / len(values))


def mean_last_window(tref: List[float], values: List[float], window_sec: float = 20.0) -> float:
    if not tref or not values:
        return float("nan")
    t_end = tref[-1]
    t_start = max(tref[0], t_end - window_sec)
    vals = [v for (t, v) in zip(tref, values) if t >= t_start]
    if not vals:
        return float("nan")
    return sum(vals) / len(vals)


def ensure_dir(d: str):
    os.makedirs(d, exist_ok=True)


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_circle_results.py <odom_data_*.csv> [out_dir]")
        sys.exit(1)

    csv_path = sys.argv[1]
    out_dir = sys.argv[2] if len(sys.argv) >= 3 else "results"
    ensure_dir(out_dir)

    times, xs, ys, cx, cy, R, omega, t0 = load_csv(csv_path)
    tref, mode = align_time(times, t0)
    xref, yref = ref_circle(cx, cy, R, omega, tref)
    e = compute_error(xs, ys, xref, yref)

    rmse_e = rmse(e)
    max_e = max(e) if e else float("nan")
    mean_ss = mean_last_window(tref, e, window_sec=20.0)

    base = os.path.splitext(os.path.basename(csv_path))[0]
    traj_png = os.path.join(out_dir, f"{base}_trajectory.png")
    err_png = os.path.join(out_dir, f"{base}_error.png")

    # ---- Plot 1: Trajectory overlay ----
    plt.figure()
    plt.plot(xref, yref, linestyle="--", label="reference")
    plt.plot(xs, ys, label="actual")
    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title(f"Trajectory: {base}\nR={R:.2f}, omega={omega:.3f} | {mode}")
    plt.legend()
    plt.tight_layout()
    plt.savefig(traj_png, dpi=200)
    plt.close()

    # ---- Plot 2: Error vs time ----
    plt.figure()
    plt.plot(tref, e)
    plt.xlabel("t [s] (aligned)")
    plt.ylabel("|e| [m]")
    plt.title(
        f"Tracking Error: {base}\nRMSE={rmse_e:.3f} m, mean(last20s)={mean_ss:.3f} m, max={max_e:.3f} m"
    )
    plt.tight_layout()
    plt.savefig(err_png, dpi=200)
    plt.close()

    print(f"File: {csv_path}")
    print(f"Meta: cx={cx:.3f}, cy={cy:.3f}, R={R:.3f}, omega={omega:.5f}, t0={t0:.3f}")
    print(f"TimeAlign: {mode} | time_sec range [{min(times):.3f}, {max(times):.3f}]")
    print(f"RMSE(|e|): {rmse_e:.4f} m")
    print(f"Max(|e|):  {max_e:.4f} m")
    print(f"Mean(|e|) last ~20s: {mean_ss:.4f} m")
    print(f"Samples: {len(e)}")
    print(f"Saved: {traj_png}")
    print(f"Saved: {err_png}")


if __name__ == "__main__":
    main()
