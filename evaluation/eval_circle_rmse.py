import csv
import math
import sys


def rmse(values):
    if not values:
        return float("nan")
    return math.sqrt(sum(v * v for v in values) / len(values))


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 eval_circle_rmse.py <odom_data_*.csv>")
        sys.exit(1)

    path = sys.argv[1]

    times = []
    xs = []
    ys = []

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

            if cx is None and row.get("cx", "") != "":
                cx = float(row["cx"])
                cy = float(row["cy"])
                R = float(row["R"])
                omega = float(row["omega"])
                t0 = float(row["t0_sec"])

    if cx is None:
        print("ERROR: ref meta not found in CSV.")
        sys.exit(2)

    if len(times) < 2:
        print("ERROR: not enough samples.")
        sys.exit(3)

    t_min = min(times)
    t_max = max(times)

    if (t0 is not None) and ((t_min - 1.0) <= t0 <= (t_max + 1.0)):
        tref_list = [t - t0 for t in times]
        mode = "ABS(time_sec - t0)"
    else:
        t0_local = times[0]
        tref_list = [t - t0_local for t in times]
        mode = "ELAPSED(time_sec - time_sec[0])"

    e_list = []
    for tref, x, y in zip(tref_list, xs, ys):
        xd = cx + R * math.cos(omega * tref)
        yd = cy + R * math.sin(omega * tref)
        e_list.append(math.hypot(xd - x, yd - y))

    rmse_xy = rmse(e_list)
    max_e = max(e_list)

    t_end = tref_list[-1]
    t_ss_start = max(tref_list[0], t_end - 20.0)
    ss_errors = [e for (tr, e) in zip(tref_list, e_list) if tr >= t_ss_start]
    mean_ss = sum(ss_errors) / len(ss_errors) if ss_errors else float("nan")

    print(f"File: {path}")
    print(f"Meta: cx={cx:.3f}, cy={cy:.3f}, R={R:.3f}, omega={omega:.5f}, t0={t0:.3f}")
    print(f"TimeAlign: {mode} | time_sec range [{t_min:.3f}, {t_max:.3f}]")
    print(f"RMSE(|e|): {rmse_xy:.4f} m")
    print(f"Max(|e|):  {max_e:.4f} m")
    print(f"Mean(|e|) last ~20s: {mean_ss:.4f} m")
    print(f"Samples: {len(e_list)}")


if __name__ == "__main__":
    main()
