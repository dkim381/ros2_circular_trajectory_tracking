import csv
import math
import os
import sys
from typing import List, Dict, Tuple


def load_csv_meta_and_series(path: str) -> Tuple[Dict[str, float], List[float], List[float], List[float]]:
    times, xs, ys = [], [], []
    meta = {"cx": None, "cy": None, "R": None, "omega": None, "t0_sec": None}

    with open(path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get("time_sec", "") == "":
                continue

            times.append(float(row["time_sec"]))
            xs.append(float(row["x"]))
            ys.append(float(row["y"]))

            if meta["cx"] is None and row.get("cx", "") != "":
                meta["cx"] = float(row["cx"])
                meta["cy"] = float(row["cy"])
                meta["R"] = float(row["R"])
                meta["omega"] = float(row["omega"])
                meta["t0_sec"] = float(row["t0_sec"])

    if meta["cx"] is None:
        raise RuntimeError(f"Meta not found in {path}")

    return meta, times, xs, ys


def align_time(times: List[float], t0: float) -> Tuple[List[float], str]:
    t_min, t_max = min(times), max(times)
    if (t_min - 1.0) <= t0 <= (t_max + 1.0):
        return [t - t0 for t in times], "ABS"
    return [t - times[0] for t in times], "ELAPSED"


def ref_circle(cx: float, cy: float, R: float, omega: float, tref: List[float]):
    xref = [cx + R * math.cos(omega * t) for t in tref]
    yref = [cy + R * math.sin(omega * t) for t in tref]
    return xref, yref


def compute_error(xs, ys, xref, yref):
    return [math.hypot(rx - x, ry - y) for x, y, rx, ry in zip(xs, ys, xref, yref)]


def rmse(values: List[float]) -> float:
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else float("nan")


def mean_last_window(tref: List[float], values: List[float], window_sec: float = 20.0) -> float:
    if not tref:
        return float("nan")
    t_end = tref[-1]
    t_start = max(tref[0], t_end - window_sec)
    vals = [v for (t, v) in zip(tref, values) if t >= t_start]
    return sum(vals) / len(vals) if vals else float("nan")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 make_results_table.py <odom_data_*.csv> [more_csv ...]")
        sys.exit(1)

    files = sys.argv[1:]

    rows = []
    for path in files:
        meta, times, xs, ys = load_csv_meta_and_series(path)
        tref, mode = align_time(times, meta["t0_sec"])
        xref, yref = ref_circle(meta["cx"], meta["cy"], meta["R"], meta["omega"], tref)
        e = compute_error(xs, ys, xref, yref)

        rows.append({
            "file": os.path.basename(path),
            "R": meta["R"],
            "omega": meta["omega"],
            "t_align": mode,
            "rmse": rmse(e),
            "max": max(e) if e else float("nan"),
            "mean_last20": mean_last_window(tref, e, 20.0),
            "samples": len(e),
        })

    # Markdown table
    print("| file | R | omega | RMSE (m) | mean last 20s (m) | max (m) | N |")
    print("|---|---:|---:|---:|---:|---:|---:|")
    for r in rows:
        print(
            f"| {r['file']} | {r['R']:.2f} | {r['omega']:.3f} | "
            f"{r['rmse']:.4f} | {r['mean_last20']:.4f} | {r['max']:.4f} | {r['samples']} |"
        )


if __name__ == "__main__":
    main()
