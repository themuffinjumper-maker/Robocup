"""
DataAnalysisCLI.py

Headless analysis tool for Robot Debug Console .rdbg recordings.

Examples:
    python DataAnalysisCLI.py summary Data/Test.rdbg
    python DataAnalysisCLI.py faults Data/Test.rdbg
    python DataAnalysisCLI.py context Data/Test.rdbg --time 42.6 --window 3
    python DataAnalysisCLI.py stats Data/Test.rdbg --signal drive.left_rpm
    python DataAnalysisCLI.py correlate Data/Test.rdbg --a drive.left_rpm --b drive.right_rpm
    python DataAnalysisCLI.py pid Data/Test.rdbg --setpoint drive.target_rpm --measured drive.left_rpm
    python DataAnalysisCLI.py anomalies Data/Test.rdbg --signal drive.left_rpm
    python DataAnalysisCLI.py report Data/Test.rdbg --output analysis_report.md

Uses only the Python standard library.
"""

from __future__ import annotations

import argparse
import json
import math
import sqlite3
import statistics
from pathlib import Path
from typing import Any


def connect(path: str | Path) -> sqlite3.Connection:
    conn = sqlite3.connect(Path(path))
    conn.row_factory = sqlite3.Row
    return conn


def table_exists(conn, name):
    return conn.execute(
        "SELECT 1 FROM sqlite_master WHERE type='table' AND name=?",
        (name,),
    ).fetchone() is not None


def metadata(conn):
    if not table_exists(conn, "metadata"):
        return {}
    return {r["key"]: r["value"] for r in conn.execute("SELECT key,value FROM metadata")}


def signal_names(conn, numeric_only=False):
    if numeric_only:
        rows = conn.execute(
            "SELECT DISTINCT signal FROM telemetry WHERE value_num IS NOT NULL ORDER BY signal"
        )
    else:
        rows = conn.execute("SELECT DISTINCT signal FROM telemetry ORDER BY signal")
    return [r[0] for r in rows]


def series(conn, signal, start=None, end=None):
    sql = """SELECT elapsed_s,wall_time,robot_time,value_num,value_text,value_type
             FROM telemetry WHERE signal=?"""
    args = [signal]
    if start is not None:
        sql += " AND elapsed_s>=?"
        args.append(start)
    if end is not None:
        sql += " AND elapsed_s<=?"
        args.append(end)
    sql += " ORDER BY elapsed_s"
    rows = []
    for r in conn.execute(sql, args):
        value = r["value_num"] if r["value_num"] is not None else r["value_text"]
        rows.append({
            "elapsed_s": r["elapsed_s"],
            "wall_time": r["wall_time"],
            "robot_time": r["robot_time"],
            "value": value,
            "value_type": r["value_type"],
        })
    return rows


def numeric_series(conn, signal, start=None, end=None):
    sql = "SELECT elapsed_s,value_num FROM telemetry WHERE signal=? AND value_num IS NOT NULL"
    args = [signal]
    if start is not None:
        sql += " AND elapsed_s>=?"
        args.append(start)
    if end is not None:
        sql += " AND elapsed_s<=?"
        args.append(end)
    sql += " ORDER BY elapsed_s"
    return [(float(r[0]), float(r[1])) for r in conn.execute(sql, args)]


def percentile(values, p):
    if not values:
        return None
    s = sorted(values)
    if len(s) == 1:
        return s[0]
    pos = (len(s)-1) * p
    lo = int(math.floor(pos)); hi = int(math.ceil(pos))
    if lo == hi:
        return s[lo]
    return s[lo] * (hi-pos) + s[hi] * (pos-lo)


def stats_for_signal(conn, signal, start=None, end=None):
    rows = numeric_series(conn, signal, start, end)
    vals = [v for _, v in rows if math.isfinite(v)]
    if not vals:
        return {"signal": signal, "samples": 0}
    times = [t for t, v in rows if math.isfinite(v)]
    duration = times[-1] - times[0] if len(times) > 1 else 0
    return {
        "signal": signal,
        "samples": len(vals),
        "min": min(vals),
        "max": max(vals),
        "mean": statistics.fmean(vals),
        "median": statistics.median(vals),
        "stdev": statistics.stdev(vals) if len(vals) > 1 else 0.0,
        "p05": percentile(vals, .05),
        "p25": percentile(vals, .25),
        "p75": percentile(vals, .75),
        "p95": percentile(vals, .95),
        "first": vals[0],
        "last": vals[-1],
        "delta": vals[-1] - vals[0],
        "duration_s": duration,
        "sample_rate_hz": ((len(vals)-1)/duration) if duration > 0 else None,
    }


def faults(conn, start=None, end=None):
    if not table_exists(conn, "faults"):
        return []
    sql = "SELECT elapsed_s,wall_time,label FROM faults WHERE 1=1"
    args = []
    if start is not None:
        sql += " AND elapsed_s>=?"; args.append(start)
    if end is not None:
        sql += " AND elapsed_s<=?"; args.append(end)
    sql += " ORDER BY elapsed_s"
    return [dict(r) for r in conn.execute(sql, args)]


def parameter_snapshots(conn):
    if not table_exists(conn, "parameter_snapshots"):
        return []
    out = []
    for r in conn.execute("SELECT elapsed_s,wall_time,snapshot_json FROM parameter_snapshots ORDER BY elapsed_s"):
        try:
            snap = json.loads(r["snapshot_json"])
        except Exception:
            snap = r["snapshot_json"]
        out.append({"elapsed_s": r["elapsed_s"], "wall_time": r["wall_time"], "values": snap})
    return out


def events(conn, start=None, end=None):
    out = []
    def bounds(sql, args):
        if start is not None:
            sql += " AND elapsed_s>=?"; args.append(start)
        if end is not None:
            sql += " AND elapsed_s<=?"; args.append(end)
        return sql, args

    if table_exists(conn, "faults"):
        sql, args = bounds("SELECT elapsed_s,wall_time,label FROM faults WHERE 1=1", [])
        for r in conn.execute(sql, args):
            out.append({"elapsed_s": r[0], "type": "fault", "label": r[2]})

    if table_exists(conn, "commands"):
        sql, args = bounds("SELECT elapsed_s,wall_time,command,arguments_json FROM commands WHERE 1=1", [])
        for r in conn.execute(sql, args):
            try: a = json.loads(r[3])
            except Exception: a = r[3]
            out.append({"elapsed_s": r[0], "type": "command", "name": r[2], "arguments": a})

    if table_exists(conn, "parameters"):
        sql, args = bounds("SELECT elapsed_s,wall_time,name,value_json FROM parameters WHERE 1=1", [])
        for r in conn.execute(sql, args):
            try: v = json.loads(r[3])
            except Exception: v = r[3]
            out.append({"elapsed_s": r[0], "type": "parameter", "name": r[2], "value": v})

    if table_exists(conn, "logs"):
        sql, args = bounds("SELECT elapsed_s,wall_time,level,message FROM logs WHERE 1=1", [])
        for r in conn.execute(sql, args):
            out.append({"elapsed_s": r[0], "type": "log", "level": r[2], "message": r[3]})

    if table_exists(conn, "states"):
        sql, args = bounds("SELECT elapsed_s,wall_time,state_json FROM states WHERE 1=1", [])
        for r in conn.execute(sql, args):
            try: state = json.loads(r[2])
            except Exception: state = r[2]
            out.append({"elapsed_s": r[0], "type": "state", "state": state})

    if table_exists(conn, "annotations"):
        sql, args = bounds("SELECT elapsed_s,wall_time,label,note FROM annotations WHERE 1=1", [])
        for r in conn.execute(sql, args):
            out.append({"elapsed_s": r[0], "type": "annotation", "label": r[2], "note": r[3]})

    return sorted(out, key=lambda x: x["elapsed_s"])


def summary(conn):
    duration = conn.execute("SELECT COALESCE(MAX(elapsed_s),0) FROM telemetry").fetchone()[0]
    count = conn.execute("SELECT COUNT(*) FROM telemetry").fetchone()[0]
    return {
        "metadata": metadata(conn),
        "duration_s": duration,
        "telemetry_rows": count,
        "signal_count": len(signal_names(conn)),
        "numeric_signals": signal_names(conn, True),
        "fault_marker_count": len(faults(conn)),
        "faults": faults(conn),
        "parameter_snapshots": parameter_snapshots(conn),
    }


def value_near(conn, signal, t):
    row = conn.execute(
        """SELECT elapsed_s,value_num,value_text FROM telemetry
           WHERE signal=? ORDER BY ABS(elapsed_s-?) LIMIT 1""",
        (signal, t),
    ).fetchone()
    if row is None:
        return None
    return {
        "elapsed_s": row[0],
        "value": row[1] if row[1] is not None else row[2],
    }


def context_at_time(conn, t, window=2.0):
    lo, hi = t-window, t+window
    return {
        "time_s": t,
        "window_s": window,
        "faults": faults(conn, lo, hi),
        "events": events(conn, lo, hi),
        "nearest_values": {s: value_near(conn, s, t) for s in signal_names(conn)},
        "numeric_stats": {
            s: stats_for_signal(conn, s, lo, hi)
            for s in signal_names(conn, True)
        },
    }


def _interp(points, t):
    if not points:
        return None
    if t <= points[0][0]:
        return points[0][1]
    if t >= points[-1][0]:
        return points[-1][1]
    lo, hi = 0, len(points)-1
    while hi-lo > 1:
        mid = (lo+hi)//2
        if points[mid][0] <= t: lo = mid
        else: hi = mid
    t0, v0 = points[lo]; t1, v1 = points[hi]
    if t1 == t0:
        return v0
    f = (t-t0)/(t1-t0)
    return v0 + f*(v1-v0)


def correlate(conn, a, b, start=None, end=None):
    aa = numeric_series(conn, a, start, end)
    bb = numeric_series(conn, b, start, end)
    if len(aa) < 3 or len(bb) < 3:
        return {"a": a, "b": b, "samples": 0, "correlation": None}
    pairs = []
    for t, av in aa:
        bv = _interp(bb, t)
        if bv is not None and math.isfinite(av) and math.isfinite(bv):
            pairs.append((av, bv))
    if len(pairs) < 3:
        return {"a": a, "b": b, "samples": len(pairs), "correlation": None}
    xs = [p[0] for p in pairs]; ys = [p[1] for p in pairs]
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    num = sum((x-mx)*(y-my) for x, y in pairs)
    den = math.sqrt(sum((x-mx)**2 for x in xs) * sum((y-my)**2 for y in ys))
    return {"a": a, "b": b, "samples": len(pairs), "correlation": num/den if den else None}


def anomalies(conn, signal, start=None, end=None):
    pts = numeric_series(conn, signal, start, end)
    if len(pts) < 8:
        return []
    vals = [v for _, v in pts]
    med = statistics.median(vals)
    absdev = [abs(v-med) for v in vals]
    mad = statistics.median(absdev)
    out = []
    if mad > 0:
        for (t, v) in pts:
            rz = 0.6745*(v-med)/mad
            if abs(rz) > 6:
                out.append({"elapsed_s": t, "value": v, "kind": "level_outlier", "score": rz})

    diffs = [(pts[i][0], pts[i][1]-pts[i-1][1], pts[i][1]) for i in range(1, len(pts))]
    if len(diffs) >= 5:
        dvals = [d for _, d, _ in diffs]
        dmed = statistics.median(dvals)
        dmad = statistics.median([abs(d-dmed) for d in dvals])
        if dmad > 0:
            for t, d, v in diffs:
                rz = 0.6745*(d-dmed)/dmad
                if abs(rz) > 8:
                    out.append({"elapsed_s": t, "value": v, "kind": "sudden_change", "score": rz})
    return sorted(out, key=lambda x: x["elapsed_s"])


def pid_metrics(conn, setpoint, measured, start=None, end=None, tolerance=0.02):
    sp = numeric_series(conn, setpoint, start, end)
    mv = numeric_series(conn, measured, start, end)
    if len(sp) < 2 or len(mv) < 3:
        return {"error": "Not enough samples."}

    t0 = max(sp[0][0], mv[0][0])
    t1 = min(sp[-1][0], mv[-1][0])
    samples = [(t, _interp(sp, t), v) for t, v in mv if t0 <= t <= t1]
    samples = [(t, s, v) for t, s, v in samples if s is not None]
    if len(samples) < 3:
        return {"error": "No overlapping samples."}

    initial_sp = samples[0][1]
    final_sp = statistics.median([s for _, s, _ in samples[-max(3, len(samples)//10):]])
    step = final_sp - initial_sp
    direction = 1 if step >= 0 else -1

    errors = [s-v for _, s, v in samples]
    abs_errors = [abs(e) for e in errors]
    sq_errors = [e*e for e in errors]

    rise_time = None
    if step != 0:
        y10 = initial_sp + 0.1*step
        y90 = initial_sp + 0.9*step
        t10 = t90 = None
        for t, _, v in samples:
            if t10 is None and direction*(v-y10) >= 0:
                t10 = t
            if t90 is None and direction*(v-y90) >= 0:
                t90 = t
                break
        if t10 is not None and t90 is not None:
            rise_time = t90-t10

    peak = max(v for _, _, v in samples) if direction > 0 else min(v for _, _, v in samples)
    overshoot = 0.0
    if step != 0:
        overshoot = max(0.0, direction*(peak-final_sp)/abs(step)*100.0)

    tol_abs = max(abs(final_sp)*tolerance, abs(step)*tolerance, 1e-12)
    settling = None
    for i, (t, _, _) in enumerate(samples):
        if all(abs(final_sp-v) <= tol_abs for _, _, v in samples[i:]):
            settling = t - samples[0][0]
            break

    iae = 0.0
    ise = 0.0
    for i in range(1, len(samples)):
        dt = samples[i][0]-samples[i-1][0]
        iae += 0.5*(abs_errors[i]+abs_errors[i-1])*dt
        ise += 0.5*(sq_errors[i]+sq_errors[i-1])*dt

    tail = errors[-max(3, len(errors)//10):]
    return {
        "setpoint_signal": setpoint,
        "measured_signal": measured,
        "samples": len(samples),
        "initial_setpoint": initial_sp,
        "final_setpoint": final_sp,
        "step_size": step,
        "rise_time_s_10_90": rise_time,
        "overshoot_percent": overshoot,
        "settling_time_s": settling,
        "steady_state_error": statistics.fmean(tail),
        "mae": statistics.fmean(abs_errors),
        "rmse": math.sqrt(statistics.fmean(sq_errors)),
        "iae": iae,
        "ise": ise,
    }


def full_export(conn):
    return {
        "summary": summary(conn),
        "statistics": {s: stats_for_signal(conn, s) for s in signal_names(conn, True)},
        "events": events(conn),
    }


def markdown_report(conn, path):
    s = summary(conn)
    meta = s["metadata"]
    lines = [
        "# Robot Recording Analysis",
        "",
        f"- Duration: **{s['duration_s']:.3f} s**",
        f"- Telemetry rows: **{s['telemetry_rows']}**",
        f"- Signals: **{s['signal_count']}**",
        f"- Fault markers: **{s['fault_marker_count']}**",
        f"- Test name: **{meta.get('test_name','—')}**",
        f"- Git branch: **{meta.get('git_branch','—')}**",
        f"- Git commit: **{meta.get('git_commit','—')}**",
        "",
        "## Faults",
    ]
    if s["faults"]:
        for f in s["faults"]:
            lines.append(f"- {f['elapsed_s']:.3f} s — {f['label']}")
    else:
        lines.append("- None")

    lines += ["", "## Numeric signal statistics", ""]
    for sig in s["numeric_signals"]:
        st = stats_for_signal(conn, sig)
        lines.append(
            f"- **{sig}**: n={st.get('samples',0)}, min={st.get('min')}, max={st.get('max')}, "
            f"mean={st.get('mean')}, stdev={st.get('stdev')}"
        )

    lines += ["", "## Candidate anomalies", ""]
    count = 0
    for sig in s["numeric_signals"]:
        aa = anomalies(conn, sig)
        if aa:
            count += len(aa)
            lines.append(f"### {sig}")
            for a in aa[:20]:
                lines.append(f"- {a['elapsed_s']:.3f} s — {a['kind']} (score {a['score']:.2f})")
    if not count:
        lines.append("No robust outliers detected.")

    Path(path).write_text("\n".join(lines), encoding="utf-8")
    return {"output": str(path), "anomaly_count": count}


def dump(obj):
    print(json.dumps(obj, indent=2, ensure_ascii=False, default=str))


def parser():
    p = argparse.ArgumentParser(description="Analyse Robot Debug .rdbg recordings")
    sub = p.add_subparsers(dest="command", required=True)

    def filecmd(name):
        q = sub.add_parser(name)
        q.add_argument("file")
        return q

    filecmd("summary")
    q = filecmd("signals"); q.add_argument("--numeric-only", action="store_true")
    q = filecmd("faults"); q.add_argument("--start", type=float); q.add_argument("--end", type=float)
    q = filecmd("events"); q.add_argument("--start", type=float); q.add_argument("--end", type=float)
    q = filecmd("context"); q.add_argument("--time", type=float, required=True); q.add_argument("--window", type=float, default=2.0)
    q = filecmd("stats"); q.add_argument("--signal", required=True); q.add_argument("--start", type=float); q.add_argument("--end", type=float)
    q = filecmd("range"); q.add_argument("--signal", required=True); q.add_argument("--start", type=float); q.add_argument("--end", type=float)
    q = filecmd("correlate"); q.add_argument("--a", required=True); q.add_argument("--b", required=True); q.add_argument("--start", type=float); q.add_argument("--end", type=float)
    q = filecmd("pid"); q.add_argument("--setpoint", required=True); q.add_argument("--measured", required=True); q.add_argument("--start", type=float); q.add_argument("--end", type=float); q.add_argument("--tolerance", type=float, default=.02)
    q = filecmd("anomalies"); q.add_argument("--signal", required=True); q.add_argument("--start", type=float); q.add_argument("--end", type=float)
    q = filecmd("export-json"); q.add_argument("--output", required=True)
    q = filecmd("report"); q.add_argument("--output", required=True)
    return p


def main():
    args = parser().parse_args()
    conn = connect(args.file)
    try:
        if args.command == "summary":
            dump(summary(conn))
        elif args.command == "signals":
            dump(signal_names(conn, args.numeric_only))
        elif args.command == "faults":
            dump(faults(conn, args.start, args.end))
        elif args.command == "events":
            dump(events(conn, args.start, args.end))
        elif args.command == "context":
            dump(context_at_time(conn, args.time, args.window))
        elif args.command == "stats":
            dump(stats_for_signal(conn, args.signal, args.start, args.end))
        elif args.command == "range":
            dump(series(conn, args.signal, args.start, args.end))
        elif args.command == "correlate":
            dump(correlate(conn, args.a, args.b, args.start, args.end))
        elif args.command == "pid":
            dump(pid_metrics(conn, args.setpoint, args.measured, args.start, args.end, args.tolerance))
        elif args.command == "anomalies":
            dump(anomalies(conn, args.signal, args.start, args.end))
        elif args.command == "export-json":
            data = full_export(conn)
            Path(args.output).write_text(json.dumps(data, indent=2, default=str), encoding="utf-8")
            dump({"output": args.output})
        elif args.command == "report":
            dump(markdown_report(conn, args.output))
    finally:
        conn.close()


if __name__ == "__main__":
    main()
