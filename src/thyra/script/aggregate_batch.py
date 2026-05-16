#!/usr/bin/env python3
"""
aggregate_batch.py
------------------
Reads a master_summary_*.csv produced by start_batch_sim.sh and writes
per-combination (mode × scenario) statistics to aggregate_batch_*.csv.

Usage:
  ros2 run thyra aggregate_batch.py <input.csv> [<output.csv>]
  python3 aggregate_batch.py <input.csv> [<output.csv>]

If <output.csv> is omitted, the output filename is derived from the input
by replacing 'master_summary_' with 'aggregate_batch_' so the timestamp
suffix matches.

Statistics computed per group, per metric (linear_error_m,
rotation_error_deg, engagement_s):
  n           — number of successful runs (excludes STARTUP_FAIL / TIMEOUT)
  mean, std   — sample mean and standard deviation
  min, max    — extremes

The script is dependency-free (uses only csv + math + statistics from
the stdlib) so it runs anywhere the rest of the ROS stack runs.
"""

import csv
import math
import os
import statistics
import sys


METRIC_COLS = ('linear_error_m', 'rotation_error_deg', 'engagement_s')
KEY_COLS    = ('mode', 'scenario')
SENTINELS   = ('STARTUP_FAIL', 'TIMEOUT', 'N/A', '')


def _coerce(value):
    """Return float(value) or None if value is a sentinel/empty/non-numeric."""
    if value is None:
        return None
    s = str(value).strip()
    if s in SENTINELS:
        return None
    try:
        v = float(s)
    except ValueError:
        return None
    if math.isnan(v) or math.isinf(v):
        return None
    return v


def _stats(values):
    """Return (n, mean, std, min, max) for a list of floats. Empty → ('0','','','','')."""
    vals = [v for v in values if v is not None]
    if not vals:
        return (0, None, None, None, None)
    n = len(vals)
    mean = statistics.fmean(vals)
    std  = statistics.stdev(vals) if n > 1 else 0.0
    return (n, mean, std, min(vals), max(vals))


def _fmt(v, places=4):
    return '' if v is None else f'{v:.{places}f}'


def aggregate(input_path, output_path):
    if not os.path.isfile(input_path):
        print(f'error: input not found: {input_path}', file=sys.stderr)
        return 1

    # ── Read rows, group by (mode, scenario) ────────────────────────
    groups = {}      # key → list of dicts with float metrics (or None)
    raw_count = 0
    with open(input_path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            raw_count += 1
            key = tuple(row.get(c, '') for c in KEY_COLS)
            metrics = {m: _coerce(row.get(m)) for m in METRIC_COLS}
            groups.setdefault(key, []).append(metrics)

    if not groups:
        print(f'error: no rows in {input_path}', file=sys.stderr)
        return 1

    # ── Compute statistics ──────────────────────────────────────────
    rows_out = []
    for key, runs in sorted(groups.items()):
        mode, scenario = key
        total_runs = len(runs)
        record = {'mode': mode, 'scenario': scenario,
                  'total_runs': total_runs}
        for m in METRIC_COLS:
            n, mean, std, mn, mx = _stats([r[m] for r in runs])
            record[f'{m}_n']    = n
            record[f'{m}_mean'] = mean
            record[f'{m}_std']  = std
            record[f'{m}_min']  = mn
            record[f'{m}_max']  = mx
        rows_out.append(record)

    # ── Write aggregate CSV ─────────────────────────────────────────
    fieldnames = ['mode', 'scenario', 'total_runs']
    for m in METRIC_COLS:
        for suffix in ('n', 'mean', 'std', 'min', 'max'):
            fieldnames.append(f'{m}_{suffix}')

    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(fieldnames)
        for r in rows_out:
            writer.writerow([
                r['mode'], r['scenario'], r['total_runs'],
                *(
                    r[f'{m}_n'] if suffix == 'n' else _fmt(r[f'{m}_{suffix}'])
                    for m in METRIC_COLS
                    for suffix in ('n', 'mean', 'std', 'min', 'max')
                )
            ])

    # ── Print pretty table to stdout ────────────────────────────────
    print(f'\nAggregated {raw_count} rows -> {len(rows_out)} combinations')
    print(f'Input : {input_path}')
    print(f'Output: {output_path}\n')

    header = f'{"mode":<8}{"scenario":<14}{"n":>4}   '
    header += f'{"lin_err mean ± std":<22}{"rot_err mean ± std":<22}{"engage mean ± std":<22}'
    print(header)
    print('-' * len(header))
    for r in rows_out:
        lin = (f'{_fmt(r["linear_error_m_mean"], 3)} ± '
               f'{_fmt(r["linear_error_m_std"], 3)} m')
        rot = (f'{_fmt(r["rotation_error_deg_mean"], 2)} ± '
               f'{_fmt(r["rotation_error_deg_std"], 2)}°')
        eng = (f'{_fmt(r["engagement_s_mean"], 2)} ± '
               f'{_fmt(r["engagement_s_std"], 2)} s')
        n_total = r['total_runs']
        n_lin   = r['linear_error_m_n']
        n_str   = f'{n_lin}/{n_total}'
        line = f'{r["mode"]:<8}{r["scenario"]:<14}{n_str:>4}   '
        line += f'{lin:<22}{rot:<22}{eng:<22}'
        print(line)
    print()
    return 0


def main():
    if len(sys.argv) < 2:
        print(__doc__, file=sys.stderr)
        return 2
    input_path = sys.argv[1]
    if len(sys.argv) >= 3:
        output_path = sys.argv[2]
    else:
        base = os.path.basename(input_path)
        dirn = os.path.dirname(input_path) or '.'
        out_base = base.replace('master_summary_', 'aggregate_batch_')
        if out_base == base:
            out_base = 'aggregate_batch_' + base
        output_path = os.path.join(dirn, out_base)
    return aggregate(input_path, output_path)


if __name__ == '__main__':
    sys.exit(main())
