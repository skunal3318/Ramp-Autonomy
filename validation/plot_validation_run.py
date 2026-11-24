#!/usr/bin/env python3
"""Plot one guarded validation-run CSV as a diagnostic timeline."""
import argparse
import csv
import math
from pathlib import Path

try:
    import matplotlib
except ModuleNotFoundError as exc:
    raise SystemExit(
        'matplotlib is required; on Ubuntu run: '
        'sudo apt install python3-matplotlib'
    ) from exc

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch


REQUIRED_COLUMNS = {
    't', 'state', 'margin', 'slip',
    'odom_x', 'odom_y', 'gz_x', 'gz_y',
}
STATE_COLORS = {
    'APPROACH': '#4C78A8',
    'ASCENT': '#F58518',
    'TRANSITION': '#EEC943',
    'DESCENT': '#54A24B',
    'EXIT': '#B279A2',
    'HALT': '#E45756',
}


def _number(row, column, row_number):
    raw_value = row.get(column)
    if raw_value is None:
        raise ValueError(f'row {row_number}: missing value for {column}')
    value = raw_value.strip()
    if not value:
        return math.nan
    try:
        number = float(value)
    except ValueError as exc:
        raise ValueError(
            f'row {row_number}: {column} must be numeric, got {value!r}'
        ) from exc
    if math.isinf(number):
        raise ValueError(f'row {row_number}: {column} must be finite or blank')
    return number


def load_run(csv_path):
    with csv_path.open(newline='', encoding='utf-8-sig') as stream:
        reader = csv.DictReader(stream, strict=True)
        columns = set(reader.fieldnames or [])
        missing = sorted(REQUIRED_COLUMNS - columns)
        if missing:
            raise ValueError('missing required columns: ' + ', '.join(missing))

        rows = []
        for row_number, row in enumerate(reader, start=2):
            if row.get(None):
                raise ValueError(f'row {row_number}: too many values')
            raw_state = row.get('state')
            if raw_state is None:
                raise ValueError(f'row {row_number}: missing value for state')
            rows.append({
                't': _number(row, 't', row_number),
                'state': raw_state.strip() or 'UNKNOWN',
                'margin': _number(row, 'margin', row_number),
                'slip': _number(row, 'slip', row_number),
                'odom_x': _number(row, 'odom_x', row_number),
                'odom_y': _number(row, 'odom_y', row_number),
                'gz_x': _number(row, 'gz_x', row_number),
                'gz_y': _number(row, 'gz_y', row_number),
            })

    if len(rows) < 2:
        raise ValueError('CSV must contain at least two data rows')
    if any(not math.isfinite(row['t']) for row in rows):
        raise ValueError('time column t must contain a number on every row')
    if any(rows[i]['t'] < rows[i - 1]['t'] for i in range(1, len(rows))):
        raise ValueError('time column t must be in nondecreasing order')

    if rows[-1]['t'] == rows[0]['t']:
        raise ValueError('time column t must span a positive duration')

    signal_columns = ('margin', 'slip', 'odom_x', 'odom_y', 'gz_x', 'gz_y')
    empty = [
        column for column in signal_columns
        if not any(math.isfinite(row[column]) for row in rows)
    ]
    if empty:
        raise ValueError('columns contain no usable values: ' + ', '.join(empty))

    start = rows[0]['t']
    for row in rows:
        row['t'] -= start
    return rows


def state_segments(rows):
    segments = []
    segment_start = rows[0]['t']
    state = rows[0]['state']
    for row in rows[1:]:
        if row['state'] != state:
            segments.append((segment_start, row['t'], state))
            segment_start = row['t']
            state = row['state']

    if rows[-1]['t'] > segment_start:
        segments.append((segment_start, rows[-1]['t'], state))
    return segments


def align_position_origins(rows, truth_key, estimate_key):
    """Shift the estimate onto the truth's initial origin, preserving drift."""
    truth = [row[truth_key] for row in rows]
    estimate = [row[estimate_key] for row in rows]
    for truth_value, estimate_value in zip(truth, estimate):
        if math.isfinite(truth_value) and math.isfinite(estimate_value):
            offset = truth_value - estimate_value
            return truth, [
                value + offset if math.isfinite(value) else value
                for value in estimate
            ]
    raise ValueError(
        f'{truth_key} and {estimate_key} have no row with paired values'
    )


def plot_run(csv_path, output_path):
    rows = load_run(csv_path)
    time_s = [row['t'] for row in rows]
    truth_x, estimate_x = align_position_origins(rows, 'gz_x', 'odom_x')
    truth_y, estimate_y = align_position_origins(rows, 'gz_y', 'odom_y')

    figure, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    position_ax, margin_ax, slip_ax = axes

    position_ax.plot(time_s, truth_x,
                     label='Ground truth x', color='#1F77B4')
    position_ax.plot(time_s, estimate_x,
                     label='EKF x (initial offset removed)', color='#FF7F0E')
    position_ax.plot(time_s, truth_y,
                     label='Ground truth y', color='#1F77B4', linestyle='--')
    position_ax.plot(time_s, estimate_y,
                     label='EKF y (initial offset removed)', color='#FF7F0E',
                     linestyle='--')
    position_ax.set_ylabel('Baseline-aligned position (m)')
    position_legend = position_ax.legend(loc='upper left', ncols=2)

    margin_ax.plot(time_s, [row['margin'] for row in rows],
                   label='Stability margin', color='#2CA02C')
    margin_ax.axhline(0.15, color='#D62728', linestyle=':',
                      label='HALT threshold (0.15)')
    margin_ax.set_ylabel('Stability margin')
    margin_ax.legend(loc='upper left')

    slip_ax.plot(time_s, [row['slip'] for row in rows],
                 label='Wheel slip', color='#9467BD')
    slip_ax.axhline(0.5, color='#D62728', linestyle=':',
                    label='HALT threshold (0.5)')
    slip_ax.set_ylabel('Wheel slip')
    slip_ax.set_xlabel('Elapsed time (s)')
    slip_ax.legend(loc='upper left')

    states = []
    fallback_colors = plt.get_cmap('tab20').colors
    for start, end, state in state_segments(rows):
        if state not in states:
            states.append(state)
        color = STATE_COLORS.get(
            state,
            fallback_colors[states.index(state) % len(fallback_colors)],
        )
        alpha = 0.18 if state == 'HALT' else 0.08
        for axis in axes:
            axis.axvspan(start, end, color=color, alpha=alpha, linewidth=0)

    state_handles = [
        Patch(
            facecolor=STATE_COLORS.get(
                state,
                fallback_colors[index % len(fallback_colors)],
            ),
            alpha=0.35,
            label=state,
        )
        for index, state in enumerate(states)
    ]
    position_ax.legend(
        handles=state_handles,
        title='FSM state',
        loc='upper right',
        ncols=min(3, len(state_handles)),
    )
    position_ax.add_artist(position_legend)

    for axis in axes:
        axis.grid(True, color='#D9D9D9', linewidth=0.6, alpha=0.7)
        axis.set_axisbelow(True)

    figure.suptitle(f'Validation run: {csv_path.name}')
    figure.tight_layout()
    figure.savefig(output_path, format='png', dpi=150, bbox_inches='tight')
    plt.close(figure)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Plot ground truth, EKF, safety signals, and FSM states.'
    )
    parser.add_argument('csv_path', type=Path, help='guarded-run CSV to plot')
    parser.add_argument(
        '-o', '--output', type=Path,
        help='output PNG path (default: CSV path with a .png suffix)',
    )
    args = parser.parse_args(argv)
    output_path = args.output or args.csv_path.with_suffix('.png')
    if not output_path.suffix:
        output_path = output_path.with_suffix('.png')
    elif output_path.suffix.lower() != '.png':
        parser.error('output path must use a .png suffix')

    try:
        plot_run(args.csv_path, output_path)
    except (csv.Error, OSError, ValueError) as exc:
        parser.error(str(exc))
    print(f'wrote {output_path}')


if __name__ == '__main__':
    main()
