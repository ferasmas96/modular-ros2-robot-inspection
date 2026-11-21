#!/usr/bin/env python3
"""Re-render saved diagnostic CSV data; does not launch ROS or modify raw data."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--version', default='v01')
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
summary = json.loads((root/'data/summary.json').read_text())
data = np.genfromtxt(root/'data/aligned_poses.csv', delimiter=',', names=True)
ref = np.genfromtxt(root/'data/reference_for_plot.csv', delimiter=',', names=True)
with (root/'data/commands_motion_window.csv').open() as src:
    commands = list(csv.DictReader(src))
plt.rcParams.update({'font.size': 10, 'axes.grid': True, 'grid.alpha': .22,
                     'svg.fonttype': 'none', 'savefig.facecolor': 'white'})


def save(fig, subject):
    for extension in ['png', 'svg']:
        path = root/f'2026-11-21_SLAM-004C_{subject}_{args.version}.{extension}'
        if path.exists():
            raise FileExistsError(path)
        fig.savefig(path, dpi=180, bbox_inches='tight')
    plt.close(fig)


fig, ax = plt.subplots(figsize=(7.1, 5.8))
ax.add_patch(Circle(summary['spawn_xy_m'], summary['region_radius_m'], fill=False,
                    color='#aa7417', linestyle='--', linewidth=1.3, label='Reviewed 1.25 m center region'))
ax.plot(ref['x_m'], ref['y_m'], ':', color='#b44f4f', linewidth=2,
        label='Gazebo reference, including shutdown tail')
ax.plot(data['ref_x_m'], data['ref_y_m'], color='#333333', linewidth=1.5,
        label='Gazebo at GLIM timestamps')
ax.plot(data['glim_x_m'], data['glim_y_m'], color='#2076ad', linewidth=1.6,
        label='Live GLIM base pose, initial SE(3) alignment')
ax.scatter(*summary['spawn_xy_m'], color='#35784a', s=30, zorder=5, label='Start')
ax.scatter(ref['x_m'][-1], ref['y_m'][-1], marker='x', color='#b12828', s=55,
           zorder=6, label='Last simulator reference')
ax.set(xlabel='Warehouse world X (m)', ylabel='Warehouse world Y (m)',
       title='SLAM-004C: captured base trajectories\nExperiment aborted at the region limit')
ax.set_aspect('equal', adjustable='box')
ax.legend(loc='upper left', fontsize=8)
fig.text(.12, -.01, 'Offline alignment only; no TF broadcast. Dotted tail has no matching GLIM poses.', fontsize=8)
save(fig, 'aligned-base-trajectories')

t0 = summary['motion_start_s']
x = data['simulation_seconds'] - t0
select = (x >= -2) & (x <= summary['motion_end_s'] - t0 + 2)
fig, axes = plt.subplots(2, 1, figsize=(8.3, 5.4), sharex=True)
axes[0].plot(x[select], 1000*data['translation_error_m'][select], color='#2076ad')
axes[1].plot(x[select], np.rad2deg(data['orientation_error_rad'][select]), color='#75529c')
for ax in axes:
    ax.axvline(summary['abort_clock_s'] - t0, color='#b12828', linestyle='--',
               linewidth=1, label='Region-stop request')
    ax.set_xlim(-2, summary['motion_end_s'] - t0 + 1)
axes[0].set(ylabel='Translation error (mm)',
            title=f"SLAM-004C: synchronized base-pose errors\nCaptured motion ATE RMSE: {1000*summary['ate_motion_rmse_m']:.2f} mm")
axes[1].set(xlabel='Simulation seconds from first nonzero robot command', ylabel='Orientation error (deg)')
axes[0].legend(fontsize=8)
fig.tight_layout()
save(fig, 'synchronized-pose-errors')

fig, axes = plt.subplots(2, 2, figsize=(10, 5.5), sharex=True)
for topic, color, label in [('/a300_00000/manual/raw_cmd_vel', '#858585', 'Raw keyboard'),
                            ('/a300_00000/cmd_vel', '#2076ad', 'Guard / mux input'),
                            ('/a300_00000/platform/cmd_vel', '#d08224', 'Controller input')]:
    rows = [r for r in commands if r['topic'] == topic]
    raw = topic.endswith('/manual/raw_cmd_vel')
    time_key = 'bag_receipt_seconds' if raw else 'sensor_seconds'
    times = np.array([float(r[time_key])-t0 for r in rows])
    for column, field in enumerate(['linear_x_m_s', 'angular_z_rad_s']):
        values = [float(r[field]) for r in rows]
        axes[0 if raw else 1, column].step(times, values, where='post', color=color,
                                            linewidth=1.1, linestyle='-' if raw or 'Guard' in label else '--',
                                            label=label)
for j in range(2):
    axes[1, j].axhline(.1, color='#ad3737', linestyle=':', label='Hard limits ±0.10')
    axes[1, j].axhline(-.1, color='#ad3737', linestyle=':')
    axes[1, j].set_ylim(-.12, .12)
    axes[1, j].set_xlabel('Simulation seconds from first nonzero robot command')
    axes[1, j].legend(fontsize=7, loc='upper left')
    for ax in axes[:, j]:
        ax.set_xlim(-2, summary['motion_end_s']-t0+1)
        ax.axvline(summary['abort_clock_s']-t0, color='#b12828', linestyle='--', linewidth=.8)
axes[0, 0].set(title='Raw keyboard forward command', ylabel='Linear X (m/s)')
axes[0, 1].set(title='Raw keyboard turn command', ylabel='Angular Z (rad/s)')
axes[1, 0].set_ylabel('Delivered linear X (m/s)')
axes[1, 1].set_ylabel('Delivered angular Z (rad/s)')
fig.suptitle('SLAM-004C: measured command limiting during the captured motion', y=1.02)
fig.tight_layout()
fig.text(.06, -.03, 'Raw headers are zero: raw traces use bag receipt simulation time; delivered traces use header time.', fontsize=8)
save(fig, 'actual-command-limits')
