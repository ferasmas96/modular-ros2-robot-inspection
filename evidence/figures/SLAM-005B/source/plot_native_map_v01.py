#!/usr/bin/env python3
"""Reproduce a stationary native RTAB-Map cloud preview, no ROS initialization."""
from pathlib import Path
import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    root = Path(__file__).resolve().parents[1]
    with (root / 'data/native_cloud.csv').open() as f:
        points = np.array([[float(v) for v in row] for row in list(csv.reader(f))[1:]])
    colors = points[:, 3:6] / 255
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for ax, horizontal, vertical, title in [
            (axes[0], 0, 1, 'Top view'), (axes[1], 0, 2, 'Side view')]:
        ax.scatter(points[:, horizontal], points[:, vertical], c=colors, s=5,
                   edgecolors='none', rasterized=True)
        ax.plot(0, 0, 'r+', markersize=10, markeredgewidth=1.6, label='Initial base origin')
        ax.set(xlabel='X [m]', ylabel=['X [m]', 'Y [m]', 'Z [m]'][vertical], title=title)
        ax.set_aspect('equal', adjustable='box')
        ax.grid(alpha=0.2)
        ax.set_facecolor('#f1f3f5')
    axes[0].legend(loc='upper right', fontsize=8)
    fig.suptitle('SLAM-005B · stationary native RGB-D map preview', fontsize=14)
    fig.text(0.5, 0.035,
             '2,411 finite points · frame: rtabmap_map · one stationary viewpoint\n'
             'No moving-SLAM accuracy or GLIM/world alignment established',
             ha='center', fontsize=9)
    fig.tight_layout(rect=(0, 0.13, 1, 0.92))
    stem = root / '2026-11-21_SLAM-005B_stationary-native-map_v01'
    for extension in ['png', 'svg']:
        target = stem.with_suffix('.' + extension)
        if target.exists():
            raise FileExistsError(target)
        fig.savefig(target, dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    main()
