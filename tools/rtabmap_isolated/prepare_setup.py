#!/usr/bin/python3
"""Copy a private validated setup and add only the approved RGB-D extension."""
import argparse
from pathlib import Path
import shutil


def prepare(source, destination):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if destination.exists() or source in destination.parents:
        raise ValueError('Destination must be new and outside the original setup')
    if not (source / 'robot.urdf.xacro').is_file():
        raise ValueError('No baseline robot Xacro')
    shutil.copytree(source, destination, symlinks=True, ignore=shutil.ignore_patterns('__pycache__'))
    for p in destination.rglob('*.py'):
        text = p.read_text()
        updated = text.replace(str(source), str(destination))
        if text != updated:
            if p.is_symlink():
                p.unlink()
            p.write_text(updated)
    p = destination / 'robot.urdf.xacro'
    original = p.read_text()
    if original.count('</robot>') != 1:
        raise ValueError('Expected one robot closing element')
    addition = ('\n  <!-- SLAM-005B: isolated RGB-D extension; baseline sensor nodes unchanged. -->\n'
                '  <xacro:include filename="$(find rtabmap_husky_integration)/urdf/front_left_rgbd.urdf.xacro"/>\n'
                '  <xacro:husky_front_left_rgbd/>\n')
    if p.is_symlink():
        p.unlink()
    p.write_text(original.replace('</robot>', addition + '</robot>'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--destination', type=Path, required=True)
    args = parser.parse_args()
    prepare(args.source, args.destination)


if __name__ == '__main__':
    main()
