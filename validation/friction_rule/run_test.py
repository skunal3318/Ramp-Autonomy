#!/usr/bin/env python3
"""Measure whether the test box slides, at a given incline angle.

Generates the world, runs gz-sim headless, records the box pose, and
reports how far it travelled along the slope. A box that holds station
moves a millimetre or two as the contact settles; a box that lets go
travels centimetres and keeps going.

Usage: run_test.py <angle_deg> [seconds]
"""
import math
import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
# A box only settling into contact moves well under this; a sliding box
# passes it long before the capture window closes.
SLIDE_THRESHOLD_M = 0.02


def capture(world_path, world_name, samples):
    """Run the server, collect pose messages, stop the server."""
    server = subprocess.Popen(
        ['gz', 'sim', '-s', '-r', world_path],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    try:
        out = subprocess.run(
            ['gz', 'topic', '-e', '-t', f'/world/{world_name}/dynamic_pose/info',
             '-n', str(samples)],
            capture_output=True, text=True, timeout=120,
        ).stdout
    finally:
        server.terminate()
        try:
            server.wait(timeout=10)
        except subprocess.TimeoutExpired:
            server.kill()
    return out


def box_positions(dump, model='test_box'):
    """Pull (x, z) for each pose message naming the model."""
    positions = []
    blocks = dump.split('pose {')
    for block in blocks:
        if f'name: "{model}"' not in block:
            continue
        pos = re.search(r'position\s*\{([^}]*)\}', block)
        if not pos:
            continue
        body = pos.group(1)
        x = re.search(r'\bx:\s*(-?[\d.eE+-]+)', body)
        z = re.search(r'\bz:\s*(-?[\d.eE+-]+)', body)
        positions.append((float(x.group(1)) if x else 0.0,
                          float(z.group(1)) if z else 0.0))
    return positions


def main():
    if len(sys.argv) < 2:
        raise SystemExit(
            'usage: run_test.py <angle_deg> [seconds] [box_mu] [ramp_mu]\n'
            '       either mu may be "default" to omit that <surface> block')
    angle_deg = float(sys.argv[1])
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
    mu_args = sys.argv[3:5]

    with tempfile.TemporaryDirectory() as tmp:
        world = os.path.join(tmp, f'friction_{angle_deg:g}.sdf')
        subprocess.run([sys.executable, os.path.join(HERE, 'make_world.py'),
                        str(angle_deg), world, *mu_args], check=True)
        # dynamic_pose/info publishes once per render update, not per physics
        # step, so ask for generous coverage of the requested window.
        dump = capture(world, 'friction_rule', int(seconds * 60))

    pts = box_positions(dump)
    if len(pts) < 2:
        raise SystemExit(f'only {len(pts)} box poses captured, cannot measure')

    (x0, z0), (x1, z1) = pts[0], pts[-1]
    # Distance travelled along the incline surface, which is what the
    # friction cone governs. Downhill is +x for a positive pitch.
    pitch = math.radians(angle_deg)
    along = (x1 - x0) * math.cos(pitch) - (z1 - z0) * math.sin(pitch)

    slid = abs(along) > SLIDE_THRESHOLD_M
    print(f'  angle            {angle_deg} deg   (tan = {math.tan(pitch):.4f})')
    print(f'  poses captured   {len(pts)}')
    print(f'  start            x={x0:.4f} z={z0:.4f}')
    print(f'  end              x={x1:.4f} z={z1:.4f}')
    print(f'  along slope      {along:+.4f} m')
    print(f'  verdict          {"SLID" if slid else "held"}')
    return 0 if not slid else 2


if __name__ == '__main__':
    sys.exit(main())
