#!/usr/bin/env python3
"""Add the ForceTorque system to a world so wheel sensors publish.

The shared worlds are left alone: the plugin is injected into a copy.

Usage: make_world.py <base_world.sdf> <out.sdf>
"""
import sys

PLUGIN = '    <plugin filename="gz-sim-forcetorque-system" name="gz::sim::systems::ForceTorque"/>\n'
ANCHOR = '<plugin filename="gz-sim-imu-system"'


def main():
    if len(sys.argv) < 3:
        raise SystemExit('usage: make_world.py <base_world.sdf> <out.sdf>')
    base, out = sys.argv[1], sys.argv[2]

    sdf = open(base, encoding='utf-8').read()
    if 'gz-sim-forcetorque-system' in sdf:
        raise SystemExit(f'{base} already loads the ForceTorque system')
    if ANCHOR not in sdf:
        raise SystemExit(f'no {ANCHOR}...> line in {base} to insert beside')

    line_start = sdf.index(ANCHOR)
    line_end = sdf.index('\n', line_start) + 1
    open(out, 'w', encoding='utf-8').write(sdf[:line_end] + PLUGIN + sdf[line_end:])
    print(f'{out}: ForceTorque system added')


if __name__ == '__main__':
    main()
