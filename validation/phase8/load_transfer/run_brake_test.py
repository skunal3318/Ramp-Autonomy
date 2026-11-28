#!/usr/bin/env python3
"""Measure how much normal load shifts between axles under braking.

Issue #25 predicts the uphill axle unloads by m*a*h/L and therefore saturates
before the whole-robot traction budget says it should. That is paper maths.
This measures the transfer directly from per-wheel force/torque sensors.

Flat ground, not the ramp: the transfer is m*a*h/L either way, and the flat
baseline is a clean zero so the braking signal stands on its own.

Per-wheel loads are summed per axle. The solver splits load unevenly left to
right on a symmetric robot (see README), but that cancels in the axle totals.

Usage: run_brake_test.py [cruise_speed] [out.json]
"""
import json
import math
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.environ.get('RAMP_WS', os.path.abspath(os.path.join(HERE, '..', '..', '..')))
WORLD_NAME = 'ramp_world_sharp'
ROBOT = 'ramp_robot'
CMD_TOPIC = f'/model/{ROBOT}/cmd_vel'
ODOM_TOPIC = '/odom'       # DiffDrive's odom_topic, not namespaced by the model
WHEELS = ('front_left', 'front_right', 'rear_left', 'rear_right')

# Zone A runs x 0..4.0 and the robot spawns at x=1.0. Everything has to finish
# before the ramp foot or the "flat ground" baseline stops being flat.
SETTLE_S = 12.0
CRUISE_S = 3.0
BRAKE_S = 2.0
CHASSIS_KG = 40.0          # base_link; the wheels are unsprung and below the joints
CG_HEIGHT_M = 0.25         # base_link origin above the contact plane at rest
WHEELBASE_M = 0.56


def sh(cmd, **kw):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True, **kw)


def start_capture(topic, path):
    f = open(path, 'w')
    return subprocess.Popen(['gz', 'topic', '-e', '-t', topic],
                            stdout=f, stderr=subprocess.DEVNULL), f


def parse_ft(path):
    """[(sim_seconds, Fz)] from a gz topic dump of a force_torque sensor."""
    out = []
    for blk in open(path).read().split('header {')[1:]:
        sec = re.search(r'sec:\s*(\d+)', blk)
        nsec = re.search(r'nsec:\s*(\d+)', blk)
        f = re.search(r'force\s*\{([^}]*)\}', blk, re.S)
        if f:
            g = lambda k: float((re.search(rf'\b{k}:\s*(-?[\d.eE+-]+)', f.group(1)) or [0, '0'])[1])
            t = (int(sec.group(1)) if sec else 0) + (int(nsec.group(1)) if nsec else 0) * 1e-9
            out.append((t, abs(g('z')), g('x')))
    return out


def parse_speed(path):
    """[(sim_seconds, forward speed)] from the DiffDrive odometry dump.

    Only twist carries a `linear` block, so no need to scope the search.
    """
    out = []
    for blk in open(path).read().split('header {')[1:]:
        sec = re.search(r'sec:\s*(\d+)', blk)
        nsec = re.search(r'nsec:\s*(\d+)', blk)
        vx = re.search(r'linear\s*\{[^}]*?\bx:\s*(-?[\d.eE+-]+)', blk, re.S)
        if vx:
            t = (int(sec.group(1)) if sec else 0) + (int(nsec.group(1)) if nsec else 0) * 1e-9
            out.append((t, float(vx.group(1))))
    return out


def mean_between(series, t0, t1, idx=1):
    vals = [p[idx] for p in series if t0 <= p[0] <= t1]
    return sum(vals) / len(vals) if vals else None


def stats_between(series, t0, t1):
    """(mean, standard error, n). Contact chatter is large, so the error matters."""
    vals = [p[1] for p in series if t0 <= p[0] <= t1]
    if not vals:
        return None, None, 0
    m = sum(vals) / len(vals)
    if len(vals) < 2:
        return m, None, len(vals)
    var = sum((v - m) ** 2 for v in vals) / (len(vals) - 1)
    return m, math.sqrt(var / len(vals)), len(vals)


def main():
    speed = float(sys.argv[1]) if len(sys.argv) > 1 else 0.8
    out_path = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, 'brake_test.json')
    tmp = os.path.join('/tmp', f'load_transfer_{os.getpid()}')
    os.makedirs(tmp, exist_ok=True)

    world = os.path.join(tmp, 'world.sdf')
    urdf = os.path.join(tmp, 'robot.urdf')
    base_world = os.path.join(WS, 'src/ramp_autonomy_gazebo/worlds/ramp_world_sharp.sdf')

    subprocess.run([sys.executable, os.path.join(HERE, 'make_world.py'), base_world, world], check=True)
    with open(urdf, 'w') as f:
        r = sh(f"xacro {WS}/src/ramp_autonomy_description/urdf/robot.urdf.xacro wheel_force_sensors:=true")
        if r.returncode:
            raise SystemExit(f'xacro failed:\n{r.stderr[-400:]}')
        f.write(r.stdout)

    server = subprocess.Popen(['gz', 'sim', '-s', '-r', world],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    caps = []
    try:
        time.sleep(6)
        sh(f"ros2 run ros_gz_sim create -world {WORLD_NAME} -file {urdf} "
           f"-name {ROBOT} -x 1.0 -y 0 -z 0.23")
        time.sleep(3)

        for w in WHEELS:
            caps.append((w, *start_capture(f'/wheel_ft/{w}', os.path.join(tmp, f'{w}.txt'))))
        caps.append(('odom', *start_capture(ODOM_TOPIC, os.path.join(tmp, 'odom.txt'))))

        t_settle = time.time()
        time.sleep(SETTLE_S)
        t_cruise = time.time()
        sh(f"gz topic -t {CMD_TOPIC} -m gz.msgs.Twist -p 'linear: {{x: {speed}}}'")
        time.sleep(CRUISE_S)
        t_brake = time.time()
        sh(f"gz topic -t {CMD_TOPIC} -m gz.msgs.Twist -p 'linear: {{x: 0.0}}'")
        time.sleep(BRAKE_S)
    finally:
        for _, p, f in caps:
            p.terminate(); f.close()
        server.terminate()
        try:
            server.wait(timeout=10)
        except subprocess.TimeoutExpired:
            server.kill()

    ft = {w: parse_ft(os.path.join(tmp, f'{w}.txt')) for w in WHEELS}
    odom = parse_speed(os.path.join(tmp, 'odom.txt'))
    if not all(ft.values()) or not odom:
        raise SystemExit('no samples captured, cannot measure')

    # Phases come from the speed trace, not from the sleeps that produced it.
    # Sim time and wall time diverge whenever the real-time factor is not 1,
    # which shifted every window late on the first attempt.
    v_max = max(s for _, s in odom)
    if v_max < speed * 0.5:
        raise SystemExit(f'robot never reached speed (max {v_max:.2f} of {speed})')

    cruising = [t for t, s in odom if s > v_max * 0.95]
    t_brake = cruising[-1]
    stopped = [t for t, s in odom if t > t_brake and s < v_max * 0.05]
    t_stop = stopped[0] if stopped else odom[-1][0]
    t_move = next(t for t, s in odom if s > v_max * 0.05)

    w_settle = (odom[0][0], t_move - 0.5)
    w_cruise = (cruising[0], t_brake)
    w_brake = (t_brake, t_stop)

    def axles(win):
        """(front, rear, standard error on the front-rear gap, n)."""
        vals, errs, ns = {}, {}, []
        for w in WHEELS:
            m, se, n = stats_between(ft[w], *win)
            vals[w], errs[w], _ = m or 0.0, se or 0.0, ns.append(n)
        f = vals['front_left'] + vals['front_right']
        r = vals['rear_left'] + vals['rear_right']
        gap_se = math.sqrt(sum(e ** 2 for e in errs.values()))
        fx = sum(mean_between(ft[w], *win, idx=2) or 0.0 for w in WHEELS)
        return f, r, gap_se, min(ns), fx

    rest_f, rest_r, rest_se, rest_n, rest_fx = axles(w_settle)
    cru_f, cru_r, cru_se, cru_n, cru_fx = axles(w_cruise)
    brk_f, brk_r, brk_se, brk_n, brk_fx = axles(w_brake)

    decel = (v_max - 0.0) / (t_stop - t_brake) if t_stop > t_brake else None

    predicted = CHASSIS_KG * decel * CG_HEIGHT_M / WHEELBASE_M if decel else None
    # Transfer is half the change in the front-rear gap between cruise and brake.
    measured = ((brk_f - brk_r) - (cru_f - cru_r)) / 2.0
    measured_se = math.sqrt(brk_se ** 2 + cru_se ** 2) / 2.0

    res = dict(cruise_speed=speed,
               rest=dict(front=round(rest_f, 2), rear=round(rest_r, 2),
                         imbalance=round(rest_f - rest_r, 2)),
               cruise=dict(front=round(cru_f, 2), rear=round(cru_r, 2),
                           imbalance=round(cru_f - cru_r, 2)),
               braking=dict(front=round(brk_f, 2), rear=round(brk_r, 2),
                            imbalance=round(brk_f - brk_r, 2)),
               decel_mps2=round(decel, 3) if decel else None,
               transfer_measured_N=round(measured, 2),
               transfer_measured_se_N=round(measured_se, 2),
               transfer_predicted_N=round(predicted, 2) if predicted else None,
               h_over_L=round(CG_HEIGHT_M / WHEELBASE_M, 4))
    with open(out_path, 'w') as f:
        json.dump(res, f, indent=2)

    print(f"  rest      front {rest_f:7.2f}  rear {rest_r:7.2f}  gap {rest_f - rest_r:+7.2f} +/- {rest_se:.2f} N  (n={rest_n})")
    print(f"  cruising  front {cru_f:7.2f}  rear {cru_r:7.2f}  gap {cru_f - cru_r:+7.2f} +/- {cru_se:.2f} N  (n={cru_n})")
    print(f"  braking   front {brk_f:7.2f}  rear {brk_r:7.2f}  gap {brk_f - brk_r:+7.2f} +/- {brk_se:.2f} N  (n={brk_n})")
    print()
    print(f"  longitudinal Fx sum    rest {rest_fx:7.2f}   cruise {cru_fx:7.2f}   brake {brk_fx:7.2f} N")
    print(f"  expected during brake  Mc*a = {CHASSIS_KG * (decel or 0):7.2f} N")
    print(f"  measured deceleration  {decel if decel else float('nan'):.3f} m/s^2")
    print(f"  transfer measured      {measured:7.2f} +/- {measured_se:.2f} N per axle")
    if predicted:
        sigma = abs(measured - predicted) / measured_se if measured_se else float('inf')
        print(f"  transfer predicted     {predicted:7.2f} N   (m*a*h/L, h/L = {CG_HEIGHT_M / WHEELBASE_M:.3f})")
        print(f"  difference             {abs(measured - predicted):7.2f} N = {sigma:.1f} sigma")
    print(f"\n  written to {out_path}")


if __name__ == '__main__':
    main()
