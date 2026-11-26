#!/usr/bin/env python3
"""Trial-health guard for the sim test harness.

Why this exists: an earlier characterization run cleaned up between trials
with name-based `pkill -f` on only "gz sim"/"ros2 launch". That silently left
every child ROS node (state_machine_node, motion_stall_detector_node,
cmd_vel_watchdog_node, stability_monitor_node, ros_gz_bridge, robot_state_
publisher, ...) running, ~57 stale processes, host load 71-76 on 12 cores,
and trials that timed out under CPU starvation were recorded as "stalls".
Nothing checked, so nothing failed loudly.

Rules enforced here (all by PID, from /proc, never by matching the name of a
command that might not have died):
  preflight  - BEFORE launching a sim: zero stale sim/ROS processes, and the
               host is actually idle (instantaneous CPU, not just the lagging
               load average).
  verify     - AFTER launching, BEFORE any motion: exactly one sim instance,
               no duplicated node processes, every REQUIRED_NODES entry
               actually running, load not starved, and the simulator's
               real-time factor ~1.0 measured over a window.
  cleanup    - kill every sim/ROS process by PID and prove they are gone.
  monitor    - sample RTF + load during a trial so a starved trial is
               identifiable afterward instead of silently counted, and watch
               REQUIRED_NODES so a trial that lost one is not counted at all.
Every failure prints a loud banner and exits non-zero (2): the caller must
stop, not run the trial anyway. `summarize` grades a finished trial instead
and exits 3 if it was starved, 4 if it lost a required node. Those are kept
apart deliberately: a starved trial is a slow one whose numbers may still mean
something, a trial that lost a required node is not a trial.

None of this is a robot safety system. It does not stop a robot, it stops a
run from being trusted. See issue #4.

CLI: harness_guard.py {preflight|verify|cleanup|monitor <out.json> [secs]|summarize <out.json>}
"""
import os, re, sys, time, json, signal, subprocess, threading

# (family label, substring of the FULL command line). Matched against /proc/<pid>/cmdline.
FAMILIES = [
    ('gz_sim',        'gz sim'),
    ('gz_wrapper',    'gz_tools_vendor'),
    ('ros2_launch',   'ros2 launch'),
    ('ros_gz_bridge', 'parameter_bridge'),
    ('robot_state_publisher', 'robot_state_publisher'),
    ('ramp_autonomy_node', '/install/ramp_autonomy_'),   # installed nodes only, not any path mentioning the workspace
    ('ros_node',      '/opt/ros/jazzy/lib/'),
]
# stale trial drivers are also killed by cleanup (never the caller itself)
TRIAL_SCRIPTS = ('stall_diagnosis_trials.py', 'stall_highrate_trial.py', 'stall_scan_trial.py', 'freeze_session.py', 'crest_pause_injector.py',
                 'drift_trials.py', 'arc_turn_followpath_test_ekf.py', 'validation_test_ekf_groundtruth.py')
# tool-harness shell wrappers whose command line merely *contains* the launch command
WRAPPER_MARK = 'shell-snapshots'

# Node executables that must stay alive for the whole of a trial, checked by
# `monitor`. This is a TEST-HARNESS check, not a robot safety system: it does
# not stop a robot, it stops a run from being trusted.
#
# cmd_vel_watchdog_node is here because it is the sole publisher of the robot's
# actuation topic, and the gz-sim DiffDrive plugin holds its last target
# indefinitely when commands stop arriving (verified: the plugin exposes no
# timeout, stale-command or deadman parameter). So its death does not stop the
# robot, it freezes the last command and leaves the trial still apparently
# running. Without this check that run would be recorded as a result. See
# issue #4, where the actual fix, a consumer that times out, is discussed.
REQUIRED_NODES = ('cmd_vel_watchdog_node',)

MAX_IDLE_CPU_PCT = 25.0     # host busy % (all cores) allowed before launching a sim
MAX_LOAD1_IDLE = 12.0       # sanity cap on 1-min load before launch
MAX_LOAD1_RUNNING = 16.0    # one healthy sim measured 6-12 on this 12-core host; starved runs were 20-75
MIN_RTF = 0.95              # single healthy instance measured 1.000 steady
NODE_DUP_MAX = 1            # at most one process per ROS node executable


def banner(msg):
    bar = '!' * 78
    sys.stderr.write(f"\n{bar}\n!! HARNESS GUARD FAILED - DO NOT RUN THE TRIAL\n!! {msg}\n{bar}\n\n")
    sys.stderr.flush()


def _ancestors():
    out, pid = set(), os.getpid()
    while pid > 1:
        out.add(pid)
        try:
            with open(f'/proc/{pid}/stat') as f:
                pid = int(f.read().rsplit(')', 1)[1].split()[1])
        except Exception:
            break
    return out


def _cmdline(pid):
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            return f.read().replace(b'\0', b' ').decode(errors='replace').strip()
    except Exception:
        return ''


def list_procs(include_trial_scripts=False, exclude=None):
    """[(pid, family, cmdline)] for every live sim/ROS process (by /proc, excluding self/ancestors)."""
    skip = _ancestors() | set(exclude or ())
    found = []
    for name in os.listdir('/proc'):
        if not name.isdigit():
            continue
        pid = int(name)
        if pid in skip:
            continue
        cmd = _cmdline(pid)
        if not cmd or WRAPPER_MARK in cmd:
            continue
        fam = next((label for label, sub in FAMILIES if sub in cmd), None)
        if fam is None and include_trial_scripts and any(s in cmd for s in TRIAL_SCRIPTS):
            fam = 'trial_script'
        if fam:
            found.append((pid, fam, cmd))
    return found


def missing_nodes(required=REQUIRED_NODES):
    """Which REQUIRED_NODES have no live process, by executable name in the cmdline.

    Matched the same way list_procs() matches everything else, from /proc, never
    by asking the node whether it is alive. A node that has hung rather than
    exited still has a PID, so this catches death and not hangs; the RTF and
    load sampling alongside it is what catches the rest.
    """
    live = ' '.join(cmd for _, _, cmd in list_procs())
    return tuple(n for n in required if n not in live)


def load1():
    return os.getloadavg()[0]


def cpu_busy_pct(window=3.0):
    def snap():
        with open('/proc/stat') as f:
            v = list(map(int, f.readline().split()[1:8]))
        idle = v[3] + v[4]
        return idle, sum(v)
    i0, t0 = snap(); time.sleep(window); i1, t1 = snap()
    return 100.0 * (1.0 - (i1 - i0) / max(1, (t1 - t0)))


def _gz_stats(world):
    out = subprocess.run(['gz', 'topic', '-e', '-t', f'/world/{world}/stats', '-n', '1'],
                         capture_output=True, text=True, timeout=6).stdout
    m = lambda blk: (lambda g: int(g[0]) + int(g[1]) * 1e-9)(
        re.search(rf'{blk} \{{\s*sec: (\d+)\s*(?:nsec: (\d+))?', out).groups(default='0'))
    return m('sim_time'), m('real_time')


def detect_world():
    try:
        out = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True, timeout=6).stdout
        worlds = sorted(set(re.findall(r'/world/([^/\s]+)/stats', out)))
        return worlds[0] if len(worlds) == 1 else (worlds or [None])[0]
    except Exception:
        return None


def measure_rtf(world=None, window=5.0):
    """Sim-time advanced per real-time second over `window` (robust; not the instantaneous field)."""
    try:
        world = world or detect_world()
        if not world:
            return None
        s0, r0 = _gz_stats(world); time.sleep(window); s1, r1 = _gz_stats(world)
        return (s1 - s0) / (r1 - r0) if r1 > r0 else None
    except Exception:
        return None


def preflight(fail_exit=True):
    """Before launching a sim."""
    stale = list_procs(include_trial_scripts=True)
    if stale:
        lines = '\n!!   '.join(f'pid {p:>7} [{f}] {c[:80]}' for p, f, c in stale[:12])
        banner(f'{len(stale)} stale sim/ROS process(es) already running (first shown):\n!!   {lines}\n'
               f'!! Run `harness_guard.py cleanup` and re-check; refusing to launch on a dirty host.')
        return _die(fail_exit)
    busy = cpu_busy_pct(3.0)
    if busy > MAX_IDLE_CPU_PCT or load1() > MAX_LOAD1_IDLE:
        # a just-killed sim's 1-min load average decays slowly; give the *instantaneous* CPU a chance to settle
        deadline = time.time() + 120
        while time.time() < deadline and (busy > MAX_IDLE_CPU_PCT or load1() > MAX_LOAD1_IDLE):
            time.sleep(5); busy = cpu_busy_pct(3.0)
        if busy > MAX_IDLE_CPU_PCT or load1() > MAX_LOAD1_IDLE:
            banner(f'host not idle: cpu busy {busy:.0f}% (max {MAX_IDLE_CPU_PCT:.0f}%), load1 {load1():.1f} '
                   f'(max {MAX_LOAD1_IDLE}) with ZERO sim processes - something else is consuming CPU.')
            return _die(fail_exit)
    print(f'preflight OK: 0 stale processes, cpu busy {busy:.0f}%, load1 {load1():.1f}')
    return True


def _ppid(pid):
    try:
        return int(open(f'/proc/{pid}/stat').read().rsplit(')', 1)[1].split()[1])
    except Exception:
        return -1


def count_gz_instances(procs):
    """Number of simulator INSTANCES (not processes). Headless: one `gz sim ... -s` process = 1 instance. With the GUI enabled a run is three
    processes - the `gz sim <world>` launcher, its `gz sim server` child and its `gz sim gui` client - which is still ONE instance. So: the GUI client
    is not an instance, and a launcher that has a `gz sim server` child is not an instance either (its server is). Two servers, or two headless
    processes, are still two instances and still fail the guard."""
    cand = [(p, c) for p, f, c in procs if f == 'gz_sim' and 'gz sim' in c and 'ruby' not in c and not c.startswith('/bin/sh')]
    servers = {p for p, c in cand if c.rstrip().endswith('gz sim server')}
    guis = {p for p, c in cand if c.rstrip().endswith('gz sim gui')}
    launchers_of_servers = {_ppid(p) for p in servers}
    return sum(1 for p, c in cand if p not in guis and p not in launchers_of_servers)


def verify(world=None, exclude=None, fail_exit=True):
    """After launching, before any motion. Returns dict of measurements."""
    procs = list_procs(exclude=exclude)
    n_gz = count_gz_instances(procs)
    n_launch = sum(1 for _, f, c in procs if f == 'ros2_launch')
    dup = {}
    for p, f, c in procs:
        if f in ('ramp_autonomy_node', 'ros_node', 'ros_gz_bridge', 'robot_state_publisher'):
            toks = c.split(' --ros-args')[0].split()
            exe = toks[1] if len(toks) > 1 and toks[0].endswith(('python3', 'python')) else toks[0]
            key = os.path.basename(exe)
            dup[key] = dup.get(key, 0) + 1
    dups = {k: v for k, v in dup.items() if v > NODE_DUP_MAX}
    absent = tuple(n for n in REQUIRED_NODES if n not in ' '.join(c for _, _, c in procs))
    l1 = load1()
    rtf = measure_rtf(world, 5.0)
    res = dict(n_gz_sim=n_gz, n_ros2_launch=n_launch, duplicate_nodes=dups, load1=round(l1, 2),
               rtf=None if rtf is None else round(rtf, 3), n_procs=len(procs),
               missing_nodes=list(absent))
    problems = []
    if absent: problems.append(f'required node(s) not running before motion: {", ".join(absent)}')
    if n_gz != 1: problems.append(f'{n_gz} gz sim server processes (need exactly 1)')
    if n_launch != 1: problems.append(f'{n_launch} ros2 launch processes (need exactly 1)')
    if dups: problems.append(f'duplicated node processes: {dups}')
    if l1 > MAX_LOAD1_RUNNING: problems.append(f'load1 {l1:.1f} > {MAX_LOAD1_RUNNING} (CPU-starved host)')
    if rtf is None: problems.append('could not measure the simulator real-time factor')
    elif rtf < MIN_RTF: problems.append(f'real-time factor {rtf:.2f} < {MIN_RTF} (simulator cannot keep real time)')
    if problems:
        banner('; '.join(problems) + f'\n!! measurements: {res}')
        res['ok'] = False
        if fail_exit:
            sys.exit(2)
        return res
    res['ok'] = True
    print(f'verify OK: {res}')
    return res


def cleanup(fail_exit=True):
    for rnd in range(6):
        procs = list_procs(include_trial_scripts=True)
        if not procs:
            print(f'cleanup OK after {rnd} round(s): 0 sim/ROS processes remain')
            return True
        for pid, _, _ in procs:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            except PermissionError:
                pass
        time.sleep(2.0)
    left = list_procs(include_trial_scripts=True)
    banner(f'{len(left)} process(es) survived SIGKILL x6: ' + '; '.join(f'{p}:{f}' for p, f, _ in left[:8]))
    return _die(fail_exit)


def _die(fail_exit):
    if fail_exit:
        sys.exit(2)
    return False


class HealthMonitor:
    """Samples RTF + load during a trial; a starved trial identifies itself in the data."""
    def __init__(self, world=None, period=20.0, out_path=None):
        self.world, self.period, self.out = world, period, out_path
        self.samples = []            # (t, rtf, load1)
        # First moment a required node was seen to be gone, and which. Recorded
        # once and never cleared: a node that dies and is somehow restarted
        # still invalidates the trial, so the first loss is what matters.
        self.lost_at = None
        self.lost_nodes = ()
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._th.start(); return self

    def _run(self):
        while not self._stop.is_set():
            if self.world is None:
                self.world = detect_world()
            rtf = measure_rtf(self.world, 4.0)
            self.samples.append((round(time.time(), 1), None if rtf is None else round(rtf, 3), round(load1(), 2)))
            if self.lost_at is None:
                gone = missing_nodes()
                if gone:
                    self.lost_at, self.lost_nodes = round(time.time(), 1), gone
                    print(f'\n{"=" * 72}\n  REQUIRED NODE GONE DURING TRIAL: {", ".join(gone)}\n'
                          f'  This trial is not a result. See issue #4.\n{"=" * 72}\n',
                          file=sys.stderr, flush=True)
            if self.out:
                with open(self.out, 'w') as f:
                    json.dump(self.summary(), f)
            self._stop.wait(self.period)

    def stop(self):
        self._stop.set(); self._th.join(timeout=15)
        summ = self.summary()
        if self.out:
            with open(self.out, 'w') as f:
                json.dump(summ, f)
        return summ

    def summary(self):
        r = [s[1] for s in self.samples if s[1] is not None]
        l = [s[2] for s in self.samples]
        # HG_LOW_RTF (default 0.90 = unchanged): the 'two samples below this' clause was calibrated on the Floor-1-only world (RTF ~0.98). The Floor-2 world runs at a steady
        # ~0.91 with a load average of ~6 on an idle host, which trips that clause without any contention; sessions that need to run on it set HG_LOW_RTF=0.85 (the
        # hard-floor clause 'min < 0.85' and the load clause stay in force). Every result run that way says so.
        low_lim = float(os.environ.get('HG_LOW_RTF', '0.90'))
        low = sum(1 for x in r if x < low_lim)
        starved = bool(r) and (min(r) < 0.85 or low >= 2) or (bool(l) and max(l) > 20.0)
        return dict(n=len(self.samples), min_rtf=min(r) if r else None,
                    mean_rtf=round(sum(r) / len(r), 3) if r else None,
                    max_load1=max(l) if l else None, n_low_rtf=low, starved=starved,
                    # Kept separate from 'starved' on purpose: a starved trial is
                    # a slow one whose numbers may still mean something, a trial
                    # that lost a required node is not a trial.
                    lost_nodes=list(self.lost_nodes), lost_at=self.lost_at)


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else ''
    if cmd == 'preflight': preflight()
    elif cmd == 'verify': verify()
    elif cmd == 'cleanup': cleanup()
    elif cmd == 'monitor':
        out = sys.argv[2]; secs = float(sys.argv[3]) if len(sys.argv) > 3 else 3600
        m = HealthMonitor(period=20.0, out_path=out).start()
        signal.signal(signal.SIGTERM, lambda *a: (m.stop(), sys.exit(0)))
        time.sleep(secs); m.stop()
    elif cmd == 'summarize':
        s = json.load(open(sys.argv[2])); print(json.dumps(s))
        # 4 before 3: losing a required node is the more serious of the two, and
        # a trial that did both should report the one that voids it outright.
        if s.get('lost_nodes'):
            bar = '!' * 78
            gone = ', '.join(s['lost_nodes'])
            when = s.get('lost_at')
            at = '' if when is None else f' (first seen missing at {when})'
            sys.stderr.write(f'\n{bar}\n'
                             f'!! TRIAL LOST A REQUIRED NODE - THIS IS NOT A RESULT\n'
                             f'!! gone: {gone}{at}\n'
                             f'!! see issue #4\n'
                             f'{bar}\n\n')
            sys.stderr.flush()
            sys.exit(4)
        sys.exit(3 if s.get('starved') else 0)
    else:
        print(__doc__); sys.exit(1)


if __name__ == '__main__':
    main()
