#!/usr/bin/env python3
"""
descent_brake_node.py  [PHASE 7]

Active only while /ramp_state == 'DESCENT': continuously adjusts the
live MPPI controller's ACCELERATION ceiling (FollowPath.ax_max/az_max)
to keep /wheel_slip inside a target band, using the same
/controller_server/set_parameters mechanism nav2_param_switcher_node.py
already uses. HALT (state_machine_node.py, unaffected by this node)
stays the hard backstop if slip or stability margin gets bad enough
despite this - this loop is meant to reduce how often HALT needs to
fire, not replace it.

Leaving DESCENT restores the full ceiling. The parameters this node
pushes outlive it: it stops acting as soon as the state is not DESCENT,
but whatever reduced ax_max/az_max it last pushed stay live on
controller_server, and nav2_param_switcher_node.py will not push over
them because it owns vx_max/vx_min/wz_max and deliberately not the
acceleration limits. Without the restore, any descent with enough slip
to throttle the ceiling left the robot acceleration-limited through
EXIT and any flat driving after it. The entry and exit rules are in
descent_brake_policy.py so they can be tested directly.

===================== Phase 7 (continued): why ax_max/az_max, not vx_max =====================
Phase 7's first version capped FollowPath.vx_max (the steady-state
speed ceiling) instead. A later phase-7 pass re-ran both approaches side by side
with a corrected test methodology (phase 7's own numbers turned out to
be invalid - see the README's "FSM bug" section for why) and the
result was not just "no improvement", it was actively catastrophic:

    baseline (no braking):  5.50 m, 17.01 s, max_slip 1.000, 0 halts
    vx_max approach:        5.50 m, 76.44 s, max_slip 11.148, 146 halts
    ax_max approach:        5.50 m, 17.77 s, max_slip 1.000, 0 halts

vx_max capping didn't just fail to help, it triggered a feedback
spiral: cutting the speed ceiling forces the controller to decelerate
HARDER to comply (a bigger acceleration transient, not a smaller one),
which on this low-friction slope induces MORE slip, which this node
(old version) read as "still slipping, cut vx_max further", which
demanded still harder deceleration next tick - a vicious cycle that
repeatedly pushed slip past HALT's threshold (146 times in one 5.5 m
descent). Root cause, found live: a smooth, constant-velocity raw
cmd_vel probe across the SAME low-friction ramp produced ~zero slip,
so slip here is tied to ACCELERATION transients, not to how fast the
robot is cruising - constraining the velocity ceiling doesn't touch
the actual cause and, worse, forces exactly the kind of sharp
correction that IS the cause. Capping ax_max (forward/backward
acceleration) and az_max (angular acceleration - included because on
a skid-steer/diff-drive base, a STEERING correction is itself a
differential WHEEL acceleration event, exactly as slip-inducing as a
forward accel spike) targets the actual mechanism directly, so it
never needs to fight the controller for speed at all: the ax_max run
above is statistically indistinguishable from the unbraked baseline
(17.77 s vs. 17.01 s, identical max_slip) because slip never crossed
SLIP_BRAKE_THRESHOLD in the first place - the braking logic simply had
nothing to do, which is itself the result: it does no harm when
traction is fine and (per phase 7's original design intent, still
believed correct though not re-stressed by this particular run) is
still there to intervene if slip does spike. vx_max/vx_min/wz_max/
heading_tolerance are left entirely alone by this node now -
nav2_param_switcher_node.py's normal ramp_mode profile still applies
during DESCENT unmodified. See the README for full methodology and
the FSM-bug context these numbers depend on.

===================== Why set_parameters, not direct cmd_vel =====================
Two ways to cap descent behavior: (a) push updated MPPI parameters via
controller_server's live set_parameters service, same mechanism
nav2_param_switcher_node.py already uses, or (b) publish directly to
/model/ramp_robot/cmd_vel (or the pre-smoother cmd_vel_nav), bypassing
the controller. Went with (a): publishing cmd_vel directly would make
this a THIRD node able to command the robot, racing controller_server
and the velocity smoother for the same topic with no arbitration, and
would silently override or interleave with MPPI's own path-following/
obstacle-avoidance output rather than incorporating it. Adjusting MPPI's
own acceleration limits keeps it as the one actual driver: this node
constrains the envelope MPPI samples trajectories within (the way a
traction-control system constrains a car's engine/brake authority
without taking over steering), rather than fighting it for the output
topic.

===================== Peak, not instantaneous, slip per control tick =====================
Found live in phase 7, still applies here: a fixed-rate sampler
checking "what is slip right now" once per control tick can miss a
brief spike entirely if it lands between two ticks - this sim's slip
events are brief, not sustained. Fixed by tracking the PEAK slip seen
since the last tick (updated on every /wheel_slip message, not just at
tick time) and resetting that peak after each control tick reads it -
a spike is caught if it happened at all during the window, independent
of control-loop rate.

===================== Target slip band - based on phase 6/7's live data =====================
Unchanged from phase 7 - this sim's wheel-ground contact is plain
Coulomb friction (fixed mu, binary-ish stick/slip), not a slip-ratio-
-dependent curve real tire models are tuned against, so there's no
"peak-mu-around-15%-slip" ratio to target as a stable equilibrium. The
band below is a threshold/hysteresis pair:
    SLIP_BRAKE_THRESHOLD   = 0.25  (half of WHEEL_SLIP_HALT_THRESHOLD's
                              0.5 - real room to react before HALT's
                              hard backstop)
    SLIP_RELEASE_THRESHOLD = 0.08  (comfortably above ordinary near-zero
                              noise, comfortably below anything seen
                              once traction actually broke)

Published:
    (none)

Subscribed:
    /ramp_state   (std_msgs/String)   current FSM state
    /wheel_slip   (std_msgs/Float32)  from stability_monitor_node.py

Calls:
    /controller_server/set_parameters  (rcl_interfaces/srv/SetParameters)
"""
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import Float32, String

from .descent_brake_policy import (
    POST_HALT_RESTART_FRACTION,
    ceiling_fraction_for_transition,
)

CONTROLLER_PLUGIN = 'FollowPath'

# Hysteresis band - see the module docstring for why these specific
# numbers and why this is a threshold pair rather than a target ratio.
SLIP_BRAKE_THRESHOLD = 0.25
SLIP_RELEASE_THRESHOLD = 0.08

# Per-control-tick adjustment, in m/s^2 (ax) and rad/s^2 (az). Braking
# (reducing the ceiling) is proportional to how far over threshold the
# current slip is, so a severe event gets a bigger cut than a marginal
# one; releasing is a smaller fixed step - "brake fast, release
# cautiously". Scaled down from phase 7's vx_max steps (0.05/0.02 m/s)
# to reflect these are acceleration units on a 2.0 m/s^2 ceiling rather
# than velocity units on a 0.35 m/s one - roughly the same fraction of
# ceiling per step.
BRAKE_STEP_BASE = 0.25   # m/s^2 (ax); az scaled proportionally, see _push
RELEASE_STEP = 0.10      # m/s^2 (ax); az scaled proportionally, see _push
CONTROL_PERIOD_SEC = 0.5

# Floor - HALT is the true hard zero, not this loop. Chosen as ~1/5 of
# the ceiling (mirroring phase 7's MIN_DESCENT_VX_MAX_MPS being ~1/3 of
# its ceiling) - gentle enough to meaningfully suppress correction
# transients, not so low that MPPI can no longer track a path at all
# (phase 7's floor-of-near-zero lesson: an over-aggressive floor caused
# FollowPath to stall and give up, which was worse than doing nothing).
MIN_ACCEL_FRACTION = 0.2

# POST_HALT_RESTART_FRACTION and the rest of the DESCENT entry/exit
# rules live in descent_brake_policy.py, so they can be tested without
# rclpy or a running controller_server.


def _double_param(name: str, value: float) -> Parameter:
    return Parameter(
        name=name,
        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)),
    )


class DescentBrake(Node):
    def __init__(self):
        super().__init__('descent_brake')

        params_path = Path(get_package_share_directory('ramp_autonomy_nav')) / 'config' / 'ramp_mppi_params.yaml'
        with open(params_path) as f:
            profiles = yaml.safe_load(f)
        ramp_mode = profiles['ramp_mode']
        self._ceiling_ax_max = float(ramp_mode['max_accel_x'])
        self._ceiling_az_max = float(ramp_mode['max_accel_theta'])
        self._floor_ax_max = self._ceiling_ax_max * MIN_ACCEL_FRACTION
        self._floor_az_max = self._ceiling_az_max * MIN_ACCEL_FRACTION

        self._state = 'APPROACH'  # what we're transitioning FROM in _on_state, not a duplicate tracker
        self._peak_slip_since_tick = 0.0
        # Both ceilings move together, scaled by the same fraction each
        # tick - one slip-band controller adjusting one "how aggressive
        # can MPPI be" knob, not two independently-tuned loops.
        self._ceiling_fraction = 1.0
        self._push_in_flight = False

        self._set_params_client = self.create_client(SetParameters, '/controller_server/set_parameters')

        self.create_subscription(String, '/ramp_state', self._on_state, 10)
        self.create_subscription(Float32, '/wheel_slip', self._on_wheel_slip, 10)
        self.create_timer(CONTROL_PERIOD_SEC, self._on_control_tick)

        self.get_logger().info(
            f'Descent brake running (phase 7, ax_max/az_max): slip band '
            f'[{SLIP_RELEASE_THRESHOLD}, {SLIP_BRAKE_THRESHOLD}], '
            f'ceiling ax_max={self._ceiling_ax_max} m/s^2, az_max={self._ceiling_az_max} rad/s^2'
        )

    def _on_wheel_slip(self, msg: Float32):
        # Peak-since-last-tick, not instantaneous - see the module
        # docstring's "Peak, not instantaneous" section for why this
        # matters more than the control period itself.
        self._peak_slip_since_tick = max(self._peak_slip_since_tick, msg.data)

    def _on_state(self, msg: String):
        # self._state is still whatever we're transitioning FROM at this point.
        new_state = msg.data
        fraction = ceiling_fraction_for_transition(self._state, new_state)

        if fraction is not None:
            if new_state == 'DESCENT' and self._state == 'HALT':
                self.get_logger().warn(
                    f'Re-entering DESCENT after a HALT - starting cautious at '
                    f'{POST_HALT_RESTART_FRACTION:.0%} of the acceleration ceiling'
                )
            elif new_state != 'DESCENT' and self._ceiling_fraction != fraction:
                # Only worth a line when the ceiling was actually throttled
                # down, which is the case this restore exists for.
                self.get_logger().info(
                    f'Leaving DESCENT for {new_state} - restoring the acceleration '
                    f'ceiling from {self._ceiling_fraction:.0%} to {fraction:.0%}'
                )
            self._ceiling_fraction = fraction
            self._push(fraction)  # apply immediately, don't wait for the next tick

        self._state = new_state

    def _on_control_tick(self):
        # Always drain the peak-since-last-tick accumulator, even when not
        # in DESCENT, so a stale peak from before entering it never leaks
        # into the first real control decision.
        slip = self._peak_slip_since_tick
        self._peak_slip_since_tick = 0.0

        if self._state != 'DESCENT':
            return
        if self._push_in_flight:
            return  # previous push hasn't resolved yet - don't stack requests

        previous = self._ceiling_fraction
        if slip > SLIP_BRAKE_THRESHOLD:
            overshoot = slip - SLIP_BRAKE_THRESHOLD
            reduction = (BRAKE_STEP_BASE * (1.0 + overshoot)) / self._ceiling_ax_max
            self._ceiling_fraction = max(MIN_ACCEL_FRACTION, self._ceiling_fraction - reduction)
        elif slip < SLIP_RELEASE_THRESHOLD:
            self._ceiling_fraction = min(1.0, self._ceiling_fraction + RELEASE_STEP / self._ceiling_ax_max)
        # else: within the band - hold steady, no push needed unless it changed

        if self._ceiling_fraction != previous:
            self.get_logger().info(
                f'slip={slip:.2f} -> accel ceiling {previous:.0%} -> {self._ceiling_fraction:.0%} '
                f'(ax_max={self._ceiling_fraction * self._ceiling_ax_max:.2f} m/s^2)'
            )
            self._push(self._ceiling_fraction)

    def _push(self, ceiling_fraction: float):
        if not self._set_params_client.service_is_ready():
            self.get_logger().error(
                f'/controller_server/set_parameters not available - could not push accel ceiling '
                f'fraction={ceiling_fraction:.2f}'
            )
            return

        ax_max = max(self._floor_ax_max, ceiling_fraction * self._ceiling_ax_max)
        az_max = max(self._floor_az_max, ceiling_fraction * self._ceiling_az_max)
        request = SetParameters.Request(parameters=[
            _double_param(f'{CONTROLLER_PLUGIN}.ax_max', ax_max),
            _double_param(f'{CONTROLLER_PLUGIN}.ax_min', -ax_max),
            _double_param(f'{CONTROLLER_PLUGIN}.az_max', az_max),
        ])
        self._push_in_flight = True
        future = self._set_params_client.call_async(request)
        future.add_done_callback(lambda f, frac=ceiling_fraction: self._on_push_response(f, frac))

    def _on_push_response(self, future, ceiling_fraction: float):
        self._push_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'set_parameters call raised pushing accel ceiling fraction={ceiling_fraction:.2f}: {exc}'
            )
            return
        failed = [r.reason for r in response.results if not r.successful]
        if failed:
            self.get_logger().error(
                f'set_parameters REJECTED pushing accel ceiling fraction={ceiling_fraction:.2f}: {failed}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = DescentBrake()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
