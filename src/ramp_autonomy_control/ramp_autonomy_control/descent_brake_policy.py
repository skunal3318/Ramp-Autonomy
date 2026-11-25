"""Dependency-free DESCENT entry/exit policy for descent_brake_node.py.

Kept out of the node module so the transition rules can be tested
without rclpy, a controller_server, or a simulator - the acceleration
ceiling is a safety-relevant number and its lifecycle is worth pinning
down in a test rather than only in a live run.
"""

# Fraction of the ceiling to (re-)start at after a slip/margin HALT
# recovers straight back into DESCENT (state_machine_node.py) rather
# than through APPROACH - see that node's HALT-recovery comment for why
# DESCENT specifically needs a cautious restart, not the full ceiling.
POST_HALT_RESTART_FRACTION = 0.4

# What an unthrottled ceiling is, as a fraction of the profile value.
FULL_CEILING_FRACTION = 1.0


def ceiling_fraction_for_transition(previous_state: str, new_state: str):
    """Ceiling fraction to apply on a /ramp_state transition.

    Returns None when the transition does not call for a change, in
    which case the caller leaves the current fraction and its last
    pushed parameters alone.

    Entering DESCENT starts unrestricted, or cautious if we arrived
    straight from a HALT, and lets the control loop throttle down only
    if slip actually shows up.

    Leaving DESCENT restores the full ceiling. This matters because the
    node stops acting the moment the state is not DESCENT, while the
    reduced ax_max/az_max it last pushed stay live on controller_server
    until something pushes over them. nav2_param_switcher_node.py will
    not: it owns vx_max/vx_min/wz_max and deliberately not the
    acceleration limits (see both docstrings). So without this, any
    descent that saw enough slip to throttle the ceiling left the robot
    acceleration-limited for the whole of EXIT and any flat driving
    after it, until the next DESCENT happened to reset it.
    """
    entering_descent = new_state == 'DESCENT' and previous_state != 'DESCENT'
    leaving_descent = new_state != 'DESCENT' and previous_state == 'DESCENT'

    if entering_descent:
        if previous_state == 'HALT':
            return POST_HALT_RESTART_FRACTION
        return FULL_CEILING_FRACTION

    if leaving_descent:
        # Includes leaving to HALT. The restore is harmless there, since
        # HALT zeroes vx_max anyway, and re-entering DESCENT from HALT
        # goes through the cautious branch above and lowers it again.
        return FULL_CEILING_FRACTION

    return None
