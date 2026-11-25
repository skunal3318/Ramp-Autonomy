"""Entry and exit rules for the descent brake's acceleration ceiling."""
import pytest

from ramp_autonomy_control.descent_brake_policy import (
    FULL_CEILING_FRACTION,
    POST_HALT_RESTART_FRACTION,
    ceiling_fraction_for_transition,
)


FSM_STATES = ['APPROACH', 'ASCENT', 'TRANSITION', 'DESCENT', 'EXIT', 'HALT']
NON_DESCENT = [s for s in FSM_STATES if s != 'DESCENT']


@pytest.mark.parametrize('previous', [s for s in NON_DESCENT if s != 'HALT'])
def test_entering_descent_starts_unrestricted(previous):
    assert ceiling_fraction_for_transition(previous, 'DESCENT') == FULL_CEILING_FRACTION


def test_entering_descent_from_halt_starts_cautious():
    assert ceiling_fraction_for_transition('HALT', 'DESCENT') == POST_HALT_RESTART_FRACTION


@pytest.mark.parametrize('new_state', NON_DESCENT)
def test_leaving_descent_restores_the_full_ceiling(new_state):
    """The bug this guards: nothing else pushes ax_max back up."""
    assert ceiling_fraction_for_transition('DESCENT', new_state) == FULL_CEILING_FRACTION


@pytest.mark.parametrize('previous', FSM_STATES)
def test_staying_in_the_same_state_changes_nothing(previous):
    assert ceiling_fraction_for_transition(previous, previous) is None


@pytest.mark.parametrize('previous', NON_DESCENT)
@pytest.mark.parametrize('new_state', NON_DESCENT)
def test_transitions_that_do_not_touch_descent_change_nothing(previous, new_state):
    assert ceiling_fraction_for_transition(previous, new_state) is None


def test_a_full_round_trip_ends_unrestricted():
    """APPROACH to EXIT, with a mid-descent HALT, must not leave it throttled."""
    sequence = ['APPROACH', 'ASCENT', 'TRANSITION', 'DESCENT',
                'HALT', 'DESCENT', 'EXIT']
    fraction = FULL_CEILING_FRACTION
    for previous, new_state in zip(sequence, sequence[1:]):
        decided = ceiling_fraction_for_transition(previous, new_state)
        if decided is not None:
            fraction = decided
    assert fraction == FULL_CEILING_FRACTION


def test_halt_recovery_is_cautious_but_exit_still_restores():
    """The cautious restart must not survive the end of the descent."""
    assert ceiling_fraction_for_transition('HALT', 'DESCENT') == POST_HALT_RESTART_FRACTION
    assert ceiling_fraction_for_transition('DESCENT', 'EXIT') == FULL_CEILING_FRACTION


def test_cautious_restart_is_a_real_reduction():
    """Guards against the two constants being set equal and the branch going silent."""
    assert 0.0 < POST_HALT_RESTART_FRACTION < FULL_CEILING_FRACTION
