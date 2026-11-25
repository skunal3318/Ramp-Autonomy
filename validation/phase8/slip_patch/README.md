# Testing the descent brake against real slip

Harness for [#28](https://github.com/skunal3318/Ramp-Autonomy/issues/28). Nothing
has been run yet; this is the world generator and the protocol.

## Why the existing low-friction worlds do not answer this

`ramp_world_sharp_mu025_TESTONLY.sdf` and `ramp_world_sharp_mu030_TESTONLY.sdf`
drop the friction of the **whole** ramp. That answers a different question. A
robot that never had traction in the first place will have a bad descent
whatever the brake does, so the run says nothing about whether the brake
helped.

What is actually missing is evidence that the brake's intervention branch does
any good. The published `ax_max` result was measured on a run where slip never
crossed `SLIP_BRAKE_THRESHOLD` at all, so the brake correctly did nothing, and
`docs/development-log.md` records it as statistically indistinguishable from
doing nothing. That is a demonstration of harmlessness, not of effectiveness.
Two external reviewers reached that same point independently within a day of
the project going public.

So the robot needs to enter the descent with normal traction, build speed, and
**then** meet a patch.

## Generating a world

```
./make_patch_world.py <base_world.sdf> <out.sdf> [mu] [fraction] [length_m]
```

The patch is derived from the base world's own `zone_b_ramp` slab rather than
hardcoded, so it follows the ramp if the geometry is ever corrected again, as
it has been once already (see `validation/phase8/crest_freeze/`). The script
refuses a patch whose friction is not below the ramp's own.

Defaults put a 1.0 m patch at `mu` 0.35, 40% of the way down from the crest,
which the robot meets after 1.46 m of descent. For reference, the ramp is
`mu` 0.55, the static floor is `tan(16 deg) = 0.287`, and
[#3](https://github.com/skunal3318/Ramp-Autonomy/issues/3) brackets the
practical climbing floor at (0.35, 0.45].

Placement is checked: the generated patch sits 0.0547 m off the slab mid-plane
along the ramp normal, against an expected 0.055, and 0.364 m along the slope
from the slab centre, against an expected 0.364.

## Protocol

Paired runs, everything else held constant:

1. brake enabled
2. `descent_brake_node.py` killed, exactly as the phase 7 baseline was taken

Report for both: max slip, HALT count, traverse time, max lateral offset.

## What counts as a result

The brake helps if the braked run shows lower max slip or fewer HALTs at
comparable traverse time.

If it shows the same slip and a longer traverse, that is the `vx_max` failure
reappearing in a new place and should be reported as such.

**A null result is publishable and gets published.** The credibility of this
project rests on reporting that honestly rather than on the brake working.

## Before running

- **The recorder must capture `/motion_stalled`.** The guarded-run CSV schema
  does not, which is why the figure in
  [#31](https://github.com/skunal3318/Ramp-Autonomy/pull/31) draws seven HALT
  episodes and cannot explain any of them. A slip experiment whose output
  cannot say why the robot stopped is not worth running. See
  [#32](https://github.com/skunal3318/Ramp-Autonomy/issues/32).
- **`harness_guard.py` will refuse.** It enforces `MIN_RTF = 0.95` and the full
  stack holds roughly 0.91 to 0.93 on ordinary hardware. Either the floor is
  addressed or these runs are unguarded, and if unguarded that belongs in the
  write-up.
- **Hold the host still.** Both wrong conclusions recorded in
  `docs/retrospective.md` came from measurements confounded by host conditions,
  once by running on battery, which halved the real-time factor. Run the pair
  back to back in one session, on mains, with nothing else loaded.
