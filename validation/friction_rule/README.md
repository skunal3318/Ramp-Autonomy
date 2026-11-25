# What the contact friction coefficient actually is

Measured for [#33](https://github.com/skunal3318/Ramp-Autonomy/issues/33).

Every `mu` in this project is authored on one surface, but contact friction
is a single number the physics engine derives from **both** surfaces in
contact. Until this was measured, nothing in the repo established what that
number was, which left every quoted friction figure describing a surface
rather than a contact.

Two things needed settling:

1. how the engine combines two surface coefficients into one
2. what the robot's wheels contribute, given their `<collision>` blocks carry
   no `<surface>` element and inherit an engine default nobody chose

## Method

A rigid body on a slope slides when `tan(angle) > mu_eff`, so the angle at
which a box lets go measures the effective coefficient directly. Give the box
and the ramp deliberately different values, bisect the slide angle, and the
result says which rule produced it.

```
./run_test.py <angle_deg> [seconds] [box_mu] [ramp_mu]
```

Either `mu` may be `default`, which omits that body's `<surface>` block
entirely, reproducing how the wheels are authored. The runner generates the
world, runs gz-sim headless, records the box pose and reports how far it
travelled along the slope. Settling moves a tenth of a millimetre; letting go
moves metres.

## Result 1: the rule is the minimum

Box `mu` 0.8 on ramp `mu` 0.5. Candidate rules predict different slide angles:

| rule | mu_eff | slides above |
|---|---|---|
| product | 0.400 | 21.80 deg |
| minimum | 0.500 | 26.57 deg |
| geometric mean | 0.632 | 32.31 deg |
| arithmetic mean | 0.650 | 33.02 deg |

Measured:

| angle | tan | along slope | verdict |
|---|---|---|---|
| 15 deg | 0.268 | +0.0001 m | held (control) |
| 24 deg | 0.445 | +0.0002 m | held |
| 26 deg | 0.488 | +0.0002 m | held |
| 27 deg | 0.510 | +0.67 m | slid |
| 30 deg | 0.577 | +7.81 m | slid |
| 45 deg | 1.000 | +42.14 m | slid (control) |

Holding at 24 rules out the product. Sliding at 30 rules out both means. The
transition between 26 and 27 brackets `mu_eff` to **[0.4877, 0.5095]**, which
contains `min(0.8, 0.5) = 0.500` and nothing else on the list.

## Result 2: the unset default is at or above 0.9

Box with no `<surface>` block at all, on ramp `mu` 0.9:

| angle | tan | verdict |
|---|---|---|
| 40 deg | 0.839 | held |
| 41 deg | 0.869 | held |
| 43 deg | 0.933 | slid |
| 44 deg | 0.966 | slid |

That brackets `mu_eff` to **[0.8693, 0.9325]**, containing the ramp's own 0.9.
So the default is at least 0.9, and the minimum rule returns the ramp value.

## What this means for the project's numbers

The wheels inherit a default of at least 0.9. Every ramp surface in this
project is authored at 0.55 or 0.9. Under the minimum rule the contact
coefficient is therefore the ramp's value in every case.

**The authored friction figures are the contact figures.** The missing wheel
`<surface>` block is a real gap and should still be made explicit, but it has
not affected any published result, and the traction work in
[#25](https://github.com/skunal3318/Ramp-Autonomy/issues/25) can use the
authored `mu` as the contact `mu` without a correction.

Had the rule turned out to be the product, every contact would have been
running well below its quoted figure and both #25 and
[#3](https://github.com/skunal3318/Ramp-Autonomy/issues/3) would have needed
restating. It is worth saying that this was not the expected answer so much as
the lucky one.

## Scope

Measured on Gazebo Sim 8.11.0, gz-physics 7.8.0, DART 6.13.2, which is the
engine actually in use (see #33 for why that is not the engine the world files
name). Box on box contact, static ramp, no rolling and no suspension. A tyre
rolling under drive torque is a different contact problem, so this establishes
what `mu` means at a contact, not that a wheel behaves like a sliding block.
