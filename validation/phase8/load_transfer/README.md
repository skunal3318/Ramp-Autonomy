# Does the simulator reproduce braking load transfer?

Measured for [#25](https://github.com/skunal3318/Ramp-Autonomy/issues/25).

**Answer: no, and the reason matters.** Gravity-driven load transfer is reproduced
accurately. Braking-driven load transfer is absent, because `DiffDrive` imposes
the deceleration without passing the force through the wheel joints.

## Why this was measured

#25 predicts the uphill axle unloads under braking by `m*a*h/L`, saturating at
0.663 m/s^2 against a configured ceiling of 2.0. That is paper maths. It needed
either confirming or killing.

## Method

`wheel_force_sensors:=true` adds a force/torque sensor to each wheel joint,
off by default. `make_world.py` injects the ForceTorque system into a copy of a
world. `run_brake_test.py` drives the robot on flat ground, brakes, and reports
per-axle load with a standard error.

Per-wheel loads are summed per axle. The solver splits load unevenly left to
right on a mass-symmetric robot resting level (111.44 N against 84.56 N), which
cancels in the axle totals but means **per-wheel normal loads from this
simulator should not be trusted individually**. That is worth knowing for
[#8](https://github.com/skunal3318/Ramp-Autonomy/issues/8).

## Result 1: gravity transfer is reproduced, and precisely

Robot parked on the ramp, pitch varied, nothing moving:

| slope | front axle | rear axle | total | imbalance | `Mc*g*cos` |
|---|---|---|---|---|---|
| 8 deg | 167.89 N | 220.30 N | 388.19 N | 52.41 N | 388.58 N |
| 16 deg | 136.50 N | 240.31 N | 376.81 N | 103.81 N | 377.20 N |
| 25 deg | 98.05 N | 257.21 N | 355.26 N | 159.16 N | 355.64 N |

Total matches `Mc*g*cos(theta)` at every angle, so the sensor is sound. The
imbalance follows

```
imbalance = 0.960 * Mc * g * sin(theta)
```

to within 0.0001 across all three angles.

## Result 2: braking transfer is absent

Flat ground, cruise at 0.8 m/s, commanded stop. Deceleration measured at
2.105 m/s^2, matching `DiffDrive`'s `max_linear_acceleration` of 2.0.

| | front | rear | imbalance |
|---|---|---|---|
| at rest | 196.00 N | 196.00 N | 0.00 N |
| cruising | 195.88 N | 197.30 N | -1.42 N |
| braking | 200.86 N | 192.70 N | +8.16 N |

Transfer measured: **4.79 +/- 1.77 N**. By the relationship fitted above, a
longitudinal force of `Mc*a` = 84.2 N should give about 81 N.

## Why: the force never reaches the joints

Summed longitudinal force through all four wheel joints:

| phase | sum of Fx |
|---|---|
| at rest | 0.00 N |
| cruising | -3.46 N |
| braking | 2.52 N |

Decelerating a 40 kg chassis at 2.105 m/s^2 requires 84.2 N. The joints carry
2.5 N of it.

The chassis really is decelerating, so `DiffDrive` is imposing that motion as a
velocity constraint rather than reacting it through the wheel contacts. No
longitudinal force at axle height means no pitching moment, which means no
vertical transfer. Gravity behaves differently because it acts on the chassis as
a body force, so the joints have to react it, which is why result 1 works.

## What this means for #25

**The axle-unloading prediction cannot be validated in this simulator.** It is
not that the prediction is wrong; it is that the mechanism it depends on is not
modelled in the actuation path. Any sim run would confirm the prediction by
construction or deny it by construction, and neither would be evidence.

It needs hardware, and specifically per-wheel torque or force feedback, which is
what the platform in [#11](https://github.com/skunal3318/Ramp-Autonomy/issues/11)
is being built with.

Also worth noting for anyone reading the trusted matrix: if braking does not
transfer load in simulation, then every descent result in this project was
produced with the axle loads staying closer to even than they would on a real
robot. That makes the simulated descents *easier* than reality, not harder.

## Reproducing

```
./make_world.py ../../../src/ramp_autonomy_gazebo/worlds/ramp_world_sharp.sdf /tmp/w.sdf
RAMP_WS=$(pwd)/../../.. ./run_brake_test.py 0.8 out.json
```

The static slope figures came from spawning the robot on a pitched slab and
reading the sensors at rest; the generator for that is inline in the issue
thread rather than kept here, since it only patches one pose value.

## Caveats

Contact sensors are declared on the wheels under the same flag, intended to give
ground normal force directly. The topics appear with correctly matched collision
names but never publish. Unresolved, and not needed for the result above.
