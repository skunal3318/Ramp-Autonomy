# The simulation world

Two ramp-only worlds (`ramp_world_sharp.sdf`, `ramp_world_curved.sdf`) exist for regression testing against this project's earliest validated
results. `construction_site.sdf` is the primary test environment since phase 8 - a structural blockout of a small construction site, grounded in
cited real-world standards rather than guessed dimensions. This is a summary; `docs/construction_site_design.md` has the full session-by-session
design record - every citation in full, the complete Fuel asset sourcing and placement-verification tables, and the live traversal-test results
each design decision was checked against.

## Layout

```
                                          N (+Y)
        +-----------------------------------------------------------+
        |                    site office (8x6 m)                    |
        |                                                            |
GATE -> | access  |      laydown area       | ramp |  elevated level |
(6.1 m) | road    |      (20x10 m, ~11%)     | (16deg, filleted) | (15x12 m, z=3.3 m)
        |         |                          |      |                |
        |                    waste/debris staging (10x6 m)          |
        +-----------------------------------------------------------+
        X=0      X=8                       X=28  X=41            X=56
```

Overall footprint: 59 m x 32 m = 1,888 m², built around the robot's fixed spawn pose (x=1.0, y=0.0, z=0.23, facing +X) - the gate sits at x=0 so
the spawn point lands just inside it.

Each dimension traces to a cited source rather than an arbitrary choice:

- **Perimeter hoarding, 2.4 m, one 6.1 m gate.** 2.4 m is the commonly recommended height for construction hoarding (2.0 m is the typical
  minimum), above OSHA's 1.83 m minimum for commercial/industrial perimeters. A single controlled gate matches standard site-security guidance
  that all traffic should funnel through one trackable access point. 6.1 m (20 ft) is the standard fire-apparatus access road width, sized for
  two-way delivery traffic plus emergency access.
- **Elevated level, 3.2 m rise.** Standard single-storey floor-to-floor height is commonly cited in the 3.0-3.6 m range; 3.2 m sits centrally in
  it.
- **The ramp, 16°, filleted, 0.45 friction, 4.5 m wide.** 16° is this project's own already-validated slope from the earlier ramp worlds, well
  inside OSHA 1926.451(e)(5)(ii)'s hard ceiling of roughly 20° for ramps and walkways. The filleted (not sharp) transition reuses the curved
  world's validated 3-step design (0° -> 5.33° -> 10.67° -> 16°) because a sharp, unfilleted seam is a known, live-reproduced tip-over mechanism
  in this project's own history (see `retrospective.md`). 0.45 friction is this project's own measured practical climbing floor, not a generic
  "dry concrete" guess - 0.35 is above the theoretical static-friction minimum (`tan(16°) ≈ 0.287`) but was confirmed live to not reliably climb.
  4.5 m width, wider than the earlier ramp worlds' 3.0 m, is a direct response to a real physical tip-over this project found on a 3.0 m panel
  from lateral drift during descent.
- **Laydown area, ~11% of footprint.** Site-logistics guidance commonly cites 10-15% of total project footprint for laydown; this site's 200 m²
  against 1,888 m² total is 10.6%.
- **Site office and waste staging**, placed per standard layout guidance: the office near the gate but clear of the laydown area's main
  corridor, waste/debris staging as its own distinct zone, separated from both laydown and the office.

## Floor 2: a two-floor building

The site extends past Floor 1 (the elevated level) with a second storey 3.2 m higher, added without touching the validated Floor-1 route - the
only pre-existing line of the world file that changed is the east perimeter fence, moved 31 m east to make room.

Ramp 2 is a translated copy of ramp 1's exact validated geometry (same 7-element fillet chain, same 16°, same 0.45 friction, same 4.5 m width) -
reusing a design already proven rather than inventing a new one to validate. Floor 2 (18 m x 16 m, top at z=6.55) carries guardrails (42 in top
rail, midrail, toeboard per OSHA 1926.502(b)), a debris net on the open edge, a 5-bay facade scaffold, a mason tower scaffold against a racked
CMU wall, six structural columns with impalement-capped starter bars, a scenery stair tower (robot-inaccessible - the robot only ever uses
ramps), and a material staging area near the ramp-2 exit. A 3x3 m marked delivery pad sits in line with the ramp-2 exit lane; nothing attaches
or detaches there yet beyond the delivery session's own detach mechanism (see `results.md`).

Every join between flat slabs and ramp elements is checked by exact top-face corner math, not visual inspection - the project's own history
includes a live tip-over caused by exactly this kind of unchecked seam (see `retrospective.md`). Every new object's placement is checked against
every existing model (0.3 m overlap margin) and against the robot's actual logged ground-truth path (1.5 m buffer).

Floor 2's static collision shapes cost roughly 2.5 microseconds of physics-step time each - a first, fully detailed pass dropped headless
real-time factor to about 0.4. Scenery now uses envelope collisions instead of each prop's own detailed multi-shape collision (a scaffold is a
solid base box, not individually modeled tubes), while the robot's lane, ramps, slabs, columns, walls, and barriers keep exact collision
geometry. That brought max-speed real-time factor from 0.41 to 1.12; during an actual verification run it holds a mean of roughly 0.92-0.93.

## Physics engine, and what the friction numbers mean

**The engine is DART, whatever the world files say.** Every world here carries a `<physics>` element whose `type` attribute
names an engine, and that attribute does not select anything. Gazebo Harmonic ships `dartsim`, `bullet`, `bullet-featherstone`
and `tpe`, and loads its default when asked for something else, without a warning. Confirmed by reading the startup log:
`Loaded [gz::physics::dartsim::Plugin]`. The attribute now reads `dart` so the files describe what runs, but changing it is
cosmetic: both spellings load the same plugin. See [`#33`](https://github.com/skunal3318/Ramp-Autonomy/issues/33).

**Friction is a contact property, and the engine takes the lower of the two surfaces.** Every `mu` in this project is authored
on one surface, while the coefficient that governs a contact is derived from both. That derivation was measured by bisecting
the slide angle of a box on an incline, with the two surfaces deliberately given different values:

- the rule is the **minimum**. Box 0.8 on ramp 0.5 holds at 26° and slides at 27°, bracketing the effective coefficient to
  [0.4877, 0.5095]. The product (0.400) and both means (above 0.63) are excluded.
- a body with **no `<surface>` block inherits at least 0.9**. With no block at all, on a ramp of 0.9, it holds at 41° and
  slides at 43°, which is the ramp's own value returned by the minimum.

The robot's wheel `<collision>` blocks carry no `<surface>` element, so they inherit that default. Every ramp surface in this
project is authored at 0.45, 0.55 or 0.9, all below it. **So the authored figure is the contact figure in every case**, and the
friction values quoted throughout these docs mean what they appear to mean.

That is a result rather than an assumption, and it could have gone the other way: had the rule been the product, every contact
would have been running well below its quoted value. The wheel surface block should still be made explicit rather than
inherited. Harness, method and full result tables are in [`validation/friction_rule/`](../validation/friction_rule/README.md).

## Props

22 props populate Floor 1 and a further set populate Floor 2, sourced from real Gazebo Fuel assets - queried against Fuel's own API and checked
for real dimensions before placement, not assumed to exist or guessed at. See `THIRD_PARTY_LICENSES.md` for the full list, sources, and
licenses. Three props placed near the gate (two fence panels, three cones, one speed sign) were later removed permanently once a drift
characterization effort showed no position on the access road's width could satisfy this project's own margining convention against the
return leg's open-loop lateral drift - a safety finding, documented in `retrospective.md`, not a licensing concern.

## Images

`docs/images/` has GUI screenshots: `floor2_overview.png`, `floor2_delivery.png`, `floor2_staging.png`, `floor2_stair_core.png`,
`floor2_top_view.png`, `ramp2_side.png` (the two-floor site), and `mesh_gate_closeup.png`, `mesh_ramp_climb.png`, `mesh_elevated_site.png` (the
MiR100 chassis mesh, live GUI, loaded).
