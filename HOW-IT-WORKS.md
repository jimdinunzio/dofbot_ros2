# How it works — mechanism and derivations

The companion to [ARCHITECTURE.md](ARCHITECTURE.md). That file is the *map*:
what's where, how to run it, what to watch out for. This one is the
*mechanism*: the actual math, the sequence step by step, and why each derivation
is shaped the way it is.

Read the module docstrings for the fullest treatment — `gripper.py` and
`dofbot_kinematics.py` in particular carry the complete argument including the
attempts that failed. This file connects them.

---

## Can I just read the plan instead?

**For the vendor archaeology, yes. For how the system works, no — it will
mislead you.**

`~/.claude/plans/ok-we-have-an-bright-orbit.md` is the original implementation
plan (27 July 2026). It remains genuinely valuable for one thing: it is the
distilled result of a whole session spent reading Yahboom's three source trees
(`dofbot_pro_ws_src`, `DOFBOT_AI_Code`, `LargeModel_ws_src`). That work should
not be redone. Go there for:

- Which vendor lesson lives in which file, and the URLs
- Why their `set_scene.cpp` attach recipe is shaped as it is
- How their mono-camera "quasi-3D" demos actually work, and why they can't carry
  this task
- The eye-in-hand transform chain in `grasp_desktop.py`, for when the Oak-D
  arrives
- That the nano URDF is byte-identical to ours, so nano-sourced constants apply
  directly while Pro-sourced ones must be re-derived

**But it was written before implementation, and measurement disproved several of
its load-bearing claims.** A future session that reads it as fact will inherit
those errors. Specifically:

| plan says | reality, measured |
|---|---|
| L108: "KDL cannot solve this arm, and that is the real blocker" | KDL solves **116/120** exactly-reachable pose goals at the 5 ms timeout it shipped with, 119/120 at 0.5 s |
| L124: "MoveIt's own `computeCartesianPath` is unusable here" | Returns `fraction = 1.000` on the approach and lift segments actually used |
| L115: shoulder at **Z0 = 0.1255** | **0.1075** — this arm's standoffs are exactly 18 mm shorter than the CAD |
| L121: "All five limits are ±1.57" | `arm1_Joint` is **−1.9024 / +1.8850** — a 217° sector, not 180° |
| L289: "60 mm maximum opening ... cannot clear the body at all" | True of the stock jaws; the **extended** profile reaches 105 mm and takes the can |
| L399: "`floor_z_top:=-0.015` is correct — leave it alone" | Fitted against the old Z0. Now suspect, needs a hardware re-check |

The analytic IK is still the right design — but for a different reason than the
plan gives. Not "KDL fails", which is false. It is that a pose goal needs a
quaternion, and on a 5-DOF arm only orientations on the reachable manifold have
solutions, so to *ask* MoveIt for a pose you must already have solved this arm's
kinematics. Plus: exact, branch-aware, limit-aware, and fast enough to run at
every Cartesian waypoint.

---

## The kinematic chain

From `dofbot_description/urdf/dofbot.urdf`, all metres:

```
base_link
  │  z 0.0745                    ← 18 mm shorter than CAD; measured, not vendor
  ▼
arm1  yaw   theta1  about z
  │  z 0.033
  ▼
arm2  pitch theta2  about y      shoulder, Z0 = 0.1075
  │  z 0.08285                                        L1
  ▼
arm3  pitch theta3  about y
  │  z 0.08285                                        L2
  ▼
arm4  pitch theta4  about y
  │  x −0.00215, z 0.078149
  ▼
arm5  roll  theta5  about z
  │  x −0.00265, z 0.068091
  ▼
Gripping_point_Link             the TCP frame — but NOT where the jaws grip
```

Yaw, three pitches, roll. The three pitch joints share an axis, which is the
fact everything else rests on.

**Small y offsets** (5e-05, 0.00055, …) lie *along* the pitch axis, so they never
rotate relative to each other and simply sum to a constant 0.605 mm lateral
offset of the tool from the plane theta1 sweeps. Modelled exactly — it costs three
lines and takes the FK/IK round-trip from ~1 mm to machine precision.

The TCP's own x/y offset sits **after** the theta5 roll, so it swings in and out
of the pitch plane as the wrist rolls. Also modelled exactly.

`Gripping_Joint`'s fixed `rpy="3.1416 -1.5708 0"` reorients the TCP frame but
does not move it, so it never enters the position kinematics.

---

## Forward kinematics

Because the pitch joints share an axis, `phi = theta2 + theta3 + theta4` is the
tool's tilt from vertical, and the arm collapses to a planar 3R chain in the
half-plane that theta1 aims:

```
r = L1·sin θ2 + L2·sin(θ2+θ3) + L3·sin(φ + Δ)
z = Z0 + L1·cos θ2 + L2·cos(θ2+θ3) + L3·cos(φ + Δ)

x = r·cos θ1 − lat·sin θ1
y = r·sin θ1 + lat·cos θ1
```

`L3` and `Δ` come from the last two link offsets combined: `L3 = hypot(dx, dz)`,
`Δ = atan2(dx, dz)` — a small tilt because those offsets have an x component.
`lat` is the accumulated 0.605 mm lateral term.

`fk(joints, tip)` returns `(x, y, z, phi, roll)`. `tip` selects `'tcp'` or
`'arm5'` purely by switching L3 — the second exists because Yahboom's solver
targets the arm5 rotation centre, so it enables a like-for-like comparison.

**Verified against `/compute_fk` to ~1e-16 m** over 200 random states, and
against an independent 4×4 transform chain built by parsing the URDF inside
`test_kinematics.py`. Two different algorithms reading two different sources.

---

## Inverse kinematics

Given `(x, y, z, phi, roll)`:

1. **theta1 from the bearing, corrected for the lateral offset.** The tool sits
   `lat` to one side of the theta1 plane, so the plane must be aimed slightly off
   the bearing to the target:
   `rho = hypot(x,y)`, `r = sqrt(rho² − lat²)`, `theta1 = atan2(y,x) − atan2(lat, r)`

2. **Wrist centre.** Subtract the tool segment, whose direction phi fixes:
   `rw = r − L3·sin(φ+Δ)`, `zw = z − Z0 − L3·cos(φ+Δ)`

3. **Planar 2R** for theta2/theta3 by law of cosines. Two branches — `elbow='up'`
   takes the negative root, bending away from the base.

4. **theta4 = phi − theta2 − theta3**, and **theta5 = roll**.

5. **Per-joint limit check.** Not one symmetric constant — `JOINT_LIMITS` is a
   tuple of `(lo, hi)` read from the URDF's `<limit>` tags, because `arm1` is
   both wider and asymmetric.

`ik_best()` tries both elbow branches and returns the one nearest a seed, which
is what keeps a Cartesian segment from flipping configuration halfway through.

### The one documented limitation

Only poses with **non-negative in-plane radius** are recovered. Configurations
folded back *through* the base axis are valid on the real arm but alias onto a
yaw flipped by pi, because theta1 comes from `atan2(y, x)`.

Measured over 4000 random valid joint vectors: 1988 round-trip exactly, 2012
fail, and **every single failure is a negative-r configuration** — zero from any
other cause. The forward working volume is entirely r > 0. Recorded so the ~50%
figure is never mistaken for a bug.

### Two fixed-point traps, both real

- A fully-extended elbow (θ3 = 0) puts the wrist centre at exactly `L1+L2`, which
  comes back from FK as `0.16570000000000001` and failed a bare `>`. That rejects
  an ordinary posture — the SRDF `up` and `down` states both have θ3 = 0. Hence
  `_REACH_EPS = 1e-9`.
- `SAFE_MAX_WIDTH` is `round(MAX_WIDTH - CLEARANCE, 6)` because `0.060 - 0.003`
  is `0.056999999999999995`, which rejected a request for exactly 57 mm.

---

## The gripper model

### Why the jaw width cannot be derived

The gripping face is mesh geometry on the link2/link3 coupler and **no joint
frame lies on it**. An attempt to compute widths from the Rlink1/2/3 mimic chain
gave 25.8–85 mm: the FK was correct, but the fingertip contact point was
*assumed* to be 30 mm along `Rlink2_Link` — a number with no basis (0.03 is the
length of link 1). Widths are caliper measurements. Full stop.

The extended profile's measured table is strongly non-linear: the first 1.05 rad
gives up only 21 mm of span, the last 0.5 rad gives up 34 mm. A two-point fit is
~15 mm out mid-travel, which is why the stub the stock profile still uses is
flagged `CALIBRATED = False`.

### Why the tip offset *can* be derived

The fingertip is just the end of the finger, and the mesh does know that. Run the
`arm5 → Rlink1_Joint → Rlink2_Joint` chain onto the outermost vertex of the
finger mesh. That is legitimate — it is a different question from where the flat
face sits.

The offset varies with opening because the four-bar swings the fingers **outward
along the tool axis** as it closes:

```
opening   stock tip offset   extended
60 mm     +11.7 mm           —
30 mm     +33.0 mm           +89.0 mm
0 mm      +42.1 mm           —
```

Positive means *past* the TCP frame, so the TCP must sit that far **back** from
the contact point — at a downward grasp pitch, "hover a bit higher".

### Tip versus throat

`tip_offset_for()` puts the contact at the fingertip. `throat_offset_for()`
seats the object against the back of the finger, and that is what the pick
sequence uses:

```
advance = FINGER_DEPTH − width/2 − BACK_STOP_CLEARANCE
throat_offset = tip_offset − max(0, advance)
```

**Half the object is behind the contact line.** The jaws touch a round object at
its *widest* point, so a 66 mm can already fills 33 mm of the 40 mm face and
leaves 5 mm of advance — not 40, and not a fixed 10. Both wrong versions were
tried on hardware: using the whole face drove the arm 35 mm too deep; a fixed
observed gap is width-blind, so it under-uses a narrow object's room and drives a
wide one into the stop.

It **never retreats** — an object wider than `2·(FINGER_DEPTH − BACK_STOP_CLEARANCE)`
is already against the stop, and the right answer there is the fingertip.

`width/2` assumes the object is round or square in the grip plane, true of
everything in the catalogue. A long flat box would have to supply its own
half-extent.

### Both are lookups, never searches

An earlier version searched for the smallest backoff that was merely
collision-free. That finds the **deepest reach that doesn't trip a contact** —
which is not gripping at the tips, it's the arm buried to the knuckles. Visible
immediately in RViz and wrong in kind: *not colliding is not gripping.*

**Corollary:** never collision-check the *commanded* squeeze angle.
`grip_angle_for()` deliberately asks for narrower than the object so the servo
loads up, but the object stops the jaws at its own width. Check
`jaw_angle_for(width)` — the state the gripper actually occupies.

---

## The pick sequence, step by step

`pick(x, y, z)` — `(x, y, z)` is the object **centre** in `base_link`.

**1. Clear stale scene objects** (`_clear`). Must happen *before* the approach
search: the sweep collision-checks each candidate against the live scene, and a
grasp puts the jaws around the object, so a leftover copy rejects every
candidate. Leftovers are the normal case — `--plan-only` deliberately leaves the
object standing, `--no-place` ends with it attached, and any failed run leaves it
wherever it got to. The scene is read before anything is deleted, because
`detach`/`remove` are not no-ops on an absent object. Detach before remove:
removing a *world* object doesn't touch an *attached* one.

**2. Choose a feasible approach** (`_feasible_approach`). Three things are being
chosen, not one — the tool pitch, the grip height, and how long the final
straight-line approach is. For every (grasp_height, phi) pair:

```
contact  = grasp_point(obj@height, x, y, z)   # object geometry only
hover    = throat_offset_for(grasp_width)     # gripper geometry only
grasp    = back_off(contact, phi, hover)              ← the TCP target
standoff = the LONGEST that solves, from `standoff` down to `min_standoff`
pre      = back_off(contact, phi, hover + standoff)
mid      = midpoint(pre, grasp)
```

All three poses are screened by analytic IK (microseconds); the whole nested
sweep is 64 ms and ~4300 `ik_best` calls, so nothing here is worth optimising.
The midpoint is not redundant: the reachable set is not convex, so two good
endpoints do not imply a good line between them.

**The standoff is measured, not demanded, and it is what sets the near edge.** A
fixed 80 mm rules out every target inside 0.263 m while the grasp itself solves
from 0.164 m. The standoff pose is the grasp pulled back *along the tool axis*,
which at phi ≈ 2.2 points up and **inward** — into the arm's inability to fold up
tight. So it runs out before the grasp does, and the unsolvable target is the one
too *close*, which is the opposite of what "cannot reach" suggests.

**Candidates are ranked, not taken first-fit.** The score is the tightest joint's
distance from its limit (`_joint_margin`), then the standoff, then the object's
preferred grip height, then the configured pitch — the last two as tie-breaks
among postures that are already comfortable. Both leading terms are quantised
(`MARGIN_ENOUGH`, `STANDOFF_ENOUGH`), so a difference smaller than the arm's own
backlash cannot walk the grip height off the value proven on hardware.

Only the ranked survivors cost a `/check_state_validity` call, at most
`MAX_STATE_CHECKS` of them, and each is checked **with the jaws at
`jaw_angle_for(width)`**.

Then `_reachable_lift` measures how far straight up the tool can actually go —
another lookup-by-measurement rather than a fixed 100 mm, because at a steep
pitch the reachable band in z is only a few centimetres deep. **It is reported,
never required.** Gating on it would reject good grasps for a step that is not
load-bearing: interpolating from the grasp to `carry` raises the can
monotonically, +27 mm of base height in the first tenth of the move and never a
dip, measured at x = 0.19, 0.20 and 0.24. Step 10 clears the floor on its own.

**3. Add to the scene** — after the search, before the `plan_only` branch. Not
earlier or the search rejects everything; not later or `--plan-only` checks its
poses against a world with no object in it.

**4. `move_named('ready')`, `open_gripper()`** — with the object present and
still *forbidden*. The transit has no business passing through it.

**5. Allow object↔gripper collision.** The last few centimetres are "in
collision" by definition. Only that pair is allowed, so the floor and chassis
stay checked. This edits the ACM by **reading the current matrix and modifying
it** — sending an ACM in a scene diff *replaces* it, which would wipe every
`disable_collisions` pair from the SRDF.

**6. `move_pose(*pre)`** — OMPL plans this, collision-aware, joint-space goal.

**7. `cartesian_move(grasp)`** — we plan this. Interpolate, IK every waypoint with
seed carry-over, validate each against the live scene, refuse the whole segment
if any fails.

**8. `set_gripper(obj.grip_angle())`** — the object's own tuned squeeze if it has
one, else `DEFAULT_SQUEEZE`.

**9. Attach**, with the held pose derived (below), then **forbid** the pair
again — the attached object's `touch_links` now cover finger contact, and leaving
the allowance would let the object pass through the fingers after it's put down.

**10. `cartesian_move` straight up** by the measured lift — which may be zero —
then **`move_named('carry')`**, which is what actually gets the object clear of
the floor. The Cartesian lift is the *controlled* first few centimetres, not the
clearing move; see step 2.

`place()`: `over_trash` → open → detach + remove → `carry`.

---

## Where the held object goes

When the jaws close, the object must be attached to the gripper so MoveIt carries
it. Its pose in the `arm5_Link` frame:

The object was standing upright when grasped, so its own axis is world-vertical
and the gripper has carried it rigidly since. World +z expressed in `arm5_Link`:

```
up = (−sin(φ)·cos(roll),  sin(φ)·sin(roll),  cos(φ))
```

since arm4 is base rotated by `Ry(φ)` and arm5 adds `Rz(roll)`. The jaws hold the
object at `TOOL_OFFSET + tip_offset` along the tool axis, and its centre is
`centre_offset()` further along `up`.

This **generalises** Yahboom's `set_scene.cpp` recipe rather than copying it.
Theirs puts a 0.03-tall cylinder at `y = 0.075` in `Arm5_Link` — correct only for
their frame (+y is the tool axis on the Pro, ours is +z), their TCP distance, and
an object gripped at its very top with its axis along the tool. Put
`grasp_height = height` and `phi = 0` into the general form and you get exactly
their `z = TOOL_OFFSET_z − height/2`.

Verified against move_group: attaching a 120 mm cylinder gripped 30 mm up at
phi = 2.6 and reading it back from `/get_planning_scene` matched our arm5-frame
pose transformed through `Gripping_Joint`'s rpy, to five decimals. That check is
frozen in `test_grasp_model.py`.

**Touch links must name every end-effector link exactly.** MoveIt silently
ignores names it doesn't recognise — Yahboom's `"llink2"`/`"rlink2"` are Pro
names against our `Llink2_Link`/`Rlink2_Link`, so a copy-paste leaves the held
object colliding with the gripper holding it and every later plan fails for no
stated reason. `arm5_Link` is in the list too: its mesh runs to z = 0.0503 while
the TCP is at 0.0681 and the fingers are only ~50 mm long, so a held object is
20 mm from the wrist body.

---

## Trajectory timing

`IterativeParabolicTimeParameterization` has no Python binding on Humble, so
`cartesian_move` assigns `time_from_start` itself: a trapezoidal profile over the
waypoint list, distance measured as the max-norm over joints (the quantity the
speed cap applies to).

`max_joint_speed` defaults to ~30 °/s and **is not cosmetic** — `moveit_bridge`
writes at 10 Hz and skips deltas under 0.5 servo-degrees, so a faster trajectory
is one the real arm cannot follow; it lags and arrives late.

---

## Data flow

```
                    ┌──────────────┐
   simulation:      │ ros2_control │  mock joints, invents state
                    │   (fake)     │
                    └──────┬───────┘
                           │ /joint_states
                           ▼
  pick_place ──► move_group ──► robot_state_publisher ──► /tf ──► RViz
      │              ▲                    ▲
      │              │ /check_state_validity
      │              │ /apply_planning_scene
      └──────────────┘ /move_action, follow_joint_trajectory, gripper_cmd

   hardware:  /joint_states ──► moveit_bridge ──► servos
              moveit_bridge ──► /servo_states     (encoder_rate > 0, default off)
              servos ──► joint_state_mirror ──► /joint_states  (read-only)
```

The two hardware nodes are **mutually exclusive** — both want `/dev/ttyTHS1`, and
pyserial takes no exclusive lock, so two owners interleave bytes and corrupt
reads *silently*. Both now detect a rival on the port and name its PID, through
the shared `serial_port.rival_warning`.

Execution is **open-loop**: MoveIt believes the mock joints, not the encoders. A
stalled or blocked servo goes undetected.

`moveit_bridge` can read the encoders back onto **`/servo_states`** — never onto
`/joint_states`, because a second publisher there makes MoveIt's idea of the
robot state flicker between two sources. Off by default (`encoder_rate: 0.0`),
since reads and writes share one bus and one single-threaded executor.

That topic exists so the disagreement can be measured. It is: the arm tracks the
commanded path to within a few mrad but runs **~225 ms behind**, which is the
bridge's own `track_time_ms`, not the hardware. The servos close their own
position loops, so there is no missing control loop to add — see **Closing the
loop, and why it is not needed** in [ARCHITECTURE.md](ARCHITECTURE.md).

---

## What perception will attach to

Exactly one seam: `pick(x, y, z)`.

`graspable.py` already supplies size, grasp width and grip height — a 355 ml can
is 66 × 122 mm and no camera needs to measure that. Perception supplies only the
**position**.

A depth camera returns a point on the near *surface*, so the caller steps along
the view ray by the object radius to reach the centre. Deliberately, once, in the
caller — rather than Yahboom's blind `+0.02/+0.01/+0.01` constants, which partly
encode that correction and partly encode calibration error.

Three upgrades worth building in, all cheap because the can is a *known* object:
aggregate depth over the bbox ROI (an empty can is shiny metal and a prime stereo
dropout); correct near-surface → centre explicitly; and set grasp height from
known geometry rather than from the detection point. See the plan's
`LargeModel_ws_src` section for the vendor's transform chain — ours is
eye-to-hand (chassis-mounted), so their FK term drops out and `EndToCamMat`
becomes a static `base_link → camera` TF.
