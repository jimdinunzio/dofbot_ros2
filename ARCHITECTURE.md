# DOFBOT manipulation stack — how it fits together

Orientation for anyone (human or agent) picking this up. It says what each piece
does, where the load-bearing decisions live, and which numbers are measured
versus derived. It is not API documentation — the modules carry that in their
docstrings, and those docstrings are the primary source. This is the map.

Last verified against the tree on 2026-09-01: 160 tests pass, 3 skipped.

---

## The one-paragraph version

`pick(x, y, z)` takes an object's centre in `base_link` and executes
approach → grasp → attach → lift → carry → place, with no RViz in the loop. It
works because the arm's kinematics are solved in closed form in pure Python
rather than handed to MoveIt as a pose goal, which lets the whole sequence be
parameterised as *position + tool pitch* — five numbers for five joints. MoveIt
still does collision-aware planning for the long transits, and still owns the
planning scene. Perception will eventually plug in by calling `pick()`. The wrist
camera now looks twice inside that sequence — at the standoff and at carry —
and can abort it, but only ever on a definite no; see Seeing the pick.

---

## Packages

| package | what it is |
|---|---|
| `dofbot_description` | URDF, meshes, collision geometry. The physical model. |
| `dofbot_moveit` | Generated MoveIt config: SRDF, controllers, kinematics, RViz. |
| `dofbot_ctrl` | Everything we wrote. Kinematics, gripper model, pick sequence, hardware bridges. |
| `dofbot_arm_lib` | Vendor serial driver (`Arm_Lib`), repackaged. |
| `dofbot_interface` | Message/service definitions. |
| `arm_service` | **Not a colcon package** — no `package.xml`, so `colcon` passes over it. The XML-RPC front door: a remote caller asks for a pick, this runs the `ros2` commands. |

---

## dofbot_ctrl module map

Read in this order; each depends only on the ones above it.

```
joint_map.py         servo degrees <-> URDF radians. Zero offsets, signs, gripper range.
dofbot_kinematics.py closed-form FK/IK. Pure math, no ROS. The foundation.
gripper.py           jaw model: opening <-> angle, and where the fingers actually grip.
graspable.py         object catalogue. Dimensions, grasp width, grip height.
vision.py            wrist camera -> nanoOWL. present / absent / unknown.
scene_objects.py     catalogue object -> planning-scene geometry + held pose.
scene_markers.py     catalogue object -> RViz mesh marker. Visual only.
moveit_client.py     DofbotMoveIt: all ROS interaction with move_group + controllers.
pick_place.py        the sequence. Entry point `pick_place`.
```

Hardware-facing, separate from the above:

```
joint_state_mirror.py  reads servos -> /joint_states     (read-only, for RViz)
moveit_bridge.py       /joint_states -> servos           (drives the arm)
serial_port.py         who else has /dev/ttyTHS1 open. Shared diagnostic.
gui_teleop.py          manual jogging
calibrate_zero.py      per-servo zero offsets by encoder read
calibrate_view.py      the two camera gates, read off a real frame
vision_check.py        one camera question, answered on stdout; arm_server runs it
chassis_collision.py   SUPERSEDED — chassis/floor are real URDF links now
```

Measurement tools, both read-only:

```
measure_bus.py       servo-bus latency, sustained sweep rate, encoder noise floor
measure_tracking.py  /joint_states vs /servo_states — how far behind the arm runs
```

---

## The three ideas that explain most of the code

### 1. phi — tool tilt from vertical

`phi = theta2 + theta3 + theta4`, the sum of the three pitch joints, because they
share an axis. `phi = 0` points the tool straight up, `pi/2` horizontal,
`pi` straight down.

This is the arm's real parameterisation. A 5-DOF arm cannot hit an arbitrary
orientation, but it can hit any *tilt*, so `(x, y, z, phi, roll)` is exactly five
numbers for five joints and admits a closed-form solution.

**MoveIt's KDL solver is not broken** — measured, 116/120 exactly-reachable pose
goals solved at the 5 ms timeout it shipped with, and `computeCartesianPath`
returns `fraction = 1.000` on the segments used here. The reason we don't use
pose goals is that you must already know an achievable quaternion to ask for one,
and working that out *is* this arm's kinematics.

### 2. Hover — the TCP is not where the jaws grip

`Gripping_point_Link` is a fixed frame 68.09 mm from the wrist. The fingertips
are nowhere near it, and the four-bar swings them further out along the tool axis
as the jaws close. So the TCP target is the grasp point pulled **back** along the
tool axis.

Two offsets, and the distinction matters:

- `tip_offset_for(width)` — contact at the fingertip.
- `throat_offset_for(width)` — object seated against the back of the finger.
  This is what the pick sequence uses.

The throat derivation is `FINGER_DEPTH - width/2 - BACK_STOP_CLEARANCE`. **Half
the object is behind the contact line** — the jaws touch a round object at its
widest point, so a 66 mm can already fills 33 mm of the 40 mm face. Two earlier
attempts left that term out; one drove the arm 35 mm too deep.

Both are **lookups, never searches**. An earlier version searched for the
smallest collision-free backoff, which finds the deepest reach that doesn't trip
a contact — "not colliding" is not "gripping".

Corollary: never collision-check the *commanded* squeeze angle. The object stops
the jaws at its own width, so `grip_angle_for()` is a pose the gripper never
occupies while holding something. Check `jaw_angle_for(width)`.

### 3. The approach sweep

The feasible band of phi for a given target is narrow — sometimes 0.02 rad — and
it moves with reach. So `_feasible_approach` sweeps phi from `pi/2` to `pi` in
0.01 rad steps, screening each candidate with analytic IK (free, microseconds)
before spending a `/check_state_validity` call on the survivors.

The step size is not over-caution: a 0.05 step straddled a real 0.02-wide band
and reported "no workable approach" for a pose the arm reaches comfortably.

**It chooses three things.** Along with phi it picks the *grip height*, from
whatever band the catalogue entry allows, and the *standoff* — how long the final
straight-line approach is. Both are measured rather than demanded, the same shape
as `_reachable_lift`, which measures how far straight up the tool can actually
go. (`ik_best` is the third instance: it tries both elbow branches and keeps the
one nearest the seed.)

**The standoff is what sets the near edge, not the reach.** The grasp alone
solves from 0.164 m out; demanding a full 80 mm standoff at the same pitch walls
off everything inside 0.263 m. The reason is not obvious: the standoff pose is
the grasp pulled *back along the tool axis*, and at phi ≈ 2.2 that direction is
up and **inward**, into the same fold-up limit. So the standoff runs out before
the grasp does, and the target that will not solve is the one too **close**.
Measured, the can's band is 0.20–0.39 m and the near edge is set by the
`min_standoff` floor — a policy choice about how short an approach is acceptable,
since that final straight line is what makes the jaws slide over the object
instead of swinging into it.

**Being inside the band is not the same as being in a good place in it.** Only
0.28–0.32 m gets the full 80 mm standoff, the 80 mm grip height proven on
hardware, `grasp_pitch` on its preferred 2.2 and over 0.12 rad of joint margin
at once; 0.20–0.23 solves with none of them. Whoever drives the base should aim
for 0.30 — see `arm_service/README.md`, "Where to put the robot", for the whole
band and the constants that gate it.

**Ranked, not first-fit.** Candidates are scored on how much room the tightest
joint has left (`_joint_margin`) and on the standoff they support, because near
the inner edge "solvable" and "workable" come apart: solutions there sit hard
against a stop, with nothing left to absorb the error in where the object really
is. Both quantities are quantised, so that a difference smaller than the arm's
own backlash cannot outvote the grip height proven on hardware. `grasp_pitch` is
a tie-break among comfortable postures, not a starting point; at 2.2 it sits in
the 2.05–2.35 band a floor pick actually uses.

**The lift is measured and reported, never required.** It is not what gets the
object clear of the floor: interpolating from the grasp to `carry` raises the can
monotonically — measured at x = 0.19, 0.20 and 0.24, the can's base gains 27 mm
in the first tenth of that move and never dips. `move_named('carry')` does the
clearing; the Cartesian lift is only the safest *first* few centimetres, worth
taking when available and not worth failing a pick over.

---

## Seeing the pick

The sequence is otherwise blind. It is told where the object is, plans a grasp
to that coordinate, and closes the jaws whether or not anything is there. Two
camera checks confirm it: one at the pre-grasp standoff — *is the object where
I was told?* — and one at carry — *did it come with me?*

**The camera is the one on the wrist.** `Camera_Link` hangs off `arm4_Link`, so
it moves with the arm and looks past the jaws. It is a 640×480 UVC camera and
the only camera on this machine; the Oak-D in the Frames section is on the
chassis, wired to the robot's brain, and is not here.

### Three answers, and the third one is the point

| verdict | meaning | what a pick does |
|---|---|---|
| `present` | seen, where it has to be | carries on |
| `absent` | the view was good and it was **not** there | **aborts** |
| `unknown` | the question could not be asked | carries on, and says so |

`unknown` covers nanoOWL running something else and a dark frame. It is
deliberately not folded into `absent`: an unanswerable question is not a
negative answer, and treating it as one would make the GPU service a hard
dependency of every pick. **Vision is a confirmation added to the sequence,
never a prerequisite for it.**

### The camera cannot see its own grip point

**Measured 2026-09-05, and it is the thing standing in the way.** With a can in
the jaws, only the lid and the top centimetre or two reach the bottom edge of
the frame; no part of the gripper is visible; and nanoOWL detects nothing for
any wording — `a soda can`, `a coke can`, `a drink can`, `a beverage can`,
`the top of a soda can` all return nothing. Only `a red object` fires, at 0.12,
on the sliver that is visible. The detector itself is fine: `a picture frame`
scores 0.61 on the picture behind the arm, in 60 ms.

The URDF says why. In `arm4_Link`'s frame, z is the tool axis:

| joint | | xyz |
|---|---|---|
| `Camera_Joint` | arm4 → `Camera_Link` | −0.0481, 0, 0.0707 |
| `arm5_Joint` | arm4 → `arm5_Link` | −0.00215, 0, 0.078149 |
| `Gripping_Joint` | arm5 → grip point | z 0.068091 |

From the camera the grip point lies **43.3 mm sideways and 75.5 mm along the
tool axis — 29.8° off the camera's own mounting axis.** The horizontal field is
about 63° (estimated from the 66 mm can filling 66% of the frame width at that
range), putting the vertical half-angle near 25°. **The grip point sits roughly
five degrees outside the frame.** The camera is mounted parallel to the tool
axis, offset to one side, and never toes back in.

**No arm pose fixes it, and no ROI would either.** `arm5_Joint` is revolute
about z — the tool axis itself — and the grip point is 2.65 mm off that axis,
so rolling the wrist spins the jaws in place and moves nothing in the image.
The gripper projects to the same region of the frame in *every* posture the arm
can strike. That invariance is exactly what would make a fixed pixel rectangle
a sound rule; the trouble is the region is off the bottom of the sensor, and a
rectangle cannot filter detections that were never made.

The approach check is different, because **angle falls with range** — the same
43.3 mm offset subtends less the further down the tool axis the target sits:

| standoff | range | off-axis | |
|---|---|---|---|
| 0 mm | 75.5 mm | 29.8° | the grasp itself — outside the frame |
| 20 mm | 95.5 mm | 24.4° | at the edge |
| 40 mm | 115.5 mm | 20.5° | inside |
| 80 mm | 155.5 mm | 15.6° | comfortably inside |

So `approach` sees its subject comfortably, and `held` — which asks about the
grip point — sees only a sliver. **That is why `held` is a classification and
`approach` is a detection**, and it is the whole reason the two checks are not
the same mechanism. Aiming the camera down and inward by ten or fifteen degrees
would let both be detections, but it is a hardware change and nothing currently
needs it.

### The two checks work differently, and have to

| check | asks | how |
|---|---|---|
| `approach` | is the object where I was told? | **detection** — `[a soda can]`, take the box nearest the image centre |
| `held` | did it come with me? | **classification** — `(an empty gripper, a gripper holding a soda can)` |

`approach` is a detection because at a standoff the target is 15–25° off axis
and fully in frame: measured, a can on the floor detects at **0.797**. Of the
cans in view it takes the one **nearest the image centre**, because that is
where the arm is pointed; others sit off to the sides. No calibration needed.

`held` cannot be a detection — the held can is a truncated sliver at the bottom
edge and detection returns nothing for it under any wording. Classification
needs no detectable object, and measured on the sliver it returns **0.992
holding / 0.927 empty**.

**The held check requires a pose with no floor in frame, and `carry` is one** —
it points the tool well up, so the only can that can appear is the one being
held. That is what makes the question answerable, because classification reads
the *whole picture* and cannot tell a can in the jaws from one lying behind it:
measured, one in view reads as held at **1.000**. Designing the scene out beats
arguing with it, and it costs nothing, because **raising the arm does not move
the held can in the image** (camera and jaws are rigidly linked) — only the
background moves.

**So the answer is meaningful at `carry` and nowhere else.** The pick sequence
asks there. `is_holding()` can be called at any moment from any pose, and asked
at `ready` over a littered floor it will report `held` at 1.000 with empty
jaws — not a degraded answer, a confident wrong one.

`CLASSIFY_MIN_SCORE` (0.70) is the one floor: a classification **always**
returns a winner, so below it the answer is `unknown` rather than a verdict.

### Wording a classification prompt — three rules, each bought by a wrong answer

1. **The alternatives must cover the whole space, absence included.** `(a small
   can, a huge can)` calls an empty frame "a small can" at 0.79.
2. **They must differ only in the thing being asked about.** Any alternative
   that also describes the *scene* gets won by the scene: every pairing
   containing "a can on the floor" was a constant — all three frames are of a
   floor — and it took the **empty** one at 0.994.
3. **A prompt that is right on the frame you tried it on is not yet evidence.**
   Half the wordings tried were constants or inversions that read perfectly
   sensibly. `(a whole can, a cut off can)` called the truncated held can "a
   whole can" at 0.991.

Negation is weak wording besides: `(no soda can, soda can)` gets its negative at
**0.527**, a coin flip in a two-way softmax, because CLIP embeds a negated noun
close to the noun.

### The detector is nanoOWL, and it is already running

`nano-owl-service` (XML-RPC, port 8000, `nano_owl.service`) is an
open-vocabulary detector on this machine. Nothing in `dofbot_ctrl` loads a
model or touches the GPU. Two facts make it usable from here:

- **It is started in `network` frame-source mode**, so it opens no camera of
  its own. `/dev/video0` stays ours to open. (The older `tree_demo.py` grabs
  the camera *and* returns only annotated JPEGs, with the detections drawn on
  and then discarded — it cannot answer this question at all.)
- **It answers with structured boxes**, and `get_detections()` reports the
  `frame_seq` of the frame it ran on.

It is shared with the robot's brain, which uses it to *find* cans through the
Oak-D. The server holds **one prompt and one latest-detection slot, globally**,
so two things keep the uses from reading each other's answers: every pushed
frame carries a sequence number seeded from the clock, and we poll until *ours*
comes back; and the previous prompt is read before ours is set and put back
afterwards. Neither makes concurrent use safe — the server has no notion of a
session — and neither is meant to. Concurrency is not expected: the brain finds
a can and the arm then picks it, in that order.

### Asking from outside

`arm_server.is_holding()` runs `vision_check` and returns `held` as
**True / False / None**, the same three answers. `ok` says whether the question
was *asked*, not what the answer was — a can that is not there is a successful
query. It needs no `move_group`, so it answers whether or not the arm is
enabled.

---

## The two grippers

**The arm has two gripper configurations and they physically swap.** Bolt-on
finger extensions screw onto the stock fingers and come off with a screwdriver.

| profile | range | takes |
|---|---|---|
| `stock` | 0–60 mm, closes fully | the 30 mm test block, not the can |
| `extended` | 50–105 mm, never shuts | the 355 ml can, not the test block |

They are near-complements. Reaching the 66 mm can costs the bottom of the range,
so the 30 mm block falls straight between the extended fingers. This is why
`graspable.fits()` is a live function and not a stored verdict.

**One switch drives both sides:** environment variable `DOFBOT_GRIPPER`
(`stock` | `extended`, default `extended`). `gripper.py` picks its table from it;
`dofbot.urdf` reads the *same* variable via `$(optenv DOFBOT_GRIPPER extended)`
to include or drop the extension meshes.

An env var rather than a `xacro:arg` because `urdf_launch` invokes xacro with no
arguments at all, and display/mirror/gui_teleop all load the URDF through it.
**If the two sides disagree, every grasp is planned short by the length of the
extensions** and the arm drives into the object.

Current state: `extended`, `CALIBRATED = True`, 50–105 mm (safe 53–102),
`DEFAULT_SQUEEZE` 5.4 mm, `FINGER_DEPTH` 40 mm.

---

## Which numbers are measured, and which are derived

This distinction has caused more trouble than anything else in the project.

**Measured on hardware — do not recompute:**

- Extended jaw widths (`grip_span_table.txt`). Strongly non-linear: the first
  1.05 rad gives up 21 mm of span, the last 0.5 rad gives up 34 mm. A two-point
  fit is ~15 mm out mid-travel.
- `DEFAULT_SQUEEZE` = 5.4 mm, from the can held at a commanded 1.434 rad.
- The 52 mm the extension tips rise beyond the stock ones.
- Per-servo zero offsets in `joint_map.py`.
- Base geometry: plate 145.0 × 120.0 × 3.0 mm offset −13.5 mm in x; arm base
  r 40 mm cylinder z 3.0 → 82.8 mm.
- `Z0` = 107.5 mm. **This arm's standoffs are exactly 18 mm shorter than
  Yahboom's CAD**, so it differs from every shipped URDF. Shorter is *better*
  here — raising the shoulder buys height but costs floor radius.

- The camera's field of view — **estimated, not measured**: about 63°
  horizontal, from the 66 mm can filling 66% of the frame width at a known
  range in one frame. Good enough to explain why the grip point is out of
  shot; not good enough to compute with.
- `vision.APPROACH_MAX_OFFSET` — **not yet measured**. There is nothing to
  derive it from either: `Camera_Link`'s URDF
  pose gives the camera body's mounting, not the lens direction or the field of
  view, and the camera has no intrinsics. Both are `None`, which is *off*, not
  broken — the checks fall back to "is one in view at all". `calibrate_view`
  reads them off a frame.

**Legitimately derived from the URDF:**

- Stock tip offsets (11.7–42.1 mm), from the `arm5 → Rlink1 → Rlink2` chain onto
  the outermost vertex of the finger mesh. That's just the end of the finger.
- Everything in `dofbot_kinematics`, transcribed from URDF joint origins and
  cross-checked against `/compute_fk` to ~1e-16 m.

**Traps that have actually bitten:**

- Deriving jaw *width* from the URDF gave 25.8–85 mm. The FK was right; the
  fingertip contact point was *assumed* 30 mm along `Rlink2_Link`, a number with
  no basis. The real jaw face is coupler mesh geometry.
- Recomputing extended tip offsets from `[RL]link2_Link_Ext.STL` gave 11.1 mm
  instead of 52. **Those meshes are not in their link's frame** — they mirror
  about y = −6.5 and occupy z = 0..7.5 while the stock fingers occupy z = −8..−2.
  It was self-consistent, passed its own unit test, and would have driven the arm
  41 mm into every object.
- Deriving the standoff difference from a measured shoulder height gave 17.5 mm
  when two direct measurements of the same feature gave exactly 18.0. **Anchor on
  the difference between two like measurements, not on a difference against a
  value you computed.**

---

## Frames

`base_link` is the URDF root, and every number in this stack is expressed in it:
**+x forward** into the pick zone, **-x** back toward the chassis, **+z up**,
origin at the **underside** of the arm's mounting plate.

That is the arm's own frame. The arm is bolted to a mobile robot, and the
**robot frame is what the arm and the Oak-D camera share** — the camera is
eye-to-hand on the chassis, so it cannot be reached through the arm's FK. This
workspace has never defined a robot frame: the chassis is modelled *backwards*,
hanging off `base_link` as a child (`chassis.xacro`). The transform below is
what the robot-side session needs, and it is the inverse of that modelling.

Taking the robot origin as the usual `base_footprint` — the chassis axis,
projected to the ground:

```
robot_origin -> base_link      xyz = 0.265  0  <ground>      rpy = 0 0 0
```

| term | value | how it is known |
|---|---|---|
| x | 0.265 | chassis axis to plate, measured; `back_offset` in `chassis.xacro` |
| y | 0 | arm sits on the robot centreline — confirmed, not assumed |
| yaw | 0 | arm points dead forward — confirmed, not assumed |
| z | 0.022 carpet / 0.026 hard floor | plate underside above ground, measured |

**The z is not a constant.** The wheels sink, so a soft surface drops the whole
robot and brings the ground *closer* to the plate. Carpet is the smaller number
for exactly that reason. `DOFBOT_FLOOR` selects it (see `chassis.xacro`);
unset means carpet, which is where picking is currently tested.

Getting that env var wrong is not symmetric. Carpet's 22 mm models the ground
*higher* than a hard floor really is, which only costs 4 mm of low reach.
The hard-floor number on carpet puts the collision plane *below* the real
surface and lets the gripper plan into it.

---

## Collision geometry

The shipped URDF reused raw CAD visual meshes for `<collision>`: 1,049,211
triangles, `arm4_Link` alone at 246,098 against MoveIt's 10,000-vertex threshold.
move_group took 45–60 s to start and `/check_state_validity` was slow enough to
trip 10 s client timeouts.

Now ~166k triangles. `scripts/make_collision_meshes.py` writes convex hulls of
`arm1`–`arm4` (scipy only, no trimesh/open3d installed). `base_link` uses two
primitives instead — it is mostly air, and its hull filled 62% of the bounding
box.

**A convex hull is wrong for anything with a functional concavity.** `arm5_Link`
was hulled and then reverted: it is the gripper mount, a U-shaped yoke whose
opening is exactly where the fingers sit. Same reason the finger links keep full
meshes — hulling a finger fills the gap the object sits in, so the jaws read as
permanently closed.

**Judge a hull assembled, not per-part.** Numeric checks said the hulls were
conservative and correctly contained; only looking at them on the robot showed
arm5's was useless. `mirror.rviz` carries a second RobotModel display,
"RobotModel (collision)", off by default at Alpha 0.5, for exactly this.

Two facts that made hulls the right tool: several meshes are **not watertight**
(`base_link` had 1,565 non-manifold edges), which topology-preserving decimation
fights and qhull ignores; and **all ten arm-link pairs are already
`disable_collisions` in the SRDF**, so hull inflation between arm links cannot
cause a false positive.

---

## Running it

```bash
# simulation only, touches no serial port
ros2 launch dofbot_ctrl pick_place.launch.py rviz:=false bridge:=false

# on the robot, driving the real arm
ros2 launch dofbot_ctrl pick_place.launch.py

# then, in another terminal
ros2 run dofbot_ctrl pick_place -- --check-states
ros2 run dofbot_ctrl pick_place -- --plan-only 0.30 0.0 0.039

# one half per invocation, and one of --pick/--place is required. --pick ends
# holding the object at 'carry'; what is being carried lives in the planning
# scene until --place, so the two share no process and the pause between them
# is where the caller decides where the object goes
ros2 run dofbot_ctrl pick_place -- --pick 0.30 0.0 0.039   # upright can on carpet
ros2 run dofbot_ctrl pick_place -- --place

# recovery after a run that died partway: clear the scene, let go, go home
ros2 run dofbot_ctrl pick_place -- --reset

# a greeting gesture, planned and collision-checked like any other move
ros2 run dofbot_ctrl wave_arm -- --waves 3

# needs no launch and no move_group, only the wrist camera and nanoOWL beside it
ros2 run dofbot_ctrl vision_check -- --held
ros2 run dofbot_ctrl calibrate_view -- --detect --shot /tmp/view.png
```

`x y z` is the object **centre** in `base_link` — for something resting on the
floor that is half its height up, not 0.

Read-only mirror (physical arm → RViz):

```bash
ros2 launch dofbot_ctrl mirror.launch.py
```

**For a remote display, run RViz and nothing else** on the other machine:
`ros2 launch dofbot_moveit moveit_rviz.launch.py`. Running `pick_place.launch.py`
with `bridge:=false` there starts a *second whole stack* — the symptom is two
`/move_action` servers, "Ignoring unexpected goal response", and `CONTROL_FAILED`.

Tests are pure Python, no ROS graph needed:

```bash
python3 -m pytest src/dofbot_ros2/dofbot_ctrl/test/ -q
```

---

## Named poses

`up`, `down`, `init`, `ready`, `carry`, `over_trash` — defined in **two places
that must agree**: `NAMED_STATES` in `moveit_client.py` and the `group_state`
entries in `dofbot_description.srdf`.

Validate with `pick_place --check-states`, which checks each against the live
planning scene. This is what stops a pose being eyeballed: the vendor's original
`init` collided with `chassis_link` and nobody noticed until it was checked.

`init` was hand-posed on the real arm and read out with `calibrate_zero`, so its
joint values are physical fact. `over_trash` is a **placeholder for a detected
bin**: the posture was measured on the arm off the left front, then rotated
on-axis (`theta1` is the only joint that moved), so it now drops straight in
front at TCP (0.167, 0.001, 0.334). It assumes a rim below ~0.3 m and 0.167 m
out, which is barely past the chassis. The bin's real position is to come from
perception, and `place()` will then take coordinates the way `pick()` does.

---

## Driving it from another machine

`arm_service/` is the seam for a caller that has no ROS: an XML-RPC server that
turns `pick_can(x, y, z)` into the `ros2` commands above. It holds no rclpy state
of its own — one long-lived `ros2 launch` it supervises, one short-lived
`ros2 run` per command — so it restarts freely and the stack it drives is the
same one a terminal drives.

```bash
src/dofbot_ros2/arm_service/start_arm_server.sh      # or the systemd unit
python3 arm_client.py --url http://192.168.55.1:8001/ pick 0.30 0.0 0.039
```

`enable_arm`, `disable_arm`, `pick_can`, `place_can`, `move_to_state`,
`reset_arm`, `wave_arm`. Motion is serialized: a second request is answered `busy` rather
than queued, because a queued arm command executes against a world that has
moved on. See `arm_service/README.md`.

---

## Working agreements

- **Do not measure from the mesh. Ask Jim.** He has the physical part and Fusion.
- He measures heights from the mounting plate's **top** face; `base_link` z=0 is
  the **underside**. He adds the 3 mm himself.
- Don't tell him to support the arm — it never falls.
- He verifies in RViz; don't burn tokens on exhaustive automated testing.
- Keep symbol names in one font (plain `phi`, not switching to code font).

---

## Known open items

- **The extension meshes render detached in RViz.** The URDF places them at
  `origin 0 0 0` but they were exported in a different frame. Cosmetic —
  `gripper.py` does not depend on it — but it looks wrong.
- **Stock jaw widths are still a two-point stub.** `CALIBRATED` is False for that
  profile only. Never caliper-swept.
- **One finger has noticeable backlash**, so the pair is not symmetric whatever
  the URDF's mimic multiplier says. The gap at a commanded angle has hysteresis
  (the table was swept closing, and grasps close, so the normal path matches),
  and the object does not sit on the gripper centreline — but `scene_objects`
  attaches it symmetrically, so a small pose error is carried into the place.
- **The place target is a fixed pose, not a found one.** `over_trash` drops
  straight in front at 0.167 m; nothing looks for the bin, and `place()` takes
  no coordinates. This is the next thing perception plugs into after `pick()`.
- **The held numbers were measured at a pose with floor in view, not at
  `carry`.** Carry's background is plainer, which should only help, but the
  holding/empty pair wants re-shooting there before 0.99/0.93 are quoted as
  carry's figures.
- **`APPROACH_MAX_OFFSET` is unset**, so the approach check accepts a can
  anywhere in frame rather than only near where the arm is pointed. It needs
  a real standoff frame first; `MIN_SCORE` wants tuning on the same frames.
- **The classification numbers are one frame per state.** Decisive as far as
  they go, but they want repeating across lighting and can placements.
- **Execution is open-loop.** MoveIt believes the mock joints, not the encoders.
  A stalled or blocked servo goes undetected. Whether to change that is an open
  question, now being answered by measurement rather than by argument — see
  [Closing the loop](#closing-the-loop-what-is-known-and-what-is-being-measured).

---

## Closing the loop, and why it is not needed

**The servos already close their own loop.** Each YB-SD15M runs its own position
controller; `_write_pos` hands it a target and a duration and it gets there and
holds. There is no missing control loop — an outer ROS loop over a 12 ms serial
sweep would be a slower cascade around a faster one, which adds lag, not
authority. What making the encoders authoritative would buy is *state* and
*fault detection*, not control.

Measured 2026-08-22, all read-only, tools in `dofbot_ctrl/tuning/`:

| | measured |
|---|---|
| Six-servo sweep | 12.2 ms → **82 Hz**, no drops in 400+ sweeps |
| Per-joint poll ceiling | a servo re-queried inside **8–10 ms** does not answer |
| Encoder noise at rest | **1 count = 0.082°** (servo 5: 2 counts) |
| Tracking during a pick | **~225 ms lag**, residual 5.5–15.3 mrad |

The bus is not a constraint. The per-joint gap is: a sweep faster than ~10 ms
per servo starts dropping, so 82 Hz sits above a cliff rather than having
headroom — 50 Hz is comfortable, 100 Hz is not.

**The arm tracks well, just late.** Removing a single fitted delay drops RMS
error from 72–110 mrad to 5.5–15.3 mrad, against a 1.4 mrad noise floor. The
225 ms is our own tuning, not the hardware: the bridge writes at 10 Hz with
`track_time_ms = 200`, so every write is superseded before the servo arrives.

So closed loop is not the next move:

- Encoders authoritative *today* would be **harmful** — `current_joints()` feeds
  `cartesian_move`'s seed, and 225 ms stale means planning from where the arm was.
- A JTC path tolerance would have to exceed 236 mrad (13.5°) not to abort every
  trajectory, which detects nothing.
- There is no torque or current feedback in the protocol (only 0x2A write
  position, 0x38 read position, 0x28 torque enable, 0x05 set id), so a stall can
  only ever be inferred from position error.
- A real `SystemInterface` would have to reimplement the `grip_time_ms` special
  case in C++, not retire it: re-commanding servo 6 every cycle is exactly what
  makes a slow close snap shut.

If the lag is ever worth cutting, that is a `track_time_ms` and write-rate
question, and it helps whether or not the loop is ever closed.

---

## Gotchas that have cost real time

- **Orphaned processes.** `pkill -f "...launch.py"` kills the launcher, not its
  children. Two orphaned `joint_state_mirror` processes both held
  `/dev/ttyTHS1` and corrupted each other's reads — indistinguishable from a dead
  arm. `joint_state_mirror` now detects this and names the rival PID.
- **A powered-off arm looks like a mesh problem.** No `/joint_states` → no TF →
  RViz reports "No transform" for every link and draws only `base_link`. Both
  hardware nodes now say so explicitly.
- **`ET.write` strips comments** and reformats the whole file. Use it for
  structural URDF edits, then re-insert comments. **Never use regex on the
  URDF** — it was corrupted twice by append-instead-of-replace.
- **XML comments cannot contain `--`.** Bit us three times.
- **RViz rewrites its own config on exit** and strips YAML comments from it.
- `mirror.rviz` needs `Durability Policy: Transient Local` — `robot_state_publisher`
  latches `/robot_description`, so a Volatile subscriber only gets it by winning a
  startup race. Speeding up the stack exposed this.

---

## Deliberately out of scope

Oak-D integration, the camera TF (see **Frames** for the robot-frame half of
it that is already measured), hand-eye calibration, and
visual servoing. The layer above attaches at exactly one seam: `pick(x, y, z)`.

When perception arrives it supplies the **position**; `graspable.py` already
supplies the size, grasp width and grip height. A depth camera returns a point on
the near *surface*, so the caller steps along the view ray by the object radius to
reach the centre — deliberately, once, in the caller.
