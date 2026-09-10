# How docking fails, and which guard catches it

Docking looks like a solved problem until you try to make it work every time.
The behaviour is short, the geometry is simple, and the tolerance is a couple
of centimetres -- and then a robot that docks perfectly ninety times in a row
parks itself against the funnel lip and flattens its battery overnight.

This note lists the failure modes that actually occur, in the order they bite,
and names the guard in `src/autodock/states.py` that is responsible for each.
Everything here is modelled in the simulator, so every claim can be reproduced
by running the examples.

---

## 1. The mechanical tolerance, and where the numbers come from

None of the thresholds in `DockingPolicy` are free parameters. They are all
derived from one piece of metal and one camera mounting.

The dock modelled in `DockGeometry` is a V-funnel that narrows onto two
spring-loaded charging contacts:

| quantity | value | what it is |
|---|---|---|
| `funnel_outer_half_width` | 90 mm | half-width of the dock's outer face; wider than this and the robot misses the dock body entirely |
| `lateral_tolerance` | 25 mm | half-width of the funnel throat minus half the docking shoe |
| `yaw_tolerance` | 6 deg | past this the shoe jams on the funnel wall instead of sliding along it |
| `shoe_length` | 120 mm | length of shoe that has to pass the throat |
| `DockingPolicy.overtravel` | 40 mm | how far the drive may keep pushing past where it expected contact before the attempt is failed |

The criterion for a successful mate is **not** two independent budgets. A shoe
of length `L` held at heading `psi` sweeps a corridor of half-width

```
swept = |lateral| + L * |sin(psi)|      must be <= 25 mm,   and |psi| <= 6 deg
```

At the 6 degree heading limit the second term alone is 12.5 mm, so heading
error eats half the lateral budget before the robot is off-centre at all. That
coupling is the reason `ALIGN` exists as a separate state and the reason the
blind-segment planner optimises `swept` rather than lateral offset (see
`DockingController.plan_blind_arc`).

The two behaviour-level windows are then derived from that number:

* **coarse window at the standoff** -- `align_lateral_tol` 20 mm,
  `align_yaw_tol` 3 deg. Failing it means the approach went badly enough that
  another run at it is cheaper than continuing.
* **fine gate at the commit range** -- `commit_swept_budget` 13 mm,
  `commit_yaw_budget` 3.5 deg, applied to the *predicted contact pose* under
  the planned blind segment, not to where the robot is standing. 13 mm is the
  25 mm mechanical tolerance minus roughly 12 mm of allowance for everything
  that happens after the gate: residual estimator error, and drift over the
  blind segment.

---

## 2. Lateral error cannot be removed at the end

A differential drive has no sideways degree of freedom. There is no command
that moves the robot 10 mm to its left. Lateral offset can only be traded
against heading and against distance travelled, so removing 10 mm of it needs
tens of centimetres of path.

That makes the ordering of the behaviour forced rather than stylistic:

* lateral error is removed during `APPROACH`, by pure pursuit onto the dock
  axis, where there are metres of path to spend;
* `ALIGN` turns on the spot, which fixes heading and **cannot** fix lateral
  offset -- so if the estimate says the robot is off the axis at the standoff,
  the machine sends it to `REPOSITION` rather than letting it rotate uselessly
  (`lateral_out_of_tolerance`);
* by the commit range there is only 350 mm of path left, which buys a few
  millimetres of correction on a gentle arc and nothing more.

**Guards:** `aligned`, `lateral_out_of_tolerance`, `realign_exhausted`.

---

## 3. The last stretch is blind

The marker is a fixed size and the camera is not on the floor. Below
`MarkerModel.min_range` (300 mm by default) the tag fills or leaves the frame
and there are no observations at all. Whatever error exists at that moment is
the error the robot arrives with, plus drift.

The design response is a `COMMIT` state: stop, average ten stationary
detections, plan the blind segment, and make an explicit go/no-go decision.
Standing still is not wasted time -- it removes motion blur and pose latency,
which are the two largest error sources in a moving fiducial fix.

**Guards:** `commit_range_reached`, `commit_locked`, `commit_rejected`,
`commit_abandoned`.

---

## 4. Wheel mismatch: the error that retrying does not fix

The dominant odometry error on a small indoor base is not encoder resolution.
It is that the two wheels are not identical -- tyre wear, tyre pressure, an
uneven payload, one wheel slipping slightly more on the same carpet. A
fractional difference `g` between the effective wheel radii makes a robot
commanded to drive dead straight follow an arc of curvature
`g / track_width`, so its lateral departure after `d` metres of blind travel is

```
lateral drift ~ (g / track) * d^2 / 2          heading drift ~ (g / track) * d
```

**Quadratic in the blind distance.** Doubling the blind segment quadruples the
lateral error; the measured version of this is
`tests/test_odometry.py::test_lateral_drift_over_a_blind_segment_grows_quadratically`.

The part that matters operationally: this is a *property of the robot*, not of
the attempt. It is drawn once per episode in `DriveTrain.sample` and held for
the whole run, so a robot whose wheels are badly matched fails, backs off,
and fails again the same way. Retries buy nothing against it. That is visible
in the sweep: at drift scale 5 the success rate plateaus around 50 % no matter
how many attempts the policy allows, because the failures are not independent
draws.

The cures are all outside the retry policy: calibrate the wheel radii,
shorten the blind segment by moving the camera lower and further forward, or
estimate the systematic curvature online from previous approaches.

**Guards:** none. This is the failure mode that no guard catches before
contact, which is exactly why it dominates the boundary.

---

## 5. Planar pose ambiguity

A square marker viewed nearly head-on has two pose solutions that project to
almost the same four corners. When corner noise is comparable to the
difference between them, the solver returns the wrong one: the true pose
mirrored about the viewing ray, so the reported heading is out by roughly twice
the incidence angle. It is a large, structured, *confident* error, it is more
likely at long range where the tag is small in the image, and averaging does
not remove it.

Two things handle it: the estimator gates each fix on its innovation, and
`pose_locked` refuses to act on a single detection during acquisition.

**Guards:** `pose_locked`, `acquisition_lost`.

---

## 6. A gated filter that wedges itself shut

Gating creates its own failure. If one bad fix is accepted while the covariance
is still wide -- a flipped fiducial pose during acquisition is the usual culprit
-- the state is now wrong, the covariance shrinks anyway, and every subsequent
*correct* detection sits more than `gate_sigmas` away and is thrown out. The
filter quietly rejects the only data that could fix it, and the behaviour above
it flips between `SEARCH` and `ACQUIRE` at the control rate forever.

This happened during development, and it is why `marker_detected` is defined as
"a fix was *accepted*" rather than "the detector reported something", and why
`DockEstimator` re-initialises itself after `reset_after_rejections`
consecutive rejections. Any filter with a gate needs a watchdog like this.

**Guards:** `marker_detected`, `acquisition_lost`, `fix_stale`.

---

## 7. An over-confident filter

The same covariance that lets the gate work will collapse below the error the
filter actually has, because the dominant odometry error is a *bias* and a
random-walk process model cannot represent a bias. Left alone, the estimator
here reported 0.15 degrees of heading sigma while carrying 1.8 degrees of
heading error -- and the commit gate believed it, because the gate checks
sigma.

`EstimatorNoise.pos_floor` and `yaw_floor` put a floor under the covariance, so
the filter never becomes more certain than the level at which its own model is
wrong. Fixing this moved the nominal success rate from 92 % to 100 %.

**Guards:** `commit_locked` (which checks `lateral_sigma`, and is only as good
as that number is honest).

---

## 8. Contact is not docking

A bump switch says the robot hit something. It does not say the contacts
mated. A robot that treats contact as success ends up parked against its dock
overnight with a flat battery, and the fault report says "it docked fine".

`ENGAGE` holds a gentle push for 300 ms to let the spring contacts seat, then
`VERIFY` waits for charging voltage. Contact without charge is a failed
attempt and goes to `RETREAT`.

**Guards:** `bump_detected`, `engage_settled`, `charge_confirmed`,
`charge_absent`.

---

## 9. Stopping short, and pushing forever

The blind segment is dead reckoned, so the robot's idea of how far it still has
to go is wrong by the odometry range error (measured 5th/95th percentile at
nominal drift: -7.6 mm / +4.3 mm). If odometry over-reports distance the robot
stops short of the contacts; if the robot is too far off the axis it drives
straight past the dock and touches nothing at all.

Either way the drive must stop. `travel_budget_exceeded` caps the blind
segment at the estimated remaining range plus `DockingPolicy.overtravel`, which is
both the "stopped short" detector and the protection against a robot grinding
its wheels against a dock it has already reached.

**Guards:** `travel_budget_exceeded`.

---

## 10. Never finding the dock, and never stopping

A robot handed a "go dock" command has whatever heading its last task left it
with, and the marker has a 70 degree field of view and a 60 degree incidence
limit. If it is dropped somewhere the marker cannot be seen from, nothing in
the behaviour will ever start. The measured consequence is in the misalignment
sweep: performance is flat until the start position is far enough off the axis
that the marker is too foreshortened to detect, and then acquisition fails
outright.

Three bounds stop the robot trying forever: a search timeout, a retry budget,
and a hard episode timeout that is listed first in every state so it always
wins.

**Guards:** `search_exhausted`, `attempts_exhausted`, `episode_timeout`.

---

## Summary table

| failure | what it looks like | guard |
|---|---|---|
| Off the axis at the standoff | robot rotates on the spot achieving nothing | `lateral_out_of_tolerance`, `realign_exhausted` |
| Committed on a noisy fix | confident single detection, wrong pose | `pose_locked`, `commit_locked` |
| Flipped fiducial pose | heading out by twice the incidence | `pose_locked` + estimator innovation gate |
| Filter wedged shut by its own gate | search/acquire flip-flop at the control rate | `marker_detected` + estimator reset watchdog |
| Over-confident covariance | small sigma, large error, gate passes it | `commit_locked` (checks sigma) |
| Wheel mismatch over the blind segment | arrives millimetres off, retries fail identically | none before contact -- `charge_absent` after |
| Stopped short / drove past | no bump, drive still running | `travel_budget_exceeded` |
| Touching but not charging | "it docked fine", flat battery | `charge_absent` |
| Dock never visible | robot spins forever | `search_exhausted` |
| Anything else | robot never gives up | `attempts_exhausted`, `episode_timeout` |
