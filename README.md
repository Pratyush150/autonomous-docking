# autonomous-docking

A differential-drive robot drives itself onto a charging dock and ends up
within a few millimetres and a couple of degrees of a fixed piece of metal,
using a camera that stops working before it gets there.

![Python](https://img.shields.io/badge/Python-3.9%2B-3776AB?logo=python&logoColor=white)
![NumPy](https://img.shields.io/badge/NumPy-only%20hard%20dependency-013243?logo=numpy&logoColor=white)
![Tests](https://img.shields.io/badge/tests-142%20passing-2f6f4e)
![License](https://img.shields.io/badge/License-MIT-blue)

---

## The problem

Docking looks like a solved problem right up until you need it to work every
time. The behaviour is five states on a whiteboard, the geometry is a straight
line, and the tolerance is a couple of centimetres. Then a robot that docked
ninety times in a row parks itself against the funnel lip, the fault report
says "it docked fine", and the battery is flat in the morning.

Three things make it hard, and they are all in the last half metre.

**The sensor gets worse as the requirement gets tighter.** A fiducial big
enough to detect at 3 m fills or leaves the frame at 0.3 m, and on most robots
the camera is mounted above and behind the docking shoe, so the marker is gone
before contact. The final approach is dead reckoning. Whatever error the robot
has when it loses the marker is the error it arrives with, plus drift.

**A differential drive cannot correct lateral error in place.** There is no
command that moves the robot 10 mm to its left. Lateral offset can only be
traded against heading and against distance travelled, so an approach that
arrives at the dock 20 mm off the axis has no move left that fixes it. It has
to be fixed metres earlier, or not at all.

**Contact is not docking.** A bump switch says the robot hit something. Only
charging voltage says the contacts mated. Every docking system that skips that
distinction eventually reports success while sitting there discharging.

This repository is the docking *software* -- perception model, dock-relative
estimator, guarded state machine, controller -- and a Monte Carlo harness that
measures where the design stops working. It is a simulation study. There is no
real robot and no real dock behind these numbers.

---

## What it does

- **A state machine, not a tangle of ifs.** Thirteen states, 35 transitions, and
  every transition guarded by a named pure function of one context object. The
  guards live in a registry, so the test suite can assert that every guard is
  tested, that every guard is used by the table, and that the abort transition
  is evaluated first in every state.
- **A go/no-go commit gate.** The robot stops at the last range where the
  marker is still observable, averages ten stationary detections, plans its
  blind segment, and refuses to enter the dock if the *predicted contact pose*
  falls outside a budget derived from the mechanical tolerance.
- **A blind segment that is planned, not just driven.** The last 300 mm is one
  constant-curvature arc chosen by a scan over its single free parameter,
  minimising the swept half-width at the contact plane -- the criterion the
  dock actually applies.
- **Differential-drive kinematics done honestly.** Non-holonomic constraints,
  wheel-speed and wheel-acceleration limits, and a clamp that scales `v` and
  `omega` together so saturation slows the robot down instead of straightening
  its path.
- **Perception that degrades the way fiducials really degrade.** Range-dependent
  noise, a minimum range below which nothing is reported at all, an incidence
  limit, dropouts that cluster near the range limits, two-frame latency, and
  planar pose ambiguity that returns a confidently mirrored heading.
- **An estimator with the three fixes a gated filter needs.** Latency
  compensation by odometry replay, an innovation gate for flipped poses, a
  covariance floor so a systematic wheel mismatch cannot be modelled away as
  white noise, and a divergence watchdog so the gate cannot lock the filter out
  of its own data.
- **A bounded retry policy.** Three attempts, two re-approaches per attempt, a
  search timeout, and a hard episode timeout listed first in every state.
- **Monte Carlo over seeds, and sweeps that find the boundary.** Distributions
  rather than one run, and a two-dimensional success surface over odometry
  drift and blind-segment length.

---

## Quickstart

```bash
git clone https://github.com/Pratyush150/autonomous-docking
cd autonomous-docking
pip install -r requirements.txt

python3 examples/demo.py --demo      # one narrated run + 80 episodes, ~20 s
python3 -m pytest -q                 # 142 tests, ~3 min, no network, no display
```

Use it from source, no install needed:

```python
import sys; sys.path.insert(0, "src")

from autodock import EpisodeConfig, run_campaign, summarise
from autodock.montecarlo import with_blind_radius

config = with_blind_radius(EpisodeConfig(), 0.5)   # camera loses the tag at 0.5 m
results = run_campaign(config, episodes=200, workers=8)
print(summarise(results, config.dock).headline())
```

Or drive the same core one tick at a time, which is what a robot would do:

```python
from autodock.runtime import DockingRuntime

runtime = DockingRuntime()
command = runtime.step(now=t, dt=0.04, observation=fix_or_none, bump=switch, charging=voltage)
send_twist(command.v, command.omega)
runtime.feed_odometry(measured_v, measured_omega, 0.04)
```

---

## Figures

Every figure is generated by a script in `examples/`. To regenerate all of them:

```bash
python3 examples/sweep.py --workers 8          # writes docs/sweep.csv
python3 examples/make_figures.py --workers 8   # writes docs/figures/*.png
```

![Approach geometry with the dock axis, the standoff pose, the commit gate and the blind segment](docs/figures/approach-geometry.png)
The geometry every guard is written in. The standoff is computed from the dock
pose, not chosen in the world frame, so the final approach is a straight line
along the dock's own axis by construction. Right: the last 450 mm, with the
+/-25 mm mechanical tolerance band and the commit gate at 350 mm.

![Sixty trajectories from randomised start poses, converging onto the dock axis, with a close-up of the last 450 mm for a nominal and a worn drive train](docs/figures/trajectories.png)
Sixty randomised starts, heading uniform over the full circle. Pure pursuit
spends metres of path removing lateral error, because that is the only currency
a differential drive can pay it in. The two close-ups are the same 60 seeds on
a nominal drive train and on one three times worse matched. The difference
between the two panels is drift, and almost all of it is spent in the last
350 mm, where the robot cannot see the marker.

![Contact error against initial lateral offset and initial heading, and the distribution of contact error](docs/figures/contact-error.png)
Where the robot started barely matters. How well its wheels are matched
decides everything, and it decides it in the tail rather than the median.

![Success rate against odometry drift, blind-segment length and initial misalignment](docs/figures/success-sweep.png)
Success rate along each axis, with the 95th percentile contact error on the
right-hand scale.

![Success rate as a heat map over odometry drift and blind-segment length](docs/figures/success-boundary.png)
The boundary. The black contour is 95 % success.

![State machine timeline and dock-frame error for a run that had to retry](docs/figures/state-timeline.png)
A run on a badly matched drive train: it reaches the dock, the contact switch
closes, no charging voltage appears, and `charge_absent` sends it back out for
a second attempt that succeeds. The shaded bands are stretches with no
detection -- the acquisition sweep at the start, and the blind segment at the
end of each attempt.

---

## How it works

```
      camera + fiducial detector                    contact switch     charge sense
      (an ArUco/AprilTag pipeline;                        |                  |
       not part of this repo)                             |                  |
                 |                                        |                  |
                 v                                        |                  |
        MarkerObservation                                 |                  |
        late 2 ticks, range-dependent noise,              |                  |
        dropouts, sometimes a mirrored pose               |                  |
                 |                                        |                  |
   wheel         v                                        |                  |
   encoders --> DockEstimator  <-- odometry twist         |                  |
                 innovation gate                          |                  |
                 covariance floor                         |                  |
                 divergence watchdog                      |                  |
                 latency replay                           |                  |
                 |                                        |                  |
                 v                                        v                  v
          DockError(s, lateral, yaw) + sigmas + age  --> GuardContext <-------+
                 |                                        |
                 |                              DockingStateMachine
                 |                              35 guarded transitions,
                 |                              evaluated in order,
                 |                              abort first in every state
                 |                                        |
                 |                                        v
                 |                                    DockState
                 |                                        |
                 +--------------------> DockingController <+
                                              |
                                        (v, omega)
                                              |
                                        clamp_twist        scale both together,
                                              |            preserving curvature
                                        body_to_wheels
                                              |
                                        rate_limit_wheels  per-wheel slew limit
                                              |
                                              v
                                         differential drive base
```

**The dock frame is the only frame that matters.** Origin at the centre of the
contact plane, `+x` along the direction the robot must travel to enter. Every
guard, every control law and every metric is written in it as
`(s, lateral, yaw)`: remaining distance along the axis, signed offset from the
axis, heading relative to the axis. The simulator is the only thing that knows
about a world frame.

**One tick, in order.** Take a fiducial observation (which may be absent, is
two ticks old, and may be a mirrored solution). Fold it into the estimator,
re-propagated forward through the odometry that has accumulated since its
timestamp. Build the guard context. Evaluate the transition table. Ask the
controller for a twist. Clamp it to the wheel envelope, slew-limit the wheels,
and convert back to the twist the base can actually produce. Push that through
the imperfect drive train to move the robot, and integrate the *commanded*
wheels through *nominal* geometry to get odometry. The robot acts on the
second; it is graded on the first.

**The behaviour.**

```
SEARCH ----------marker_detected-------> ACQUIRE
ACQUIRE ---------pose_locked-----------> APPROACH          pure pursuit onto the dock axis
APPROACH --------standoff_reached------> ALIGN             turn in place, 0.75 m out
ALIGN -----------aligned---------------> FINAL_APPROACH    line following, closed loop
ALIGN -----------lateral_out_of_tol----> REPOSITION -> APPROACH
FINAL_APPROACH --commit_range_reached--> COMMIT            stop, average, plan, decide
COMMIT ----------commit_locked---------> BLIND_APPROACH    350 mm, open loop, one arc
COMMIT ----------commit_rejected-------> REPOSITION
BLIND_APPROACH --bump_detected---------> ENGAGE            seat the spring contacts
ENGAGE ----------engage_settled--------> VERIFY
VERIFY ----------charge_confirmed------> DOCKED
VERIFY ----------charge_absent---------> RETREAT -> SEARCH
RETREAT ---------attempts_exhausted----> ABORTED
<any state> -----episode_timeout-------> ABORTED
```

`ALIGN` turns on the spot. That fixes heading and, by construction, fixes
nothing else -- so when the estimate says the robot is off the axis at the
standoff, the machine sends it back out to fly the approach again rather than
letting it rotate uselessly. That transition is the non-holonomic constraint
written down as a guard.

`COMMIT` is where the design lives. The robot stops, because a stationary robot
is the only configuration in which a fiducial pose is free of motion blur and
latency error. It averages ten detections. It plans the blind segment -- one
constant curvature, chosen by scanning its single free parameter to minimise
the predicted swept half-width at the contact plane. And then it checks that
prediction against a budget, not its current position: the question is not
"am I close to the axis", it is "does the best blind segment available from
here still land inside the mechanical tolerance with margin for the drift that
comes after". Those are different questions and the second one is the useful
one.

### The mechanical tolerance, and why it is coupled

A shoe of length `L` held at heading `psi` and offset `lateral` sweeps a
corridor of half-width `|lateral| + L*|sin(psi)|`. With `L` = 120 mm and a
25 mm throat, six degrees of heading error costs 12.5 mm -- half the lateral
budget -- before the robot is off-centre at all. Lateral tolerance and heading
tolerance are not independent budgets, which is why the blind-segment planner
optimises the coupled quantity rather than nulling lateral offset. The full
derivation is in [docs/FAILURE_MODES.md](docs/FAILURE_MODES.md).

---

## Worked example

Real output, pasted unedited:

```
$ python3 examples/demo.py --demo
```

```text
------------------------------------------------------------------------------
Single episode, seed 1
------------------------------------------------------------------------------
start: 2.58 m out along the dock axis, -716 mm off it, heading +52 deg
drive train: commanded straight, actually curves at -0.0445 1/m

  t [s]  from            guard                    to             
   7.16  search          marker_detected          acquire        
   7.28  acquire         pose_locked              approach       
  17.44  approach        standoff_reached         align          
  17.76  align           aligned                  final_approach 
  22.32  final_approach  commit_range_reached     commit         
  22.84  commit          commit_locked            blind_approach 
  27.76  blind_approach  bump_detected            engage         
  28.08  engage          engage_settled           verify         
  28.52  verify          charge_confirmed         docked         

contact pose:  lateral +2.8 mm, heading +0.92 deg, swept half-width 4.7 mm (tolerance 25 mm)
blind segment: 291 mm driven with no marker in view
dead reckoning was off by +5.8 mm in range at contact
detections: 377 used, 34 dropped
outcome: docked after 28.5 s, 0 retries

------------------------------------------------------------------------------
Monte Carlo, 80 randomised starts and disturbance draws
------------------------------------------------------------------------------
80 episodes, success 100.0%, contact 100.0%, mate|contact 100.0%, median 23.5 s

| metric | 5th | median | 95th |
|---|---|---|---|
| |lateral| at contact (mm) | 0.32 | 2.74 | 12.19 |
| |heading| at contact (deg) | 0.16 | 0.90 | 3.18 |
| swept half-width at contact (mm) | 1.08 | 4.60 | 19.30 |
| longitudinal offset at stop (mm) | -0.00 | -0.00 | 0.00 |
| dead-reckoning range error (mm) | -7.41 | -1.09 | 5.20 |
| blind segment driven (mm) | 291.20 | 299.60 | 305.34 |
| time to dock (s) | 16.39 | 23.48 | 29.16 |
| retries used | 0.00 | 0.00 | 0.00 |

outcomes: {'docked': 80}
retries:  {0: 80}

state entries across the campaign:
  search           3
  acquire          83
  approach         82
  align            80
  final_approach   80
  commit           80
  blind_approach   80
  engage           80
  verify           80
  docked           80
```

---

## Measured results

All numbers below are produced by the commands shown. Nothing is quoted from
anywhere else.

### Nominal drive train

```bash
python3 examples/monte_carlo.py --episodes 400 --workers 8
```

```text
400 episodes, success 100.0%, contact 100.0%, mate|contact 100.0%, median 23.2 s

| metric | 5th | median | 95th |
|---|---|---|---|
| |lateral| at contact (mm) | 0.30 | 3.68 | 11.80 |
| |heading| at contact (deg) | 0.11 | 0.95 | 3.11 |
| swept half-width at contact (mm) | 1.08 | 5.50 | 17.95 |
| longitudinal offset at stop (mm) | -0.00 | -0.00 | 0.00 |
| dead-reckoning range error (mm) | -7.64 | -1.31 | 4.30 |
| blind segment driven (mm) | 294.00 | 299.60 | 308.00 |
| time to dock (s) | 17.67 | 23.24 | 29.21 |
| retries used | 0.00 | 0.00 | 0.00 |

outcomes:          {'docked': 400}
retries used:      {0: 400}
inside tolerance:  100.0% of episodes ended charging (mechanical tolerance 25 mm swept, 6 deg heading)
```

How to read that table:

- **`swept half-width` is the quantity the dock actually applies**:
  `|lateral| + 120 mm * |sin(heading)|` at the contact plane, against a 25 mm
  tolerance. Lateral offset and heading error are not separate budgets.
- **Longitudinal offset at the stop is zero because the dock is a physical
  stop.** The robot drives until the contact switch closes, so range is
  resolved mechanically rather than by odometry. The informative longitudinal
  number is the line below it: the dead-reckoning range error, how wrong the
  robot's idea of the remaining distance was when it arrived. Get that wrong in
  the other direction and the robot stops short of the contacts, which is what
  `travel_budget_exceeded` is for.
- **Contact-pose statistics are taken over the episodes that actually touched
  the dock.** An episode that drove straight past it has no contact pose, and
  averaging it in as zero error would flatter the result.
- **Time is reported over successful episodes.** A failed episode's duration is
  a property of the timeout, not of the design.

Four hundred episodes with no failures does not mean the design cannot fail.
By the rule of three it bounds the failure rate at about 0.75 % with 95 %
confidence, and no further than that. The useful question is not what the
nominal number is, it is how much margin sits behind it -- 18 mm of swept error
at the 95th percentile against a 25 mm tolerance is not a lot -- and that is
what the sweeps below measure.

### The same design on a worn drive train

Drive-train asymmetry three times the nominal spread -- the fleet a year after
commissioning, with mismatched tyre wear and uneven payloads.

```text
400 episodes, success 83.2%, contact 99.8%, mate|contact 83.5%, median 24.6 s

| metric | 5th | median | 95th |
|---|---|---|---|
| |lateral| at contact (mm) | 1.40 | 9.77 | 30.47 |
| |heading| at contact (deg) | 0.19 | 2.13 | 6.54 |
| swept half-width at contact (mm) | 2.19 | 14.13 | 44.01 |
| longitudinal offset at stop (mm) | -0.00 | -0.00 | 0.00 |
| dead-reckoning range error (mm) | -20.43 | -1.42 | 12.73 |
| blind segment driven (mm) | 285.60 | 299.60 | 316.40 |
| time to dock (s) | 17.87 | 24.56 | 52.30 |
| retries used | 0.00 | 0.00 | 2.00 |

outcomes:          {'abort:attempts_exhausted': 65, 'abort:episode_timeout': 2, 'docked': 333}
retries used:      {0: 296, 1: 27, 2: 77}
inside tolerance:  83.2% of episodes ended charging (mechanical tolerance 25 mm swept, 6 deg heading)
```

The interesting line is the retry histogram. First-attempt success is 74.0 %
and three attempts get it to 83.2 %, so retrying recovers 37 of the 400
episodes. The other 67 failures use the entire retry budget and fail the same
way each time, because drive-train mismatch is a property of the robot and not
of the attempt: the second and third approaches curve off the axis exactly as
the first one did. No retry policy fixes that. Calibration, a shorter blind
segment or an online estimate of the robot's own straight-line curvature
would.

---

## Where it stops working

```bash
python3 examples/sweep.py --episodes 150 --grid-episodes 100 --workers 8
```

150 episodes per point, the same seed block at every point so the difference
between two rows is the parameter and not the draw. Full table in
[docs/sweep.csv](docs/sweep.csv).

### Odometry drift

| odometry drift scale | success | median swept (mm) | 95th swept (mm) | mean retries | median time (s) |
|---|---|---|---|---|---|
| 0.5 | 100.0% | 4.5 | 12.5 | 0.00 | 23.4 |
| 1 | 100.0% | 6.3 | 16.4 | 0.01 | 23.4 |
| 2 | 94.0% | 10.1 | 27.0 | 0.18 | 23.6 |
| 3 | 77.3% | 15.1 | 43.9 | 0.58 | 24.5 |
| 4 | 62.7% | 19.5 | 59.6 | 0.85 | 25.2 |
| 5 | 52.7% | 22.3 | 68.2 | 0.97 | 26.0 |
| 6 | 46.7% | 20.2 | 68.5 | 1.12 | 26.5 |

Drift scale 1 is the nominal drive train: a 1.5 % standard deviation on the
mismatch between the two effective wheel radii. Success is flat until about
twice that and then falls off a cliff. It plateaus near 50 % rather than going
to zero, and the plateau is retries doing what little they can -- but note the
mean retry count barely passes 1, because a robot whose wheels are mismatched
approaches wrongly the same way three times.

### Blind-segment length

| blind segment | success | median swept (mm) | 95th swept (mm) | mean retries | median time (s) |
|---|---|---|---|---|---|
| 0.15 m | 100.0% | 3.8 | 9.2 | 0.00 | 22.9 |
| 0.25 m | 100.0% | 4.2 | 14.5 | 0.00 | 23.2 |
| 0.35 m | 99.3% | 7.3 | 19.2 | 0.05 | 23.6 |
| 0.5 m | 90.0% | 11.9 | 34.4 | 0.30 | 24.8 |
| 0.65 m | 68.7% | 18.3 | 53.4 | 0.73 | 26.8 |
| 0.8 m | 56.7% | 21.9 | 71.6 | 0.99 | 28.9 |
| 0.95 m | 49.3% | 24.8 | 89.7 | 1.19 | 32.0 |

This is the same physics seen from the other side. Lateral drift over a blind
segment of length `d` grows as `(g / track) * d^2 / 2` -- quadratic -- so the
blind segment is the more expensive of the two axes. Going from 0.25 m to
0.50 m of blind travel costs ten points of success rate on a *nominal* drive
train. The blind segment is set by where the camera is mounted and how big the
marker is, which makes it the cheapest thing on this list to fix.

### Initial misalignment

| initial lateral spread scale | success | median swept (mm) | 95th swept (mm) | mean retries | median time (s) |
|---|---|---|---|---|---|
| 0.25 | 100.0% | 7.3 | 17.1 | 0.00 | 21.4 |
| 0.5 | 100.0% | 6.3 | 17.8 | 0.00 | 22.0 |
| 1 | 100.0% | 6.3 | 16.4 | 0.01 | 23.4 |
| 1.5 | 100.0% | 6.3 | 17.7 | 0.01 | 24.5 |
| 2 | 100.0% | 6.4 | 17.8 | 0.00 | 25.9 |
| 2.5 | 99.3% | 6.5 | 17.5 | 0.01 | 26.8 |
| 3 | 94.0% | 5.5 | 19.7 | 0.00 | 27.4 |

Flat, which is the useful finding here. Pure pursuit has metres of path in
which to remove lateral error, and it removes it. Whether the navigation stack
drops the robot 0.3 m or 2.2 m off the dock axis makes no difference to where
it ends up, only to how long it takes -- 21.4 s against 27.4 s. The tail-off at
the widest spread is not the controller running out of authority; it is the
marker becoming undetectable, because from 3.3 m off the axis at 2 m out the
tag is seen at close to the 60 degree incidence limit and acquisition never
starts.

The practical reading: do not spend effort tightening the pose the navigation
stack hands over. Spend it on the wheels and on the camera bracket.

### The boundary

100 episodes per cell.

| drift | 0.20 m | 0.35 m | 0.50 m | 0.65 m | 0.80 m |
|---|---|---|---|---|---|
| 1 | 100% | 99% | 90% | 75% | 61% |
| 2 | 99% | 85% | 62% | 44% | 33% |
| 3 | 94% | 71% | 43% | 28% | 22% |
| 4 | 82% | 54% | 32% | 22% | 16% |
| 5 | 75% | 47% | 25% | 17% | 12% |

The two axes are not independent, and the boundary is roughly a curve of
constant `drift x blind^2` -- the quadratic drift law showing through. Cells
with similar products have similar success rates: 61 % at (drift 1, 0.80 m,
product 0.64) against 54 % at (drift 4, 0.35 m, product 0.49). The match is
approximate rather than exact because the estimator's own error at the commit
gate also grows with the commit range, so the blind axis carries a little more
weight than the pure geometry predicts.

**The operational conclusion.** The 95 % contour sits between 0.40 and 0.45 m of
blind segment on a nominal drive train, and has collapsed to under 0.20 m by the time
drive-train mismatch is three times nominal. Moving the camera down and forward
so the marker stays visible to 0.20 m instead of 0.50 m takes a nominal robot
from 90 % to 100 %, and a robot with twice the nominal wheel mismatch from 62 %
to 99 %. Nothing available in the controller, the filter or the retry policy is
worth anywhere near that much. If a docking system is failing in the field, the
first two questions are how long the blind segment is and how well the wheel
radii are calibrated -- in that order.

---

## What this handles that a tutorial does not

- **Lateral error is removed where there is room to remove it.** Not with a
  sideways nudge at the end, because there is no such command on a
  differential drive. `ALIGN` is allowed to fix heading and explicitly not
  allowed to pretend it fixed anything else.
- **The commit decision is explicit and reversible.** Most docking code drives
  until it hits something. This one stops at the last observable range, decides
  whether the approach is good enough, and backs out if it is not. Refusing to
  enter the dock is a normal outcome, not an error.
- **Contact and charge are different sensors and different guards.** `ENGAGE`
  seats the contacts, `VERIFY` waits for voltage, and contact without voltage
  is a failed attempt.
- **The blind segment is planned against the mechanical criterion.** Driving
  straight is only the right answer when the robot is already on the axis.
- **Planar pose ambiguity is modelled, and gated.** A mirrored fiducial pose is
  a large, structured, confident error. Averaging does not remove it; rejecting
  it does, and `pose_locked` refuses to act on a single detection.
- **The gate has a watchdog.** A gated filter that accepts one bad fix while
  its covariance is wide will then reject every correct detection forever, and
  the behaviour above it flips between searching and acquiring at the control
  rate. That happened during development. `DockEstimator` now re-initialises
  after five consecutive rejections, and `marker_detected` means "a fix was
  *accepted*", not "the detector said something".
- **The covariance has a floor.** The dominant odometry error is a bias -- this
  robot's left wheel is a per cent or so larger than its right for the whole run
  -- and a random-walk process model cannot represent a bias. Without a floor
  the filter reported 0.15 degrees of heading sigma while carrying 1.8 degrees
  of heading error, and the commit gate believed it. Adding the floor moved the
  nominal success rate from 92 % to 100 %.
- **Retries are bounded, and known not to help against some faults.** Drive-train
  mismatch is a property of the robot, not of the attempt, so three tries fail
  the same way. That shows up in the sweep as a success-rate plateau rather
  than a slow decay.
- **Saturation preserves the path.** `clamp_twist` scales `v` and `omega`
  together, so a saturated command follows the same geometric path more slowly
  instead of a straighter path at the requested speed.

---

## Limitations

**This is a simulation study.** There is no real robot, no real dock and no
field trials behind any number in this repository. Every figure comes from the
model in `src/autodock`, and a model is only as good as the failure modes
someone thought to put in it.

Specifically not modelled, and each of these can dominate on real hardware:

- **Floor interaction.** No carpet-to-tile transitions, no dock cable to run
  over, no wheel slip events, no ramp at the dock lip. Slip is modelled only as
  a small multiplicative noise on wheel speed.
- **Funnel contact physics.** The dock is a pass/fail geometric criterion at the
  contact plane, not a rigid-body contact model. A real funnel guides a shoe
  that enters slightly off-centre, and can also jam it. We chose an explicit
  tolerance and graded against it rather than pretend to simulate the sliding.
- **Vision itself.** The fiducial detector is a statistical model of a detector,
  not a detector. No images, no lighting, no motion blur beyond its effect being
  asserted rather than simulated, no calibration error, no rolling shutter.
- **The charger.** Charge is a deterministic function of the contact pose.
  Intermittent contacts, contact resistance and dirty pads are real and absent.
- **Obstacles and other robots.** Nothing crosses the approach path.
- **Battery state.** No relationship between how flat the robot is and how many
  retries it can afford.

Design-level limits:

- The estimator's covariance is diagonal. Yaw error turns into lateral error as
  soon as the robot moves, and that coupling is handled by re-propagating the
  mean through an exact arc model rather than in the covariance.
- The blind segment is a single constant-curvature arc. A two-phase manoeuvre
  would correct more lateral error for the same heading cost.
- The robot does not learn its own drive-train curvature from previous
  attempts, which is the single change that would most improve the boundary and
  is left out deliberately so the boundary is visible.
- The mechanical tolerances describe one plausible dock. They are stated in one
  place, `DockGeometry`, and every derived threshold follows from them, but they
  are a design we chose rather than a dock we measured.

**The ROS 2 node in `src/autodock/ros_node.py` has been written but not run.**
ROS 2 is not installed in the environment this repository was developed and
measured in. The `rclpy` import is guarded, the docking logic itself lives in
`autodock.runtime` and has no ROS import at all, and the message-conversion
maths is unit tested -- but nobody has spun that node against a live robot, and
this README does not claim otherwise.

What a real deployment would additionally need: extrinsic calibration between
camera and drive frame, a wheel-radius calibration procedure, dock-occupied and
dock-visible checks against a map, recovery from being pushed while docked,
thermal and current limits on the final push, and a way to report "I cannot
dock" upstream that is more useful than a boolean.

---

## Testing

```bash
env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q
```

142 tests across 11 files. No network, no display, no plotting stack. The suite
runs the closed-loop simulator end to end, so it takes about two minutes on
one core.

They assert behaviour, not the absence of exceptions:

- **Every transition guard is tested by name**, true in the case it exists for
  and false next to it, and a meta-test fails if a guard is ever added without
  a test or without a use in the transition table.
- **The abort transition is asserted to be listed first in every state**, and a
  context that satisfies both an abort and a normal transition is asserted to
  abort.
- **The robot never begins the blind approach outside its own commit gate.**
  An observer hook is attached to whole episodes at four times nominal drift,
  and every single entry into `BLIND_APPROACH` is checked against all six
  conditions of `commit_locked`.
- **Retries are bounded even when nothing works.** A dock with a 0.2 mm
  tolerance is run to completion: every episode ends `abort:attempts_exhausted`,
  uses exactly the retry budget, and enters `ENGAGE` no more than
  `max_attempts` times.
- **The abort condition fires** for a marker that can never be seen
  (`abort:search_exhausted`, within the search timeout) and for a three-second
  episode budget (`abort:episode_timeout`, within one tick of it).
- **Kinematics are checked against closed forms**: a constant twist integrated
  over one period returns to its start to 1e-9 and is one diameter away at half
  a period; the clamp is asserted to preserve `omega/v` exactly; and the
  non-holonomic property is asserted directly, by checking that body-frame
  sideways displacement is second order in `dt` for every twist.
- **The quadratic drift law is measured**, not assumed: doubling the blind
  distance is asserted to quadruple the lateral error to within 5 %.
- **The latency compensation is asserted to be worth having** -- the same fix
  applied without replay is checked to drag the estimate backwards.
- **The divergence watchdog is asserted to fire** after exactly the configured
  number of rejections, and the covariance floor is asserted to keep the filter
  tracking a step it would otherwise ignore.
- **Success is graded on truth.** Every successful episode is re-checked
  against `DockGeometry.engagement_ok` on the *true* contact pose, never on the
  robot's estimate.

---

## Layout

```
src/autodock/
  geometry.py     SE(2), the dock frame, the coupled engagement criterion
  kinematics.py   wheel limits, curvature-preserving clamp, exact arc integration
  perception.py   fiducial model: min range, noise growth, dropouts, latency, ambiguity
  odometry.py     drive-train asymmetry, and the odometry that cannot see it
  estimator.py    dock-frame filter: latency replay, gate, covariance floor, watchdog
  states.py       DockState, DockingPolicy, and every guard as a pure function
  machine.py      the transition table and the retry bookkeeping
  controller.py   pure pursuit, turn in place, line following, blind-arc planning
  runtime.py      the online core: one tick in, one twist out. No ROS, no simulator
  sim.py          closed-loop episode: truth, sensor, estimator, machine, controller
  metrics.py      distributions, percentiles, conditional rates
  montecarlo.py   campaigns and sweeps
  ros_node.py     ROS 2 wrapper, guarded import, written but not run
examples/         demo, monte_carlo, sweep, make_figures
tests/            11 files, 142 tests
docs/             FAILURE_MODES.md, sweep.csv, figures/
```

---

## Related work

- **[ros2-diffdrive-robot](https://github.com/Pratyush150/ros2-diffdrive-robot)**
  -- the ROS 2 differential-drive base this docking behaviour would sit on top
  of: URDF, Gazebo, and the serial motor interface where the wheel-speed limits
  and the encoder feedback modelled here actually come from.
- **[robot-sim-test-harness](https://github.com/Pratyush150/robot-sim-test-harness)**
  -- scenario-driven regression testing for robot behaviour, which is what the
  Monte Carlo harness here becomes once you have more than one behaviour to
  keep honest.
- **[drone-control-toolkit](https://github.com/Pratyush150/drone-control-toolkit)**
  -- the control and estimation side in more depth: anti-windup, cascaded
  loops, and the EKF machinery this repository's deliberately small filter is a
  simplification of.
- **[lidar-slam-toolkit](https://github.com/Pratyush150/lidar-slam-toolkit)**
  -- where the robot's pose comes from before the docking behaviour is handed
  control, and the drift diagnostics that tell you how well placed it will be.

---

## License

MIT. See [LICENSE](LICENSE).
