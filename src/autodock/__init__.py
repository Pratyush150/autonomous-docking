"""Autonomous docking of a differential-drive robot onto a charging station.

Simulation study of the docking *software*: perception model, dock-relative
estimator, guarded state machine, controller, and a Monte Carlo harness that
reports where the design stops working.
"""

from .controller import BlindPlan, ControllerGains, DockingController
from .estimator import DockEstimator, EstimatorNoise
from .geometry import DockError, DockGeometry, Pose2D, wrap_angle
from .kinematics import DiffDriveLimits, body_to_wheels, clamp_twist, integrate, wheels_to_body
from .machine import TRANSITIONS, DockingStateMachine, Transition
from .metrics import CampaignSummary, summarise
from .montecarlo import SweepPoint, run_campaign, run_sweep
from .odometry import DriveTrain, DriveTrainSpec, WheelOdometry
from .perception import FiducialSensor, MarkerModel, MarkerObservation
from .runtime import DockingRuntime, RuntimeCommand
from .sim import EpisodeConfig, EpisodeResult, StartDistribution, run_episode
from .states import GUARDS, DockingPolicy, DockState, GuardContext

__version__ = "0.1.0"

__all__ = [
    "BlindPlan",
    "ControllerGains",
    "DockingController",
    "DockEstimator",
    "EstimatorNoise",
    "DockError",
    "DockGeometry",
    "Pose2D",
    "wrap_angle",
    "DiffDriveLimits",
    "body_to_wheels",
    "clamp_twist",
    "integrate",
    "wheels_to_body",
    "TRANSITIONS",
    "DockingStateMachine",
    "Transition",
    "CampaignSummary",
    "summarise",
    "SweepPoint",
    "run_campaign",
    "run_sweep",
    "DriveTrain",
    "DriveTrainSpec",
    "WheelOdometry",
    "FiducialSensor",
    "MarkerModel",
    "MarkerObservation",
    "DockingRuntime",
    "RuntimeCommand",
    "EpisodeConfig",
    "EpisodeResult",
    "StartDistribution",
    "run_episode",
    "GUARDS",
    "DockingPolicy",
    "DockState",
    "GuardContext",
    "__version__",
]
