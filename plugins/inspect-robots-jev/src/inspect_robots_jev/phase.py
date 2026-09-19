"""Deterministic phase machine for the cube-into-bowl task.

Code owns task progress (rule R4): Jev picks small moves inside a phase, the
machine decides from tag poses and gripper opening when a phase is finished.
Every rule is a tolerance test on numbers the perceiver already produced.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from inspect_robots_jev.menu import MenuKind
from inspect_robots_jev.world import ARMS, Arm, Vec3, WorldState, distance

PhaseName = Literal[
    "approach", "descend", "grasp", "lift", "carry", "lower", "release", "retreat", "done"
]
#: Which arm/target/menu each phase uses. Target is the object the menu talks about.
_TARGET: dict[PhaseName, str] = {
    "approach": "cube",
    "descend": "cube",
    "grasp": "cube",
    "lift": "cube",
    "carry": "bowl",
    "lower": "bowl",
    "release": "bowl",
    "retreat": "bowl",
    "done": "bowl",
}
_MENU: dict[PhaseName, MenuKind] = {
    "approach": "xyz",
    "descend": "z",
    "grasp": "grip_close",
    "lift": "z",
    "carry": "xyz",
    "lower": "z",
    "release": "grip_open",
    "retreat": "z",
    "done": "done",
}


@dataclass(frozen=True)
class TaskConfig:
    """Object names and tolerances for cube-into-bowl, metres unless noted."""

    cube: str = "cube"
    bowl: str = "bowl"
    hover_m: float = 0.05
    xy_tol_m: float = 0.01
    z_tol_m: float = 0.01
    lift_m: float = 0.08
    bowl_depth_m: float = 0.04
    closed_on_object: float = 0.35
    closed_on_air: float = 0.02
    open_threshold: float = 0.8
    cube_half_m: float = 0.0127


@dataclass(frozen=True)
class Phase:
    """The current stage: who moves, toward what, with which menu, to which point."""

    name: PhaseName
    arm: Arm
    target: str
    menu_kind: MenuKind
    goal: Vec3 | None

    @property
    def positional(self) -> bool:
        """True when Jev is steering toward ``goal`` rather than operating the gripper."""
        return self.goal is not None


def _add(p: Vec3, dz: float) -> Vec3:
    return (p[0], p[1], p[2] + dz)


class PhaseMachine:
    """Track the cube-into-bowl stage and advance it on tolerance rules."""

    def __init__(self, config: TaskConfig) -> None:
        self._cfg = config
        self._name: PhaseName = "approach"
        self._arm: Arm = "left"
        self._grasp_point: Vec3 | None = None
        self._phase: Phase | None = None

    @property
    def phase(self) -> Phase:
        """The phase computed by the last ``reset``/``advance`` call."""
        if self._phase is None:
            raise RuntimeError("PhaseMachine.reset() has not been called")
        return self._phase

    def reset(self, world: WorldState) -> Phase:
        """Start over, choosing the arm whose gripper is nearer the cube."""
        cube = world.objects[self._cfg.cube]
        self._arm = min(
            ARMS, key=lambda arm: distance(cube.in_frame[arm], world.grippers[arm].position)
        )
        self._name = "approach"
        self._grasp_point = None
        self._phase = self._build(world)
        return self._phase

    def advance(self, world: WorldState) -> Phase:
        """Apply at most one transition for this world snapshot and return the phase."""
        cfg = self._cfg
        arm = self._arm
        grip = world.grippers[arm]
        current = self._build(world)
        goal = current.goal
        if self._name == "approach" and goal is not None and self._near(grip.position, goal):
            self._name = "descend"
        elif self._name == "descend" and goal is not None and self._near_z(grip.position, goal):
            self._name = "grasp"
        elif self._name == "grasp":
            if grip.opening <= cfg.closed_on_air:
                self._name = "approach"
            elif grip.opening <= cfg.closed_on_object:
                self._grasp_point = grip.position
                self._name = "lift"
        elif (
            self._name == "lift" and goal is not None and grip.position[2] >= goal[2] - cfg.z_tol_m
        ):
            self._name = "carry"
        elif self._name == "carry" and goal is not None and self._near_xy(grip.position, goal):
            self._name = "lower"
        elif self._name == "lower" and goal is not None and self._near_z(grip.position, goal):
            self._name = "release"
        elif self._name == "release" and grip.opening >= cfg.open_threshold:
            self._name = "retreat"
        elif (
            self._name == "retreat"
            and goal is not None
            and grip.position[2] >= goal[2] - cfg.z_tol_m
        ):
            self._name = "done"
        self._phase = self._build(world)
        return self._phase

    # -- helpers -----------------------------------------------------------

    def _target_name(self) -> str:
        return self._cfg.cube if _TARGET[self._name] == "cube" else self._cfg.bowl

    def _goal(self, world: WorldState) -> Vec3 | None:
        cfg = self._cfg
        arm = self._arm
        name = self._name
        if _MENU[name] in ("grip_close", "grip_open", "done"):
            return None
        target = world.objects[self._target_name()].in_frame[arm]
        if name == "approach":
            return _add(target, cfg.hover_m)
        if name == "descend":
            return target
        if name == "lift":
            base = self._grasp_point if self._grasp_point is not None else target
            return _add(base, cfg.lift_m)
        if name == "carry":
            return _add(target, cfg.hover_m + cfg.cube_half_m)
        if name == "lower":
            return _add(target, cfg.cube_half_m - cfg.bowl_depth_m)
        return _add(target, cfg.lift_m)  # retreat

    def _build(self, world: WorldState) -> Phase:
        return Phase(
            name=self._name,
            arm=self._arm,
            target=self._target_name(),
            menu_kind=_MENU[self._name],
            goal=self._goal(world),
        )

    def _near_xy(self, p: Vec3, goal: Vec3) -> bool:
        tol = self._cfg.xy_tol_m
        return abs(p[0] - goal[0]) <= tol and abs(p[1] - goal[1]) <= tol

    def _near_z(self, p: Vec3, goal: Vec3) -> bool:
        return abs(p[2] - goal[2]) <= self._cfg.z_tol_m

    def _near(self, p: Vec3, goal: Vec3) -> bool:
        return self._near_xy(p, goal) and self._near_z(p, goal)
