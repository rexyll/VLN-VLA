"""Small, dependency-free behavior-tree primitives for manipulation recovery.

This module deliberately does not command ROS/Gazebo.  Motion code supplies
callbacks that perform its existing preflight, collision, IK and servo checks.
The supervisor only controls bounded retry/backtrack decisions.
"""
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence as TypingSequence, Tuple


class Status(str, Enum):
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"
    RUNNING = "RUNNING"


class Node:
    def tick(self, context: Dict[str, Any]) -> Status:
        raise NotImplementedError

    def reset(self) -> None:
        pass


class Action(Node):
    def __init__(self, callback: Callable[[Dict[str, Any]], Status]):
        self.callback = callback

    def tick(self, context: Dict[str, Any]) -> Status:
        result = self.callback(context)
        if not isinstance(result, Status):
            raise TypeError("behavior-tree actions must return Status")
        return result


class Sequence(Node):
    """Memory sequence: resumes a RUNNING child on the next tick."""
    def __init__(self, *children: Node):
        self.children = list(children)
        self.index = 0

    def tick(self, context: Dict[str, Any]) -> Status:
        while self.index < len(self.children):
            result = self.children[self.index].tick(context)
            if result == Status.RUNNING:
                return result
            if result == Status.FAILURE:
                self.reset()
                return result
            self.index += 1
        self.reset()
        return Status.SUCCESS

    def reset(self) -> None:
        self.index = 0
        for child in self.children:
            child.reset()


class Selector(Node):
    """Memory selector: advances only after a child fails."""
    def __init__(self, *children: Node):
        self.children = list(children)
        self.index = 0

    def tick(self, context: Dict[str, Any]) -> Status:
        while self.index < len(self.children):
            result = self.children[self.index].tick(context)
            if result == Status.RUNNING:
                return result
            if result == Status.SUCCESS:
                self.reset()
                return result
            self.index += 1
        self.reset()
        return Status.FAILURE

    def reset(self) -> None:
        self.index = 0
        for child in self.children:
            child.reset()


class Retry(Node):
    """Retry a child at most max_attempts times; RUNNING does not spend budget."""
    def __init__(self, child: Node, max_attempts: int):
        if max_attempts < 1:
            raise ValueError("max_attempts must be >= 1")
        self.child = child
        self.max_attempts = int(max_attempts)
        self.attempts = 0

    def tick(self, context: Dict[str, Any]) -> Status:
        result = self.child.tick(context)
        if result == Status.RUNNING:
            return result
        if result == Status.SUCCESS:
            self.reset()
            return result
        self.attempts += 1
        self.child.reset()
        if self.attempts >= self.max_attempts:
            self.reset()
            return Status.FAILURE
        return Status.RUNNING

    def reset(self) -> None:
        self.attempts = 0
        self.child.reset()


@dataclass(frozen=True)
class FailureCandidate:
    """One diagnostic IK candidate; only hard_valid candidates may be executed."""
    candidate_id: str
    position_residual_mm: float
    rotation_residual_deg: float
    score: float
    hard_valid: bool = False
    target_xyz: Tuple[float, float, float] = ()
    target_rpy: Tuple[float, float, float] = ()
    joint_seed: Tuple[float, ...] = ()
    evidence: Dict[str, Any] = field(default_factory=dict, compare=False)

    @property
    def dedupe_key(self) -> Tuple[Any, ...]:
        return (self.candidate_id,
                tuple(round(v, 5) for v in self.target_xyz),
                tuple(round(v, 5) for v in self.target_rpy),
                tuple(round(v, 4) for v in self.joint_seed))


class CandidateBuffer:
    """Keep the best 3-5 distinct records; never equate a low score with validity."""
    def __init__(self, capacity: int = 5):
        if capacity < 3 or capacity > 5:
            raise ValueError("candidate capacity must be between 3 and 5")
        self.capacity = capacity
        self._items: Dict[Tuple[Any, ...], FailureCandidate] = {}

    def add(self, candidate: FailureCandidate) -> None:
        key = candidate.dedupe_key
        previous = self._items.get(key)
        if (previous is None
                or (candidate.hard_valid and not previous.hard_valid)
                or (candidate.hard_valid == previous.hard_valid
                    and candidate.score < previous.score)):
            self._items[key] = candidate
        ordered = sorted(
            self._items.items(),
            key=lambda item: (not item[1].hard_valid, item[1].score))
        self._items = dict(ordered[:self.capacity])

    def ranked(self) -> List[FailureCandidate]:
        return sorted(self._items.values(), key=lambda item: item.score)

    def executable(self) -> List[FailureCandidate]:
        return [item for item in self.ranked() if item.hard_valid]

    def __len__(self) -> int:
        return len(self._items)


@dataclass(frozen=True)
class Checkpoint:
    checkpoint_id: str
    action_name: str
    scene_token: str
    base_pose: Tuple[float, float, float]
    object_state: str
    irreversible_epoch: int
    replayable: bool
    safe_recovery_pose_verified: bool


@dataclass(frozen=True)
class WorldSnapshot:
    scene_token: str
    base_pose: Tuple[float, float, float]
    object_state: str
    irreversible_epoch: int
    state_known: bool = True


class RecoverySupervisor:
    """Fail-closed gate for replaying an upstream action."""
    def __init__(self, max_backtracks: int = 1, pose_tolerance_m: float = 0.005,
                 yaw_tolerance_rad: float = 0.01):
        if max_backtracks < 0:
            raise ValueError("max_backtracks must be >= 0")
        self.max_backtracks = max_backtracks
        self.pose_tolerance_m = float(pose_tolerance_m)
        self.yaw_tolerance_rad = float(yaw_tolerance_rad)
        self.backtracks = 0

    def can_backtrack(self, checkpoint: Checkpoint,
                      current: WorldSnapshot) -> Tuple[bool, str]:
        if self.backtracks >= self.max_backtracks:
            return False, "backtrack_budget_exhausted"
        if not current.state_known:
            return False, "world_state_unknown"
        if not checkpoint.replayable:
            return False, "upstream_action_not_replayable"
        if not checkpoint.safe_recovery_pose_verified:
            return False, "safe_recovery_pose_not_verified"
        if checkpoint.irreversible_epoch != current.irreversible_epoch:
            return False, "irreversible_event_boundary_crossed"
        if checkpoint.scene_token != current.scene_token:
            return False, "scene_changed"
        if checkpoint.object_state != current.object_state:
            return False, "object_state_changed"
        dx = current.base_pose[0] - checkpoint.base_pose[0]
        dy = current.base_pose[1] - checkpoint.base_pose[1]
        dyaw = current.base_pose[2] - checkpoint.base_pose[2]
        import math
        dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
        if math.hypot(dx, dy) > self.pose_tolerance_m:
            return False, "base_pose_mismatch"
        if abs(dyaw) > self.yaw_tolerance_rad:
            return False, "base_yaw_mismatch"
        return True, "checkpoint_revalidated"

    def authorize_backtrack(self, checkpoint: Checkpoint,
                            current: WorldSnapshot) -> Tuple[bool, str]:
        allowed, reason = self.can_backtrack(checkpoint, current)
        if allowed:
            self.backtracks += 1
        return allowed, reason

    def reset(self) -> None:
        self.backtracks = 0
