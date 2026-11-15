# DrinkTask Behavior-Tree and IK Recovery Design

## Status

Design baseline for the isolated experiment branch `experiment/drinktask-bt-ik-recovery`.
This document does not change runtime behavior. Runtime integration is gated on unit and
headless Gazebo validation.

## Objective

Combine per-action adaptive IK retries with behavior-tree supervision and bounded
upstream recovery. When action D cannot be completed from the current state, the system
must determine whether D itself is locally recoverable or whether the terminal state of
the preceding reversible action C created an unfavorable initial condition.

The behavior tree improves recovery structure and diagnosability; it does not guarantee
success and must never convert an unsafe or inaccurate result into success.

## Core execution model

1. Capture a checkpoint before each action: measured joint state, measured end-effector
   pose, base pose, task phase, object ownership/pose, gripper state, and relevant scene
   state.
2. Execute the action's exact nominal target first.
3. On IK failure, run a bounded adaptive search over:
   - solver seeds and redundant joint posture (including elbow/up-roll branches);
   - small, action-specific Cartesian target offsets;
   - small, action-specific end-effector orientation offsets.
4. For each candidate, record independent position and rotation residuals, solver,
   seed, FK-recomputed residual, joint-limit margin, posture-rule result, collision
   result, swept-path result, and execution result.
5. Retain the best 3-5 *distinct* candidates for diagnosis. A candidate that violates
   any hard gate is diagnostic only and is never executable.
6. If local recovery fails, ask the behavior-tree recovery policy whether the previous
   action is reversible and safe to revisit. If so, use the retained D-failure data to
   select a feasible terminal-state adjustment for C, replan C, verify C, and retry D.
7. Bound both local and upstream retries. Detect repeated candidates and recovery
   oscillation. On exhaustion or state uncertainty, stop and fail closed.

## Behavior-tree structure

- TaskRoot: Sequence
  - Preconditions: validate scene, base lock, object ownership, and measured state
  - ExecuteAction(C)
  - ExecuteAction(D) with LocalIKRecovery
  - On D failure: RecoverySelector
    - LocalRecovery: retry D with a bounded candidate set
    - UpstreamRecovery: only if C is explicitly marked reversible and checkpoint
      invariants still hold
      - AnalyzeFailureSet(D)
      - ReplanTerminalState(C)
      - ExecuteAdjusted(C)
      - RevalidateCheckpoint
      - Retry(D)
    - SafeStop: preserve base lock and object ownership; report failure
  - Postconditions: verify actual pose, gripper/object state, and task invariants

This is a manipulation-task behavior tree, not a replacement for Nav2's navigation
behavior tree. It should initially be implemented as a small in-package Python BT
supervisor to avoid introducing an unverified runtime dependency.

## Candidate ranking and feedback

Store at least the following per candidate:

- nominal target and candidate target (XYZ + quaternion or rotation matrix);
- measured start joint state and base pose;
- solver name and seed identifier;
- FK position residual (mm) and orientation residual (degrees);
- joint-limit margin and redundant-posture metrics;
- hard posture-rule result;
- endpoint collision and swept-path collision results;
- whether the candidate was executed and measured final error.

Rank only candidates that pass all hard constraints for executable recovery. Keep the
best 3-5 distinct safe candidates, ranked by task-space error, joint margin, path
clearance, and distance from the nominal target. If no candidate passes hard constraints,
retain the nearest failures for diagnosis but do not use them to move the robot.

D's failure data must be interpreted as a constraint on C's terminal state, not copied
blindly as C's target. The planner should seek a C terminal state from which D has a
safe feasible solution, while preserving C's own task semantics.

## Backtracking safety rules

Upstream backtracking is allowed only when all of these hold:

- C is explicitly classified as reversible and replay-safe.
- No irreversible event occurred (grasp/release, pour, handover, object transfer,
  attachment ownership change, or other externally visible side effect).
- Object ownership and measured object pose match the checkpoint invariants.
- The robot can reach a verified safe recovery pose without collision.
- The base remains locked when the task requires it.
- The adjusted C terminal state remains within C's own task tolerance and safety limits.

If any condition is false or uncertain, do not replay C. Enter a safe-stop/recovery state.
In particular, never replay a pre-release action after release has been acknowledged, and
never assume a held object still exists based only on software state.

## Hard gates that remain unchanged

- IK position/orientation acceptance thresholds remain unchanged.
- R1-R7 human-posture constraints remain hard constraints.
- Joint limits remain hard constraints.
- Full swept-path collision checks remain mandatory.
- Existing action-specific servo and measured-arrival gates remain mandatory.
- Grasp/release ACK and object-pose verification remain mandatory.
- Base lock and handback ownership protocol remain unchanged.

No retry may loosen any of these gates.

## Initial P104 scope

Start with the P104 PREPARE kettle post-release right-arm withdrawal failure:

- preserve the confirmed released-object state;
- treat the existing direct retreat and waypoint candidates as baseline evidence;
- add adaptive wrist orientation and redundant-posture search only within a
  collision-checked, empty-hand retreat envelope;
- retain 3-5 best distinct candidates with full diagnostics;
- do not backtrack across the release boundary;
- only consider revisiting the preceding reversible approach/preparation action if the
  object is confirmed released, the scene is unchanged, and the arm can first enter a
  verified safe recovery pose. Otherwise stop safely.

After P104 withdrawal is validated, generalize the supervisor to other action nodes.

## Validation plan

1. Pure unit tests for BT status propagation, bounded retries, candidate deduplication,
   ranking, checkpoint invariants, and forbidden backtracking across irreversible events.
2. Deterministic IK tests for nominal success, seed-only recovery, target micro-adjustment,
   orientation micro-adjustment, and hard-gate rejection.
3. Regression test reproducing the P104 withdrawal failure trace.
4. Headless Gazebo E2E: verify successful task completion only when all existing
   postconditions pass; verify fail-closed behavior when all recovery budgets are
   exhausted.
5. Repeat full E2E runs and report success count separately from partial/local recovery
   success. Do not count a task as successful unless it reaches DONE and all acceptance
   checks pass.
