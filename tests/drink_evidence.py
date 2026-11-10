#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pure helpers for DrinkTask evidence acceptance; safe to import without ROS."""

REQUIRED_SHOT_LABELS = frozenset({
    'arrive', 'present', 'grip', 'drink', 'takeback', 'placed', 'handback',
})


def missing_required_shots(labels):
    """Return required event labels not represented by a saved screenshot."""
    return sorted(REQUIRED_SHOT_LABELS - set(labels or ()))

def should_stop_terminal_observer(state, saved_labels, terminal_elapsed,
                                  grace_seconds=3.0):
    """Stop at terminal states, allowing DONE's final image a bounded retry."""
    if state in ('ERROR', 'FAIL_PREPARE', 'FAIL_HANDOVER'):
        return True
    if state != 'DONE':
        return False
    if 'placed' in set(saved_labels or ()):
        return True
    return terminal_elapsed >= grace_seconds

def summarize_round_evidence(result, root):
    """Validate that every screenshot referenced by a round was archived."""
    import os

    shots = result.get('shots', [])
    evidence = result.get('evidence') or {}
    declared = evidence.get('screenshots_archived')
    paths_ok = (
        isinstance(shots, list)
        and all(isinstance(path, str) and path
                and os.path.isfile(os.path.join(root, path))
                for path in shots)
    )
    archive_ok = (
        isinstance(declared, int)
        and not isinstance(declared, bool)
        and declared == len(shots)
        and paths_ok
    )
    task_ok = bool(result.get('ok'))
    return {
        'task_ok': task_ok,
        'archive_ok': archive_ok,
        'ok': task_ok and archive_ok,
        'screenshots_archived': declared if archive_ok else 0,
        'shots': list(shots) if isinstance(shots, list) else [],
    }
