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
    """Validate required event labels and unique, in-repository archived files."""
    import os

    if not isinstance(result, dict):
        return {
            'task_ok': False, 'archive_ok': False, 'ok': False,
            'screenshots_archived': 0, 'shots': [],
            'missing_shot_labels': sorted(REQUIRED_SHOT_LABELS),
        }

    shots = result.get('shots', [])
    evidence = result.get('evidence')
    if not isinstance(evidence, dict):
        evidence = {}
    declared = evidence.get('screenshots_archived')
    labels = result.get('shot_labels', [])
    labels_well_formed = (
        isinstance(labels, list)
        and all(isinstance(label, str) for label in labels)
    )
    missing = set(missing_required_shots(
        labels if labels_well_formed else []))
    labels_ok = (
        labels_well_formed
        and len(labels) == len(REQUIRED_SHOT_LABELS)
        and set(labels) == REQUIRED_SHOT_LABELS
        and not missing
    )

    root_real = os.path.realpath(root)
    resolved = []
    paths_ok = isinstance(shots, list)
    if paths_ok:
        for path in shots:
            if (not isinstance(path, str) or not path or '\x00' in path
                    or os.path.isabs(path)):
                paths_ok = False
                break
            try:
                candidate = os.path.realpath(os.path.join(root_real, path))
                inside_root = (
                    os.path.commonpath((root_real, candidate)) == root_real)
                exists = os.path.isfile(candidate)
            except (OSError, ValueError):
                inside_root = False
                exists = False
            if not inside_root or not exists:
                paths_ok = False
                break
            resolved.append(candidate)

    unique_paths = paths_ok and len(set(resolved)) == len(resolved)
    archive_ok = (
        isinstance(declared, int)
        and not isinstance(declared, bool)
        and declared == len(REQUIRED_SHOT_LABELS)
        and isinstance(shots, list)
        and len(shots) == len(REQUIRED_SHOT_LABELS)
        and labels_ok
        and paths_ok
        and unique_paths
    )
    task_ok = result.get('ok') is True
    return {
        'task_ok': task_ok,
        'archive_ok': archive_ok,
        'ok': task_ok and archive_ok,
        'screenshots_archived': declared if archive_ok else 0,
        'shots': list(shots) if isinstance(shots, list) else [],
        'missing_shot_labels': sorted(missing),
    }
