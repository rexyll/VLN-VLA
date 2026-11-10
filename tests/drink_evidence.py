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


def validate_observer_result(result):
    """Validate the observer JSON contract before screenshot archival."""
    if not isinstance(result, dict):
        raise ValueError('expected a JSON object')
    shots = result.get('shots', [])
    if not isinstance(shots, list):
        raise ValueError('shots must be a JSON array')
    if not all(isinstance(path, str) and path for path in shots):
        raise ValueError('screenshot paths must be non-empty strings')
    if 'evidence' in result and not isinstance(result['evidence'], dict):
        raise ValueError('evidence must be a JSON object when present')
    return shots


def archive_observer_screenshots(result, root, shot_dir):
    """Atomically archive observer screenshots and attach archive metadata."""
    import os
    import shutil
    import tempfile

    shots = validate_observer_result(result)
    root_real = os.path.realpath(root)
    # Resolve existing parent symlinks before creating staging directories;
    # lexical containment alone would allow an in-repository symlink to escape.
    shot_dir = os.path.realpath(os.path.abspath(shot_dir))
    try:
        if os.path.commonpath((root_real, shot_dir)) != root_real:
            raise ValueError('screenshot archive must be inside repository root')
    except ValueError:
        raise ValueError('screenshot archive must be inside repository root')
    parent = os.path.dirname(shot_dir)
    os.makedirs(parent, exist_ok=True)
    if os.path.lexists(shot_dir):
        raise FileExistsError('screenshot archive already exists: %s' % shot_dir)

    staging = tempfile.mkdtemp(prefix='.drink-shots-', dir=parent)
    try:
        archived = []
        for index, source in enumerate(shots, 1):
            source_real = os.path.realpath(source)
            try:
                inside_root = (
                    os.path.commonpath((root_real, source_real)) == root_real)
            except ValueError:
                inside_root = False
            if not inside_root:
                raise ValueError('screenshot source must be inside repository root: %s'
                                 % source)
            if not os.path.isfile(source_real):
                raise FileNotFoundError('missing screenshot evidence: %s' % source)
            name = '%02d_%s' % (index, os.path.basename(source_real))
            destination = os.path.join(staging, name)
            shutil.copy2(source_real, destination)
            archived.append(os.path.relpath(
                os.path.join(shot_dir, name), root_real))

        # Same-filesystem directory rename publishes the complete archive at once.
        os.rename(staging, shot_dir)
        staging = None
        result['shots'] = archived
        result.setdefault('evidence', {})['screenshots_archived'] = len(archived)
        return result
    finally:
        if staging is not None:
            shutil.rmtree(staging, ignore_errors=True)
