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
