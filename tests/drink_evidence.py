#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pure helpers for DrinkTask evidence acceptance; safe to import without ROS."""

REQUIRED_SHOT_LABELS = frozenset({
    'arrive', 'present', 'grip', 'drink', 'takeback', 'placed', 'handback',
})


def missing_required_shots(labels):
    """Return required event labels not represented by a saved screenshot."""
    return sorted(REQUIRED_SHOT_LABELS - set(labels or ()))
