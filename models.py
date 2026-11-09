"""Motion model and measurement model for Part II.

State x = [p_x, p_y, psi]: position of the robot centre (m) and heading
(rad, counterclockwise from the x-axis), in the global frame of the arena.
"""
import numpy as np


def f(x, u, dt):
    """Motion model: the state dt seconds later (Task 6)."""
    raise NotImplementedError


def F(x, u, dt):
    """Jacobian of f with respect to x."""
    raise NotImplementedError


def g(x, tags):
    """Measurement model: what the camera should measure from pose x (Task 5)."""
    raise NotImplementedError


def G(x, tags):
    """Jacobian of g with respect to x."""
    raise NotImplementedError
