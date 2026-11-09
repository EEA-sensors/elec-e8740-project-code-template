"""Estimation methods for Part II.

The models are passed in as arguments, so you can test each method with the
models from models.py.
"""
import numpy as np


def gauss_newton(y, g, G, x0, R):
    """Nonlinear weighted least squares for a static pose (Task 5)."""
    raise NotImplementedError


def dead_reckoning(x0, u, t, f):
    """Pose over time from the inputs only, without the camera (Task 6)."""
    raise NotImplementedError


def ekf_predict(x, P, u, dt, f, F, Q):
    """Prediction step of an extended Kalman filter (Task 7)."""
    raise NotImplementedError


def ekf_update(x, P, y, g, G, R):
    """Update step with camera measurements (Task 7)."""
    raise NotImplementedError
