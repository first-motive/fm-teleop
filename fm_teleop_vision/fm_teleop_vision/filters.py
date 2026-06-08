"""One-Euro filter (Casiez et al., 2012) — pure scalar smoothing, no ROS, no OpenCV.

The vision source tracks a wrist's world position; MediaPipe's per-frame estimate is
noisy, and feeding that jitter straight into a velocity command makes the arm buzz at
rest. A One-Euro filter is the standard low-lag answer: it smooths hard when the signal
is still and loosens as motion speeds up, so it kills rest-jitter without adding lag to a
deliberate reach.

Reference: https://gery.casiez.net/1euro/. ``Vec3OneEuro`` wraps three independent
scalar filters for the wrist's (x, y, z); this module imports nothing heavy so it is
unit-tested on the host without a ROS graph, a camera, or the pose model.
"""

import math


class LowPassFilter:
    """First-order exponential low-pass: ``y = alpha*x + (1-alpha)*y_prev``."""

    def __init__(self):
        self._y = None

    def __call__(self, x, alpha):
        if self._y is None:
            self._y = x
        else:
            self._y = alpha * x + (1.0 - alpha) * self._y
        return self._y

    @property
    def last(self):
        return self._y


class OneEuroFilter:
    """One scalar One-Euro filter.

    ``min_cutoff`` lowers to smooth more (at the cost of lag); ``beta`` raises to track
    fast motion more tightly. Defaults match the reference perception pipeline.
    """

    def __init__(self, min_cutoff=1.0, beta=0.02, d_cutoff=1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self._x_lp = LowPassFilter()
        self._dx_lp = LowPassFilter()
        self._x_prev = None

    @staticmethod
    def _alpha(dt, cutoff):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return dt / (dt + tau)

    def __call__(self, x, dt):
        if dt <= 0:
            # No time elapsed; return the last filtered value if we have one.
            return self._x_lp.last if self._x_lp.last is not None else x
        if self._x_prev is None:
            # Seed on the first sample.
            self._x_prev = x
            self._x_lp(x, 1.0)
            return x
        dx = (x - self._x_prev) / dt
        dx_hat = self._dx_lp(dx, self._alpha(dt, self.d_cutoff))
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        x_hat = self._x_lp(x, self._alpha(dt, cutoff))
        self._x_prev = x
        return x_hat

    def reset(self):
        """Forget history so the next sample seeds afresh (e.g. after tracking dropout)."""
        self._x_lp = LowPassFilter()
        self._dx_lp = LowPassFilter()
        self._x_prev = None


class Vec3OneEuro:
    """Three independent One-Euro filters, one per axis, for a 3D point."""

    def __init__(self, min_cutoff=1.0, beta=0.02, d_cutoff=1.0):
        self._filters = [
            OneEuroFilter(min_cutoff, beta, d_cutoff) for _ in range(3)
        ]

    def __call__(self, xyz, dt):
        """Filter a 3-vector; returns the smoothed ``[x, y, z]``."""
        return [f(v, dt) for f, v in zip(self._filters, xyz)]

    def reset(self):
        """Reset all three axes (call when tracking drops, to avoid a resume jump)."""
        for f in self._filters:
            f.reset()
