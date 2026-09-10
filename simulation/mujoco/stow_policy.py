"""Shared simulation-only Stow: stop driving before inter-leg contact.

Derived from full CAD triangles by stow_clearance.py (18 mm design margin).
At completion, release motor torque so gravity may gently seat the legs.
This is not a physical-servo calibration or hardware approval.
"""
LANDING = [0., 40., 130.] * 4
FOLDED = [0., -254.63, 4.42] * 2 + [0., -80.75, 4.42] * 2
FOLD_SECONDS = 12.
SETTLE_SECONDS = 3.
RELEASE_TORQUE_AT_STOW = True
CAPABILITY = 'simstow'
