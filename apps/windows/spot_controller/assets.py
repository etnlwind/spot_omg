"""Bundled artwork shared with the Apple app, also resolved in PyInstaller."""
from pathlib import Path


def resource_path(name):
    # PyInstaller preserves this package directory under _internal; build.ps1
    # places resources beside this module just as in a source checkout.
    return Path(__file__).resolve().parent / "resources" / name
