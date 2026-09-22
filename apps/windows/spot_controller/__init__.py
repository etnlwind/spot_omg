"""Spot OMG Windows controller."""
__version__ = "95.0.0"
__build__ = 69

def display_version():
    """Same release/revision/build convention as Apple ControlView."""
    import sys
    major, revision, _ = __version__.split('.')
    release = f"V{major}" + (f"-R{revision}" if revision != '0' else '')
    configuration = 'Release' if getattr(sys, 'frozen', False) else 'Debug'
    return f"{release} ({__build__}) - {configuration}"
