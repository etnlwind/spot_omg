"""Build host-only C bindings on Windows, macOS and Linux; no firmware changes."""
import os
from pathlib import Path
import platform
import shutil
import subprocess
import tempfile


def library_suffix():
    return {"Windows": ".dll", "Darwin": ".dylib"}.get(platform.system(), ".so")


def _compiler_command():
    root = Path(__file__).resolve().parents[3]
    compiler = os.environ.get("CC") or shutil.which("cc") or shutil.which("gcc")
    zig = root / ".toolchain" / "zig" / "zig.exe"
    if not compiler and platform.system() == "Windows" and zig.exists():
        compiler = str(zig)
    if not compiler:
        raise RuntimeError("C compiler missing. On Windows run apps/windows/setup.ps1; otherwise install cc or set CC.")
    command = [compiler]
    if Path(compiler).stem.lower() == "zig":
        command += ["cc"]
        if platform.system() == "Windows":
            command += ["-target", "x86_64-windows-gnu"]
    return command


def build_executable(sources, include, destination, extra=()):
    """Build a host test with the same available toolchain as the simulator."""
    destination = Path(destination)
    if platform.system() == "Windows":
        destination = destination.with_suffix(".exe")
    destination.parent.mkdir(parents=True, exist_ok=True)
    command = _compiler_command() + ["-std=c11", "-O2", *extra]
    command += [*(str(p) for p in sources), "-I", str(include), "-o", str(destination), "-lm"]
    result = subprocess.run(command, capture_output=True, text=True,
                            creationflags=0x08000000 if os.name == "nt" else 0)
    if result.returncode:
        raise RuntimeError("Host C test compilation failed:\n" + result.stderr)
    return destination


def build_shared(sources, include, destination, extra=()):
    command = _compiler_command() + ["-std=c11", "-O2", *extra]
    if platform.system() != "Windows":
        command += ["-fPIC"]
    command += ["-dynamiclib" if platform.system() == "Darwin" else "-shared"]
    if platform.system() == "Windows":
        command += ["-Wl,--export-all-symbols"]
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as directory:
        output = Path(directory) / ("binding" + library_suffix())
        command += [*(str(p) for p in sources), "-I", str(include), "-o", str(output), "-lm"]
        result = subprocess.run(command, capture_output=True, text=True,
                                creationflags=0x08000000 if os.name == "nt" else 0)
        if result.returncode:
            raise RuntimeError("Host C library compilation failed:\n" + result.stderr)
        try:
            output.replace(destination)
        except PermissionError:
            if not destination.exists():
                raise
    return destination
