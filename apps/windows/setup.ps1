param([string]$Python = "python")
$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
& $Python -m pip install -r "$PSScriptRoot/requirements.txt" -e "$repo/tools/servo_tool" "mujoco==3.11.0"
if ($LASTEXITCODE -ne 0) { throw "Python dependency installation failed" }
$toolchain = Join-Path $repo ".toolchain"
$zig = Join-Path $toolchain "zig/zig.exe"
if (-not (Test-Path -LiteralPath $zig)) {
    New-Item -ItemType Directory -Force -Path $toolchain | Out-Null
    $archive = Join-Path $toolchain "zig-0.15.2.zip"
    Invoke-WebRequest "https://ziglang.org/download/0.15.2/zig-x86_64-windows-0.15.2.zip" -OutFile $archive
    if ((Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash -ne "3a0ed1e8799a2f8ce2a6e6290a9ff22e6906f8227865911fb7ddedc3cc14cb0c") { throw "Zig checksum mismatch" }
    Expand-Archive -LiteralPath $archive -DestinationPath $toolchain -Force
    Rename-Item -LiteralPath (Join-Path $toolchain "zig-x86_64-windows-0.15.2") -NewName "zig"
}
& $Python -X utf8 -c "from servo import SharedGaitPolicy; SharedGaitPolicy(); from pathlib import Path; import sys; sys.path.insert(0,str(Path(sys.argv[1]))); from simulation.mujoco.runtime.body_stabilizer import BodyStabilizer; BodyStabilizer(); print('Windows C bindings ready')" $repo
if ($LASTEXITCODE -ne 0) { throw "C binding verification failed" }
