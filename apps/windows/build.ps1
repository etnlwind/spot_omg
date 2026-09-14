param([string]$Python = "python")
$ErrorActionPreference = "Stop"
$runtime = (Get-Command $Python -ErrorAction Stop).Source
$pythonDirectory = Split-Path -Parent $runtime
$previousBuildPath = $env:PATH
# Do not bundle unrelated ICU/Qt DLLs from tools such as Poppler on the host PATH.
$env:PATH = "$pythonDirectory;$pythonDirectory/Scripts;$pythonDirectory/Library/bin;$env:WINDIR/System32;$env:WINDIR"
try {
    & $runtime -m PyInstaller --clean --noconfirm --windowed --onedir --name SpotOMGController `
        --distpath "$PSScriptRoot/dist" --workpath "$PSScriptRoot/build" --specpath $PSScriptRoot `
        --collect-submodules bleak.backends.winrt --collect-submodules winrt "$PSScriptRoot/main.py"
    if ($LASTEXITCODE -ne 0) { throw "Desktop build failed" }
} finally {
    $env:PATH = $previousBuildPath
}
