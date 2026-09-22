param([string]$Python = "python", [string]$DistPath = "$PSScriptRoot/dist")
$ErrorActionPreference = "Stop"
$runtime = (Get-Command $Python -ErrorAction Stop).Source
$pythonDirectory = Split-Path -Parent $runtime
$resources = Join-Path $PSScriptRoot "spot_controller/resources"
$versionSource = [System.IO.File]::ReadAllText((Join-Path $PSScriptRoot "spot_controller/__init__.py"))
$versionMatch = [regex]::Match($versionSource, '__version__\s*=\s*"(\d+)\.(\d+)\.(\d+)"')
if (-not $versionMatch.Success) { throw "App version must be a numeric major.minor.patch" }
$appVersion = "$($versionMatch.Groups[1].Value).$($versionMatch.Groups[2].Value).$($versionMatch.Groups[3].Value)"
$buildMatch = [regex]::Match($versionSource, '__build__\s*=\s*(\d+)')
if (-not $buildMatch.Success) { throw "App build number is required" }
$buildNumber = [int]$buildMatch.Groups[1].Value
if ($buildNumber -lt 1 -or $buildNumber -gt 65535) { throw "App build must be in 1..65535" }
$appleProject = [System.IO.File]::ReadAllText((Join-Path $PSScriptRoot "../ios/SpotOMGController/SpotOMGController.xcodeproj/project.pbxproj"))
$appleVersions = [regex]::Matches($appleProject, 'MARKETING_VERSION\s*=\s*([\d.]+);')
$appleBuilds = [regex]::Matches($appleProject, 'CURRENT_PROJECT_VERSION\s*=\s*(\d+);')
if ($appleVersions.Count -eq 0 -or $appleBuilds.Count -eq 0) { throw "Apple app version metadata missing" }
foreach ($match in $appleVersions) { if ($match.Groups[1].Value -ne $appVersion) { throw "Windows and Apple app versions differ" } }
foreach ($match in $appleBuilds) { if ([int]$match.Groups[1].Value -ne $buildNumber) { throw "Windows and Apple build numbers differ" } }
$fileVersion = "$appVersion.$buildNumber"
$versionTuple = "$($versionMatch.Groups[1].Value), $($versionMatch.Groups[2].Value), $($versionMatch.Groups[3].Value), $buildNumber"
$buildDirectory = Join-Path $PSScriptRoot "build"
New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null
$versionFile = Join-Path $buildDirectory "windows-version.txt"
$versionMetadata = @"
VSVersionInfo(
  ffi=FixedFileInfo(filevers=($versionTuple), prodvers=($versionTuple),
    mask=0x3f, flags=0x0, OS=0x40004, fileType=0x1, subtype=0x0, date=(0, 0)),
  kids=[StringFileInfo([StringTable('040904B0', [
    StringStruct('CompanyName', 'Spot OMG'),
    StringStruct('FileDescription', 'Spot OMG!'),
    StringStruct('FileVersion', '$fileVersion'),
    StringStruct('InternalName', 'SpotOMGController'),
    StringStruct('OriginalFilename', 'SpotOMGController.exe'),
    StringStruct('ProductName', 'Spot OMG!'),
    StringStruct('ProductVersion', '$appVersion')
  ])]), VarFileInfo([VarStruct('Translation', [1033, 1200])])]
)
"@
[System.IO.File]::WriteAllText($versionFile, $versionMetadata, [System.Text.UTF8Encoding]::new($false))
$previousBuildPath = $env:PATH
# Do not bundle unrelated ICU/Qt DLLs from tools such as Poppler on the host PATH.
$env:PATH = "$pythonDirectory;$pythonDirectory/Scripts;$pythonDirectory/Library/bin;$env:WINDIR/System32;$env:WINDIR"
try {
    & $runtime -m PyInstaller --clean --noconfirm --windowed --onedir --name SpotOMGController `
        --distpath $DistPath --workpath "$PSScriptRoot/build" --specpath $PSScriptRoot `
        --icon "$resources/SpotOMG.ico" --version-file $versionFile `
        --add-data "$resources;spot_controller/resources" `
        --collect-submodules bleak.backends.winrt --collect-submodules winrt "$PSScriptRoot/main.py"
    if ($LASTEXITCODE -ne 0) { throw "Desktop build failed" }
} finally {
    $env:PATH = $previousBuildPath
}
