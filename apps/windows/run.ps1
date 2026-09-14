param([string]$Python = "python")
$env:PYTHONUTF8 = "1"
& $Python -X utf8 "$PSScriptRoot/main.py"
