# Run in VS2019 Developer PowerShell after scripts/Setup-OpenCV.ps1.
$ErrorActionPreference = 'Stop'
$project = Split-Path -Parent $PSScriptRoot
$props = Join-Path $project 'OpenCV.local.props'
if (-not (Test-Path -LiteralPath $props)) { throw '先运行 Setup-OpenCV.ps1。' }
[xml]$xml = Get-Content -LiteralPath $props -Raw
$opencv = [string]$xml.Project.PropertyGroup.OpenCVRoot
$temporary = Join-Path ([IO.Path]::GetTempPath()) ('SmokeTests_' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $temporary | Out-Null
Push-Location $temporary
try {
    & cl.exe /nologo /std:c++17 /EHsc /utf-8 /O2 /MD "/I$project" "/I$opencv\include" (Join-Path $project 'SmokeEngine.cpp') (Join-Path $project 'tests\SmokeTests.cpp') /FeSmokeTests.exe /link "/LIBPATH:$opencv\x64\vc16\lib" opencv_world4100.lib
    if ($LASTEXITCODE -ne 0) { throw 'C++ 测试编译失败。' }
    Copy-Item -LiteralPath (Join-Path $opencv 'x64\vc16\bin\opencv_world4100.dll') -Destination $temporary
    & (Join-Path $temporary 'SmokeTests.exe') $project (Join-Path $project 'samples\upstream_smoke_annotated.jpg') (Join-Path $project 'samples\upstream_mixed_missed_smoke.jpg')
    if ($LASTEXITCODE -ne 0) { throw 'C++ 测试失败。' }
}
finally { Pop-Location }
