# Run from an x64 Native Tools Command Prompt for VS 2019 / Developer PowerShell.
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$build = Join-Path ([System.IO.Path]::GetTempPath()) ('PCDCoreTests_' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $build | Out-Null
$source = Join-Path $root 'VC2019\ResizableDialog'
& cl.exe /nologo /std:c++17 /EHsc /utf-8 /O2 "/I$source" (Join-Path $source 'PointCloud.cpp') (Join-Path $PSScriptRoot 'CoreTests.cpp') "/Fo$build\" "/Fe$build\CoreTests.exe"
if ($LASTEXITCODE -ne 0) { throw 'Core test build failed.' }
& (Join-Path $build 'CoreTests.exe') (Join-Path $build 'fixtures')
if ($LASTEXITCODE -ne 0) { throw 'Core tests failed.' }
