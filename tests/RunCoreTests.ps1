# Run in VS2019 Developer PowerShell with cl.exe available.
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$build = Join-Path ([System.IO.Path]::GetTempPath()) ('PCDTests_' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $build | Out-Null
$source = Join-Path $root 'VC2019\PointCloudWorkbench'
Push-Location $build
try {
    foreach ($suite in @('CoreTests', 'CylinderTests')) {
        & cl.exe /nologo /std:c++17 /EHsc /utf-8 /O2 "/I$source" (Join-Path $source 'PointCloud.cpp') (Join-Path $source 'Cylinder.cpp') (Join-Path $PSScriptRoot ($suite + '.cpp')) "/Fe$suite.exe"
        if ($LASTEXITCODE -ne 0) { throw "$suite build failed." }
        & (Join-Path $build ($suite + '.exe')) (Join-Path $build ($suite + '-fixtures'))
        if ($LASTEXITCODE -ne 0) { throw "$suite failed." }
    }
}
finally { Pop-Location }
