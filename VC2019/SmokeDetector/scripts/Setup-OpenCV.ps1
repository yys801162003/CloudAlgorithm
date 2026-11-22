#requires -Version 5.1
param([string]$OpenCVRoot)
$ErrorActionPreference = 'Stop'
$project = Split-Path -Parent $PSScriptRoot
if (-not $OpenCVRoot) {
    $thirdParty = Join-Path $project 'third_party'
    $OpenCVRoot = Join-Path $thirdParty 'opencv\build'
    if (-not (Test-Path -LiteralPath (Join-Path $OpenCVRoot 'x64\vc16\lib\opencv_world4100.lib'))) {
        New-Item -ItemType Directory -Path $thirdParty -Force | Out-Null
        $installer = Join-Path $thirdParty 'opencv-4.10.0-windows.exe'
        $url = 'https://github.com/opencv/opencv/releases/download/4.10.0/opencv-4.10.0-windows.exe'
        $expected = 'bff38466091c313dac21a0b73eea8278316a89c1d434c6f0b10697e087670168'
        if (-not (Test-Path -LiteralPath $installer)) {
            [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
            Invoke-WebRequest -Uri $url -OutFile ($installer + '.part') -UseBasicParsing
            Move-Item -LiteralPath ($installer + '.part') -Destination $installer
        }
        if ((Get-FileHash -LiteralPath $installer -Algorithm SHA256).Hash.ToLowerInvariant() -ne $expected) {
            throw "OpenCV 安装包校验失败，请删除后重试：$installer"
        }
        # OpenCV's official package is a 7-Zip self-extracting archive.
        $arguments = '-y -o"' + $thirdParty + '"'
        $process = Start-Process -FilePath $installer -ArgumentList $arguments -Wait -PassThru
        if ($process.ExitCode -ne 0) { throw "OpenCV 解压失败：$($process.ExitCode)" }
    }
}
$OpenCVRoot = [IO.Path]::GetFullPath($OpenCVRoot)
foreach ($relative in @('include\opencv2\core.hpp', 'x64\vc16\lib\opencv_world4100.lib', 'x64\vc16\lib\opencv_world4100d.lib', 'x64\vc16\bin\opencv_world4100.dll', 'x64\vc16\bin\opencv_world4100d.dll')) {
    if (-not (Test-Path -LiteralPath (Join-Path $OpenCVRoot $relative) -PathType Leaf)) {
        throw "缺少 OpenCV 4.10.0 / x64 / vc16 文件：$relative。OpenCVRoot 应指向 opencv\build。"
    }
}
$escaped = [Security.SecurityElement]::Escape($OpenCVRoot)
$xml = '<?xml version="1.0" encoding="utf-8"?>' + [Environment]::NewLine +
    '<Project xmlns="http://schemas.microsoft.com/developer/msbuild/2003"><PropertyGroup><OpenCVRoot>' + $escaped + '</OpenCVRoot></PropertyGroup></Project>'
[IO.File]::WriteAllText((Join-Path $project 'OpenCV.local.props'), $xml, (New-Object Text.UTF8Encoding($true)))
Write-Host "OpenCV 配置完成：$OpenCVRoot" -ForegroundColor Green
Write-Host '打开 SmokeDetector.sln，选择 x64 / Release，按 F5 编译运行。'
