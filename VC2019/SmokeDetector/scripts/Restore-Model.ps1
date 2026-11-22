#requires -Version 5.1
$ErrorActionPreference = 'Stop'
$project = Split-Path -Parent $PSScriptRoot
$model = Join-Path $project 'models\firewatch-yolo11s.onnx'
$expected = '1ab25ae6a26e711542e5b5e9670611476719532bc00ef1bd1a1657c01f24c2b4'
if ((Test-Path -LiteralPath $model) -and (Get-FileHash -LiteralPath $model -Algorithm SHA256).Hash.ToLowerInvariant() -eq $expected) {
    Write-Host '模型已存在，SHA256 校验通过。' -ForegroundColor Green
    exit 0
}
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
New-Item -ItemType Directory -Path (Split-Path -Parent $model) -Force | Out-Null
$url = 'https://github.com/leskoont/firewatch/releases/download/v0.1.0/firewatch-yolo11s.onnx'
Invoke-WebRequest -Uri $url -OutFile ($model + '.part') -UseBasicParsing
if ((Get-FileHash -LiteralPath ($model + '.part') -Algorithm SHA256).Hash.ToLowerInvariant() -ne $expected) {
    throw '下载模型的 SHA256 校验失败，未替换原模型。'
}
Move-Item -LiteralPath ($model + '.part') -Destination $model -Force
Write-Host '模型已恢复，SHA256 校验通过。' -ForegroundColor Green
