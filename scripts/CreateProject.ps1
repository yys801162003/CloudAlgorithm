#requires -Version 5.1
param(
    [string]$Destination = (Join-Path (Get-Location).Path 'ResizableDialog')
)
$ErrorActionPreference = 'Stop'
$Destination = [System.IO.Path]::GetFullPath($Destination)
$zipPath = $Destination + '.zip'
if ((Test-Path -LiteralPath $Destination) -or (Test-Path -LiteralPath $zipPath)) {
    throw '目标目录或 ZIP 已存在，请用 -Destination 指定其他位置，避免覆盖。'
}
New-Item -ItemType Directory -Path $Destination | Out-Null
$utf8 = New-Object System.Text.UTF8Encoding($true)
function Write-ProjectFile([string]$Name, [string]$Content) {
    [System.IO.File]::WriteAllText((Join-Path $Destination $Name), $Content, $utf8)
}

Write-ProjectFile 'README.md' @'
# VC2019 可调整大小的对话框

## 打开和运行

1. Visual Studio Installer 中安装“使用 C++ 的桌面开发”，包含 MSVC v142 和 Windows 10 SDK。无需 MFC。
2. 解压整个压缩包，使用 Visual Studio 2019 打开 `ResizableDialog.sln`。
3. 选择 `Debug` 或 `Release`，选择 `x64` 或 `Win32`。
4. 按 F5 编译运行。程序位于 `bin/平台/配置/ResizableDialog.exe`。

如果提示缺少 Windows SDK，右键解决方案选择“重定解决方案目标”，选择本机已安装的 Windows 10 SDK。

## 行为

- 拖动四边和四角调整大小，支持最小化、最大化和还原。
- 文本框随窗口自动伸缩，关闭按钮保持右下角，显示当前客户区尺寸。
- 有一个 220 × 120 对话框单位的最小客户区，保证控件仍可使用；没有额外的自定义最大尺寸限制。
- 按 Esc、关闭按钮或标题栏 X 退出。

## 关键代码

`ResizableDialog.rc` 中的 `WS_THICKFRAME` 允许拖动边框，`WS_MAXIMIZEBOX` 允许最大化。
`main.cpp` 响应 `WM_SIZE`，在 `LayoutControls` 中更新控件位置和尺寸。
如果不需要自定义最小尺寸，删除 `WM_GETMINMAXINFO` 分支即可。

本示例采用原生 Win32 API，并非 MFC 工程。源文件使用 UTF-8；C++ 启用 /utf-8，资源使用 code_page(65001)。

## 验证情况

已检查工程 XML、配置映射、源文件引用和压缩包完整性。
当前生成环境为 Linux，没有 Visual Studio/Windows SDK，因此尚未实际编译或运行 Windows 程序。
'@

Write-ProjectFile 'ResizableDialog.rc' @'
#pragma code_page(65001)
#include <windows.h>
#include "resource.h"

LANGUAGE LANG_CHINESE, SUBLANG_CHINESE_SIMPLIFIED

IDD_MAIN_DIALOG DIALOGEX 0, 0, 360, 240
STYLE DS_SETFONT | DS_CENTER | WS_POPUP | WS_CAPTION | WS_SYSMENU | WS_THICKFRAME | WS_MINIMIZEBOX | WS_MAXIMIZEBOX | WS_CLIPCHILDREN
EXSTYLE WS_EX_APPWINDOW
CAPTION "VC2019 可调整大小的对话框"
FONT 9, "Microsoft YaHei UI", 400, 0, 134
BEGIN
    LTEXT "拖动窗口边框或四角可调整大小。", IDC_SIZE_LABEL, 10, 10, 340, 14
    EDITTEXT IDC_CONTENT_EDIT, 10, 30, 340, 178, ES_MULTILINE | ES_AUTOVSCROLL | ES_WANTRETURN | WS_VSCROLL | WS_TABSTOP
    DEFPUSHBUTTON "关闭", IDCANCEL, 290, 216, 60, 16
END
'@

Write-ProjectFile 'ResizableDialog.sln' @'
Microsoft Visual Studio Solution File, Format Version 12.00
# Visual Studio Version 16
VisualStudioVersion = 16.0.28701.123
MinimumVisualStudioVersion = 10.0.40219.1
Project("{BC8A1FFA-BEE3-4634-8014-F334798102B3}") = "ResizableDialog", "ResizableDialog.vcxproj", "{6131BAF3-1A90-4658-B4B0-ABFC90619A06}"
EndProject
Global
	GlobalSection(SolutionConfigurationPlatforms) = preSolution
		Debug|Win32 = Debug|Win32
		Debug|x64 = Debug|x64
		Release|Win32 = Release|Win32
		Release|x64 = Release|x64
	EndGlobalSection
	GlobalSection(ProjectConfigurationPlatforms) = postSolution
		{6131BAF3-1A90-4658-B4B0-ABFC90619A06}.Debug|Win32.ActiveCfg = Debug|Win32
		{6131BAF3-1A90-4658-B4B0-ABFC90619A06}.Debug|Win32.Build.0 = Debug|Win32
		{6131BAF3-1A90-4658-B4B0-ABFC90619A06}.Debug|x64.ActiveCfg = Debug|x64
		{6131BAF3-1A90-4658-B4B0-ABFC90619A06}.Debug|x64.Build.0 = Debug|x64
		{6131BAF3-1A90-4658-B4B0-ABFC90619A06}.Release|Win32.ActiveCfg = Release|Win32
		{6131BAF3-1A90-4658-B4B0-ABFC90619A06}.Release|Win32.Build.0 = Release|Win32
		{6131BAF3-1A90-4658-B4B0-ABFC90619A06}.Release|x64.ActiveCfg = Release|x64
		{6131BAF3-1A90-4658-B4B0-ABFC90619A06}.Release|x64.Build.0 = Release|x64
	EndGlobalSection
EndGlobal
'@

Write-ProjectFile 'ResizableDialog.vcxproj' @'
<?xml version="1.0" encoding="utf-8"?>
<Project DefaultTargets="Build" xmlns="http://schemas.microsoft.com/developer/msbuild/2003">
  <ItemGroup Label="ProjectConfigurations">
    <ProjectConfiguration Include="Debug|Win32"><Configuration>Debug</Configuration><Platform>Win32</Platform></ProjectConfiguration>
    <ProjectConfiguration Include="Debug|x64"><Configuration>Debug</Configuration><Platform>x64</Platform></ProjectConfiguration>
    <ProjectConfiguration Include="Release|Win32"><Configuration>Release</Configuration><Platform>Win32</Platform></ProjectConfiguration>
    <ProjectConfiguration Include="Release|x64"><Configuration>Release</Configuration><Platform>x64</Platform></ProjectConfiguration>
  </ItemGroup>
  <PropertyGroup Label="Globals">
    <VCProjectVersion>16.0</VCProjectVersion>
    <ProjectGuid>{6131BAF3-1A90-4658-B4B0-ABFC90619A06}</ProjectGuid>
    <Keyword>Win32Proj</Keyword>
    <RootNamespace>ResizableDialog</RootNamespace>
    <WindowsTargetPlatformVersion>10.0</WindowsTargetPlatformVersion>
  </PropertyGroup>
  <Import Project="$(VCTargetsPath)\Microsoft.Cpp.Default.props" />
  <PropertyGroup Condition="'$(Configuration)|$(Platform)'=='Debug|Win32'" Label="Configuration">
    <ConfigurationType>Application</ConfigurationType>
    <UseDebugLibraries>true</UseDebugLibraries>
    <PlatformToolset>v142</PlatformToolset>
    <CharacterSet>Unicode</CharacterSet>
  </PropertyGroup>
  <PropertyGroup Condition="'$(Configuration)|$(Platform)'=='Debug|x64'" Label="Configuration">
    <ConfigurationType>Application</ConfigurationType>
    <UseDebugLibraries>true</UseDebugLibraries>
    <PlatformToolset>v142</PlatformToolset>
    <CharacterSet>Unicode</CharacterSet>
  </PropertyGroup>
  <PropertyGroup Condition="'$(Configuration)|$(Platform)'=='Release|Win32'" Label="Configuration">
    <ConfigurationType>Application</ConfigurationType>
    <UseDebugLibraries>false</UseDebugLibraries>
    <PlatformToolset>v142</PlatformToolset>
    <CharacterSet>Unicode</CharacterSet>
  </PropertyGroup>
  <PropertyGroup Condition="'$(Configuration)|$(Platform)'=='Release|x64'" Label="Configuration">
    <ConfigurationType>Application</ConfigurationType>
    <UseDebugLibraries>false</UseDebugLibraries>
    <PlatformToolset>v142</PlatformToolset>
    <CharacterSet>Unicode</CharacterSet>
  </PropertyGroup>
  <Import Project="$(VCTargetsPath)\Microsoft.Cpp.props" />
  <PropertyGroup>
    <OutDir>$(SolutionDir)bin\$(Platform)\$(Configuration)\</OutDir>
    <IntDir>$(SolutionDir)obj\$(Platform)\$(Configuration)\</IntDir>
  </PropertyGroup>
  <ItemDefinitionGroup>
    <ClCompile>
      <WarningLevel>Level4</WarningLevel>
      <SDLCheck>true</SDLCheck>
      <ConformanceMode>true</ConformanceMode>
      <LanguageStandard>stdcpp17</LanguageStandard>
      <AdditionalOptions>/utf-8 %(AdditionalOptions)</AdditionalOptions>
      <PreprocessorDefinitions>UNICODE;_UNICODE;%(PreprocessorDefinitions)</PreprocessorDefinitions>
    </ClCompile>
    <Link>
      <SubSystem>Windows</SubSystem>
      <AdditionalDependencies>User32.lib;Gdi32.lib;%(AdditionalDependencies)</AdditionalDependencies>
    </Link>
  </ItemDefinitionGroup>
  <ItemGroup><ClCompile Include="main.cpp" /></ItemGroup>
  <ItemGroup><ClInclude Include="resource.h" /></ItemGroup>
  <ItemGroup><ResourceCompile Include="ResizableDialog.rc" /></ItemGroup>
  <Import Project="$(VCTargetsPath)\Microsoft.Cpp.targets" />
</Project>
'@

Write-ProjectFile 'main.cpp' @'
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include "resource.h"

// Convert dialog units using the dialog's actual font.
static RECT DialogUnits(HWND dialog, int x, int y, int width, int height)
{
    RECT rect = { x, y, x + width, y + height };
    MapDialogRect(dialog, &rect);
    return rect;
}

static void LayoutControls(HWND dialog)
{
    RECT client = {};
    GetClientRect(dialog, &client);
    const RECT metrics = DialogUnits(dialog, 10, 10, 60, 16);
    const int marginX = metrics.left;
    const int marginY = metrics.top;
    const int buttonWidth = metrics.right - metrics.left;
    const int buttonHeight = metrics.bottom - metrics.top;
    const int labelHeight = DialogUnits(dialog, 0, 0, 0, 14).bottom;
    const int gap = DialogUnits(dialog, 0, 0, 0, 6).bottom;
    const int width = client.right;
    const int height = client.bottom;
    const int contentWidth = (width > 2 * marginX) ? width - 2 * marginX : 0;
    const int editTop = marginY + labelHeight + gap;
    const int buttonTop = height - marginY - buttonHeight;
    const int editHeight = (buttonTop > editTop + gap) ? buttonTop - editTop - gap : 0;

    MoveWindow(GetDlgItem(dialog, IDC_SIZE_LABEL), marginX, marginY,
        contentWidth, labelHeight, TRUE);
    MoveWindow(GetDlgItem(dialog, IDC_CONTENT_EDIT), marginX, editTop,
        contentWidth, editHeight, TRUE);
    MoveWindow(GetDlgItem(dialog, IDCANCEL), width - marginX - buttonWidth,
        buttonTop, buttonWidth, buttonHeight, TRUE);

    wchar_t text[160] = {};
    wsprintfW(text, L"拖动边框或四角调整大小；当前客户区：%d × %d", width, height);
    SetDlgItemTextW(dialog, IDC_SIZE_LABEL, text);
}

static INT_PTR CALLBACK DialogProc(HWND dialog, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
    case WM_INITDIALOG:
        SetDlgItemTextW(dialog, IDC_CONTENT_EDIT,
            L"这是一个可以自由调整大小的 Win32 对话框。\r\n\r\n"
            L"拖动四边或四角，文本框将自动伸缩，关闭按钮保持在右下角。\r\n"
            L"也可以最大化、还原，或双击标题栏。\r\n\r\n"
            L"工程使用 VS2019 / v142，不依赖 MFC。");
        LayoutControls(dialog);
        return TRUE;

    case WM_SIZE:
        if (wParam != SIZE_MINIMIZED)
            LayoutControls(dialog);
        return TRUE;

    case WM_GETMINMAXINFO:
    {
        // Small minimum preserves usable controls; there is no custom maximum.
        RECT minimum = DialogUnits(dialog, 0, 0, 220, 120);
        AdjustWindowRectEx(&minimum,
            static_cast<DWORD>(GetWindowLongPtrW(dialog, GWL_STYLE)), FALSE,
            static_cast<DWORD>(GetWindowLongPtrW(dialog, GWL_EXSTYLE)));
        auto* limits = reinterpret_cast<MINMAXINFO*>(lParam);
        limits->ptMinTrackSize.x = minimum.right - minimum.left;
        limits->ptMinTrackSize.y = minimum.bottom - minimum.top;
        return TRUE;
    }

    case WM_COMMAND:
        if (LOWORD(wParam) == IDCANCEL)
        {
            EndDialog(dialog, IDCANCEL);
            return TRUE;
        }
        break;

    case WM_CLOSE:
        EndDialog(dialog, IDCANCEL);
        return TRUE;
    }
    return FALSE;
}

int APIENTRY wWinMain(HINSTANCE instance, HINSTANCE, LPWSTR, int)
{
    const INT_PTR result = DialogBoxParamW(instance,
        MAKEINTRESOURCEW(IDD_MAIN_DIALOG), nullptr, DialogProc, 0);
    if (result == -1)
    {
        MessageBoxW(nullptr, L"无法创建对话框。请检查资源文件是否正确编译。",
            L"启动失败", MB_OK | MB_ICONERROR);
        return 1;
    }
    return 0;
}
'@

Write-ProjectFile 'resource.h' @'
#pragma once
#define IDD_MAIN_DIALOG 101
#define IDC_SIZE_LABEL 1001
#define IDC_CONTENT_EDIT 1002
'@


# 将当前生成的工程打包；排除依赖、版本库及构建缓存。
Add-Type -AssemblyName System.IO.Compression
Add-Type -AssemblyName System.IO.Compression.FileSystem
$excludedDirs = @('node_modules', '.git', '.vs', 'bin', 'obj', '.cache', 'Debug', 'Release', 'x64', 'x86')
$excludedExtensions = @('.user', '.suo', '.obj', '.pdb', '.ilk', '.ipch', '.log')
$prefix = $Destination.TrimEnd([char[]]'\/') + [System.IO.Path]::DirectorySeparatorChar
$rootName = Split-Path -Leaf $Destination
$archive = [System.IO.Compression.ZipFile]::Open($zipPath, [System.IO.Compression.ZipArchiveMode]::Create)
try {
    Get-ChildItem -LiteralPath $Destination -Recurse -File -Force | ForEach-Object {
        $file = $_
        $relative = $file.FullName.Substring($prefix.Length)
        $segments = $relative -split '[\\/]'
        $skip = $false
        foreach ($segment in $segments) {
            if ($excludedDirs -contains $segment) { $skip = $true; break }
        }
        if (($file.Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0) { $skip = $true }
        if ($excludedExtensions -contains $file.Extension) { $skip = $true }
        if (-not $skip) {
            $entryName = $rootName + '/' + $relative.Replace('\', '/')
            [System.IO.Compression.ZipFileExtensions]::CreateEntryFromFile(
                $archive, $file.FullName, $entryName,
                [System.IO.Compression.CompressionLevel]::Optimal) | Out-Null
        }
    }
}
finally { $archive.Dispose() }
Write-Host "工程：$Destination" -ForegroundColor Green
Write-Host "压缩包：$zipPath" -ForegroundColor Green
Write-Host '使用 Visual Studio 2019 打开 ResizableDialog.sln，按 F5 编译运行。'
