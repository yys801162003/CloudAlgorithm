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
