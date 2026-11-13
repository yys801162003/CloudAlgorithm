# CloudAlgorithm
The Algorithm of analysis and manupulation of Cloud Points

## VS2019 可调整大小的对话框示例

原生 Win32 C++ 工程，无需 MFC，支持拖动窗口边框和四角调整大小、最大化和还原，文本框自动适配窗口。

- [下载完整工程和 PowerShell 脚本（ZIP）](https://github.com/yys801162003/CloudAlgorithm/raw/refs/heads/main/downloads/VC2019_ResizableDialog_Package.zip)
- [PowerShell 生成脚本](scripts/CreateProject.ps1)
- [工程说明](VC2019/ResizableDialog/README.md)
- [Visual Studio 解决方案](VC2019/ResizableDialog/ResizableDialog.sln)

解压 ZIP 后，用 Visual Studio 2019 打开 `ResizableDialog/ResizableDialog.sln`，按 F5 编译运行。需要安装“使用 C++ 的桌面开发”、MSVC v142 和 Windows 10 SDK。

也可在 Windows PowerShell 5.1 或更新版本中运行生成脚本：

```powershell
.\CreateProject.ps1 -Destination 'D:\Projects\ResizableDialog'
```

生成工程目录和同名 ZIP。目标已存在时停止，避免覆盖。脚本采用 UTF-8 带 BOM；打包排除 `.git`、`node_modules`、`.vs`、`bin`、`obj` 等目录。

当前已检查工程结构、嵌入源码一致性和压缩包完整性，尚未在 Windows 上实际执行脚本或编译运行。
