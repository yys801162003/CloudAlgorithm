# CloudAlgorithm
The Algorithm of analysis and manupulation of Cloud Points

## VC2019 PCD 点云配准与差异分析

可调整大小的原生 Win32 对话框，读取两份 PCD，显示和配准点云，计算几何相似度并标记不同点。

**[下载完整工程 ZIP](https://github.com/yys801162003/CloudAlgorithm/raw/refs/heads/main/downloads/VC2019_PCD_Registration_Package.zip)**

- [工程与算法说明](VC2019/ResizableDialog/README.md)
- [Visual Studio 2019 解决方案](VC2019/ResizableDialog/ResizableDialog.sln)
- [PowerShell 工程生成脚本](scripts/CreateProject.ps1)
- [示例 PCD 文件](samples)
- [核心测试](tests/CoreTests.cpp)

### 编译与使用

1. 解压 ZIP，使用 VS2019 打开 `VC2019/ResizableDialog/ResizableDialog.sln`。
2. 推荐 x64 / Release，按 F5。需要 MSVC v142 和 Windows 10 SDK，无需 PCL/MFC/Eigen/VTK。
3. 分别点击“读取点云 A”和“读取点云 B”，选择两份 PCD。
4. 设置差异距离阈值（与 PCD 坐标同单位；0 为自动），点击“配准并比较”。
5. 青=A、绿=B、红=A 差异、紫=B 差异。可仅显示差异、切换 B 原始位置、导出 PCD 和变换矩阵。

视图支持左键旋转、右键平移、滚轮缩放。界面可拖拽调整大小、最大化和还原，加载和计算在后台进行。

### 相似度含义

使用 PCA 多姿态初始化 + 裁剪式刚体 ICP 将 B 对齐到 A。配准使用降采样点，最终对全部有效点做双向最近邻距离比较。

`相似度 = 100 × (A 重合率 + B 重合率) / 2`

大于用户距离阈值的点被标记为差异点。支持 ASCII、binary、binary_compressed（LZF）PCD；跳过非有限坐标。

该分数是指定阈值下的几何重合率，受密度、遮挡、噪声和配准质量影响。ICP 不保证全局最优；不估计缩放，不支持非刚体形变。完整限制与指标定义见工程说明。

### 验证与示例

核心自动测试和内存/未定义行为检查通过；Windows x64/x86 资源、界面、链接交叉编译通过；PowerShell 7 生成与打包测试通过。尚未在 Windows/Visual Studio 2019 桌面实际运行。

示例 `sample_A.pcd` 与 `sample_B_rigid.pcd` 配准后相似度约 100%；与 `sample_B_changed.pcd` 比较，阈值 `0.02` 时约 95.49%，标记 A 侧 80 个、B 侧 45 个差异点。

### 本机生成工程与 ZIP

下载 `scripts/CreateProject.ps1` 后，在 PowerShell 中执行：

```powershell
.\CreateProject.ps1 -Destination 'D:\Projects\PCDRegistration'
```

生成完整目录以及同名 ZIP；打开生成目录中的 `VC2019/ResizableDialog/ResizableDialog.sln`。目标已存在时停止，避免覆盖。脚本兼容 Windows PowerShell 5.1 语法，保存为 UTF-8 带 BOM。

ZIP 排除 `.git`、`node_modules`、`.vs`、`bin`、`obj`、编译产物和缓存。保留的 `downloads/VC2019_ResizableDialog_Package.zip` 是最初的基础对话框历史包，PCD 功能请使用本页顶部的新包。
