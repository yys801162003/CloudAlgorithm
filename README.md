# CloudAlgorithm
The Algorithm of analysis and manupulation of Cloud Points

## VC2019 PNG/JPEG 烟雾识别

**[下载烟雾识别完整工程 ZIP](https://github.com/yys801162003/CloudAlgorithm/raw/refs/heads/main/downloads/SmokeDetector_VS2019.zip)** · [编译与操作说明](VC2019/SmokeDetector/README.md)

独立的 `VC2019/SmokeDetector/SmokeDetector.sln` 工程提供 OpenCV 颜色/纹理规则和已训练 YOLO11s 神经网络两种方法，三栏对比输入和标注结果，支持中文路径、后台检测、阈值调整与结果导出。模型已经包含，运行时在本地 CPU 推理，无需 Python。

先运行工程内 `scripts/Setup-OpenCV.ps1` 配置官方 OpenCV 4.10.0，然后用 VS2019 选择 **x64 / Release** 编译。ZIP 包含源码、真实 ONNX 模型、示例、测试和许可，排除 OpenCV 依赖安装目录与编译缓存。

此烟雾识别模块和所附模型采用 AGPL-3.0，模型来源与 OpenCV 许可见[第三方通知](VC2019/SmokeDetector/THIRD_PARTY_NOTICES.md)。传统规则仅识别疑似区域；神经网络分数不是准确率，未检出不等于无烟，需用独立场景样本评估。

## VC2019 点云工作台

**[下载完整工程 ZIP](https://github.com/yys801162003/CloudAlgorithm/raw/refs/heads/main/downloads/PointCloudWorkbench_VS2019.zip)**

[工程及算法说明](VC2019/PointCloudWorkbench/README.md) · [VS2019 解决方案](VC2019/PointCloudWorkbench/PointCloudWorkbench.sln) · [PowerShell 生成脚本](scripts/CreateProject.ps1) · [示例 PCD](samples)

### 功能

- 可拖动边框和四角调整大小的 Win32 对话框，支持最大化/还原；点云可旋转、平移、缩放。
- 分别读取两份 PCD，支持 ASCII、binary、binary_compressed（LZF），支持中文路径。
- 使用 PCA 多姿态初始化与 ICP 将 B 刚体配准到 A，显示几何相似度、双向重合率、RMSE，红色/紫色标记两侧差异点。
- 独立对 A/B 进行单圆柱重建，显示半径、直径、轴线、长度、拟合误差及内点率，黄色线框显示圆柱侧面，绿色/红色标记圆柱内外点。
- 圆柱重建后计算最大直径、点云长度、最小截面长度、最小截面直径、最大截面椭圆率、最大截面轴心位置偏差，以及两根轴线夹角。各项测量可单独叠加到点云视图。
- 导出配准后的点云、差异点、4×4 变换矩阵，以及圆柱 OBJ 网格、表面 PCD、内外点 PCD 和参数报告。
- 后台读取、计算和导出，可取消任务；重新加载文件会清除相关旧结果。

### 编译与试用

1. 解压 ZIP，使用 Visual Studio 2019 打开 `VC2019/PointCloudWorkbench/PointCloudWorkbench.sln`。
2. 选择 **x64 / Release**，按 F5。安装 MSVC v142 和 Windows 10 SDK，无需 PCL、MFC、Eigen 或 VTK。
3. 配准测试：读取 `sample_A.pcd` 与 `sample_B_rigid.pcd`，差异阈值填 `0.02`，点击“ICP 配准 / 比较”。再换成 `sample_B_changed.pcd` 查看差异点。
4. 圆柱测试：读取 `cylinder_A_noisy.pcd`，圆柱距离阈值填 `0.012`，点击“圆柱重建 A”。真实半径为 `0.7`；再试 `cylinder_partial.pcd`。

### 指标及范围

相似度为 `100 × (A 重合率 + B 重合率) / 2`，重合由配准后的双向最近邻距离阈值判断。这是几何重合率，不是语义相同概率；单位、密度、噪声、遮挡和局部配准结果会影响分数。

圆柱重建针对单根理想直圆柱/管道的外表面，长度取轴向内点 1%/99% 分位跨度。部分扫描的完整圆周属于模型外推；不生成端盖，不重建弯管、厚度或任意自由曲面。详细算法和限制见工程说明。

### 本地生成工程和 ZIP

下载 `scripts/CreateProject.ps1` 后在 PowerShell 中运行：

```powershell
.\CreateProject.ps1 -Destination 'D:\Projects\PointCloudWorkbench'
```

生成工程目录和同名 ZIP；已有目标不覆盖。脚本保存为 UTF-8 带 BOM，语法兼容 Windows PowerShell 5.1。

### 验证

核心配准与圆柱测试、内存/未定义行为检查、Windows x64/x86 资源与链接交叉编译、PowerShell 7 生成及打包验证通过。当前没有 Windows 桌面，尚未在 MSVC v142 / Visual Studio 2019 中实际构建或交互运行。

ZIP 只包含工程、脚本、说明、示例和测试，排除 `.git`、`node_modules`、`.vs`、编译产物及缓存。
