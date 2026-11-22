# VC2019 图片烟雾识别对话框工程

分别使用**传统 OpenCV 规则**与**已训练神经网络**识别 PNG/JPEG 图片中的烟雾。三栏同时显示输入、规则标注和神经网络标注，独立显示判断、分数/面积和耗时，不将两种结果合并成一个结论。

## 1. 准备与编译

1. 使用 Visual Studio Installer 安装“使用 C++ 的桌面开发”、MSVC v142 和 Windows 10 SDK。
2. 在本目录打开 PowerShell，执行：

   ```powershell
   .\scripts\Setup-OpenCV.ps1
   ```

   脚本从 OpenCV 官方 GitHub 下载约 183 MB 的 OpenCV 4.10.0 Windows 包，验证 SHA256 后解压至 `third_party`，写入本机专用的 `OpenCV.local.props`。下载和解压需要足够磁盘空间。

   如果已经安装了对应 OpenCV，可直接指定 **build** 目录：

   ```powershell
   .\scripts\Setup-OpenCV.ps1 -OpenCVRoot 'D:\Libraries\opencv\build'
   ```

   必须包含 `x64/vc16/lib/opencv_world4100.lib` 和对应 DLL；别的 OpenCV 版本需要同步修改工程依赖。脚本会核查 Release、Debug 文件。
3. 双击 `SmokeDetector.sln`，选择 **x64 / Release**，按 F5。也提供 x64 / Debug；官方包未提供这里所需的 Win32 库，因此工程只配置 x64。
4. 编译后自动将 OpenCV DLL、模型、模型许可和通知复制到输出目录。可执行文件位于 `bin/x64/Release/SmokeDetector.exe`，运行时请保留同目录 DLL、`models` 与许可文件。

若 PowerShell 提示禁止执行脚本，可只在当前窗口设置：

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\scripts\Setup-OpenCV.ps1
```

若提示 Windows SDK 不匹配，右键解决方案选择“重定解决方案目标”。

**工程已经包含真实训练的 ONNX 模型，无需再寻找或训练模型。运行时不需要 Python、PyTorch、CUDA或网络连接，图片不会上传；初次配置 OpenCV 时需要联网。**

## 2. 操作

1. 点击“打开 PNG / JPEG”，选择 `.png`、`.jpg` 或 `.jpeg`，支持中文路径。
2. 点击“OpenCV 检测”“神经网络检测”或“两种方法对比”。读取和检测在后台执行，对话框可调整大小、最大化和还原。
3. 传统方案使用“疑似区域面积 / 分析图面积”与面积阈值比较，默认 `1.0%`。候选区域会被着色并绘框，即使总占比未达到阈值，也可能有候选框。
4. 神经网络默认分数阈值为 `0.5`。红框为烟雾，橙框为火焰；火焰只辅助显示，不参与“是否含烟雾”的判断。
5. 结果报告记录该次实际使用的阈值。修改输入框后，必须重新检测，旧结果不会自动重新计算。
6. “导出结果”在所选父目录下生成独立的 `SmokeResult_*` 子目录，包含各方法的 PNG 标注图和 UTF-8 `report.txt`，记录源路径、原图/分析图尺寸、参数、候选框、类别、分数及耗时。原始文件不修改。
7. “取消”在处理阶段边界生效，不能立即中断 OpenCV 已进入的解码/推理调用；关闭窗口会等待后台任务结束。取消导出时可能已经生成了部分或全部文件。

最长边超过 1600 像素时，两种方法使用同一份等比例缩小的分析图。候选框及导出图的像素坐标对应**分析图**，报告同时记录原图尺寸。单文件上限 128 MiB、解码后上限 4000 万像素。使用 OpenCV 的 8 位 BGR 彩色解码；PNG 透明通道被忽略，16 位内容由解码器转为 8 位，不做 HDR 分析。

## 3. 两种识别方法

### 传统 OpenCV 规则

实现位于 `SmokeEngine.cpp::detectOpenCV`：

- HSV 低饱和度与中等亮度筛选；
- 局部标准差与拉普拉斯响应筛选相对柔和的纹理；
- 形态学开闭运算；
- 连通域大小、宽高和长宽比筛选；
- 疑似像素面积百分比达到用户阈值时报告“疑似含有烟雾”。

这是一种可解释的灰白烟雾候选规则，不是训练模型。面积比例**不是置信概率**。云、蒸汽、灰色墙面等可能误报；黑烟、彩色烟、很小或很淡的烟雾可能漏检。单张静态图片不能用运动、扩散和闪烁来消除歧义。

### 神经网络

使用 firewatch 发布的 **YOLO11s v0.1.0**，在 D-Fire 烟雾/火焰数据上训练。通过 OpenCV DNN 在 CPU 上执行 ONNX 推理；这与前面的传统规则是两个独立算法，虽然都使用 OpenCV C++ 库。

固定输入/输出契约：

- 输入 `[1,3,640,640]`，NCHW、RGB、FP32 除以 255；
- 等比例 letterbox，补边值 114，不裁剪；
- 输出 `[1,6,8400]`，依次为 `cx,cy,w,h,smoke_score,fire_score`；
- 类别 `0=smoke`、`1=fire`；没有额外 objectness，不做额外 sigmoid；
- 每个候选选择分数最高的类别，按用户阈值筛选；类内 NMS 的 IoU 阈值 0.45，然后还原到分析图坐标；
- 至少保留一个烟雾框才报告“检出烟雾”。纯火焰框不会成为烟雾阳性。

“最高烟雾候选分数”取未做最终类别选择/NMS前的原始烟雾分数，包含未被保留的候选，因此它本身不是最终是否检出的判据，也不是整张图片含烟的概率或准确率。首次检测的耗时包含模型载入，后续调用复用网络。

模型包含在 `models/firewatch-yolo11s.onnx`，SHA256 为：

```text
1ab25ae6a26e711542e5b5e9670611476719532bc00ef1bd1a1657c01f24c2b4
```

输入输出元信息见 `models/model.json`。程序按该固定模型设计，不提供任意 ONNX 模型的自动识别。若模型丢失，可执行 `scripts/Restore-Model.ps1` 从固定上游版本恢复并校验；缺模型或推理失败会显示错误，不会伪装成“无烟雾”。

## 4. 示例与结果解释

`samples` 包含上游已标注的烟雾照片、火烟混合照片，以及自行生成的规则纹理 PNG 和纯色 JPEG。详见该目录说明；输入图上的上游框本来就存在，不能误当成本工程新画的框。

功能测试中，烟雾示例的最高烟雾分数约为 0.6765；火烟混合示例虽然含有上游烟雾标注，但当前模型在默认阈值下仅检出火焰，**漏检了烟雾**；空白图未检出烟雾。混合图是失败案例，不是无烟阴性样本。上游照片与模型来自同一来源，且已带标注，**不是独立留出测试集**。这些测试验证运行路径，不提供本工程准确率、召回率或误报率。需用自己的、独立标注的场景图片评估两种方法。

“未检出”不代表图片必定无烟，两种方法可能给出不同结果。

## 5. 测试与验证范围

- 已在 Linux 上使用 OpenCV 4.10 编译并执行实际 C++ 检测核心测试，包括 PNG/JPEG 解码、中文路径、长边缩放、规则正/负构造输入、阈值、letterbox、坐标还原、类内 NMS、坏输出形状、缺模型、取消和导出。
- 使用随包真实 ONNX 执行空白图片、烟雾示例和火烟混合示例推理，记录后者的漏检情形。
- 另用 ONNX Runtime CPU 1.23.2 对照 OpenCV DNN，两个示例的最大类别分数差异小于 0.000002；混合图的漏检在两种后端均存在，不是已发现的大幅推理后端差异。可选复核脚本为 `tests/check_runtime_parity.py`，不参与应用运行。
- 检测核心和测试代码通过 AddressSanitizer / UndefinedBehaviorSanitizer 检查；所链接的 OpenCV 静态库本身未开启这些检查。
- Windows x64 的对话框资源、主界面及检测代码通过交叉编译到对象文件的语法检查。已核对官方 Windows 包内 vc16/x64 库和 DLL 的布局、安装包与模型 SHA256。
- PowerShell 7 已验证脚本解析、现有 OpenCV 目录配置分支（含空格/中文/`&` 路径及 XML 转义）与已有模型校验分支；未在 Windows 上执行自动解压程序。测试记录见 `tests/VALIDATION.md`。
- **尚未在 Windows 桌面或 Visual Studio 2019 / MSVC v142 上实际完整编译、链接和交互运行。** 对象文件检查不等于 Windows 完整构建。

在 VS2019 Developer PowerShell 配置 OpenCV 后可执行：

```powershell
.\scripts\Run-Tests.ps1
```

也提供 `CMakeLists.txt` 用于支持 C++17 与 OpenCV 4.10 的平台运行核心测试：

```text
cmake -S . -B build-tests -DOpenCV_DIR=<OpenCV的CMake配置目录>
cmake --build build-tests --config Release
ctest --test-dir build-tests -C Release --output-on-failure
```

## 6. 模型与许可

本烟雾识别模块及随附模型采用 **AGPL-3.0**，不是任意用途的无条件授权。完整文本见 `LICENSE-AGPL-3.0.txt`；模型作者、原始下载链接、训练数据来源、训练/导出源码指引和 OpenCV 的 Apache-2.0 许可见 `THIRD_PARTY_NOTICES.md`、`model-source/` 与 `licenses/`。

模型来源：https://github.com/leskoont/firewatch/releases/tag/v0.1.0

源 ZIP 不包含 OpenCV 安装包、库文件、本机路径配置、`node_modules`、`.git`、`bin`、`obj` 或缓存；包含运行所需的训练模型及相应来源/许可说明。
