#requires -Version 5.1
param([string]$Destination = (Join-Path (Get-Location).Path 'PointCloudWorkbench'))
$ErrorActionPreference = 'Stop'
$Destination = [System.IO.Path]::GetFullPath($Destination)
$zipPath = $Destination + '.zip'
if ((Test-Path -LiteralPath $Destination) -or (Test-Path -LiteralPath $zipPath)) {
    throw '目标目录或 ZIP 已存在，请指定新的 -Destination，避免覆盖。'
}
New-Item -ItemType Directory -Path $Destination | Out-Null
$utf8 = New-Object System.Text.UTF8Encoding($true)
function Write-BundleFile([string]$Name, [string]$Content) {
    $path = Join-Path $Destination $Name
    $parent = Split-Path -Parent $path
    if (-not (Test-Path -LiteralPath $parent)) { New-Item -ItemType Directory -Path $parent | Out-Null }
    [System.IO.File]::WriteAllText($path, $Content + [Environment]::NewLine, $utf8)
}

Write-BundleFile 'README.md' @'
# CloudAlgorithm
The Algorithm of analysis and manupulation of Cloud Points

## VC2019 点云工作台

**[下载完整工程 ZIP](https://github.com/yys801162003/CloudAlgorithm/raw/refs/heads/main/downloads/PointCloudWorkbench_VS2019.zip)**

[工程及算法说明](VC2019/PointCloudWorkbench/README.md) · [VS2019 解决方案](VC2019/PointCloudWorkbench/PointCloudWorkbench.sln) · [PowerShell 生成脚本](scripts/CreateProject.ps1) · [示例 PCD](samples)

### 功能

- 可拖动边框和四角调整大小的 Win32 对话框，支持最大化/还原；点云可旋转、平移、缩放。
- 分别读取两份 PCD，支持 ASCII、binary、binary_compressed（LZF），支持中文路径。
- 使用 PCA 多姿态初始化与 ICP 将 B 刚体配准到 A，显示几何相似度、双向重合率、RMSE，红色/紫色标记两侧差异点。
- 独立对 A/B 进行单圆柱重建，显示半径、直径、轴线、长度、拟合误差及内点率，黄色线框显示圆柱侧面，绿色/红色标记圆柱内外点。
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
'@

Write-BundleFile '.gitignore' @'
# Dependencies and IDE/build caches
node_modules/
.vs/
bin/
obj/
Debug/
Release/
x64/
x86/
*.user
*.suo
*.obj
*.pdb
*.ilk
*.ipch

# Additional generated build and test files
*.exe
*.res
*.o
.cache/
core-test-data/
core-tests

cylinder-test-data/
cylinder-tests
'@

Write-BundleFile 'VC2019/PointCloudWorkbench/Cylinder.cpp' @'
#include "Cylinder.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>

namespace pc { namespace {
constexpr double Pi=3.14159265358979323846;
void require(bool condition,const char* message) {if(!condition) throw std::runtime_error(message);}
void cancelled(const std::atomic_bool* cancel) {if(cancel&&cancel->load()) throw std::runtime_error("Operation cancelled.");}
double dot(Vec3 a,Vec3 b) {return a.x*b.x+a.y*b.y+a.z*b.z;}
Vec3 cross(Vec3 a,Vec3 b) {return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
Vec3 unit(Vec3 p) {double n=std::sqrt(norm2(p));return n>1e-15?p*(1/n):Vec3{0,0,1};}
void frame(Vec3 axis,Vec3& u,Vec3& v) {
    u=unit(cross(axis,std::abs(axis.x)<0.8?Vec3{1,0,0}:Vec3{0,1,0}));v=cross(axis,u);
}
struct Model {Vec3 center,axis{0,0,1};double radius=1;};
double residual(const Model& m,Vec3 p) {
    Vec3 q=p-m.center;return std::sqrt(std::max(0.0,norm2(q)-dot(q,m.axis)*dot(q,m.axis)))-m.radius;
}
Model perturb(const Model& m,int parameter,double amount) {
    Vec3 u,v;frame(m.axis,u,v);Model out=m;
    if(parameter<2) out.center=out.center+(parameter==0?u:v)*amount;
    else if(parameter<4) out.axis=unit(out.axis+(parameter==2?u:v)*amount);
    else out.radius+=amount;
    out.center=out.center-out.axis*dot(out.center,out.axis);return out;
}
bool solve(double a[5][6],int n,double* x) {
    for(int i=0;i<n;++i) {
        int pivot=i;for(int j=i+1;j<n;++j) if(std::abs(a[j][i])>std::abs(a[pivot][i])) pivot=j;
        if(std::abs(a[pivot][i])<1e-15) return false;
        for(int j=i;j<=n;++j) std::swap(a[pivot][j],a[i][j]);
        double d=a[i][i];for(int j=i;j<=n;++j) a[i][j]/=d;
        for(int k=0;k<n;++k) if(k!=i) {double factor=a[k][i];for(int j=i;j<=n;++j)a[k][j]-=factor*a[i][j];}
    }
    for(int i=0;i<n;++i)x[i]=a[i][n];
    return true;
}
double score(const Model& m,const std::vector<Vec3>& points) {
    if(!std::isfinite(m.radius)||m.radius<1e-6||m.radius>10) return 1e30;
    std::vector<double> distances;distances.reserve(points.size());
    for(auto p:points) {double r=residual(m,p);distances.push_back(r*r);}
    size_t count=std::max(size_t(1),points.size()*80/100);
    std::nth_element(distances.begin(),distances.begin()+static_cast<ptrdiff_t>(count-1),distances.end());
    return std::accumulate(distances.begin(),distances.begin()+static_cast<ptrdiff_t>(count),0.0)/static_cast<double>(count);
}
Model refine(Model m,const std::vector<Vec3>& points,const std::atomic_bool* cancel) {
    double current=score(m,points),lambda=1e-4;
    for(int iteration=0;iteration<65;++iteration) {
        cancelled(cancel);
        std::vector<double> absResidual;absResidual.reserve(points.size());
        for(auto p:points) absResidual.push_back(std::abs(residual(m,p)));
        size_t index=points.size()*80/100;
        std::nth_element(absResidual.begin(),absResidual.begin()+static_cast<ptrdiff_t>(index),absResidual.end());
        double cutoff=std::max(1e-9,absResidual[index]);double normal[5][6]={};
        Model changed[5];for(int j=0;j<5;++j) changed[j]=perturb(m,j,1e-5);
        for(auto p:points) {
            double r=residual(m,p);if(std::abs(r)>cutoff) continue;
            double jacobian[5];for(int j=0;j<5;++j)jacobian[j]=(residual(changed[j],p)-r)/1e-5;
            for(int j=0;j<5;++j) {
                for(int k=0;k<5;++k)normal[j][k]+=jacobian[j]*jacobian[k];
                normal[j][5]-=jacobian[j]*r;
            }
        }
        for(int j=0;j<5;++j)normal[j][j]+=lambda*std::max(1.0,normal[j][j]);
        double step[5]={};if(!solve(normal,5,step))break;
        double largest=0;for(double s:step)largest=std::max(largest,std::abs(s));
        double factor=largest>0.4?0.4/largest:1;
        // Apply in a fixed local frame, followed by one normalization.
        Vec3 u,v;frame(m.axis,u,v);Model trial=m;
        trial.center=trial.center+u*(step[0]*factor)+v*(step[1]*factor);
        trial.axis=unit(m.axis+u*(step[2]*factor)+v*(step[3]*factor));
        trial.center=trial.center-trial.axis*dot(trial.center,trial.axis);
        trial.radius+=step[4]*factor;double candidate=score(trial,points);
        if(candidate<current) {
            double change=current-candidate;m=trial;current=candidate;lambda=std::max(1e-10,lambda*0.3);
            if(change<1e-16||largest<1e-9)break;
        } else {lambda*=8;if(lambda>1e10)break;}
    }
    return m;
}
std::vector<Vec3> principalAxes(const std::vector<Vec3>& points) {
    Vec3 mean;for(auto p:points)mean=mean+p;mean=mean*(1.0/static_cast<double>(points.size()));
    double a[3][3]={},v[3][3]={{1,0,0},{0,1,0},{0,0,1}};
    for(auto p:points){p=p-mean;for(int i=0;i<3;++i)for(int j=0;j<3;++j)a[i][j]+=p[i]*p[j];}
    for(int step=0;step<50;++step) {
        int p=0,q=1;for(int i=0;i<3;++i)for(int j=i+1;j<3;++j)if(std::abs(a[i][j])>std::abs(a[p][q])){p=i;q=j;}
        if(std::abs(a[p][q])<1e-14)break;
        double angle=0.5*std::atan2(2*a[p][q],a[q][q]-a[p][p]),c=std::cos(angle),s=std::sin(angle);
        double pp=a[p][p],qq=a[q][q],pq=a[p][q];
        for(int k=0;k<3;++k)if(k!=p&&k!=q){double x=a[k][p],y=a[k][q];a[k][p]=a[p][k]=c*x-s*y;a[k][q]=a[q][k]=s*x+c*y;}
        a[p][p]=c*c*pp-2*c*s*pq+s*s*qq;a[q][q]=s*s*pp+2*c*s*pq+c*c*qq;a[p][q]=a[q][p]=0;
        for(int k=0;k<3;++k){double x=v[k][p],y=v[k][q];v[k][p]=c*x-s*y;v[k][q]=s*x+c*y;}
    }
    return {{v[0][0],v[1][0],v[2][0]},{v[0][1],v[1][1],v[2][1]},{v[0][2],v[1][2],v[2][2]}};
}
Model circleFromThree(Vec3 axis,Vec3 u,Vec3 v,Vec3 p,Vec3 q,Vec3 s) {
    double x1=dot(p,u),y1=dot(p,v),x2=dot(q,u),y2=dot(q,v),x3=dot(s,u),y3=dot(s,v);
    double d=2*(x1*(y2-y3)+x2*(y3-y1)+x3*(y1-y2));Model m;m.axis=axis;m.radius=-1;
    if(std::abs(d)<1e-10)return m;
    double a=x1*x1+y1*y1,b=x2*x2+y2*y2,c=x3*x3+y3*y3;
    double x=(a*(y2-y3)+b*(y3-y1)+c*(y1-y2))/d;
    double y=(a*(x3-x2)+b*(x1-x3)+c*(x2-x1))/d;
    m.center=u*x+v*y;m.radius=std::hypot(x1-x,y1-y);return m;
}
Model algebraicCircle(Vec3 axis,Vec3 u,Vec3 v,const std::vector<Vec3>& points) {
    double normal[5][6]={};
    for(auto p:points) {
        double x=dot(p,u),y=dot(p,v),j[3]={2*x,2*y,1},value=x*x+y*y;
        for(int a=0;a<3;++a){for(int b=0;b<3;++b)normal[a][b]+=j[a]*j[b];normal[a][3]+=j[a]*value;}
    }
    Model m;m.axis=axis;m.radius=-1;double x[5]={};
    if(solve(normal,3,x)&&x[2]+x[0]*x[0]+x[1]*x[1]>0) {
        m.center=u*x[0]+v*x[1];m.radius=std::sqrt(x[2]+x[0]*x[0]+x[1]*x[1]);
    }
    return m;
}
void writeCloud(const std::filesystem::path& file,const std::vector<Vec3>& points,
    const std::vector<unsigned char>* mask,bool inside,const std::atomic_bool* cancel) {
    size_t count=points.size();if(mask)count=static_cast<size_t>(std::count(mask->begin(),mask->end(),inside?1:0));
    std::ofstream out(file);require(bool(out),"Cannot create cylinder PCD export.");
    out<<"VERSION 0.7\nFIELDS x y z\nSIZE 8 8 8\nTYPE F F F\nCOUNT 1 1 1\nWIDTH "<<count<<"\nHEIGHT 1\nPOINTS "<<count<<"\nDATA ascii\n"<<std::setprecision(17);
    for(size_t i=0;i<points.size();++i){if(i%1024==0)cancelled(cancel);if(!mask||(((*mask)[i]!=0)==inside))out<<points[i].x<<' '<<points[i].y<<' '<<points[i].z<<'\n';}
    require(bool(out),"Failed to write cylinder PCD.");
}
} // namespace
Cylinder reconstructCylinder(const Cloud& cloud,double distanceThreshold,const std::atomic_bool* cancel,Progress progress) {
    require(cloud.points.size()>=30,"Cylinder reconstruction requires at least 30 finite points.");
    require(std::isfinite(distanceThreshold)&&distanceThreshold>=0,"Cylinder tolerance must be finite and nonnegative.");
    cancelled(cancel);if(progress)progress("Cylinder: generating robust axis and circle hypotheses...");
    Vec3 origin;std::vector<Vec3> selected;size_t n=std::min(size_t(6000),cloud.points.size());selected.reserve(n);
    // Reservoir sampling avoids aliasing the ring/scanline order of organized PCDs.
    std::mt19937 reservoir(9123801);
    for(size_t i=0;i<cloud.points.size();++i) {
        if(i%2048==0)cancelled(cancel);
        Vec3 p=cloud.points[i];
        require(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z),"Cylinder input contains nonfinite points.");
        if(i<n)selected.push_back(p);
        else {std::uniform_int_distribution<size_t> pick(0,i);size_t j=pick(reservoir);if(j<n)selected[j]=p;}
    }
    for(auto p:selected)origin=origin+p;
    origin=origin*(1.0/static_cast<double>(n));double scale=0;for(auto p:selected)scale+=norm2(p-origin);
    scale=std::sqrt(scale/static_cast<double>(n));require(scale>1e-12&&std::isfinite(scale),"Degenerate cylinder input.");
    for(auto& p:selected)p=(p-origin)*(1/scale);
    std::vector<Vec3> coarse;for(size_t i=0;i<std::min(size_t(360),n);++i)coarse.push_back(selected[i*n/std::min(size_t(360),n)]);
    auto directions=principalAxes(selected);
    for(int i=0;i<96;++i){double z=(i+0.5)/96.0,angle=i*2.399963229728653;double r=std::sqrt(1-z*z);directions.push_back({r*std::cos(angle),r*std::sin(angle),z});}
    struct Candidate{Model m;double score;};std::vector<Candidate> candidates;
    std::mt19937 random(801162003);std::uniform_int_distribution<size_t> index(0,coarse.size()-1);
    for(auto axis:directions) {
        cancelled(cancel);axis=unit(axis);Vec3 u,v;frame(axis,u,v);
        Model best=algebraicCircle(axis,u,v,coarse);double bestScore=score(best,coarse);
        for(int trial=0;trial<36;++trial) {
            Model m=circleFromThree(axis,u,v,coarse[index(random)],coarse[index(random)],coarse[index(random)]);
            double value=score(m,coarse);if(value<bestScore){best=m;bestScore=value;}
        }
        candidates.push_back({best,bestScore});
    }
    std::sort(candidates.begin(),candidates.end(),[](const Candidate& a,const Candidate& b){return a.score<b.score;});
    if(progress)progress("Cylinder: refining center, axis and radius...");
    Model best;double bestScore=1e30;
    for(size_t i=0;i<std::min(size_t(10),candidates.size());++i) {
        Model m=refine(candidates[i].m,selected,cancel);double value=score(m,selected);
        if(value<bestScore){best=m;bestScore=value;}
    }
    require(bestScore<1e29&&best.radius>1e-6,"No stable cylinder model found.");
    Cylinder out;out.axis=best.axis;
    int dominant=0;for(int j=1;j<3;++j)if(std::abs(out.axis[j])>std::abs(out.axis[dominant]))dominant=j;
    if(out.axis[dominant]<0)out.axis=out.axis*(-1);
    out.radius=best.radius*scale;
    out.threshold=distanceThreshold>0?distanceThreshold:std::min(scale*0.01,out.radius*0.05);
    require(out.threshold<out.radius*0.3,"Tolerance is too large relative to the fitted radius; reduce it.");
    Vec3 axisPoint=best.center*scale+origin;std::vector<double> heights,angles;Vec3 u,v;frame(out.axis,u,v);
    if(progress)progress("Cylinder: validating full-resolution inliers and building mesh...");
    for(size_t i=0;i<cloud.points.size();++i) {
        if(i%2048==0)cancelled(cancel);
        Vec3 q=cloud.points[i]-axisPoint;
        if(std::abs(std::sqrt(std::max(0.0,norm2(q)-dot(q,out.axis)*dot(q,out.axis)))-out.radius)<=out.threshold) {
            heights.push_back(dot(q,out.axis));angles.push_back(std::atan2(dot(q,v),dot(q,u)));
        }
    }
    require(heights.size()>=30&&heights.size()>=cloud.points.size()*55/100,"No dominant cylinder: fewer than 55% radial inliers. Segment the pipe or check tolerance.");
    std::sort(heights.begin(),heights.end());size_t trim=heights.size()/100;
    double low=heights[trim],high=heights[heights.size()-1-trim];out.height=high-low;
    require(out.height>std::max(out.radius*0.02,out.threshold*3),"Insufficient axial extent for cylinder reconstruction.");
    std::sort(angles.begin(),angles.end());double largest=angles.front()+2*Pi-angles.back();
    for(size_t i=1;i<angles.size();++i)largest=std::max(largest,angles[i]-angles[i-1]);
    out.angularCoverage=2*Pi-largest;require(out.angularCoverage>Pi/2,"Cylinder arc coverage below 90 degrees; fit is ambiguous.");
    out.center=axisPoint+out.axis*((low+high)*0.5);out.inliers.resize(cloud.points.size());double error=0;size_t count=0;
    for(size_t i=0;i<cloud.points.size();++i) {
        if(i%2048==0)cancelled(cancel);
        Vec3 q=cloud.points[i]-axisPoint;double h=dot(q,out.axis);
        double e=std::sqrt(std::max(0.0,norm2(q)-h*h))-out.radius;
        bool in=std::abs(e)<=out.threshold&&h>=low-out.threshold&&h<=high+out.threshold;
        out.inliers[i]=static_cast<unsigned char>(in);if(in){error+=e*e;++count;}
    }
    out.inlierRatio=static_cast<double>(count)/static_cast<double>(cloud.points.size());
    require(count>=30&&out.inlierRatio>=0.5,"Cylinder support insufficient after axial trimming.");
    out.rmse=std::sqrt(error/static_cast<double>(count));
    const size_t rings=25,segments=64;
    for(size_t h=0;h<rings;++h)for(size_t a=0;a<segments;++a) {
        double angle=2*Pi*static_cast<double>(a)/segments;
        Vec3 point=out.center+out.axis*(out.height*(static_cast<double>(h)/(rings-1)-0.5))+
            (u*std::cos(angle)+v*std::sin(angle))*out.radius;
        out.vertices.push_back(point);
    }
    for(size_t h=0;h+1<rings;++h)for(size_t a=0;a<segments;++a) {
        size_t i=h*segments+a,j=h*segments+(a+1)%segments,k=i+segments,l=j+segments;
        out.triangles.push_back({i,j,k});out.triangles.push_back({j,l,k});
    }
    return out;
}
void exportCylinder(const std::filesystem::path& dir,const Cloud& source,const Cylinder& model,const std::atomic_bool* cancel) {
    require(model.inliers.size()==source.points.size()&&!model.vertices.empty(),"No valid reconstructed cylinder.");cancelled(cancel);
    require(!std::filesystem::exists(dir),"Export directory already exists.");std::filesystem::create_directories(dir);
    writeCloud(dir/"cylinder_surface.pcd",model.vertices,nullptr,true,cancel);
    writeCloud(dir/"cylinder_inliers.pcd",source.points,&model.inliers,true,cancel);
    writeCloud(dir/"cylinder_outliers.pcd",source.points,&model.inliers,false,cancel);
    std::ofstream obj(dir/"cylinder.obj");require(bool(obj),"Cannot create OBJ file.");obj<<std::setprecision(17);
    for(auto p:model.vertices)obj<<"v "<<p.x<<' '<<p.y<<' '<<p.z<<'\n';
    for(auto f:model.triangles)obj<<"f "<<f[0]+1<<' '<<f[1]+1<<' '<<f[2]+1<<'\n';
    require(bool(obj),"Failed to write OBJ file.");
    std::ofstream report(dir/"cylinder_report.txt");require(bool(report),"Cannot create cylinder report.");
    report<<std::setprecision(17)<<"Radius="<<model.radius<<"\nDiameter="<<2*model.radius<<"\nHeight="<<model.height
        <<"\nCenter="<<model.center.x<<' '<<model.center.y<<' '<<model.center.z
        <<"\nAxis="<<model.axis.x<<' '<<model.axis.y<<' '<<model.axis.z
        <<"\nDistance_threshold="<<model.threshold<<"\nInlier_ratio="<<model.inlierRatio<<"\nRadial_RMSE="<<model.rmse
        <<"\nObserved_arc_degrees="<<model.angularCoverage*180/Pi
        <<"\nOpen-ended full 360-degree analytical side surface; end caps and unobserved areas are not measured.\n";
    require(bool(report),"Failed to write cylinder report.");
}
}
'@

Write-BundleFile 'VC2019/PointCloudWorkbench/Cylinder.h' @'
#pragma once
#include "PointCloud.h"

namespace pc {
struct Cylinder {
    Vec3 center;       // Midpoint of the reconstructed finite axis segment.
    Vec3 axis;         // Unit vector in the input cloud's coordinates.
    double radius=0, height=0, threshold=0, rmse=0, inlierRatio=0, angularCoverage=0;
    std::vector<unsigned char> inliers; // One entry per original valid point.
    std::vector<Vec3> vertices;
    std::vector<std::array<size_t,3>> triangles;
};
Cylinder reconstructCylinder(const Cloud& cloud, double distanceThreshold=0,
    const std::atomic_bool* cancel=nullptr, Progress progress={});
void exportCylinder(const std::filesystem::path& directory, const Cloud& source,
    const Cylinder& cylinder, const std::atomic_bool* cancel=nullptr);
}
'@

Write-BundleFile 'VC2019/PointCloudWorkbench/PointCloud.cpp' @'
#include "PointCloud.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace pc {
Vec3 operator+(Vec3 a, Vec3 b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
Vec3 operator-(Vec3 a, Vec3 b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
Vec3 operator*(Vec3 a, double s) { return {a.x*s,a.y*s,a.z*s}; }
double norm2(Vec3 a) { return a.x*a.x+a.y*a.y+a.z*a.z; }
Vec3 Transform::apply(Vec3 p) const {
    return {r[0]*p.x+r[1]*p.y+r[2]*p.z+t.x,
        r[3]*p.x+r[4]*p.y+r[5]*p.z+t.y,
        r[6]*p.x+r[7]*p.y+r[8]*p.z+t.z};
}
namespace {
constexpr size_t MaxPoints = 2000000, MaxBytes = 512ull*1024*1024;
void check(const std::atomic_bool* cancel) {
    if (cancel && cancel->load()) throw std::runtime_error("Operation cancelled.");
}
void require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
struct Field { std::string name, type; size_t size=0, count=1, offset=0, column=0; };
size_t positive(const std::string& s) {
    require(!s.empty() && s[0]!='-', "Invalid PCD header integer.");
    size_t used=0; auto v=std::stoull(s,&used);
    require(used==s.size() && v<=MaxBytes,"PCD header value exceeds limit.");
    return static_cast<size_t>(v);
}
template<class T> double scalar(const unsigned char* p) {
    T v{}; std::memcpy(&v,p,sizeof(v)); return static_cast<double>(v);
}
double decode(const unsigned char* p,const Field& f) {
    if (f.type=="F" && f.size==4) return scalar<float>(p);
    if (f.type=="F" && f.size==8) return scalar<double>(p);
    if (f.type=="I") {
        if(f.size==1) return scalar<int8_t>(p);
        if(f.size==2) return scalar<int16_t>(p);
        if(f.size==4) return scalar<int32_t>(p);
        if(f.size==8) return scalar<int64_t>(p);
    }
    if (f.type=="U") {
        if(f.size==1) return scalar<uint8_t>(p);
        if(f.size==2) return scalar<uint16_t>(p);
        if(f.size==4) return scalar<uint32_t>(p);
        if(f.size==8) return scalar<uint64_t>(p);
    }
    throw std::runtime_error("Unsupported PCD field type/size.");
}
std::vector<unsigned char> unlzf(const std::vector<unsigned char>& src,size_t bytes,const std::atomic_bool* cancel) {
    std::vector<unsigned char> out(bytes); size_t i=0,o=0;
    while(i<src.size()) {
        if(i%4096<32) check(cancel);
        unsigned ctrl=src[i++];
        if(ctrl<32) {
            size_t len=ctrl+1;
            require(i+len<=src.size() && o+len<=bytes,"Invalid LZF literal.");
            std::memcpy(out.data()+o,src.data()+i,len); i+=len; o+=len;
        } else {
            size_t len=ctrl>>5, back=(ctrl&31)<<8;
            if(len==7) { require(i<src.size(),"Truncated LZF length."); len+=src[i++]; }
            require(i<src.size(),"Truncated LZF reference."); back+=src[i++]+1; len+=2;
            require(back<=o && len<=bytes-o,"Invalid LZF reference.");
            for(size_t j=0;j<len;++j) { out[o]=out[o-back]; ++o; }
        }
    }
    require(o==bytes,"PCD decompressed size mismatch."); return out;
}
bool finite(Vec3 p) { return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z); }
Vec3 centroid(const std::vector<Vec3>& p) {
    Vec3 c; for(auto v:p) c=c+v; return c*(1.0/static_cast<double>(p.size()));
}
double diagonal(const std::vector<Vec3>& p) {
    Vec3 lo=p.front(),hi=lo;
    for(auto v:p) {
        lo={std::min(lo.x,v.x),std::min(lo.y,v.y),std::min(lo.z,v.z)};
        hi={std::max(hi.x,v.x),std::max(hi.y,v.y),std::max(hi.z,v.z)};
    }
    return std::sqrt(norm2(hi-lo));
}
struct KDTree {
    struct Node { size_t point=0; int left=-1,right=-1,axis=0; };
    const std::vector<Vec3>& p; std::vector<size_t> order; std::vector<Node> nodes;
    int root=-1;
    explicit KDTree(const std::vector<Vec3>& v):p(v),order(v.size()) {
        std::iota(order.begin(),order.end(),size_t(0)); nodes.reserve(v.size());
        root=build(0,order.size(),0);
    }
    int build(size_t begin,size_t end,int depth) {
        if(begin==end) return -1;
        size_t mid=begin+(end-begin)/2; int axis=depth%3;
        std::nth_element(order.begin()+begin,order.begin()+mid,order.begin()+end,
            [&](size_t a,size_t b){return p[a][axis]<p[b][axis];});
        int index=static_cast<int>(nodes.size()); nodes.push_back({order[mid],-1,-1,axis});
        int left=build(begin,mid,depth+1),right=build(mid+1,end,depth+1);
        nodes[index].left=left; nodes[index].right=right; return index;
    }
    void visit(int node,Vec3 q,size_t& best,double& distance) const {
        if(node<0) return;
        const Node& n=nodes[node]; double d=norm2(q-p[n.point]);
        if(d<distance) {distance=d;best=n.point;}
        if(distance==0) return;
        double delta=q[n.axis]-p[n.point][n.axis];
        visit(delta<0?n.left:n.right,q,best,distance);
        if(delta*delta<=distance) visit(delta<0?n.right:n.left,q,best,distance);
    }
    std::pair<size_t,double> nearest(Vec3 q) const {
        size_t best=0; double distance=std::numeric_limits<double>::infinity();
        visit(root,q,best,distance); return {best,distance};
    }
};
// Jacobi eigenvectors are columns; used for PCA and Horn's quaternion fit.
template<int N> void eigen(double (&a)[N][N],double (&v)[N][N]) {
    for(int i=0;i<N;++i) for(int j=0;j<N;++j) v[i][j]=i==j?1.0:0.0;
    for(int step=0;step<100;++step) {
        int p=0,q=1; double largest=std::abs(a[0][1]),scale=0;
        for(int i=0;i<N;++i) {
            scale+=std::abs(a[i][i]);
            for(int j=i+1;j<N;++j) if(std::abs(a[i][j])>largest) {largest=std::abs(a[i][j]);p=i;q=j;}
        }
        if(largest<=std::max(1e-30,scale*1e-14)) break;
        double angle=0.5*std::atan2(2*a[p][q],a[q][q]-a[p][p]);
        double c=std::cos(angle),s=std::sin(angle);
        double app=a[p][p],aqq=a[q][q],apq=a[p][q];
        for(int k=0;k<N;++k) if(k!=p&&k!=q) {
            double kp=a[k][p],kq=a[k][q];
            a[k][p]=a[p][k]=c*kp-s*kq; a[k][q]=a[q][k]=s*kp+c*kq;
        }
        a[p][p]=c*c*app-2*c*s*apq+s*s*aqq;
        a[q][q]=s*s*app+2*c*s*apq+c*c*aqq; a[p][q]=a[q][p]=0;
        for(int k=0;k<N;++k) {double kp=v[k][p],kq=v[k][q];v[k][p]=c*kp-s*kq;v[k][q]=s*kp+c*kq;}
    }
}
Transform compose(const Transform& a,const Transform& b) {
    Transform out;
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) {
        out.r[3*i+j]=0; for(int k=0;k<3;++k) out.r[3*i+j]+=a.r[3*i+k]*b.r[3*k+j];
    }
    out.t=a.apply(b.t); return out;
}
Transform fit(const std::vector<Vec3>& from,const std::vector<Vec3>& to) {
    Vec3 c=centroid(from),d=centroid(to); double s[3][3]={};
    for(size_t i=0;i<from.size();++i) {
        Vec3 p=from[i]-c,q=to[i]-d;
        for(int j=0;j<3;++j) for(int k=0;k<3;++k) s[j][k]+=p[j]*q[k];
    }
    double tr=s[0][0]+s[1][1]+s[2][2];
    double n[4][4]={
        {tr,s[1][2]-s[2][1],s[2][0]-s[0][2],s[0][1]-s[1][0]},
        {s[1][2]-s[2][1],2*s[0][0]-tr,s[0][1]+s[1][0],s[0][2]+s[2][0]},
        {s[2][0]-s[0][2],s[0][1]+s[1][0],2*s[1][1]-tr,s[1][2]+s[2][1]},
        {s[0][1]-s[1][0],s[0][2]+s[2][0],s[1][2]+s[2][1],2*s[2][2]-tr}};
    double v[4][4]; eigen(n,v); int best=0;
    for(int i=1;i<4;++i) if(n[i][i]>n[best][best]) best=i;
    double w=v[0][best],x=v[1][best],y=v[2][best],z=v[3][best];
    Transform t; t.r={{1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),
        2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),
        2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)}};
    t.t=d-t.apply(c); return t;
}
std::array<double,9> basis(const std::vector<Vec3>& p) {
    Vec3 c=centroid(p); double a[3][3]={},v[3][3];
    for(auto q:p) {q=q-c;for(int i=0;i<3;++i) for(int j=0;j<3;++j) a[i][j]+=q[i]*q[j];}
    eigen(a,v); std::array<int,3> order{{0,1,2}};
    std::sort(order.begin(),order.end(),[&](int i,int j){return a[i][i]>a[j][j];});
    require(a[order[1]][order[1]]>std::max(1e-30,a[order[0]][order[0]]*1e-12),"Cloud is collinear: rigid orientation is ambiguous.");
    std::array<double,9> b{};
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) b[3*i+j]=v[i][order[j]];
    return b;
}
double determinant(const std::array<double,9>& r) {
    return r[0]*(r[4]*r[8]-r[5]*r[7])-r[1]*(r[3]*r[8]-r[5]*r[6])+r[2]*(r[3]*r[7]-r[4]*r[6]);
}
struct Cell { long long x,y,z; bool operator==(const Cell& b) const{return x==b.x&&y==b.y&&z==b.z;} };
struct Hash { size_t operator()(const Cell& c) const {
    size_t h=std::hash<long long>{}(c.x);h^=std::hash<long long>{}(c.y)+0x9e3779b9+(h<<6)+(h>>2);
    return h^(std::hash<long long>{}(c.z)+0x9e3779b9+(h<<6)+(h>>2));
}};
std::vector<Vec3> sample(const std::vector<Vec3>& p,double voxel,const std::atomic_bool* cancel) {
    struct Sum {Vec3 sum;size_t n=0;}; std::unordered_map<Cell,Sum,Hash> cells;
    Vec3 origin=p.front();
    for(size_t i=0;i<p.size();++i) {
        if(i%4096==0) check(cancel);
        Vec3 q=(p[i]-origin)*(1.0/voxel);
        Cell key{static_cast<long long>(std::floor(q.x)),static_cast<long long>(std::floor(q.y)),static_cast<long long>(std::floor(q.z))};
        auto& s=cells[key];s.sum=s.sum+p[i];++s.n;
    }
    std::vector<Vec3> out; out.reserve(cells.size());
    for(const auto& item:cells) out.push_back(item.second.sum*(1.0/static_cast<double>(item.second.n)));
    std::sort(out.begin(),out.end(),[](Vec3 a,Vec3 b){return a.x!=b.x?a.x<b.x:a.y!=b.y?a.y<b.y:a.z<b.z;});
    if(out.size()>10000) {
        std::vector<Vec3> limit;limit.reserve(10000);
        for(size_t i=0;i<10000;++i) limit.push_back(out[i*out.size()/10000]);
        out.swap(limit);
    }
    return out;
}
struct Candidate {Transform t;double error=0;int iterations=0;};
double cost(const KDTree& tree,const std::vector<Vec3>& b,const Transform& t,double cap) {
    double total=0;size_t count=0;
    size_t stride=std::max(size_t(1),b.size()/1500);
    for(size_t i=0;i<b.size();i+=stride) {total+=std::min(cap,tree.nearest(t.apply(b[i])).second);++count;}
    return total/static_cast<double>(count);
}
Candidate refine(const KDTree& tree,const std::vector<Vec3>& b,Transform t,
    double scale,int limit,const std::atomic_bool* cancel) {
    double prev=std::numeric_limits<double>::infinity();int iterations=0;
    for(int step=0;step<limit;++step) {
        check(cancel);
        struct Pair {Vec3 p,q;double d;};std::vector<Pair> pairs;pairs.reserve(b.size());
        double gate=scale*(step<8?0.4:0.15);double gate2=gate*gate;
        for(size_t i=0;i<b.size();++i) {
            if(i%2048==0) check(cancel);
            Vec3 p=t.apply(b[i]); auto nearest=tree.nearest(p);
            if(nearest.second<=gate2) pairs.push_back({p,tree.p[nearest.first],nearest.second});
        }
        if(pairs.size()<3) break;
        std::sort(pairs.begin(),pairs.end(),[](const Pair& a,const Pair& other){return a.d<other.d;});
        // Trim only distant residuals, retaining exact correspondences for identical clouds.
        size_t keep=std::max(size_t(3),pairs.size()*85/100);
        std::vector<Vec3> from,to;from.reserve(keep);to.reserve(keep);double error=0;
        for(size_t i=0;i<keep;++i) {from.push_back(pairs[i].p);to.push_back(pairs[i].q);error+=pairs[i].d;}
        error/=static_cast<double>(keep);t=compose(fit(from,to),t);++iterations;
        if(std::abs(prev-error)<scale*scale*1e-12) break;
        prev=error;
    }
    return {t,cost(tree,b,t,scale*scale*0.04),iterations};
}
void writePCD(const std::filesystem::path& path,const std::vector<Vec3>& points,
    const std::vector<unsigned char>* mask,const std::atomic_bool* cancel) {
    std::ofstream out(path);require(bool(out),"Cannot create export file.");
    size_t count=mask?static_cast<size_t>(std::count(mask->begin(),mask->end(),1)):points.size();
    out<<"# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 8 8 8\nTYPE F F F\nCOUNT 1 1 1\nWIDTH "<<count
       <<"\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS "<<count<<"\nDATA ascii\n"<<std::setprecision(17);
    for(size_t i=0;i<points.size();++i) {
        if(i%1024==0) check(cancel);
        if(!mask||(*mask)[i]) out<<points[i].x<<' '<<points[i].y<<' '<<points[i].z<<'\n';
    }
    require(bool(out),"Failed to write export file.");
}
}
Cloud readPCD(const std::filesystem::path& file,const std::atomic_bool* cancel) {
    std::ifstream in(file,std::ios::binary);require(bool(in),"Cannot open PCD file.");
    std::vector<std::string> names,sizes,types,counts;size_t width=0,height=1,n=0;std::string mode,line;
    size_t headerBytes=0;
    while(std::getline(in,line)) {
        check(cancel);headerBytes+=line.size();require(headerBytes<65536,"PCD header too large.");
        if(line.size()>=3 && static_cast<unsigned char>(line[0])==0xef) line.erase(0,3);
        std::istringstream row(line);std::string key;row>>key;if(key.empty()||key[0]=='#') continue;
        std::vector<std::string> values;std::string token;while(row>>token) values.push_back(token);
        if(key=="FIELDS"||key=="FIELD") names=values;
        else if(key=="SIZE") sizes=values;else if(key=="TYPE") types=values;else if(key=="COUNT") counts=values;
        else if(key=="WIDTH" && values.size()==1) width=positive(values[0]);
        else if(key=="HEIGHT" && values.size()==1) height=positive(values[0]);
        else if(key=="POINTS" && values.size()==1) n=positive(values[0]);
        else if(key=="DATA" && values.size()==1) {mode=values[0];break;}
    }
    require(!mode.empty()&&!names.empty()&&names.size()<=128,"Missing or invalid PCD header.");
    require(sizes.size()==names.size()&&types.size()==names.size(),"PCD FIELDS/SIZE/TYPE mismatch.");
    if(counts.empty()) counts.resize(names.size(),"1");
    require(counts.size()==names.size(),"PCD COUNT mismatch.");
    require(height>0 && width<=MaxPoints/height,"PCD dimensions exceed 2 million points.");
    if(n==0) n=width*height;
    require(n>0&&n<=MaxPoints,"PCD must contain 1 to 2 million points.");
    if(width) require(width*height==n,"PCD WIDTH/HEIGHT/POINTS mismatch.");
    std::vector<Field> fields;size_t step=0,columns=0;int xyz[3]={-1,-1,-1};
    for(size_t i=0;i<names.size();++i) {
        Field f{names[i],types[i],positive(sizes[i]),positive(counts[i]),step,columns};
        require((f.size==1||f.size==2||f.size==4||f.size==8)&&f.count>0&&f.count<=1024,"Invalid PCD SIZE/COUNT.");
        require((f.type=="F"&&(f.size==4||f.size==8))||f.type=="I"||f.type=="U","Unsupported PCD type.");
        for(int j=0;j<3;++j) if(f.name==std::string(1,"xyz"[j])) {
            require(xyz[j]<0&&f.count==1,"PCD x/y/z must be unique scalar fields.");xyz[j]=static_cast<int>(i);
        }
        step+=f.size*f.count;columns+=f.count;fields.push_back(f);
    }
    require(step<=4096&&n<=MaxBytes/step,"PCD payload exceeds 512 MiB.");
    require(xyz[0]>=0&&xyz[1]>=0&&xyz[2]>=0,"PCD requires x, y and z fields.");
    Cloud out;out.points.reserve(n);
    auto append=[&](Vec3 p){
        if(finite(p)) {require(std::max({std::abs(p.x),std::abs(p.y),std::abs(p.z)})<=1e12,"Coordinates exceed supported magnitude (1e12).");out.points.push_back(p);}
        else ++out.invalidPoints;
    };
    if(mode=="ascii") {
        for(size_t i=0;i<n;++i) {
            if(i%1024==0) check(cancel);
            Vec3 p;double values[3]={};
            for(size_t j=0;j<columns;++j) {
                std::string value;require(bool(in>>value),"Truncated ASCII PCD payload.");
                for(int k=0;k<3;++k) if(j==fields[xyz[k]].column) {
                    char* end=nullptr;values[k]=std::strtod(value.c_str(),&end);
                    require(end!=value.c_str()&&*end=='\0',"Invalid ASCII coordinate.");
                }
            }
            p={values[0],values[1],values[2]};append(p);
        }
    } else {
        require(mode=="binary"||mode=="binary_compressed","Unsupported PCD DATA encoding.");
        size_t bytes=n*step;std::vector<unsigned char> data;
        if(mode=="binary") {
            data.resize(bytes);in.read(reinterpret_cast<char*>(data.data()),static_cast<std::streamsize>(bytes));
            require(static_cast<size_t>(in.gcount())==bytes,"Truncated binary PCD payload.");
        } else {
            uint32_t compressed=0,uncompressed=0;
            in.read(reinterpret_cast<char*>(&compressed),4);in.read(reinterpret_cast<char*>(&uncompressed),4);
            require(bool(in)&&compressed>0&&compressed<=MaxBytes&&uncompressed==bytes,"Invalid compressed PCD lengths.");
            std::vector<unsigned char> packed(compressed);in.read(reinterpret_cast<char*>(packed.data()),compressed);
            require(static_cast<size_t>(in.gcount())==compressed,"Truncated compressed PCD payload.");
            check(cancel);data=unlzf(packed,bytes,cancel);check(cancel);
        }
        for(size_t i=0;i<n;++i) {
            if(i%4096==0) check(cancel);
            double values[3]={};
            for(int k=0;k<3;++k) {
                const Field& f=fields[xyz[k]];
                size_t offset=mode=="binary"?i*step+f.offset:f.offset*n+i*f.size*f.count;
                values[k]=decode(data.data()+offset,f);
            }
            append({values[0],values[1],values[2]});
        }
    }
    require(out.points.size()>=3,"PCD has fewer than 3 finite points.");
    return out;
}
Result compare(const Cloud& a,const Cloud& b,double threshold,const std::atomic_bool* cancel,Progress progress) {
    require(a.points.size()>=3&&b.points.size()>=3,"Load two clouds with at least 3 valid points.");
    require(std::isfinite(threshold)&&threshold>=0,"Distance threshold must be finite and nonnegative.");
    for(const auto* points:{&a.points,&b.points}) for(auto p:*points) require(finite(p),"Point cloud contains nonfinite coordinates.");
    double scale=std::max(diagonal(a.points),diagonal(b.points));
    require(std::isfinite(scale)&&scale>1e-12,"Point clouds are degenerate or have zero extent.");
    auto notify=[&](const char* s){check(cancel);if(progress) progress(s);};
    notify("Downsampling and PCA initialization...");
    auto sa=sample(a.points,scale/180,cancel),sb=sample(b.points,scale/180,cancel);
    require(sa.size()>=3&&sb.size()>=3,"Insufficient distinct points after downsampling.");
    KDTree tree(sa);auto ba=basis(sa),bb=basis(sb);Vec3 ca=centroid(sa),cb=centroid(sb);
    std::vector<Candidate> starts;Transform center;center.t=ca-cb;
    starts.push_back({center,cost(tree,sb,center,scale*scale*0.04),0});
    std::array<int,3> permutation{{0,1,2}};
    do {
        for(int signs=0;signs<8;++signs) {
            Transform t;t.r.fill(0);
            for(int i=0;i<3;++i) for(int j=0;j<3;++j) for(int k=0;k<3;++k)
                t.r[3*i+j]+=ba[3*i+permutation[k]]*((signs&(1<<k))?-1.0:1.0)*bb[3*j+k];
            if(determinant(t.r)<0.5) continue;
            t.t=ca-t.apply(cb);check(cancel);
            starts.push_back({t,cost(tree,sb,t,scale*scale*0.04),0});
        }
    } while(std::next_permutation(permutation.begin(),permutation.end()));
    std::sort(starts.begin(),starts.end(),[](const Candidate& x,const Candidate& y){return x.error<y.error;});
    notify("Rigid ICP registration (multiple initial orientations)...");
    Candidate best;best.error=std::numeric_limits<double>::infinity();
    for(size_t i=0;i<std::min(size_t(5),starts.size());++i) {
        auto current=refine(tree,sb,starts[i].t,scale,60,cancel);
        if(current.error<best.error) best=current;
    }
    notify("Computing full-resolution bidirectional differences...");
    Result out;out.bToA=best.t;out.iterations=best.iterations;
    out.threshold=threshold>0?threshold:scale*0.01;
    out.alignedB.reserve(b.points.size());
    for(size_t i=0;i<b.points.size();++i) {if(i%4096==0) check(cancel);out.alignedB.push_back(best.t.apply(b.points[i]));}
    KDTree allA(a.points);check(cancel);KDTree allB(out.alignedB);check(cancel);
    out.differentA.resize(a.points.size());out.differentB.resize(b.points.size());
    double limit=out.threshold*out.threshold,error=0;
    for(size_t i=0;i<a.points.size();++i) {
        if(i%2048==0) check(cancel);
        double d=allB.nearest(a.points[i]).second;
        out.differentA[i]=static_cast<unsigned char>(d>limit);
        if(d<=limit) {++out.matchedA;error+=d;}
    }
    for(size_t i=0;i<out.alignedB.size();++i) {
        if(i%2048==0) check(cancel);
        double d=allA.nearest(out.alignedB[i]).second;
        out.differentB[i]=static_cast<unsigned char>(d>limit);
        if(d<=limit) {++out.matchedB;error+=d;}
    }
    out.coverageA=static_cast<double>(out.matchedA)/static_cast<double>(a.points.size());
    out.coverageB=static_cast<double>(out.matchedB)/static_cast<double>(b.points.size());
    out.similarity=100.0*(out.coverageA+out.coverageB)*0.5;
    out.hasMatchedPairs=(out.matchedA+out.matchedB)>0;
    if(out.hasMatchedPairs) out.rmse=std::sqrt(error/static_cast<double>(out.matchedA+out.matchedB));
    return out;
}
void exportResult(const std::filesystem::path& dir,const Cloud& a,const Result& r,const std::atomic_bool* cancel) {
    require(r.differentA.size()==a.points.size()&&r.differentB.size()==r.alignedB.size(),"No valid comparison result.");
    check(cancel);
    require(!std::filesystem::exists(dir),"Export directory already exists; choose a new location.");
    std::filesystem::create_directories(dir);
    writePCD(dir/"aligned_B.pcd",r.alignedB,nullptr,cancel);
    writePCD(dir/"different_A.pcd",a.points,&r.differentA,cancel);
    writePCD(dir/"different_B.pcd",r.alignedB,&r.differentB,cancel);
    std::ofstream report(dir/"report.txt");require(bool(report),"Cannot create result report.");
    report<<std::setprecision(17)<<"Similarity_percent="<<r.similarity<<"\nCoverage_A="<<r.coverageA
        <<"\nCoverage_B="<<r.coverageB<<"\nDistance_threshold="<<r.threshold<<"\nMatched_pairs="<<(r.matchedA+r.matchedB)
        <<"\nICP_iterations="<<r.iterations
        <<"\nDifferent_A="<<(a.points.size()-r.matchedA)<<"\nDifferent_B="<<(r.alignedB.size()-r.matchedB)<<"\nB_to_A_matrix:\n";
    for(int i=0;i<3;++i) report<<r.bToA.r[3*i]<<' '<<r.bToA.r[3*i+1]<<' '<<r.bToA.r[3*i+2]<<' '<<r.bToA.t[i]<<'\n';
    report<<"0 0 0 1\nRMSE=";
    if(r.hasMatchedPairs) report<<r.rmse; else report<<"N/A";
    report<<"\nSimilarity = 100 * (coverage_A + coverage_B) / 2; rigid geometry overlap, not semantic identity.\n";
    require(bool(report),"Failed to write result report.");
}
}
'@

Write-BundleFile 'VC2019/PointCloudWorkbench/PointCloud.h' @'
#pragma once
#include <array>
#include <atomic>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

namespace pc {
struct Vec3 {
    double x = 0, y = 0, z = 0;
    double operator[](int axis) const { return axis == 0 ? x : axis == 1 ? y : z; }
};
Vec3 operator+(Vec3 a, Vec3 b);
Vec3 operator-(Vec3 a, Vec3 b);
Vec3 operator*(Vec3 a, double s);
double norm2(Vec3 a);
struct Cloud {
    std::vector<Vec3> points;
    size_t invalidPoints = 0;
};
struct Transform {
    std::array<double, 9> r{{1,0,0, 0,1,0, 0,0,1}};
    Vec3 t;
    Vec3 apply(Vec3 p) const;
};
struct Result {
    Transform bToA;
    std::vector<Vec3> alignedB;
    std::vector<unsigned char> differentA, differentB;
    double similarity = 0, coverageA = 0, coverageB = 0;
    double rmse = 0, threshold = 0;
    size_t matchedA = 0, matchedB = 0;
    int iterations = 0;
    bool hasMatchedPairs = false;
};
using Progress = std::function<void(const std::string&)>;
Cloud readPCD(const std::filesystem::path& file, const std::atomic_bool* cancel = nullptr);
Result compare(const Cloud& a, const Cloud& b, double differenceThreshold = 0,
    const std::atomic_bool* cancel = nullptr, Progress progress = {});
void exportResult(const std::filesystem::path& directory, const Cloud& a,
    const Result& result, const std::atomic_bool* cancel = nullptr);
}
'@

Write-BundleFile 'VC2019/PointCloudWorkbench/PointCloudWorkbench.rc' @'
#pragma code_page(65001)
#include <windows.h>
#include "resource.h"
LANGUAGE LANG_CHINESE, SUBLANG_CHINESE_SIMPLIFIED
IDD_MAIN_DIALOG DIALOGEX 0, 0, 740, 540
STYLE DS_SETFONT | DS_CENTER | WS_POPUP | WS_CAPTION | WS_SYSMENU | WS_THICKFRAME | WS_MINIMIZEBOX | WS_MAXIMIZEBOX | WS_CLIPCHILDREN
EXSTYLE WS_EX_APPWINDOW
CAPTION "PCD 点云工作台 — 配准、差异分析与圆柱重建"
FONT 9, "Microsoft YaHei UI"
BEGIN
    PUSHBUTTON "读取点云 A", IDC_OPEN_A, 8, 8, 68, 20
    PUSHBUTTON "读取点云 B", IDC_OPEN_B, 82, 8, 68, 20
    LTEXT "差异距离阈值（0 = 自动）", -1, 160, 12, 120, 12
    EDITTEXT IDC_THRESHOLD, 284, 8, 64, 20, ES_AUTOHSCROLL | WS_TABSTOP
    PUSHBUTTON "ICP 配准 / 比较", IDC_COMPARE, 354, 8, 88, 20
    PUSHBUTTON "适应视图", IDC_FIT, 448, 8, 64, 20
    PUSHBUTTON "导出配准", IDC_EXPORT, 518, 8, 64, 20
    PUSHBUTTON "取消计算", IDC_CANCEL_WORK, 588, 8, 64, 20
    PUSHBUTTON "圆柱重建 A", IDC_CYL_A, 8, 34, 76, 20
    PUSHBUTTON "圆柱重建 B", IDC_CYL_B, 90, 34, 76, 20
    PUSHBUTTON "导出圆柱 A", IDC_CYL_EXPORT_A, 172, 34, 76, 20
    PUSHBUTTON "导出圆柱 B", IDC_CYL_EXPORT_B, 254, 34, 76, 20
    LTEXT "圆柱距离阈值（0=自动）", -1, 338, 40, 104, 12
    EDITTEXT IDC_CYL_THRESHOLD, 444, 34, 64, 20, ES_AUTOHSCROLL | WS_TABSTOP
    AUTOCHECKBOX "显示圆柱模型", IDC_CYL_SHOW, 520, 36, 88, 16
    AUTOCHECKBOX "圆柱内/外点着色", IDC_CYL_LABELS, 614, 36, 112, 16
    LTEXT "A：未加载", IDC_PATH_A, 8, 60, 724, 14, SS_PATHELLIPSIS
    LTEXT "B：未加载", IDC_PATH_B, 8, 76, 724, 14, SS_PATHELLIPSIS
    AUTOCHECKBOX "显示 B 原始位置", IDC_ORIGINAL, 8, 94, 112, 16
    AUTOCHECKBOX "仅显示配准差异点", IDC_DIFF_ONLY, 130, 94, 108, 16
    LTEXT "青=A  绿=B  红/紫=差异  黄=圆柱 | 左键旋转，右键平移，滚轮缩放", IDC_HINT, 246, 96, 486, 14
    CONTROL "", IDC_VIEW, "PointCloudView", WS_CHILD | WS_VISIBLE | WS_BORDER, 8, 116, 724, 300
    EDITTEXT IDC_RESULTS, 8, 422, 724, 84, ES_MULTILINE | ES_READONLY | ES_AUTOVSCROLL | WS_VSCROLL
    LTEXT "读取 PCD 后可配准比较，或对单份点云进行圆柱重建。", IDC_STATUS, 8, 514, 724, 16, SS_ENDELLIPSIS
END
'@

Write-BundleFile 'VC2019/PointCloudWorkbench/PointCloudWorkbench.sln' @'
Microsoft Visual Studio Solution File, Format Version 12.00
# Visual Studio Version 16
VisualStudioVersion = 16.0.28701.123
MinimumVisualStudioVersion = 10.0.40219.1
Project("{BC8A1FFA-BEE3-4634-8014-F334798102B3}") = "PointCloudWorkbench", "PointCloudWorkbench.vcxproj", "{6131BAF3-1A90-4658-B4B0-ABFC90619A06}"
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

Write-BundleFile 'VC2019/PointCloudWorkbench/PointCloudWorkbench.vcxproj' @'
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
    <RootNamespace>PointCloudWorkbench</RootNamespace>
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
      <ExceptionHandling>Sync</ExceptionHandling>
      <ConformanceMode>true</ConformanceMode>
      <LanguageStandard>stdcpp17</LanguageStandard>
      <AdditionalOptions>/utf-8 %(AdditionalOptions)</AdditionalOptions>
      <PreprocessorDefinitions>UNICODE;_UNICODE;%(PreprocessorDefinitions)</PreprocessorDefinitions>
    </ClCompile>
    <Link>
      <SubSystem>Windows</SubSystem>
      <AdditionalDependencies>User32.lib;Gdi32.lib;OpenGL32.lib;Comdlg32.lib;Shell32.lib;Ole32.lib;%(AdditionalDependencies)</AdditionalDependencies>
    </Link>
  </ItemDefinitionGroup>
  <ItemGroup><ClCompile Include="main.cpp" /><ClCompile Include="PointCloud.cpp" /><ClCompile Include="Cylinder.cpp" /></ItemGroup>
  <ItemGroup><ClInclude Include="resource.h" /><ClInclude Include="PointCloud.h" /><ClInclude Include="Cylinder.h" /></ItemGroup>
  <ItemGroup><ResourceCompile Include="PointCloudWorkbench.rc" /></ItemGroup>
  <Import Project="$(VCTargetsPath)\Microsoft.Cpp.targets" />
</Project>
'@

Write-BundleFile 'VC2019/PointCloudWorkbench/README.md' @'
# VC2019 点云工作台：配准、差异分析与圆柱重建

## 编译运行

1. Visual Studio Installer 安装“使用 C++ 的桌面开发”、MSVC v142 和 Windows 10 SDK，无需 MFC、PCL、Eigen 或 VTK。
2. 使用 Visual Studio 2019 打开 `PointCloudWorkbench.sln`。
3. 推荐选择 **x64 / Release**，按 F5 编译运行。也提供 Win32、Debug 配置；大点云建议使用 x64。
4. 若 Windows SDK 版本不匹配，右键解决方案选择“重定解决方案目标”。

使用 Windows 自带的 OpenGL 1.1 接口显示点云，需有可工作的 OpenGL 驱动。程序支持中文文件路径。

## 操作

1. 点击“读取点云 A”选择参考 PCD，再点击“读取点云 B”选择待配准 PCD。
2. 两个点云立即在同一视图中显示。左键拖动旋转，右键拖动平移，滚轮缩放，“适应视图”重置视野。
3. 输入“差异距离阈值”，单位与 PCD 坐标相同。例如坐标单位是米，`0.01` 表示 1 厘米。`0` 自动取两个点云各自包围盒对角线较大值的 1%。
4. 点击“ICP 配准 / 比较”，程序估计 **B → A 的旋转和平移**，然后计算相似度、双向重合率、差异数量和阈值内 RMSE。
5. 青色表示 A，绿色表示 B；红色标记 A 中的差异点，紫色标记 B 中的差异点。“仅显示差异点”过滤非差异点。“显示 B 原始位置”切换配准前/后的坐标；差异标签仍来自配准后的比较。
6. “导出配准”选择父目录后生成独立的 `PCD_Result_*` 子目录，包含 `aligned_B.pcd`、`different_A.pcd`、`different_B.pcd` 和 `report.txt`。报告中包含 4×4 B → A 变换矩阵；无差异时对应 PCD 的 POINTS 为 0。
7. 读取、比较和导出在后台执行。“取消计算”可停止任务；关闭窗口时会先结束后台任务。导出被取消时可能留下部分文件，原输入文件始终不修改。

窗口可拖动边框和四角调整大小，可最大化和还原；设有保持工具栏可用的最小窗口尺寸，无自定义最大尺寸限制。

## 相似度与差异的定义

配准完成后，对 **全部有效原始点**做双向最近邻查询：

- `coverage_A`：A 中到配准后 B 的最近邻距离不超过阈值的点数 / A 有效点数。
- `coverage_B`：配准后 B 中到 A 的最近邻距离不超过阈值的点数 / B 有效点数。
- `similarity = 100 × (coverage_A + coverage_B) / 2`，单位为百分比。
- 最近邻距离 **大于阈值**的点标记为该侧的差异点。
- RMSE 仅统计阈值内的双向近邻距离；无匹配点时显示 N/A。

这是按用户阈值定义的**几何重合率**，不是语义识别概率。密度、噪声、遮挡、采样间距、阈值都会影响分数；重合率使用近邻查询，不执行一对一对应，不可作为结构完全相同的证明。

## 算法与范围

- 体素降采样用于配准，每个点云最多选用 10,000 个样本，减少计算量。
- 使用质心平移与 PCA 主轴的 24 个合法旋转排列作初始姿态，选取前 5 个候选进行裁剪式点到点 ICP，每个候选最多 60 次迭代。
- 刚体最小二乘拟合使用 Horn 四元数法，最近邻使用 KD 树，最终差异计算使用全部有效点。
- 不估计缩放，不支持非刚体形变。两份 PCD 应使用相同坐标单位，且应有足够重叠区域。
- ICP 是局部优化；大面积缺失、重复对称结构、极端离群点和重叠不足可能导致错误姿态。低相似度时界面提示核对参数。程序不保证全局最优，也不会将所有失败情形自动识别为失败。
- 少于 3 个有效点、零范围、共线或近共线点云会报错；普通平面点云可处理。
- 单文件最多 2,000,000 个点，解码载荷最多 512 MiB，坐标绝对值不超过 1e12。实际可处理大小受内存影响。
- 视图中每份点云的正常/差异类别各最多绘制 100,000 点，避免大点云拖慢交互；此显示抽样不会减少最终相似度与差异计算使用的点。

## PCD 支持

支持 PCD v0.7 常见的 `DATA ascii`、`binary`、`binary_compressed`（LZF）。按 FIELDS/SIZE/TYPE/COUNT 定位标量 x/y/z，可忽略 intensity、rgb、normal 等额外字段；不会用输入 RGB 或法向量进行配准。COUNT 缺失时按 1 处理。

支持浮点 F4/F8，以及 I/U 的 1/2/4/8 字节坐标，二进制按常见小端 PCD 处理。NaN/Inf 坐标点跳过并显示数量。非法头、缺少 x/y/z、截断载荷和损坏 LZF 会报错。

## 示例

完整 ZIP 的 `samples` 目录有三份点云。先用 `sample_A.pcd` 和 `sample_B_rigid.pcd`，阈值设为 `0.02`，应得到约 100% 相似度；再将 B 换为 `sample_B_changed.pcd` 观察差异点。文件路径由用户通过读取按钮选择。

## 圆柱重建

单份点云加载后即可点击“圆柱重建 A”或“圆柱重建 B”，不要求同时加载两份点云，也不要求先配准。

1. 读取一根圆柱/管道外表面的 PCD。建议先分割出单根管道，避免混入大量其他物体。
2. 设置独立的“圆柱距离阈值”，表示到无限圆柱侧面的径向距离容差，单位与 PCD 坐标一致。`0` 自动使用 `min(样本 RMS 尺度 × 1%, 拟合半径 × 5%)`。用户阈值达到半径的 30% 时拒绝，避免过宽容差造成误判。
3. 点击重建。结果显示半径、直径、轴向重建长度、原始坐标系中的单位轴方向、径向 RMSE 与内点率。
4. “显示圆柱模型”叠加黄色线框和轴线。“圆柱内/外点着色”切换为绿色内点、红色外点；它与“仅显示配准差异点”互斥，避免混淆两种标记。
5. 配准后，B 的圆柱显示会自动应用 B → A 刚体变换；勾选“显示 B 原始位置”后，B 点云和其圆柱同时回到原始位置。
6. “导出圆柱 A/B”生成新的结果子目录，包含：
   - `cylinder.obj`：1600 顶点、3072 三角面的开口圆柱侧面网格。
   - `cylinder_surface.pcd`：模型侧面采样点。
   - `cylinder_inliers.pcd` / `cylinder_outliers.pcd`：原输入点的拟合分类。
   - `cylinder_report.txt`：半径、直径、长度、中心、轴向、阈值、内点率、RMSE 和观测角覆盖。

圆柱导出始终使用对应输入点云的原始坐标系；B 的配准变换可从“导出配准”的报告取得。取消导出可能留下部分结果文件。

### 圆柱算法和限制

- 采用最多 6000 点的确定种子蓄水池抽样，避免有序扫描环被固定步长采样成一条线；最终对全部有效原始点标注内外点。
- PCA 三条主轴与 96 个半球方向提供轴向候选，每个方向结合代数圆拟合和随机三点圆假设；前 10 个候选进入带 80% 残差裁剪的非线性细化，优化轴、横向中心和半径。
- 至少需要 30 个有效点、55% 径向支持、超过 90 度的观测角覆盖、足够的轴向跨度；轴向裁剪后至少保留 50% 内点。平面、直线和窄圆弧等退化输入会尝试拒绝，但这不是可靠的物体类别识别器。
- 轴向长度取径向内点的 1% 与 99% 分位数跨度，故略短于极端端点范围；不是对端盖位置或真实加工长度的测量。
- 重建的是**单个理想直圆柱的完整 360 度开口侧面**。部分扫描没有观测到的圆周区域是解析模型外推，不是测量数据；不生成端盖，不重建管壁厚度、孔洞、弯管、多个圆柱或任意自由曲面。
- 非线性拟合是启发式局部优化，不保证全局最优。对大面积缺失、严重遮挡或离群比例过高的数据，请先分割/清理后重试；不能用拟合成功代替尺寸计量验证。

随包 `samples/cylinder_A_noisy.pcd` 和 `cylinder_partial.pcd` 可直接测试，真实半径均为 0.7，推荐圆柱距离阈值 `0.012`。`cylinder_B_rigid.pcd` 用于同时测试配准和双圆柱显示。

## 验证

- Linux 原生 C++17 测试覆盖 PCD 三种编码、额外字段、LZF 回溯、NaN 过滤、坏文件、取消操作、同云、旋转平移、8 组三维旋转、噪声、部分重叠、差异点和导出回读。
- 新增完整圆柱+离群点、短粗圆柱、部分圆弧、大坐标、384000 点有序扫描、平面/球面/直线/窄圆弧拒绝、取消以及 OBJ/PCD 导出测试。
- 使用 AddressSanitizer / UndefinedBehaviorSanitizer 检查配准与圆柱核心逻辑。
- 使用 LLVM-MinGW 分别交叉编译 x64 与 x86，检查 Windows C++ 界面、资源及链接。
- 已在 PowerShell 7 中验证生成脚本及 ZIP 打包；脚本语法面向 Windows PowerShell 5.1。当前环境没有 Windows 桌面，尚未在 Visual Studio 2019 中实际构建或交互运行；交叉编译不等同于 MSVC v142 验证。

Windows 的两组核心测试：在 VS2019 Developer PowerShell 中执行仓库 `tests/RunCoreTests.ps1`。Linux 可执行：

```sh
g++ -std=c++17 -O2 -I VC2019/PointCloudWorkbench VC2019/PointCloudWorkbench/PointCloud.cpp tests/CoreTests.cpp -o core-tests
./core-tests

g++ -std=c++17 -O2 -I VC2019/PointCloudWorkbench VC2019/PointCloudWorkbench/PointCloud.cpp VC2019/PointCloudWorkbench/Cylinder.cpp tests/CylinderTests.cpp -o cylinder-tests
./cylinder-tests
```
'@

Write-BundleFile 'VC2019/PointCloudWorkbench/main.cpp' @'
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <windowsx.h>
#include <commdlg.h>
#include <shlobj.h>
#include <GL/gl.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cwchar>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <thread>
#include "resource.h"
#include "PointCloud.h"
#include "Cylinder.h"

constexpr UINT WM_WORK_DONE = WM_APP + 1;
constexpr UINT WM_WORK_STATUS = WM_APP + 2;
struct View {
    HDC dc=nullptr; HGLRC context=nullptr;
    double yaw=-25,pitch=20,zoom=1,panX=0,panY=0,radius=1;
    pc::Vec3 center;
    POINT mouse{}; bool left=false,right=false;
};
struct App {
    HWND dialog=nullptr; View view;
    pc::Cloud a,b; pc::Result result;
    pc::Cylinder cylinderA,cylinderB;bool hasCylinderA=false,hasCylinderB=false;
    std::filesystem::path pathA,pathB;
    bool busy=false,hasResult=false,closePending=false;
    std::atomic_bool cancel{false}; std::thread worker;
};
struct WorkResult {
    int action=0; std::filesystem::path path;
    pc::Cloud cloud; pc::Result result;pc::Cylinder cylinder;
    std::wstring error;
};
static App* GetApp(HWND dialog) {
    return reinterpret_cast<App*>(GetWindowLongPtrW(dialog,DWLP_USER));
}
static std::wstring Wide(const std::string& value) {
    if(value.empty()) return {};
    int n=MultiByteToWideChar(CP_UTF8,0,value.data(),static_cast<int>(value.size()),nullptr,0);
    std::wstring out(static_cast<size_t>(n),L' ');
    MultiByteToWideChar(CP_UTF8,0,value.data(),static_cast<int>(value.size()),&out[0],n);return out;
}
static void Status(App& app,const std::wstring& text) {
    SetDlgItemTextW(app.dialog,IDC_STATUS,text.c_str());
}
static void Refresh(App& app) { InvalidateRect(GetDlgItem(app.dialog,IDC_VIEW),nullptr,FALSE); }
static void Busy(App& app,bool value) {
    app.busy=value;
    for(int id:{IDC_OPEN_A,IDC_OPEN_B,IDC_THRESHOLD,IDC_CYL_THRESHOLD,IDC_FIT}) EnableWindow(GetDlgItem(app.dialog,id),!value);
    EnableWindow(GetDlgItem(app.dialog,IDC_COMPARE),!value&&!app.a.points.empty()&&!app.b.points.empty());
    EnableWindow(GetDlgItem(app.dialog,IDC_EXPORT),!value&&app.hasResult);
    EnableWindow(GetDlgItem(app.dialog,IDC_CANCEL_WORK),value);
    EnableWindow(GetDlgItem(app.dialog,IDC_CYL_A),!value&&!app.a.points.empty());
    EnableWindow(GetDlgItem(app.dialog,IDC_CYL_B),!value&&!app.b.points.empty());
    EnableWindow(GetDlgItem(app.dialog,IDC_CYL_EXPORT_A),!value&&app.hasCylinderA);
    EnableWindow(GetDlgItem(app.dialog,IDC_CYL_EXPORT_B),!value&&app.hasCylinderB);
}
static void FitView(App& app) {
    bool first=true;pc::Vec3 lo{},hi{};
    const auto& b=(app.hasResult&&IsDlgButtonChecked(app.dialog,IDC_ORIGINAL)!=BST_CHECKED)?app.result.alignedB:app.b.points;
    const std::vector<pc::Vec3>* clouds[]={&app.a.points,&b};
    for(const auto* cloud:clouds) for(auto p:*cloud) {
        if(first) {lo=hi=p;first=false;}
        lo={std::min(lo.x,p.x),std::min(lo.y,p.y),std::min(lo.z,p.z)};
        hi={std::max(hi.x,p.x),std::max(hi.y,p.y),std::max(hi.z,p.z)};
    }
    if(IsDlgButtonChecked(app.dialog,IDC_CYL_SHOW)==BST_CHECKED) for(bool isB:{false,true}) {
        if(!(isB?app.hasCylinderB:app.hasCylinderA))continue;
        const auto& model=isB?app.cylinderB:app.cylinderA;
        for(auto point:model.vertices) {
            if(isB&&app.hasResult&&IsDlgButtonChecked(app.dialog,IDC_ORIGINAL)!=BST_CHECKED)point=app.result.bToA.apply(point);
            lo={std::min(lo.x,point.x),std::min(lo.y,point.y),std::min(lo.z,point.z)};
            hi={std::max(hi.x,point.x),std::max(hi.y,point.y),std::max(hi.z,point.z)};
        }
    }
    app.view.center=(lo+hi)*0.5;app.view.radius=std::max(1e-9,std::sqrt(pc::norm2(hi-lo))*0.6);
    app.view.zoom=1;app.view.panX=app.view.panY=0;Refresh(app);
}
static void DrawCloud(const std::vector<pc::Vec3>& points,const std::vector<unsigned char>* mask,
    bool cloudA,bool onlyDifferences,pc::Vec3 center) {
    // Display is capped per category; all loaded points participate in the final metrics.
    const size_t budget=100000;
    for(int category=0;category<(mask?2:1);++category) {
        bool difference=mask&&category==1;
        if(onlyDifferences&&!difference) continue;
        size_t count=0;
        for(size_t i=0;i<points.size();++i) if(!mask||((*mask)[i]!=0)==difference) ++count;
        size_t stride=std::max(size_t(1),(count+budget-1)/budget),seen=0;
        if(difference) glColor3f(1.0f,cloudA?0.22f:0.20f,cloudA?0.18f:0.88f);
        else glColor3f(cloudA?0.10f:0.30f,cloudA?0.78f:0.95f,cloudA?1.0f:0.42f);
        glPointSize(difference?4.0f:2.0f);glBegin(GL_POINTS);
        for(size_t i=0;i<points.size();++i) {
            if(mask&&(((*mask)[i]!=0)!=difference)) continue;
            if(seen++%stride) continue;
            pc::Vec3 p=points[i]-center;glVertex3d(p.x,p.y,p.z);
        }
        glEnd();
    }
}
static pc::Vec3 DisplayPoint(App& app,pc::Vec3 p,bool b) {
    if(b&&app.hasResult&&IsDlgButtonChecked(app.dialog,IDC_ORIGINAL)!=BST_CHECKED) p=app.result.bToA.apply(p);
    return p-app.view.center;
}
static void DrawCylinder(App& app,const pc::Cylinder& model,bool b) {
    glColor3f(1.0f,0.82f,b?0.45f:0.05f);glLineWidth(1.0f);glBegin(GL_LINES);
    // Ring edges and sparse longitudinal edges give a readable open side surface.
    for(size_t ring=0;ring<25;++ring) for(size_t angle=0;angle<64;++angle) {
        size_t i=ring*64+angle,j=ring*64+(angle+1)%64;
        if(ring%4==0||ring==24) {
            auto p=DisplayPoint(app,model.vertices[i],b),q=DisplayPoint(app,model.vertices[j],b);
            glVertex3d(p.x,p.y,p.z);glVertex3d(q.x,q.y,q.z);
        }
        if(ring<24&&angle%8==0) {
            auto p=DisplayPoint(app,model.vertices[i],b),q=DisplayPoint(app,model.vertices[i+64],b);
            glVertex3d(p.x,p.y,p.z);glVertex3d(q.x,q.y,q.z);
        }
    }
    auto p=DisplayPoint(app,model.center-model.axis*(model.height*0.6),b);
    auto q=DisplayPoint(app,model.center+model.axis*(model.height*0.6),b);
    glVertex3d(p.x,p.y,p.z);glVertex3d(q.x,q.y,q.z);glEnd();
}
static void DrawCylinderLabels(App& app,const pc::Cloud& cloud,const pc::Cylinder& model,bool b) {
    size_t stride=std::max(size_t(1),(cloud.points.size()+149999)/150000);
    glPointSize(3);glBegin(GL_POINTS);
    for(size_t i=0;i<cloud.points.size();i+=stride) {
        if(model.inliers[i])glColor3f(0.2f,0.9f,0.5f);else glColor3f(1.0f,0.25f,0.18f);
        auto p=DisplayPoint(app,cloud.points[i],b);glVertex3d(p.x,p.y,p.z);
    }
    glEnd();
}
static void Paint(HWND window) {
    PAINTSTRUCT ps{};BeginPaint(window,&ps);
    auto* app=GetApp(GetParent(window));
    if(!app||!app->view.context) {
        RECT rect{};GetClientRect(window,&rect);FillRect(ps.hdc,&rect,static_cast<HBRUSH>(GetStockObject(BLACK_BRUSH)));
        EndPaint(window,&ps);return;
    }
    auto& v=app->view;wglMakeCurrent(v.dc,v.context);
    RECT rect{};GetClientRect(window,&rect);int width=std::max(1L,rect.right),height=std::max(1L,rect.bottom);
    glViewport(0,0,width,height);glClearColor(0.035f,0.055f,0.09f,1);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);glEnable(GL_DEPTH_TEST);
    double aspect=static_cast<double>(width)/height,r=v.radius/v.zoom;
    glMatrixMode(GL_PROJECTION);glLoadIdentity();
    glOrtho(-r*aspect,r*aspect,-r,r,-v.radius*30,v.radius*30);
    glMatrixMode(GL_MODELVIEW);glLoadIdentity();
    glTranslated(v.panX,v.panY,0);glRotated(v.pitch,1,0,0);glRotated(v.yaw,0,1,0);
    bool original=IsDlgButtonChecked(app->dialog,IDC_ORIGINAL)==BST_CHECKED;
    bool only=IsDlgButtonChecked(app->dialog,IDC_DIFF_ONLY)==BST_CHECKED&&app->hasResult;
    bool cylinderLabels=IsDlgButtonChecked(app->dialog,IDC_CYL_LABELS)==BST_CHECKED;
    if(cylinderLabels&&app->hasCylinderA)DrawCylinderLabels(*app,app->a,app->cylinderA,false);
    else DrawCloud(app->a.points,app->hasResult?&app->result.differentA:nullptr,true,only,v.center);
    const auto& b=app->hasResult&&!original?app->result.alignedB:app->b.points;
    if(cylinderLabels&&app->hasCylinderB)DrawCylinderLabels(*app,app->b,app->cylinderB,true);
    else DrawCloud(b,app->hasResult?&app->result.differentB:nullptr,false,only,v.center);
    if(IsDlgButtonChecked(app->dialog,IDC_CYL_SHOW)==BST_CHECKED) {
        if(app->hasCylinderA)DrawCylinder(*app,app->cylinderA,false);
        if(app->hasCylinderB)DrawCylinder(*app,app->cylinderB,true);
    }
    SwapBuffers(v.dc);EndPaint(window,&ps);
}
static LRESULT CALLBACK ViewProc(HWND window,UINT message,WPARAM wParam,LPARAM lParam) {
    auto* app=GetApp(GetParent(window));
    switch(message) {
    case WM_PAINT: Paint(window);return 0;
    case WM_ERASEBKGND:return 1;
    case WM_LBUTTONDOWN:case WM_RBUTTONDOWN:
        if(app) {
            app->view.mouse={GET_X_LPARAM(lParam),GET_Y_LPARAM(lParam)};
            app->view.left=message==WM_LBUTTONDOWN;app->view.right=message==WM_RBUTTONDOWN;
            SetCapture(window);SetFocus(window);
        }return 0;
    case WM_LBUTTONUP:case WM_RBUTTONUP:case WM_CAPTURECHANGED:
        if(app) {app->view.left=app->view.right=false;if(message!=WM_CAPTURECHANGED) ReleaseCapture();}return 0;
    case WM_MOUSEMOVE:
        if(app&&(app->view.left||app->view.right)) {
            auto& v=app->view;int x=GET_X_LPARAM(lParam),y=GET_Y_LPARAM(lParam);
            if(v.left) {v.yaw+=(x-v.mouse.x)*0.5;v.pitch+=(y-v.mouse.y)*0.5;}
            if(v.right) {
                RECT rect{};GetClientRect(window,&rect);double pixel=2*v.radius/v.zoom/std::max(1L,rect.bottom);
                v.panX+=(x-v.mouse.x)*pixel;v.panY-=(y-v.mouse.y)*pixel;
            }
            v.mouse={x,y};InvalidateRect(window,nullptr,FALSE);
        }return 0;
    case WM_MOUSEWHEEL:
        if(app) {app->view.zoom=std::clamp(app->view.zoom*std::pow(1.15,GET_WHEEL_DELTA_WPARAM(wParam)/120.0),0.01,1000.0);Refresh(*app);}return 0;
    case WM_SIZE:InvalidateRect(window,nullptr,FALSE);return 0;
    case WM_DESTROY:
        if(app) {
            if(app->view.context) {wglMakeCurrent(nullptr,nullptr);wglDeleteContext(app->view.context);app->view.context=nullptr;}
            if(app->view.dc) {ReleaseDC(window,app->view.dc);app->view.dc=nullptr;}
        }return 0;
    }
    return DefWindowProcW(window,message,wParam,lParam);
}
static void Layout(HWND dialog) {
    RECT r{};GetClientRect(dialog,&r);int width=r.right,height=r.bottom;
    // Top controls retain font-scaled resource positions; the view and report stretch.
    RECT top{0,0,8,116};MapDialogRect(dialog,&top);
    RECT report{0,0,0,84};MapDialogRect(dialog,&report);
    RECT status{0,0,0,18};MapDialogRect(dialog,&status);
    int margin=top.right,viewTop=top.bottom,content=std::max(0,width-2*margin);
    int statusTop=height-status.bottom-margin;
    int reportTop=statusTop-report.bottom-margin;
    MoveWindow(GetDlgItem(dialog,IDC_VIEW),margin,viewTop,content,std::max(0,reportTop-viewTop-margin),TRUE);
    MoveWindow(GetDlgItem(dialog,IDC_RESULTS),margin,reportTop,content,report.bottom,TRUE);
    MoveWindow(GetDlgItem(dialog,IDC_STATUS),margin,statusTop,content,status.bottom,TRUE);
    RECT line{0,0,0,14};MapDialogRect(dialog,&line);
    RECT pathA{0,0,8,60},pathB{0,0,8,76};MapDialogRect(dialog,&pathA);MapDialogRect(dialog,&pathB);
    MoveWindow(GetDlgItem(dialog,IDC_PATH_A),margin,pathA.bottom,content,line.bottom,TRUE);
    MoveWindow(GetDlgItem(dialog,IDC_PATH_B),margin,pathB.bottom,content,line.bottom,TRUE);
}
static bool InitGL(App& app) {
    HWND view=GetDlgItem(app.dialog,IDC_VIEW);app.view.dc=GetDC(view);
    PIXELFORMATDESCRIPTOR pfd{};pfd.nSize=sizeof(pfd);pfd.nVersion=1;
    pfd.dwFlags=PFD_DRAW_TO_WINDOW|PFD_SUPPORT_OPENGL|PFD_DOUBLEBUFFER;
    pfd.iPixelType=PFD_TYPE_RGBA;pfd.cColorBits=24;pfd.cDepthBits=24;pfd.iLayerType=PFD_MAIN_PLANE;
    int format=ChoosePixelFormat(app.view.dc,&pfd);
    if(!format||!SetPixelFormat(app.view.dc,format,&pfd)) return false;
    app.view.context=wglCreateContext(app.view.dc);
    return app.view.context&&wglMakeCurrent(app.view.dc,app.view.context);
}
static void Launch(App& app,int action,const std::filesystem::path& path={},double threshold=0) {
    app.cancel=false;Busy(app,true);Status(app,L"正在处理，可点击“取消计算”停止……");
    app.worker=std::thread([&app,action,path,threshold] {
        auto work=std::make_unique<WorkResult>();work->action=action;work->path=path;
        try {
            if(action==1||action==2) work->cloud=pc::readPCD(path,&app.cancel);
            else if(action==3) {
                work->result=pc::compare(app.a,app.b,threshold,&app.cancel,[&](const std::string& s){
                    auto text=std::make_unique<std::wstring>(Wide(s));
                    if(PostMessageW(app.dialog,WM_WORK_STATUS,0,reinterpret_cast<LPARAM>(text.get()))) text.release();
                });
            } else if(action==4) pc::exportResult(path,app.a,app.result,&app.cancel);
            else if(action==5||action==6) {
                work->cylinder=pc::reconstructCylinder(action==5?app.a:app.b,threshold,&app.cancel,[&](const std::string& text){
                    auto status=std::make_unique<std::wstring>(Wide(text));
                    if(PostMessageW(app.dialog,WM_WORK_STATUS,0,reinterpret_cast<LPARAM>(status.get())))status.release();
                });
            } else if(action==7||action==8) pc::exportCylinder(path,action==7?app.a:app.b,
                action==7?app.cylinderA:app.cylinderB,&app.cancel);
        } catch(const std::exception& e) {work->error=Wide(e.what());}
        if(PostMessageW(app.dialog,WM_WORK_DONE,0,reinterpret_cast<LPARAM>(work.get()))) work.release();
    });
}
static void OpenCloud(App& app,int action) {
    wchar_t file[32768]{};OPENFILENAMEW ofn{};ofn.lStructSize=sizeof(ofn);ofn.hwndOwner=app.dialog;
    ofn.lpstrFilter=L"PCD 点云 (*.pcd)\0*.pcd\0所有文件\0*.*\0";ofn.lpstrFile=file;ofn.nMaxFile=32768;
    ofn.Flags=OFN_FILEMUSTEXIST|OFN_PATHMUSTEXIST|OFN_NOCHANGEDIR;
    ofn.lpstrTitle=action==1?L"选择参考点云 A":L"选择待配准点云 B";
    if(GetOpenFileNameW(&ofn)) Launch(app,action,file);
}
static void ResultsText(App& app) {
    const auto& r=app.result;std::wostringstream out;out<<std::fixed;
    if(app.hasResult) {
    out<<std::fixed<<std::setprecision(3)<<L"几何相似度："<<r.similarity<<L"%    阈值："<<std::setprecision(8)<<r.threshold
       <<L"（PCD 坐标单位）    ICP 迭代："<<r.iterations<<L"\r\n"
       <<L"A 重合率："<<std::setprecision(2)<<100*r.coverageA<<L"%    B 重合率："<<100*r.coverageB
       <<L"%    差异点 A："<<(app.a.points.size()-r.matchedA)<<L"    差异点 B："<<(app.b.points.size()-r.matchedB)<<L"\r\n";
    if(r.hasMatchedPairs) out<<L"阈值内双向 RMSE："<<std::setprecision(8)<<r.rmse;
    else out<<L"阈值内双向 RMSE：N/A（无匹配点）";
    out<<L"\r\n相似度 = (A 重合率 + B 重合率) / 2；仅比较刚体几何重合，不表示语义相同。";
    if(r.similarity<30) out<<L"\r\n重合率低：请核对坐标单位、距离阈值和重叠区域；ICP 可能未找到正确姿态。";
    }
    for(bool b:{false,true}) {
        if(!(b?app.hasCylinderB:app.hasCylinderA))continue;
        const auto& c=b?app.cylinderB:app.cylinderA;
        out<<L"\r\n圆柱 "<<(b?L"B":L"A")<<L"：半径="<<std::setprecision(6)<<c.radius<<L"，直径="<<2*c.radius
           <<L"，重建长度="<<c.height<<L"，径向 RMSE="<<c.rmse<<L"，内点率="<<std::setprecision(2)<<100*c.inlierRatio<<L"%"
           <<L"\r\n轴方向=("<<std::setprecision(5)<<c.axis.x<<L", "<<c.axis.y<<L", "<<c.axis.z
           <<L")，阈值="<<c.threshold<<L"，原始坐标系。黄色模型为完整圆周外推的开口侧面。";
    }
    if(!app.hasResult&&!app.hasCylinderA&&!app.hasCylinderB)out<<L"读取完成。可配准比较，或对 A/B 进行单圆柱重建。";
    SetDlgItemTextW(app.dialog,IDC_RESULTS,out.str().c_str());
}
static void Finish(App& app,std::unique_ptr<WorkResult> work) {
    if(app.worker.joinable()) app.worker.join();
    if(app.closePending) {Busy(app,false);EndDialog(app.dialog,0);return;}
    if(!work->error.empty()) {
        Status(app,app.cancel?L"操作已取消；原数据保留，若正在导出则可能留下部分文件。":L"处理失败；保留此前的数据。");
        if(!app.cancel) MessageBoxW(app.dialog,work->error.c_str(),L"点云处理失败",MB_OK|MB_ICONERROR);
    } else if(work->action==1||work->action==2) {
        bool a=work->action==1;
        (a?app.a:app.b)=std::move(work->cloud);(a?app.pathA:app.pathB)=work->path;
        app.hasResult=false;app.result={};
        if(a){app.hasCylinderA=false;app.cylinderA={};}else{app.hasCylinderB=false;app.cylinderB={};}
        const auto& cloud=a?app.a:app.b;
        std::wstring label=(a?L"A：":L"B：")+work->path.wstring()+L"  | 有效点："+std::to_wstring(cloud.points.size())+
            L"，跳过非有限点："+std::to_wstring(cloud.invalidPoints);
        SetDlgItemTextW(app.dialog,a?IDC_PATH_A:IDC_PATH_B,label.c_str());
        ResultsText(app);
        FitView(app);Status(app,L"点云读取完成。");
    } else if(work->action==3) {
        app.result=std::move(work->result);app.hasResult=true;
        CheckDlgButton(app.dialog,IDC_ORIGINAL,BST_UNCHECKED);ResultsText(app);FitView(app);
        Status(app,L"比较完成：青=A，绿=配准后的 B，红=A 差异，紫=B 差异。");
    } else if(work->action==5||work->action==6) {
        bool a=work->action==5;
        (a?app.cylinderA:app.cylinderB)=std::move(work->cylinder);
        (a?app.hasCylinderA:app.hasCylinderB)=true;
        CheckDlgButton(app.dialog,IDC_CYL_SHOW,BST_CHECKED);ResultsText(app);FitView(app);
        Status(app,L"圆柱重建完成。黄=模型/轴线；圆柱着色模式下绿=内点、红=外点。");
    } else if(work->action==4||work->action==7||work->action==8) Status(app,L"已导出："+work->path.wstring());
    Busy(app,false);Refresh(app);
}
static bool ReadThreshold(App& app,int control,double& threshold) {
    wchar_t input[128]{};GetDlgItemTextW(app.dialog,control,input,128);
    wchar_t* end=nullptr;threshold=std::wcstod(input,&end);
    while(end&&(*end==L' '||*end==L'\t'))++end;
    if(end==input||!end||*end||!std::isfinite(threshold)||threshold<0||threshold>1e12) {
        MessageBoxW(app.dialog,L"请输入非负的有限距离值；0 为自动。单位与 PCD 坐标相同。",L"距离阈值无效",MB_OK|MB_ICONWARNING);return false;
    }
    return true;
}
static void ChooseExport(App& app,int action) {
    wchar_t parent[MAX_PATH]{};BROWSEINFOW browse{};browse.hwndOwner=app.dialog;
    browse.lpszTitle=L"选择父目录，将生成新的独立结果子目录";browse.ulFlags=BIF_RETURNONLYFSDIRS|BIF_NEWDIALOGSTYLE;
    PIDLIST_ABSOLUTE item=SHBrowseForFolderW(&browse);
    if(item) {
        if(SHGetPathFromIDListW(item,parent)) {
            std::wstring name=(action==4?L"PCD_Result_":action==7?L"Cylinder_A_":L"Cylinder_B_")+std::to_wstring(GetTickCount64());
            Launch(app,action,std::filesystem::path(parent)/name);
        }
        CoTaskMemFree(item);
    }
}
static INT_PTR CALLBACK DialogProc(HWND dialog,UINT message,WPARAM wParam,LPARAM lParam) {
    App* app=GetApp(dialog);
    switch(message) {
    case WM_INITDIALOG:
        app=reinterpret_cast<App*>(lParam);app->dialog=dialog;
        SetWindowLongPtrW(dialog,DWLP_USER,reinterpret_cast<LONG_PTR>(app));
        SetDlgItemTextW(dialog,IDC_THRESHOLD,L"0");SetDlgItemTextW(dialog,IDC_CYL_THRESHOLD,L"0");
        CheckDlgButton(dialog,IDC_CYL_SHOW,BST_CHECKED);Busy(*app,false);Layout(dialog);
        if(!InitGL(*app)) {MessageBoxW(dialog,L"无法初始化 OpenGL，请检查显卡驱动或远程桌面的 OpenGL 支持。",L"启动失败",MB_OK|MB_ICONERROR);EndDialog(dialog,1);}
        else Refresh(*app);
        return TRUE;
    case WM_SIZE:if(app&&wParam!=SIZE_MINIMIZED) Layout(dialog);return TRUE;
    case WM_GETMINMAXINFO: {
        // Resource toolbar must remain usable. No custom maximum size.
        RECT rect{0,0,736,350};MapDialogRect(dialog,&rect);
        AdjustWindowRectEx(&rect,static_cast<DWORD>(GetWindowLongPtrW(dialog,GWL_STYLE)),FALSE,static_cast<DWORD>(GetWindowLongPtrW(dialog,GWL_EXSTYLE)));
        auto* limits=reinterpret_cast<MINMAXINFO*>(lParam);
        limits->ptMinTrackSize={rect.right-rect.left,rect.bottom-rect.top};return TRUE;
    }
    case WM_WORK_STATUS: {
        std::unique_ptr<std::wstring> text(reinterpret_cast<std::wstring*>(lParam));
        if(app&&!app->closePending) Status(*app,*text);return TRUE;
    }
    case WM_WORK_DONE:
        if(app) Finish(*app,std::unique_ptr<WorkResult>(reinterpret_cast<WorkResult*>(lParam)));return TRUE;
    case WM_MOUSEWHEEL:
        if(app) SendMessageW(GetDlgItem(dialog,IDC_VIEW),message,wParam,lParam);return TRUE;
    case WM_COMMAND:
        if(!app) break;
        switch(LOWORD(wParam)) {
        case IDC_OPEN_A:if(!app->busy) OpenCloud(*app,1);return TRUE;
        case IDC_OPEN_B:if(!app->busy) OpenCloud(*app,2);return TRUE;
        case IDC_COMPARE:
            if(!app->busy&&!app->a.points.empty()&&!app->b.points.empty()) {
                double threshold=0;if(ReadThreshold(*app,IDC_THRESHOLD,threshold))Launch(*app,3,{},threshold);
            }return TRUE;
        case IDC_CYL_A:case IDC_CYL_B:
            if(!app->busy) {
                bool a=LOWORD(wParam)==IDC_CYL_A;
                if(!(a?app->a:app->b).points.empty()) {
                    double threshold=0;if(ReadThreshold(*app,IDC_CYL_THRESHOLD,threshold))Launch(*app,a?5:6,{},threshold);
                }
            }return TRUE;
        case IDC_CYL_SHOW:case IDC_CYL_LABELS:
            if(LOWORD(wParam)==IDC_CYL_LABELS&&IsDlgButtonChecked(dialog,IDC_CYL_LABELS)==BST_CHECKED)
                CheckDlgButton(dialog,IDC_DIFF_ONLY,BST_UNCHECKED);
            Refresh(*app);return TRUE;
        case IDC_FIT:if(!app->busy) FitView(*app);return TRUE;
        case IDC_ORIGINAL:FitView(*app);return TRUE;
        case IDC_DIFF_ONLY:
            if(IsDlgButtonChecked(dialog,IDC_DIFF_ONLY)==BST_CHECKED)CheckDlgButton(dialog,IDC_CYL_LABELS,BST_UNCHECKED);
            Refresh(*app);return TRUE;
        case IDC_CANCEL_WORK:app->cancel=true;Status(*app,L"正在取消，请稍候……");return TRUE;
        case IDC_EXPORT:if(!app->busy&&app->hasResult)ChooseExport(*app,4);return TRUE;
        case IDC_CYL_EXPORT_A:if(!app->busy&&app->hasCylinderA)ChooseExport(*app,7);return TRUE;
        case IDC_CYL_EXPORT_B:if(!app->busy&&app->hasCylinderB)ChooseExport(*app,8);return TRUE;
        case IDCANCEL:SendMessageW(dialog,WM_CLOSE,0,0);return TRUE;
        }
        break;
    case WM_CLOSE:
        if(app&&app->busy) {app->closePending=true;app->cancel=true;Status(*app,L"正在停止后台任务并关闭……");}
        else EndDialog(dialog,0);
        return TRUE;
    }
    return FALSE;
}
int APIENTRY wWinMain(HINSTANCE instance,HINSTANCE,LPWSTR,int) {
    CoInitializeEx(nullptr,COINIT_APARTMENTTHREADED);
    WNDCLASSW wc{};wc.style=CS_OWNDC|CS_HREDRAW|CS_VREDRAW;wc.lpfnWndProc=ViewProc;
    wc.hInstance=instance;wc.hCursor=LoadCursorW(nullptr,IDC_ARROW);wc.lpszClassName=L"PointCloudView";
    if(!RegisterClassW(&wc)) {CoUninitialize();return 1;}
    App app;
    const INT_PTR result=DialogBoxParamW(instance,MAKEINTRESOURCEW(IDD_MAIN_DIALOG),nullptr,DialogProc,reinterpret_cast<LPARAM>(&app));
    if(app.worker.joinable()) {app.cancel=true;app.worker.join();}
    if(app.view.context) {wglMakeCurrent(nullptr,nullptr);wglDeleteContext(app.view.context);}
    // The dialog destroys the child and releases its CS_OWNDC device context.
    if(result==-1) MessageBoxW(nullptr,L"无法创建主对话框，请检查资源编译和 Windows SDK。",L"启动失败",MB_OK|MB_ICONERROR);
    CoUninitialize();return result==-1?1:0;
}
'@

Write-BundleFile 'VC2019/PointCloudWorkbench/resource.h' @'
#pragma once
#define IDD_MAIN_DIALOG 101
#define IDC_OPEN_A 1001
#define IDC_OPEN_B 1002
#define IDC_COMPARE 1003
#define IDC_THRESHOLD 1004
#define IDC_FIT 1005
#define IDC_EXPORT 1006
#define IDC_CANCEL_WORK 1007
#define IDC_STATUS 1008
#define IDC_VIEW 1009
#define IDC_ORIGINAL 1010
#define IDC_DIFF_ONLY 1011
#define IDC_PATH_A 1012
#define IDC_PATH_B 1013
#define IDC_RESULTS 1014
#define IDC_HINT 1015
#define IDC_CYL_A 1016
#define IDC_CYL_B 1017
#define IDC_CYL_EXPORT_A 1018
#define IDC_CYL_EXPORT_B 1019
#define IDC_CYL_THRESHOLD 1020
#define IDC_CYL_SHOW 1021
#define IDC_CYL_LABELS 1022
'@

Write-BundleFile 'samples/README.md' @'
# 示例点云

- `sample_A.pcd`：参考点云，1400 点。
- `sample_B_rigid.pcd`：A 经已知三维旋转和平移得到，应可配准至约 100% 重合。
- `sample_B_changed.pcd`：在 A 坐标系中删除 80 点并增加 45 点，再施加相同刚体变换。

先载入 A 和 B_rigid，将差异阈值设为 `0.02`，点击“配准并比较”；然后将 B 换为 B_changed，再次比较。不同点的最终数量受阈值及近邻密度影响。

坐标为演示用任意单位，B 到 A 的变换由程序估计，不需要提前输入。


## 圆柱重建示例

- `cylinder_A_noisy.pcd`：真实半径 0.7、轴向长度 3、轴方向 (0.36, 0.48, 0.8)，1600 个带径向噪声的侧面点和 180 个离群点。
- `cylinder_B_rigid.pcd`：对 A 的全部点施加同一刚体变换，供配准与双圆柱显示使用。
- `cylinder_partial.pcd`：约 200 度可见圆弧，带噪声和少量离群点。

读取 A，圆柱距离阈值填 `0.012`，点击“圆柱重建 A”，应得到约 0.7 的半径。观测长度会因 1%/99% 轴向裁剪略短于 3。黄色显示完整圆周的开口侧面网格；观测不到的区域属于模型外推。勾选“圆柱内/外点着色”后绿=内点、红=外点。

读取 B 后可执行“ICP 配准 / 比较”，也可独立“圆柱重建 B”。B 的黄色模型会随点云 B 一起切换原始/配准姿态；导出的圆柱 B 文件仍使用 B 原始坐标系。
'@

Write-BundleFile 'samples/cylinder_A_noisy.pcd' @'
VERSION 0.7
FIELDS x y z
SIZE 8 8 8
TYPE F F F
COUNT 1 1 1
WIDTH 1780
HEIGHT 1
POINTS 1780
DATA ascii
11.1538293584 -4.11692960046 5.61406893444
11.5374003723 -3.82838125726 5.95642864576
10.9853063386 -4.28285932181 6.35765708043
11.0012106164 -4.25036975827 6.37846543031
11.2032659836 -3.95893904222 6.78977393145
12.4149281522 -4.60256141497 6.84526690401
11.0520658973 -4.09802121301 6.1659312075
11.4626754674 -4.0828287434 5.50469785376
12.4904231025 -2.72104815309 7.76499771651
13.0422627259 -3.77597695884 8.04746233525
12.4340680082 -4.22442384257 7.95495247128
12.1067252349 -2.85406705073 7.71017562652
11.6030042008 -3.49989476401 7.76576827407
11.8827315549 -3.86084700113 8.18740630186
11.5928493853 -4.95325058556 6.75132355161
11.9207896661 -4.4788151215 7.52950359416
12.6206894269 -3.62689445782 6.82910875134
12.2644892696 -4.77357294697 6.08564215476
12.8811221561 -3.98681713936 7.84135363529
11.0035467246 -4.19514708273 6.27917033728
11.5634660115 -4.06064367782 5.54277358787
13.001889151 -3.828835293 8.02358795197
12.1039734895 -4.14236597439 8.06015360359
11.6827207079 -4.50576352999 7.37414475568
12.7803858445 -4.03811798744 7.1445869702
12.1557817903 -2.7634478911 7.88495736543
12.7780837859 -3.31153550762 7.24499497509
12.2885011572 -4.63399435959 5.99414001162
12.1892996288 -4.8910930369 6.43938047237
11.3974309861 -3.66605799004 7.14434086864
12.0293152179 -5.1251940017 5.79886816869
11.6751887609 -3.52254223805 7.97892676495
12.6546572756 -4.08440275025 6.77542278969
12.3547010991 -4.65012942838 6.91815544489
12.7690377639 -3.02407575014 7.4876535758
11.416429937 -3.85685518463 5.96915914938
11.4771098778 -4.20000267918 7.42380066482
11.295624367 -4.52132540003 6.97337939664
12.6069695037 -4.25482058863 6.73164250384
12.0951342018 -4.22177832345 5.7216031698
12.0490657386 -4.85088364998 6.85203082057
11.6729266037 -4.02837223087 7.80544556549
11.5881834589 -4.89754262023 6.82360361443
11.4871671926 -3.52211865937 7.22171941503
10.9062675113 -4.65836090065 6.2670236231
12.8881824032 -3.95749581628 7.49820848212
11.3664534293 -4.38843315633 7.14987341816
11.5987379196 -4.44137647374 7.37191417501
11.7720110226 -4.19364828058 5.45385653579
11.5816108253 -4.0329089119 7.67902972846
11.869257205 -5.24017894075 6.04271033155
11.4126837194 -3.85360422495 5.98333755577
12.3797622987 -2.81365130653 7.62246681612
12.4329892893 -4.45490349332 6.33125957461
11.2945823636 -5.18915759372 6.30762472342
11.6351886819 -4.96741799005 6.74053894251
11.1301166326 -4.12001424737 5.64041649153
11.6468850986 -4.77092630094 7.02530793337
11.5724114993 -4.07705437563 5.51669299067
12.5512575571 -3.58191588697 6.76301669553
12.4224837439 -2.70017761209 7.82146547329
12.6860403741 -4.06867691826 6.83600574985
11.5358159711 -3.51142805539 7.50559976406
11.3897755669 -5.0168505254 6.5782182957
11.4796201408 -3.54973512981 7.29431727503
11.1775691274 -4.09560153469 5.64458965347
11.8203358962 -3.63226099341 8.24031609742
10.8886042942 -4.84190871459 6.19416785465
11.022590342 -4.15915832535 5.83185066556
12.7067017941 -3.88419236139 6.85763534213
12.2214378704 -4.79945540577 5.91687197766
13.0306021404 -3.16109345093 7.68316687149
11.8050207484 -3.8371721137 5.92688340371
12.3424984405 -4.63312249232 6.17594943365
12.6940728291 -3.27831412215 7.17395352529
12.8992724365 -2.86204536812 7.76438390606
12.9986247313 -3.83657012795 8.0420122825
12.5168197112 -4.36693792067 7.51206879228
12.8359766516 -3.51444069372 7.20378218028
12.8128830144 -3.31175147791 7.28737919605
11.794522972 -4.40621063871 7.56336852812
12.9044725956 -2.89877131151 7.7344395792
12.2775170984 -3.80984872742 6.27398459568
11.5194881585 -3.56617956387 7.52097549959
12.5654595392 -4.01804247368 8.30870183897
12.7491021556 -4.00054175129 8.20172415798
11.7615280888 -3.53112134867 6.44984942957
12.3724049749 -4.39511840142 6.12906872713
12.2033726868 -3.34163744663 6.76170069216
12.3641513974 -4.66482802805 6.80319575406
11.4964760375 -3.61545804406 7.50678428886
11.6486166331 -3.31258287357 7.57255627082
13.080848278 -3.71664157079 7.99282694583
12.6225935115 -3.51817615145 6.90175438728
12.5003434891 -4.48228366525 6.68058858782
11.8953564611 -4.13638134317 5.59997155502
11.5605299678 -4.9032915219 6.80700486214
11.6659895572 -3.37041312917 7.85723851506
11.6083265841 -4.77736939391 6.99913467331
11.889390738 -3.74075808314 6.0910406909
12.3486124345 -3.82416338999 6.33757917597
11.6411055729 -3.77374312402 7.88008945615
11.9712391714 -4.47162454581 7.56138358448
11.548544455 -3.46720267698 7.47719388997
12.7094875059 -4.20344266142 7.56009732856
11.8841565605 -3.12598750946 7.29089867062
11.9618039552 -3.92450927202 8.20946466297
11.58027828 -3.5615688076 6.52219278898
12.5315180926 -4.26276139622 6.50507593189
12.5327935357 -3.43619728335 6.87450213129
11.2952996194 -4.94770892799 6.59394995964
11.9014131177 -3.62754667974 8.37084282595
11.5906286844 -3.66135891089 6.27602453683
12.3488249674 -4.66431797257 6.286433193
11.3051385984 -3.98650402778 7.14634214702
12.9099279332 -3.54835137891 7.32230703999
12.051497499 -3.11598777905 7.15882224771
11.4425982208 -5.07800396344 6.51871638332
11.038857686 -4.16870247332 5.70053511154
12.2071227818 -4.87551619792 6.40245803168
11.4132960646 -3.61506741872 7.00151277601
11.9871370243 -4.87019672294 6.85155671752
12.3147361436 -4.53902260383 6.00320863527
11.7461323626 -3.88170284957 7.99824291639
11.3725832975 -5.06234645079 6.51409059588
12.3873567561 -4.65207883851 6.60591642493
11.5149998626 -3.65773037117 7.58449861247
12.2926107816 -4.75637948946 6.61166842891
11.5693236867 -4.10991905242 5.46509566002
11.099766705 -4.4648341427 6.69280751133
12.8494560566 -3.81218374884 7.19859378899
12.2853788965 -4.35558106263 5.94834465411
12.6515135089 -2.99881506594 7.42909068871
12.8318955181 -3.96615055441 8.15291328173
11.5361744659 -4.93044921178 6.76678543295
12.1146918376 -4.59348958514 5.56367334522
11.9144328053 -3.26784543132 6.89755028626
11.459055507 -4.17668945823 7.41199421324
11.0309040503 -4.12703554829 6.03012288102
12.9303507913 -3.90920586794 7.67803652148
12.3928639975 -2.765788163 7.70692246035
12.3225838853 -3.96004495212 6.19914768481
12.2832430647 -4.67191226842 6.01006863835
10.9595248962 -4.29535062001 6.26543722463
12.224282113 -4.85514403267 6.08737010905
11.5264775213 -4.15947200032 7.51688545983
11.8214398554 -3.29468913411 8.29555452453
11.2618261023 -3.81948637682 6.73088933104
11.4175161015 -3.82316709752 6.03987326378
11.0256784864 -4.55441199578 6.53302924905
11.5257406645 -5.01373869526 6.63938074322
11.7463853195 -4.67503316917 7.1901002905
12.6806505212 -4.21168845125 7.64229794607
11.9101445362 -4.06041292791 5.69798913084
11.7308401072 -3.20274265638 7.78972138502
11.4093355373 -3.74620225374 6.2827613337
12.0542041608 -4.31445082345 7.80632322125
11.9129547314 -4.40991186497 7.6184080217
12.1511238951 -3.70094002856 6.2783345572
12.4786113689 -4.30092259863 7.75852023377
11.304580911 -4.04665349898 7.15682689183
11.5954163241 -3.38751604496 7.14388820557
11.9490095851 -3.37629895128 6.6758474765
13.1337120367 -3.52104459024 7.87504189724
12.1686906109 -4.09712392094 8.15348948313
12.7781002127 -3.7488696556 7.03141159562
11.536554385 -5.1700778121 6.41802099102
12.0353989362 -4.9385472316 6.65259628392
11.7539094369 -4.88419015657 6.88594537425
11.0563657677 -4.35212300475 6.60374207457
10.8662686871 -4.74546962685 6.17645192521
11.1273219536 -4.07048953817 5.82239426875
11.4363896506 -3.88449399995 5.89429023646
11.5572630596 -3.95791999409 7.6750360727
11.7783862296 -3.14311207789 7.90840311302
11.3152030918 -4.45122667801 7.04329336944
12.2621293482 -4.72333281739 6.90425017442
12.4341468551 -4.55076358483 7.0525420519
11.8035046571 -4.77710995295 7.05952814516
12.0578764276 -4.26578295971 5.64855081236
11.5453280374 -3.65536298389 6.33569304522
12.1198017452 -3.91392268547 6.02503462531
12.1937023652 -4.88931940415 6.02767173915
12.4254278136 -3.8758173367 6.38830705511
12.6797897675 -4.25387710375 7.1793476705
11.5376009361 -3.71726437501 6.18617193084
11.3742680422 -3.85837822976 6.01450921511
12.1349304533 -4.78611061602 6.91003922127
12.8714252189 -3.99760201302 7.61483601415
11.744737012 -3.68885594877 8.09669827857
11.3345943295 -3.7272908808 6.88989159111
12.2008613877 -4.32384257056 5.80891467244
12.4727743864 -4.45919896793 7.31346573606
12.0610648656 -2.92539074564 7.58203261782
11.745896544 -3.34132322819 6.89470986127
12.9597705428 -3.746128134 7.47678256895
11.7150980719 -3.21729119447 7.52630612325
12.07313277 -3.76474943938 6.15156791294
11.6202186746 -4.74885194947 7.04453091539
11.518021277 -4.2187809042 7.4666305853
11.3109376487 -4.50365793947 7.00748411629
12.0358519054 -2.81919996506 7.99383162607
11.6847536756 -4.28902014905 7.59418822902
12.2045712705 -3.78319978878 6.2273396655
12.4111882614 -2.95679655638 7.3836466521
12.445570348 -4.55830955271 6.58207555044
11.396126631 -4.3256699724 7.23370295693
11.6119081426 -3.93608874022 7.75910700674
12.7024539138 -3.7881076451 6.87565502467
12.7986844303 -4.08478341228 7.35975455745
12.7046568397 -4.22812992653 7.36980664106
11.9044234099 -3.64404509526 6.23436317911
12.2248974028 -4.15548563947 8.09503204925
12.2705652652 -4.80287946812 6.3427551508
11.0555736683 -4.24433834359 6.5657595959
12.1660923204 -3.8226197609 8.48352858357
12.0298438765 -3.6540537081 8.51683889164
11.892495899 -2.98263935598 8.12069777095
11.1642942886 -4.2342943792 6.84973287586
12.8639762144 -3.68601104037 7.21288367761
11.9605705895 -3.49927477836 6.4720841971
11.2419982993 -4.34612658003 6.98274126598
11.7206003052 -3.49558518511 6.54743013527
11.4470479594 -4.00332779452 7.46170356324
13.1584841969 -3.34857926516 7.86699493261
12.2841082825 -2.9922226243 7.31990303719
11.4989938448 -3.5017207252 7.17606994959
12.2652004852 -3.53442738386 6.55305696674
12.3086882606 -4.02245153033 8.3102173851
10.9034543924 -4.82841066877 6.21340537105
12.1045654777 -3.08504196409 7.18795567544
11.6368855158 -3.51403052958 7.87808969718
12.7977766092 -4.101814971 7.55788308773
11.881006671 -3.66119301051 8.32178877215
11.4720716852 -3.54225808055 7.19789951024
11.6736559554 -3.45705335824 7.95699980993
12.405112511 -2.93472782564 7.42261579727
12.6998340807 -3.4201675153 7.06880513411
12.5366409282 -4.04836910493 6.48921165622
12.0791001568 -4.98687662633 6.40769931473
12.2394727285 -4.64650145023 7.16291357558
12.9428294902 -3.85881154399 8.26556169087
11.610730063 -3.78115021381 6.01229478205
12.5952102975 -4.24540779358 7.73971100902
12.774866316 -4.13127123134 7.59264244703
12.0928022853 -2.82762076295 7.82922971095
12.1927815894 -2.74754568967 7.89055628067
12.4054937395 -4.62256605953 6.58886433137
11.7170676909 -4.19500159239 5.41431138467
12.3837665882 -4.39479637259 6.14770015519
11.7920618714 -5.02095651733 6.63936145765
12.8013458711 -4.08648647098 7.48286025049
12.2812788199 -4.39794393135 5.93024854407
11.7234360411 -5.20512574351 6.31662259842
11.3670301349 -5.13280343982 6.4066890198
11.7568795063 -5.18050244161 6.35425248271
11.2674303823 -3.91489980136 6.98074100693
11.8272080699 -3.14494996111 8.1852243571
11.8788616903 -5.09125023942 6.44729254977
11.6341862786 -3.32175127974 7.48882031985
12.3244065569 -4.27707190532 7.88680546032
11.2998179746 -4.27450745589 7.10520005841
11.7830116516 -3.67989330155 8.16655795833
10.987827242 -4.82580698998 6.34723077665
11.9850399064 -3.47138964258 6.52310573176
12.2027866366 -2.9048915918 7.50222423785
12.7447798142 -2.88064265895 7.62648904197
11.8441839087 -3.99849224776 8.02484215692
12.1699041758 -4.94072235194 6.04480346498
12.5412072949 -4.24602360974 6.51546860645
11.7867207516 -4.05024676025 5.63541502519
11.179167585 -4.06414357112 6.82762841138
12.4645242965 -4.43959122388 6.41334013941
10.8805356625 -4.33717397675 5.90427370138
11.787377431 -5.04975676357 6.58801653497
11.6825846985 -3.30798940252 7.82576355379
12.6224471666 -4.17299012959 7.9101145086
11.3896906249 -5.21428147175 6.31200067393
12.6765665592 -3.89889126289 8.49609662958
11.1621466795 -4.99484441065 6.42430675416
11.6663809928 -5.20781203484 6.3276075526
11.3593790888 -4.4072556023 7.13362937034
11.8007712226 -3.3860943559 6.72875391932
11.8030190604 -4.91546160605 6.82513853725
12.2027010323 -4.83762743655 5.89007200153
12.9320744459 -3.76736064508 7.40281130868
12.7656442708 -4.14581458997 7.44384236705
11.866570006 -4.25358460685 7.79178169638
12.1306000361 -4.86388711437 5.6623255763
11.270082773 -3.83072328965 6.33735156503
11.8569987183 -3.0433304412 7.69205983359
12.2418773417 -4.6958269492 7.0239690717
11.3864633924 -3.72315834531 6.4009038603
11.5970895193 -4.91722208511 6.80935435468
12.0359618672 -3.77066137409 8.41761844484
13.0201135531 -3.62561688927 7.58280919047
11.7930268365 -4.96621312348 6.74380899509
12.9577394361 -3.48743402695 7.42811830161
11.814799192 -3.87563435445 8.08878444935
12.3910626976 -4.47296290185 6.20204962478
11.2888871733 -5.22561533059 6.25402291332
12.0134652372 -3.70264200446 6.19586917778
10.9878280679 -4.36804638294 6.43481187835
11.5269452594 -4.66375116508 7.0852137755
12.4331588627 -3.43141525838 6.78103336828
11.7391731888 -3.25713834874 7.17629120367
12.1311053737 -3.0862896839 7.17766646855
11.8454468702 -3.66956590978 8.2647107051
11.1079603743 -4.02880644912 6.14386890241
12.2704464389 -4.26533052701 7.92031996029
11.9441732751 -3.73170148297 6.12370153799
12.781915279 -4.11348062445 7.67963177906
11.9248508076 -5.1785700064 6.1449136519
11.4957868974 -5.10619231663 6.4960872049
12.6501309388 -3.92550040743 8.45557257775
12.380412363 -2.85181003082 7.55904186992
11.1575184342 -4.17270213859 6.81909166276
11.2919739621 -4.59740201458 6.9106558831
12.4139558359 -4.23057177295 6.22837977955
11.1000699233 -4.04706607011 6.04108903013
12.1576990456 -4.33872535302 7.78652918493
11.6033232434 -3.56459559738 6.49402817254
12.8487278459 -3.96115332149 7.32884453614
12.4790030348 -4.52098447541 6.77031376145
11.5181705179 -3.55944547518 6.63499063016
12.7816136054 -4.08383401947 7.22424433929
12.0146951069 -3.53329098063 6.43070958425
11.1662230594 -3.94999276723 6.47040728557
11.4025398899 -3.89866723753 7.3738103271
11.5506245835 -3.815716292 7.69979294412
13.157451398 -3.52247107357 7.95211496855
11.8007995205 -4.52926704122 7.40767264348
11.9395470383 -5.08578760169 6.38496721771
12.801737739 -3.9997273375 7.19699223193
12.1060863254 -5.00218555224 6.21023199435
11.9725806928 -3.93607474878 5.88651506863
11.8742501016 -3.93180035402 8.11934213567
12.9856928811 -3.78462874355 7.63161323968
12.2702759186 -3.37857451007 6.7437110736
10.9836468673 -4.25733629514 6.2796804529
11.4760715396 -3.53504685976 7.16276489645
10.8769240179 -4.39365995935 6.05275901277
12.009461399 -5.14689600885 5.89371304926
12.1395266245 -3.63335150777 6.35064437081
12.0666790486 -3.88848743929 6.00725486604
12.1356884467 -4.94585462168 6.41593164098
12.0674131981 -4.457458505 5.52998083147
11.5991845412 -5.09858275771 6.52535034806
11.7854308582 -3.20251405662 7.26112816639
12.9421833209 -3.14399973394 7.57207753944
11.4699519306 -3.73201456487 7.49729319766
12.220672447 -4.8669476396 6.15985896747
11.0327498265 -4.13290607798 5.94716524561
11.3663223468 -4.37959092087 7.1566917186
11.1530675689 -4.48743226921 6.7809685828
11.6447012003 -5.00832813134 6.68109273121
13.0200841132 -3.77586871998 7.78881345541
11.7280784056 -4.65618147657 7.20577244523
12.7642898217 -4.09314135242 7.86535580751
12.3540556003 -4.49880357046 7.38372032844
12.7387099611 -4.10848323868 7.07017666444
12.434832538 -2.96187593223 7.38232023095
11.8308775212 -3.399418319 8.3292707573
11.056342155 -4.9919090626 6.32071159177
11.9137930023 -4.91086915875 6.81384370091
11.2073218481 -5.0015451444 6.46270632446
12.4528279811 -4.15424428685 8.07896284809
12.288675744 -3.834145527 6.26226950524
12.4901903065 -4.04382268734 6.40632344419
11.2975800359 -3.76744419086 6.63416723353
12.4457691483 -2.73728025295 7.73657286887
11.4403193321 -3.99004047365 5.67839655354
12.9672869263 -3.31760322101 7.50689454957
12.1474914752 -4.92646785383 6.45107297397
11.2609183937 -3.89964864148 6.94823440409
11.8535312447 -4.47456241152 7.51190074268
11.566664871 -5.31065778352 6.16659064743
11.2903015658 -4.21327140051 7.1152586433
12.5805220592 -3.03569887447 7.33901496861
12.8835731435 -3.60474306259 7.2686416705
11.6067395517 -3.61454803984 7.82624792585
12.1694536301 -2.9048477167 7.53195769097
12.8353537509 -3.92663405084 8.2726855282
12.7083100877 -3.46809294033 7.04039076243
12.1876897552 -4.88329809305 6.46665217082
11.7358996117 -3.34133101047 6.90610817831
11.9510204281 -3.25800766496 6.90227094706
12.3818290836 -4.62524279893 6.90788854902
12.2804551808 -4.31524234349 5.94593325213
12.4935366794 -3.53890807637 6.73098212846
12.2693667761 -4.68824887541 6.99650396646
12.0475930696 -5.09999515845 6.01882847351
12.4573266324 -3.9037984175 8.51502054202
12.2095591837 -4.13884144056 5.92721278948
11.6722933262 -4.5794806893 7.27648447821
12.8204514953 -3.88326107278 8.42361571274
11.8158398179 -3.75172943365 6.06127145595
11.9579760888 -5.04112783526 6.49164849525
12.4696374299 -4.29927591295 6.34099007951
12.3406481438 -4.6501759064 6.93718299655
12.1397883105 -4.9384811807 5.79707989233
12.3263867223 -4.12310815391 6.10883158036
13.0981201068 -3.4763913971 7.75280566442
12.5179598529 -3.6708308165 6.65445073609
12.7980836036 -2.90874871674 7.63260047091
11.6041978461 -3.45847041468 7.7176282257
12.50622327 -4.33405832144 6.45442269214
11.9579929569 -3.5898531305 6.33876653459
12.0786707557 -4.64366946722 5.47886358541
11.5105163486 -5.02022134649 6.62673293718
11.6029298231 -3.40778217866 6.98917347774
12.0822989072 -4.63974864091 5.48611492965
12.2354454924 -2.89689177284 7.5048585394
11.7845489435 -3.36926530759 6.7747983929
11.9617793817 -4.30996535358 5.50584062422
11.2909914723 -3.91667899399 7.0891092435
12.4031573287 -3.14012782693 7.11561222079
10.9732072982 -4.19870617039 5.96338541445
11.2618780312 -3.81388262948 6.61506328703
12.0574703432 -4.2566148545 5.65394297701
11.5324940462 -4.17187305655 7.51406840232
12.0205693106 -4.80882371783 6.97194166128
11.8148039242 -3.0965687497 8.00761602985
11.3185039361 -3.74265050602 6.62919674012
11.6645116337 -3.36164697517 7.84610163549
12.5009338998 -3.08996375484 7.23272251816
11.2737075492 -4.63238776677 6.85919491456
11.081269712 -4.07028257985 6.36279764053
12.397254522 -4.52547104931 6.25391816294
11.8270288085 -3.67861210275 6.17496748359
12.6471610387 -4.19109662523 7.81292871315
10.9540845631 -4.22799486323 5.78916634294
11.0658528858 -4.10165676912 5.92153133356
11.9040061877 -2.97611773031 8.24758494138
11.4163229907 -3.68779355414 6.45038244831
11.8399233767 -4.3706423538 7.63512875709
12.0802169454 -4.64505976556 7.27448278303
12.8300377426 -3.93125696423 7.21621142708
12.4946827143 -4.38360349438 6.45688770918
12.1912092231 -4.57686985156 5.71477666039
11.8070179501 -4.15379680644 7.84778409616
11.9010306512 -3.86367999917 8.19545883481
12.1463725689 -3.52720286885 6.49158140688
11.5151262914 -3.51340153903 6.85727054684
10.9065737475 -4.29484660896 5.93879458083
11.8038451065 -5.03784013973 6.61310231622
12.3582983588 -4.41287571389 6.08858961812
12.5066758343 -4.47874903028 7.04587807097
12.9297969486 -3.63285914874 7.35540201564
12.8175249671 -4.07981342138 7.59693351484
11.8284589266 -4.68458386163 7.20885643839
12.4055118489 -4.19722921562 8.01886412463
11.030108692 -4.25678549777 6.47246699404
13.0820666175 -3.46146975613 7.70151175647
12.8951389739 -3.8405968343 7.33211851047
12.1419342096 -3.34771117561 6.73766406975
12.8120030124 -4.04278439651 7.28605496822
11.6316956068 -5.10397641018 6.52042869773
12.8021307211 -4.06568756168 7.30548175674
12.2543515803 -4.73597697376 5.95208312481
12.1020784217 -3.39362500228 6.66065737791
12.5014037539 -4.22380884106 6.40816114631
12.6930035288 -4.15823642238 6.93935697783
11.5808999912 -4.08234691997 5.51112893853
12.6894869543 -4.23561590991 7.13877222931
10.9028188174 -4.4123582669 6.18509687559
11.8659655815 -3.03759445584 7.66991818491
12.0173504976 -4.93671973372 6.68394373993
11.3288573299 -5.13171304865 6.39740853963
11.9042355481 -4.91812386103 6.80988754617
11.5207847195 -3.64851128681 6.37300135575
12.0929285322 -4.49739641458 7.53454811879
12.107897534 -4.50300859075 7.51843651483
12.4464996318 -3.67795660715 6.5704210657
11.6727660483 -3.78558768495 6.00222036905
12.9750626957 -3.84150592266 8.22099974042
11.1273411469 -4.04433328285 6.57732382713
12.8587445191 -3.99088565741 7.97201305034
11.8263067415 -4.50208507202 7.45479704712
12.4506235532 -4.29757695058 7.78817090075
12.5326424142 -4.39544808203 7.38420616798
11.7131305793 -3.27551588083 7.89927591614
11.9534152987 -2.96099285107 7.68678826569
11.2696716676 -4.8385832401 6.6891586947
11.2387551089 -4.54608365832 6.87838864084
12.1086072835 -2.78819551046 7.90513260786
12.4246565754 -3.52501273094 6.68518258544
11.2828772177 -4.5622501808 6.92552307626
13.0094663579 -3.77379717271 7.72106346808
11.3688903313 -3.72767696618 6.44184976012
11.2084945178 -4.40771321322 6.90192468099
12.443185082 -4.38477323514 7.5819998653
12.0944769989 -2.9376113123 7.51322107735
12.8009475205 -3.91520742384 7.10844477901
12.051093756 -2.89079226459 7.68934149348
11.71352981 -5.03327607737 6.64271848861
12.4389846202 -3.37367468471 6.85415214909
11.2653800167 -3.83364858603 6.82835894898
12.4028106276 -4.2959556089 6.18657125367
12.8555841761 -3.7924762122 7.2149905465
12.296047028 -4.45598375274 7.55511552101
11.4112622652 -3.64232811247 6.68047832737
11.5198153766 -3.62997982076 7.59782190996
12.46923575 -2.84290401128 7.57113312986
11.2652958843 -4.52480273181 6.92561125467
12.1020076347 -4.54782220756 5.55258804739
11.6465688053 -5.12992812078 6.47242044668
11.0365362304 -4.89997780315 6.37295606229
12.2224079415 -4.8654308573 6.18905728918
12.4199490735 -4.47468230853 6.29328468976
12.5275116462 -3.18786337547 7.12685787169
11.5714096291 -5.16548294185 6.4172994512
11.6628310655 -3.50568739909 6.56914828651
11.8387296444 -4.1549513294 7.87111811505
12.3153718045 -3.86197876209 8.5336946549
11.5839893607 -3.91223686427 7.732473243
12.6940852926 -3.240931416 7.20640281368
11.5445281498 -4.83617145878 6.88525672339
11.852440149 -4.84919349152 6.9415926629
11.3162069227 -4.01658603433 7.18979760415
12.2577208446 -4.22657108612 7.97450355426
11.6220419291 -3.55734790232 6.48972317716
12.3156718069 -4.74126167623 6.60590404297
11.2841183321 -4.51828580038 6.96174507297
12.075186255 -4.30844241728 7.81970406396
12.4725618019 -4.48047708418 7.22689331935
12.5817278093 -4.14881420653 6.59954599117
11.5012816758 -3.58032259286 6.59781347972
12.006114302 -4.10627722746 8.05700548924
12.4645063183 -4.4501874102 7.36348719879
12.7039249859 -2.86293424915 7.62275037928
12.5580908599 -3.6318341536 6.73313504245
12.1509014962 -2.84708238202 7.66993791706
11.7707413878 -4.02682106646 5.65343808039
12.196696716 -4.39010048558 7.70552256514
11.1026115558 -4.0897477873 5.80704900662
11.5224626929 -4.07387851909 5.51123593014
11.5597502702 -4.81652618683 6.92456398302
13.0068738626 -3.819254017 7.86889414464
12.131811465 -4.50228831335 7.52034946179
11.9093369685 -3.03607446667 7.516109506
11.868211104 -5.20169027238 6.16365450069
11.6034316231 -3.36868364137 7.49885992205
11.1441032031 -4.33897042625 6.79649612773
11.679435092 -4.27977715178 7.60651652443
12.8286292168 -3.30738433016 7.31055509705
11.1680660641 -3.99378382675 5.98652333063
12.245602014 -4.24751012139 5.91838095226
12.7821272201 -2.9260133756 7.60469607619
11.3242330249 -4.54558205191 6.9877741624
11.8736230616 -3.09114619719 7.42259670091
12.6784369572 -4.15610157807 7.86847789992
12.2426417133 -4.24879286462 7.95158338052
11.4242288462 -4.737069017 6.91804200455
11.9941725653 -3.68134120164 8.45032293196
12.5145323322 -3.84124427859 6.54281481669
11.6374917855 -3.33892931416 7.62904498514
12.5101100702 -4.48164016185 6.99768050369
13.0050460548 -3.51469622452 7.53567492238
12.8996989133 -3.95172481118 7.54792077825
11.6874400299 -3.9157945347 7.89766706271
11.2139371267 -3.88093980632 6.52965486715
12.7717378333 -3.95594692668 7.05119009402
12.2745527448 -3.75094588497 6.31724785877
11.0179653142 -4.67411633317 6.47486563379
12.3572884064 -4.19712233564 8.02961786504
12.4025440845 -4.32086610824 6.1896843139
12.3082110863 -4.67114453434 6.09721467439
11.4901797384 -4.87405069453 6.81016163674
11.5659557012 -3.80380705344 7.73150260556
11.0325595144 -4.54323519852 6.53978314647
11.8714347219 -3.65356409826 6.22063273911
11.9500195414 -5.04502874947 6.47232146011
12.9544773773 -3.75427032217 7.46456485342
12.4043049589 -4.4284080571 7.52430604727
12.3056464824 -3.80067576496 8.6033781887
11.3976245171 -3.63212845325 6.87479108406
11.3192188823 -3.74299962104 6.7619044626
12.0679522401 -3.67112461681 8.54343665378
10.9292070619 -4.26241057101 5.91782753206
12.1920509983 -4.2641382235 7.91215258891
11.8164664729 -4.66312465628 7.23092997991
11.0720792339 -4.19166660967 6.56053335977
11.0259394219 -4.22319593559 6.43028104494
11.7340897668 -4.14945010221 5.48326651555
12.1106700502 -3.14419921944 7.07165636486
11.7734472719 -3.43069698961 6.6483987143
12.1450851617 -4.04986370472 5.9178568162
11.78867323 -3.23163157188 7.14777973935
11.5688556059 -3.52095299575 7.66594683273
12.0483261984 -4.20740729049 5.68075789284
11.343565171 -3.7960079486 7.12959506118
11.2147787958 -3.88044577215 6.58514673776
11.9245204413 -3.59540140753 8.4274368408
11.8280522347 -3.07261435599 7.83466124533
12.3543334075 -4.10864855553 6.1561453625
11.6534980093 -4.60651437826 7.22707185768
11.9841465165 -4.28508029286 5.54957716922
12.7227046361 -4.19834737286 7.2385197341
11.351316717 -3.69974441515 6.8409347974
11.801914734 -3.09512091258 7.75832465781
11.9464164526 -2.92468926028 7.9059044083
11.3808175875 -3.7361202484 6.38254113434
12.6534343567 -3.241398719 7.17003473082
11.3719021785 -4.02082262809 5.64524114658
12.8089377625 -3.89802556006 8.3820769188
13.1488444857 -3.58842699758 8.01235775326
12.7587332847 -4.14948115387 7.38964356046
12.9321750876 -3.1348116195 7.56496868136
11.5541696272 -3.59791466335 7.68106363769
11.2846355223 -3.80756136952 6.87499895908
12.6632949073 -4.28289748032 7.16792501092
12.6019616351 -4.14970046906 6.6517228661
11.4166474977 -4.81034590603 6.83392519366
12.0065945936 -4.7675610645 7.05550869374
11.6262881745 -4.00392913107 5.64380737448
10.8729404108 -4.61424536011 6.19409090661
11.9094103142 -3.81525514511 5.99794731946
11.8803462443 -2.99592528977 7.88074253143
11.4904520277 -3.71578214747 7.54833277159
11.7066015236 -4.18880873251 7.7145996026
11.5514710687 -4.87831878076 6.83754085936
10.8697990449 -4.44857400608 6.10253598158
12.103544479 -4.98713628224 5.72670849837
12.3924049252 -3.56017627042 6.61937424506
11.2474414912 -3.83882183499 6.62375194149
11.5691514062 -3.66086227527 7.74611490727
12.3110934533 -4.3790260589 7.70127514637
12.6110230901 -3.76657584713 6.72771330971
11.7532930074 -3.35739108311 6.83954355084
11.8253530015 -3.59047955986 8.2692095871
12.028921119 -3.16829947858 7.04133069797
11.7073532169 -3.46837301101 6.6213082971
12.1306505414 -4.88211078634 5.68035420925
11.7571424153 -3.65897811845 6.21999620014
11.8485667995 -3.8340180947 5.94593783215
11.4533452196 -3.67308948031 6.42315103242
12.9216203487 -3.65646998098 7.34059831834
11.6502010048 -3.68598960503 7.91973023656
12.0397553713 -3.89164482281 8.31349342562
12.4650314472 -4.28938415955 7.79726018438
10.9847527003 -4.92579478508 6.28777927195
12.0909599498 -3.63789372515 6.31724497604
12.3966264746 -3.29738037179 6.91386839063
12.3629747463 -4.68186005022 6.52851641447
11.3640745421 -4.79269343835 6.8073612755
11.1761170079 -4.83351393896 6.58382744229
12.4070085143 -4.53106083097 6.31389735545
12.4348523251 -2.84957069605 7.55384390506
12.1098068352 -5.02118232859 6.0332953429
11.9821670959 -5.14125588078 6.09116717568
11.2344537218 -4.54703876838 6.86915755609
11.9030887585 -2.97741547453 7.80561931508
12.3037048198 -4.56420333948 5.98084251268
12.4630935542 -3.57183685834 6.67738202497
12.4316702802 -4.51355022435 6.36963648537
12.1610454466 -4.94902590508 6.07930681738
11.0235988144 -4.17140071842 5.76248089206
11.2605413657 -5.04819368459 6.45428946639
11.7126056605 -5.19206600634 6.33478962519
11.4422458599 -4.04756385419 5.5615242572
11.6229862742 -3.8864033489 5.8310301549
12.0655397563 -4.81824150035 6.91930940114
12.2319997676 -2.92618921082 7.44617659611
12.932894371 -3.84083643178 7.48405963345
11.7011245402 -3.89969129604 7.92171538862
12.0910396357 -3.93596014432 8.30168256174
11.2829048781 -3.95877483904 7.07749560285
12.6685195853 -3.88501733283 6.79068936096
12.0125801356 -3.83237000455 8.34522235352
11.2933291504 -3.7829014691 6.73259494686
12.4908905452 -2.97335183641 7.38241193572
11.6371034818 -3.86396983055 7.84192249196
11.7902804153 -3.57508170024 8.21449671604
11.8007543078 -3.6920817702 8.18755706071
12.0395243305 -3.71154928739 8.48486728471
12.3342750936 -4.56507925253 6.08170309063
11.2977338686 -4.23726674998 7.12229817048
12.5290987138 -4.32635023184 7.61443431774
11.591436946 -5.2888255953 6.18810267039
11.8206874241 -3.20534729876 7.16591299694
12.4717702215 -3.69589289408 6.58870031088
11.9158691301 -5.05722161212 6.49483965445
12.0514514889 -4.46409060459 5.50713143166
11.9140409221 -3.20019966188 7.06634226053
11.9763655882 -3.80204407537 6.05364595082
11.7874528095 -5.24021000111 6.17201024633
11.2807652569 -5.02203693406 6.49837882235
11.7760475903 -4.90306540345 6.86043252353
11.4174389572 -4.92458432103 6.70300315067
11.6103774057 -3.97082246459 7.74512677581
11.7456798283 -3.20085939493 7.91606554118
12.900330319 -3.96459372552 7.77290348742
11.884953522 -3.36706354417 8.44497047929
12.0564615042 -4.18078557008 5.71356963416
11.0928315205 -4.04498537246 6.31581017543
12.6239592193 -4.04915446319 8.21576034793
11.7569670039 -3.2208157155 7.26648148744
11.7911027907 -3.30744416436 6.91800805361
11.0489888502 -4.18286041813 6.47693072355
12.4783432508 -3.86136287767 6.46665881065
11.3495402625 -4.57112956575 6.99250612791
12.4886086076 -2.79284030031 7.65322823052
11.3337609995 -4.68326052418 6.89124738766
12.0841814328 -4.93757689447 6.58800547144
11.4740275227 -3.5523842631 7.25790538667
12.612517162 -4.09467079331 8.12190397496
12.4945718088 -4.50612180558 6.78930771691
11.404029155 -4.47404504523 7.13741976515
12.2958746719 -3.92767997508 6.19333935803
12.5475417114 -3.23215978175 7.09046798864
11.8455925957 -4.81498536428 6.99889256353
12.2026878249 -4.30390688374 7.84703261463
11.5073246464 -4.93872557924 6.73665720321
12.0808137886 -4.8367800232 6.87303517013
12.7209724983 -4.20790065597 7.41260345261
12.1035767359 -4.98611324769 5.71542629793
12.5199768663 -3.94006743676 8.45370860583
11.7076657398 -3.57828342614 8.05101187643
11.1503803863 -4.26499024063 6.82117093475
11.8231548457 -3.58068732288 8.26314283782
11.4541380897 -4.82747717515 6.84534160714
12.4480994136 -4.46127794628 6.3850707667
12.7785768054 -3.57233725424 7.08773782179
11.9463242575 -3.58324512863 6.33766302479
12.0184268459 -3.98031761864 5.86566559737
12.0590813767 -4.8458878629 6.85706738991
11.5895946451 -3.3819877803 7.36019856221
11.4803408717 -4.36334801839 7.31790001054
11.970053514 -4.90967163937 6.78884765884
11.8972873091 -3.16774037114 7.1613364231
11.3163059935 -3.89504224318 5.98465686246
11.0167441719 -4.15229273008 5.89054907811
12.3049634013 -2.95905289688 7.37313385854
12.4717080957 -4.46937262173 6.4936360275
11.4460556834 -3.88975097311 7.47101153371
12.1731521996 -4.90564736101 6.43011332379
11.7668698593 -4.4133445953 7.53469936401
12.0589407465 -5.05865720639 6.16655519514
12.1336685764 -2.80020235055 7.82453823275
12.0066348927 -4.13865558641 5.7023017362
11.9666178773 -4.19122297015 7.92824845658
12.6540634209 -3.03026524404 7.39494729768
12.6469964269 -3.6971326642 6.81569505609
11.9826035657 -4.19179197976 7.93771936661
12.0755809343 -4.39149878827 7.70369507577
11.352300703 -4.99217864358 6.58373757004
12.5911195676 -4.35500023861 6.87109490537
11.5387038169 -3.90807291627 7.65441881094
11.3233842241 -3.83728995372 7.10934413145
12.3040963213 -4.7571466914 6.4287870273
13.013060116 -3.80284906993 8.21759100848
11.190238609 -4.01418473321 6.82137641103
12.1931553988 -4.8683334861 6.5157950236
12.6543670922 -4.29256282294 7.22800797026
11.9526616343 -5.09181653266 6.3437945008
11.9418668009 -3.64877618499 8.41397319566
12.8809534472 -3.98066170636 7.87161031111
12.8762197948 -4.00014425584 7.73324789217
12.1758571645 -4.03891521996 5.9657500934
11.1495839122 -3.97577507893 6.16655881351
11.7128611949 -3.74827810061 6.06408824847
12.3043910732 -4.36078317019 5.97981379192
11.0411240213 -4.94272130787 6.34762143995
11.6905016414 -3.32396110243 7.89355343172
12.7140120361 -2.94727750468 7.52707571316
11.2931553961 -3.77226264372 6.68179687028
11.4720390292 -3.83965201363 5.96780677642
12.4477761597 -4.31010856592 6.28333786605
12.6139636684 -4.11474961666 8.07040197393
11.3226928248 -3.74407781936 6.60135917412
11.9893427232 -3.73886541789 6.13449744914
11.100253732 -4.03711746988 6.26498205205
12.6994041036 -3.80745258538 6.86372853045
12.5515792455 -4.23035927412 7.84569456786
11.271995279 -4.7257190295 6.78349202705
11.8101732907 -4.92991439211 6.80897027066
12.2266329999 -3.98452205548 6.06132594945
11.3752227062 -4.45234117699 7.11432208805
12.6608265315 -3.9080550176 8.49219212723
11.2263387393 -3.94595472263 6.85808494367
12.2197482384 -4.8452320889 6.5217676365
12.2711677413 -4.17761596884 8.06089596023
11.8062836174 -3.8448186551 8.10759497992
11.0264743219 -4.14581220472 6.18528320423
12.0871622862 -4.85789264698 6.79833745663
10.8883358877 -4.78875207133 6.20651974502
11.5417074116 -3.72174641003 6.18173589324
12.3689857865 -4.51652831573 6.1639996809
12.8049999917 -3.58747860776 7.12070128241
12.2330818249 -4.85305796817 6.20188608983
12.3919812394 -3.92563864937 8.47220809524
11.3327385903 -3.74024912236 6.52610653609
11.859069324 -3.50683380374 8.35476705133
12.7499910671 -4.04199143255 7.03260095174
13.067469796 -3.6560702014 7.75570094191
11.8369451435 -5.22748417685 6.14996995537
12.2197215498 -4.64766487047 7.17066207901
12.3135689192 -4.74042979452 6.61212077726
12.2395245808 -4.8378225243 6.47779586883
11.2209052022 -4.58735954752 6.82253331235
11.7301344565 -4.50511871892 7.40623893818
12.5332754535 -4.35011075531 7.54873202801
12.3410509399 -4.68475384385 6.28739904661
11.2825366935 -3.81922790719 6.87250970858
11.9504723572 -4.49299925161 7.51686888467
11.4736340437 -3.73176453468 7.50574723629
12.1934115614 -4.90786450333 6.14155559548
11.8031764015 -3.81582588889 5.95922264166
11.228157668 -3.93365526122 6.03826848114
11.0581767828 -4.60041166987 6.57048163497
11.455974278 -4.21325083159 7.38793299151
12.8090124374 -4.08503066393 7.45914453331
12.7545551522 -3.92667805909 6.97718490059
11.0875894801 -4.11964587721 6.54960032151
12.9490720577 -3.51430493825 7.40963649281
11.3793233989 -3.66747252912 6.94958420739
11.1346845385 -4.09155241349 5.73321502557
11.8994949494 -4.47955488647 7.52211435372
11.3357318671 -3.73720191921 6.55028245202
11.6035689565 -3.37752674849 7.53577104127
11.0302834839 -4.90032502417 6.36825356969
12.2276083546 -4.43171511161 7.62637848413
12.7147629464 -4.17568991631 7.06268876734
12.3124001055 -4.28870950946 7.87546139628
11.8169889817 -5.24296958005 6.12654423993
11.6254713805 -5.0310961516 6.63201427775
11.8200744375 -3.08149700323 7.82669005466
11.8330623659 -3.73647294065 8.21234521427
11.6429620484 -3.79285787282 7.88114626891
12.0465455903 -5.01469374154 6.39193994559
12.1996791766 -2.79459152969 7.75454065166
10.9728297932 -4.20676877713 5.9881920645
11.495244919 -4.3080227929 7.37124012424
11.9025694862 -2.98879378095 7.7653409478
11.5007689004 -5.15115474762 6.42808474998
11.8911074463 -3.78385195778 8.25185771784
12.0399447844 -3.02960381078 7.3589113519
11.9863319215 -4.12848431488 8.00963525932
12.0312630744 -4.98625760067 6.52405515276
11.71868464 -3.89402417669 5.82123003792
11.8074518973 -3.53470130337 6.43257623679
11.9593550715 -4.3302302902 7.74112647017
10.9459976614 -4.25655761901 6.1054963561
11.7125831043 -3.59595750395 6.33636123077
11.5786427748 -3.45543964881 7.62580297365
11.9725792229 -5.01115187794 6.53932689513
11.8258284313 -4.14333924298 7.87573813446
12.9520095358 -3.83011326241 7.55238230955
12.064881863 -5.05751187 6.15915901646
12.1093664263 -4.9581665338 6.44886478634
12.0856698778 -3.73092266117 8.51906388735
12.8209693443 -4.0340688841 7.33507372455
11.9728750572 -2.98954419169 7.53823188132
11.9866544503 -2.87284595537 7.94665140834
12.2389828925 -4.52671267455 5.82785109564
11.7268347116 -3.22683874169 7.88879126766
13.0011467996 -3.83177964365 7.97337768585
10.9465429638 -4.42920114164 6.3351512333
12.3272642085 -3.39129917423 6.76480629615
11.4844455279 -3.88244986611 7.55439999841
12.3893640224 -4.51026822012 7.32380292341
12.6442281928 -3.71239506094 6.80503211448
12.0810855927 -4.64793124967 7.26052611963
12.1543637598 -4.05339881351 5.92319999805
11.838639986 -4.70830849251 7.17615265939
12.232220166 -4.85889905968 6.37125068978
11.3582611648 -4.93319379627 6.65542031013
11.6145021753 -4.02307927586 5.61287569955
11.7448946082 -3.55415199784 8.13171521601
11.8761409571 -4.83253569892 6.96775288474
11.2252024811 -4.02406261172 6.94202574982
11.2856323082 -4.56349542704 6.92407245261
12.3182617172 -4.26273548739 6.02688239261
11.5337786839 -3.52883246567 7.54514922804
11.4593246819 -3.60226164747 6.65214897532
12.8495103729 -3.92693271545 7.2712754391
11.7339873531 -3.22013300492 7.88051848255
11.1049117648 -5.130458982 6.24209375973
12.348479441 -4.32851625076 7.78687133139
11.1377390022 -5.14833278145 6.25191319019
12.4927864173 -3.80961422006 6.53004884705
11.085494066 -4.30758522749 6.67377206832
12.089095583 -5.01676119756 6.23581353067
11.3661730488 -3.69868550394 6.60457663405
11.8675797736 -5.14062372207 6.33607459913
12.1172278912 -4.95127506409 5.70704005285
11.7168997775 -3.8609738368 5.88295452656
12.1493105815 -3.57993769776 6.42398767766
11.8844703046 -4.08222682993 7.98260385105
12.8350616639 -3.93918257437 8.23548460614
11.6528543136 -4.67764422612 7.13811369656
12.3002437189 -4.75709564464 6.25297106063
11.4481606661 -5.21900476461 6.31831715556
11.5603226611 -4.84638076195 6.88172613259
12.21550664 -4.30022282079 7.86404099895
11.9981553624 -5.16168775753 5.84516529476
11.457748935 -3.61676111132 6.59879104739
12.2564311586 -4.59289919728 7.27392756592
11.5254328797 -3.7016144975 6.23124376616
13.0135027522 -3.59123148852 7.56410537341
11.4824266404 -4.25976640661 7.38412150412
12.4579423803 -4.42038258258 7.4751503224
11.1670201721 -3.95427442611 6.26748965256
12.3803834053 -2.84403338373 7.57120627197
12.7254541876 -3.24386261743 7.23783730143
12.9604448344 -3.8872337409 7.87668119225
12.1640076839 -3.59557426386 6.40881963422
11.6191314043 -3.4980788802 7.80511626426
12.751227987 -4.12914926762 7.17383170911
10.8853300734 -4.47499568755 6.19222402272
11.1728989592 -3.94599443328 6.25146732759
11.7793546851 -3.13736912157 7.89123310052
12.5451819348 -2.81823881757 7.62379679848
12.7862106616 -3.86307963316 7.04118083406
10.8788252415 -4.71902618083 6.20240759436
11.9246872931 -3.85132883746 5.95888291029
11.5361838273 -4.10667406238 5.4612673638
12.5417432941 -4.37723051371 6.6527555456
12.4065294325 -2.98776103368 7.33529270288
12.7925265922 -4.00010225376 8.13526231867
11.9838208707 -2.91823325262 7.76540879823
13.0003568242 -3.24313902588 7.59225727646
12.5963043833 -4.36874930141 7.0604596503
12.162805274 -4.16580377653 5.85401323711
11.8552892841 -4.15207861882 7.89024409767
12.173530609 -4.16204968101 8.06164323618
11.8466647093 -3.35128363308 6.77089024005
11.5518005233 -3.43493637634 7.11903008863
11.0225584556 -4.2230173243 6.4166539928
11.439925937 -3.69185574184 7.37506505414
12.4559958345 -4.1464250679 6.32296539688
11.9076391251 -3.94024281749 5.83292141408
12.8803089016 -3.91364103877 8.21260549544
12.0406139603 -3.78125445656 6.11426535196
11.2236467504 -4.01424729598 5.79883934052
12.7287985704 -4.06029509791 8.05674483094
11.3920292994 -3.85659822215 7.33528193923
12.0733390775 -3.05815138279 7.25808629277
12.2036494808 -4.28299956345 7.89111006104
12.3120107092 -3.94267067505 6.20264766125
11.7590035667 -4.24630451153 7.70848857767
11.7007120226 -3.63011495737 6.27657976994
11.3953762159 -3.75518733668 7.28858063683
12.3451472141 -4.17122051106 8.07386077982
12.3200630789 -3.67780721544 6.44212110482
12.5421286721 -4.19913861854 7.93584559415
11.7385511482 -4.95089498595 6.7676356858
12.793462037 -3.91462203365 8.36351754715
12.3170494144 -4.15671572328 8.09634102026
12.2180248859 -4.65104708554 7.17277871476
11.5833536742 -3.46145177218 6.84362114277
10.8382417154 -4.59298419485 6.10822372677
13.040685082 -3.75923020323 7.84252364903
12.3179714668 -4.62571863777 7.10859135648
12.1048490464 -2.9938263429 7.38278203329
12.3603085198 -3.82929065816 6.34415592724
12.6538698824 -4.28463309189 7.39161942852
11.6051992041 -3.98915002055 5.65480532492
12.044784473 -5.09776161226 5.94915935125
11.6112686659 -4.82386810066 6.93750507337
12.5864630379 -2.8135764737 7.63648103535
12.4536153648 -4.07254390117 6.33236135639
11.4390633537 -3.57778139119 6.9504081625
12.3490669652 -4.47040675402 7.46301977061
11.7789888195 -3.30180067087 8.1796227143
11.6344893144 -3.77199063062 7.87037693194
11.7546977526 -3.3203912727 8.11776525658
12.1719824604 -3.79959124296 8.51742346582
12.0490818022 -4.5230435098 5.46633056021
12.3080315778 -4.72326622574 6.19678162476
11.8221435234 -3.06676547354 7.92248689974
11.515300377 -3.89117147924 5.8396475632
12.2037846617 -3.86260756387 8.47061782435
11.4269597007 -3.64993336882 7.24480353498
12.3338022261 -4.12733724458 8.14550259085
11.5871308526 -3.45258705377 6.86478670863
12.9995207618 -3.71037858414 7.58498076983
11.2450443971 -3.91765294145 6.0632206076
11.7134237804 -3.32619712565 6.99286601382
12.5968258572 -4.21288773423 7.82939334458
12.2338590078 -4.84782828868 6.13359547541
11.7047673622 -3.314463255 7.06044882008
11.7883827192 -3.54324309188 6.42380610776
12.3786195495 -4.23556029625 6.14704958284
11.672338728 -3.59610274183 7.98583659639
12.6215804397 -4.24121598873 6.77834468106
10.8860664672 -4.34717540427 5.98632009165
11.5046690492 -3.9577660769 7.57936399746
11.2201839388 -3.92177029493 6.79042276272
12.9264326772 -2.90578112047 7.75540601024
11.8761401253 -3.00012307704 7.9222965904
12.5586755059 -3.85049624239 6.60382854648
12.5972365013 -3.92354763713 6.64372559447
11.7812618445 -3.15763361076 7.99104949618
12.3897892635 -2.92743583375 7.4307040157
12.3743966902 -4.10600113842 6.19130702982
11.1780005287 -4.84752397703 6.58244873253
12.0115577192 -2.91829692217 7.69869273716
12.0945495082 -4.57020436231 7.39664072554
12.044837597 -3.75788048644 6.1490083439
12.5364964898 -3.07068926238 7.27486459686
12.0255570992 -3.90789531028 5.95032552672
11.7992328483 -4.69044558491 7.19148466715
10.9186773195 -4.60001998267 6.30047202634
12.3502972849 -4.69910263995 6.43502222487
11.4857670994 -5.15346454254 6.42819483561
12.0540213547 -4.07221876907 5.81311897125
11.5079018832 -4.09107679023 7.52598623859
11.8289638354 -3.10364318352 8.1486947927
11.9073449554 -3.47402045841 6.51607267395
12.0198263301 -4.07788761751 8.09227861154
11.5196317289 -3.99356633856 7.59513107978
12.8508407426 -3.99145556456 7.98965793425
11.413357529 -4.10424029505 5.4613626468
11.8957420119 -3.39336772544 8.46095471965
12.3799129561 -4.14052613723 6.18516857756
12.9394732052 -3.26458094447 7.48889895695
11.3204446757 -3.74670878591 6.80926447592
11.4044748858 -4.44945561222 7.16171353316
12.4586037365 -4.31756017176 6.32354351096
11.8380645313 -3.21498256968 8.3110352142
10.8610870157 -4.48823023304 6.12595693786
12.377636836 -4.65731946076 6.76703249412
11.309266909 -3.80561924789 6.32325764765
11.5743177648 -3.57178722462 7.7113530914
12.9716260217 -3.86735439673 7.83665118089
12.2607909138 -3.05809667955 7.20966877302
12.8040513943 -3.24875595815 7.32664798344
12.6655001577 -4.01624810486 8.24811225536
11.6914854101 -3.94392532363 5.74609485186
12.1461801637 -3.62828404621 6.36552405904
11.7032611386 -5.09300416558 6.54007363315
12.2720257444 -4.80579210453 6.27791384987
12.5471881387 -4.42625487946 6.83045617131
12.6323715731 -3.96618398484 6.70477121708
12.5286325806 -3.55836724019 6.75980921176
11.666239801 -4.91911814334 6.81983497911
10.8579335643 -4.43409916696 6.04682024156
11.9501587006 -3.44511104952 8.53763116047
12.6436050009 -3.35291537202 7.05078666068
11.6418047753 -3.85729454905 7.85758318475
12.5010905845 -4.45673600899 6.61222900971
11.7533521046 -4.36339888663 7.57976612359
12.854131626 -3.13785381125 7.4711330929
12.1280117783 -3.15001784765 7.06301019301
11.3112940756 -4.17498643089 7.16085053879
11.9697640169 -4.1929849064 5.61435725485
11.7131491471 -3.93268105087 7.91741664885
12.0060042784 -3.39270631353 6.65687030559
11.2642204642 -3.9364959355 5.95796054876
12.4823657009 -3.1284496354 7.17314463427
11.6779427018 -5.19395980626 6.35039987872
11.8726719753 -5.16722041826 6.25349739766
12.3002408657 -4.76417195385 6.49271126247
11.6377607283 -3.88343443575 5.82641860055
12.6012598868 -2.81496976792 7.64701420625
10.9332503937 -4.34443960967 6.22231998747
11.5799555889 -3.4894550504 7.6774980964
12.34934319 -4.58355794369 7.16696839854
12.4056504004 -3.35690713773 6.84916087303
12.2757796706 -4.75137449629 6.75130770805
11.375334827 -3.69289365502 6.57447812946
11.9504932472 -3.47185048605 6.52313828162
12.9337394266 -3.91325369311 8.03079365539
10.9632226267 -4.42682443234 6.38910867743
12.2205928577 -3.15392057206 7.04506699413
11.9564530619 -3.41739936166 6.61434401431
11.839169368 -4.36802663074 7.63414211544
12.1697307407 -4.34116343607 5.75281319421
12.6326883971 -3.34571819423 7.05652592454
10.9961360621 -4.24401628923 6.33737230443
12.69684288 -3.65168084386 6.91475329375
11.0632865374 -4.10668730546 6.37236061986
13.1634040603 -3.53292604997 7.99493655041
10.9434965244 -4.74809814935 6.31123508128
12.44074991 -2.69161924973 7.83739700879
12.6870546343 -4.21325418413 7.61783673829
12.9844081563 -3.17544754758 7.60753072181
12.4356735188 -4.54355887581 6.46712883097
11.3800605564 -3.78624281907 6.21002316728
12.874088284 -3.09443514193 7.52526722474
11.8761046299 -3.43005160019 8.41270497153
12.4230745873 -4.49793307113 6.32797838713
11.7602543615 -3.20829343169 8.01759536519
11.7242699802 -4.45511280294 7.45045704023
11.5316430592 -3.88367583738 5.84063040382
11.103742792 -4.30866079701 6.71289038933
12.1204489294 -4.68672902655 5.56539254931
11.6149736753 -5.233538339 6.29485843696
11.4282637152 -3.60029628759 6.87830369047
11.0752593363 -4.24596638631 6.61623580132
11.4477500438 -3.79218203491 6.09150355137
12.5691131352 -4.39828743663 6.91274244219
11.6260748506 -5.17766662669 6.39666214265
11.9133935967 -4.09676292321 7.98986301178
12.5202960083 -4.45307520792 6.70475728425
11.8144347862 -4.29300377771 7.70032819594
12.1795815554 -2.90383346967 7.51465646908
11.956881111 -4.62191324655 7.32877685584
11.5796965609 -3.58420284805 7.74968401833
11.6384871488 -4.86810650893 6.88599947623
12.0067128889 -3.01465122625 7.41867594889
11.460463205 -3.78863244828 6.0921284176
11.4647799182 -3.61453193846 6.60327141059
11.6371515486 -3.32633522274 7.25059287358
12.2654068355 -3.95996854452 6.13596679091
11.9499057184 -3.02191231485 7.48215253584
11.5994255452 -3.76549568932 6.05362368388
12.317720836 -4.26238491513 7.9223795164
12.942822509 -2.9771700906 7.69864478512
11.8104404672 -3.0950476429 7.93071513865
11.5616433584 -4.0487050399 7.63813118978
11.9495318313 -4.72095472323 7.15828330796
12.2635076554 -4.73250641405 6.84661404442
11.2930876756 -3.87073989453 6.10460815314
10.9194327364 -4.5920662378 6.30982606362
10.9673628121 -4.23492068645 6.17604872853
11.1752518045 -4.10214555822 5.61856924586
12.494003463 -4.35155176737 6.44696259797
12.5100229642 -3.34531813362 6.94419918796
12.1849721524 -4.50701986342 7.49729536146
11.596678205 -3.48894620555 7.73475052963
12.4840019754 -4.4104651016 7.45845619665
12.1027727413 -4.85505274452 6.78074782209
12.6698255541 -4.20051795562 6.90441667856
12.7481651045 -3.93419403442 6.96559490873
11.4737246606 -4.0944595726 5.48229238405
11.7931789349 -3.62240502901 6.26850812605
11.3464870436 -4.02718672393 7.25357034368
11.3201448966 -4.45811213374 7.04333712269
11.5306488899 -4.78921779913 6.9331673574
11.0004548973 -4.16976174253 5.95015191337
12.4256222037 -3.97867124159 8.39822301677
11.3895062267 -4.72821127102 6.89379107377
12.7051499208 -3.98209603575 8.3029149033
11.6737382354 -3.62698260603 6.29550489059
11.9478251487 -4.69153558316 7.20302273492
12.4788657345 -4.04359877097 6.3962959467
12.3443133548 -2.70838922157 7.83811441319
12.4948565542 -4.05442615642 6.41459530539
12.546784217 -4.01670714477 8.31927951942
12.502605951 -4.44813955211 7.25971924328
12.8595413623 -2.92646322192 7.66807944744
12.4999922948 -2.76062008495 7.70640177456
12.0377164451 -4.19914497289 7.95812914587
11.402044162 -3.78531830514 7.32992517697
11.8552417412 -3.54857315617 6.39553760354
12.7690589008 -3.80407202007 7.0064150551
11.6764403625 -5.21367811603 6.31128332372
11.5101338709 -3.82054260995 5.98204532509
12.9889646782 -3.74599660503 7.5901385268
12.1459767474 -3.03060522567 7.28056510178
12.2806640916 -2.96048040422 7.38297219735
11.7484789472 -3.17869258871 7.8251283666
11.831540487 -3.47180631979 8.32201186462
12.176594855 -3.34912365517 6.74663273129
11.6686822996 -3.58202769446 7.96846286396
11.5841565324 -4.13527869516 5.43624470061
11.0012598815 -4.39476988714 6.48004725654
13.0547261677 -3.70258785709 7.80929445864
11.3312773743 -5.11143828386 6.42407263985
11.8316867394 -3.75489796179 8.19188342732
11.9673764533 -5.07632661326 6.37280928866
11.3775026522 -4.06812767813 7.30659276313
11.5316078159 -3.59859032912 7.60892319037
12.6143846782 -3.18097312893 7.20086970147
12.8564811051 -3.48094371587 7.25138186552
12.7387011023 -4.04428210234 8.09751465397
12.531117983 -4.43316404947 7.16083917993
11.135525081 -4.98762238663 6.41148916942
11.8519451417 -3.37447759378 8.37620101135
11.9490375252 -2.9149168915 7.9279988212
10.9864322157 -4.59702277837 6.44861374211
12.3413061313 -3.11912666926 7.13194658231
11.7864412942 -3.11920964547 7.79063531867
11.4259633632 -3.77799837444 7.40515295443
11.9300153216 -3.39504556745 8.51682552329
11.7860133788 -3.53415278564 8.21049592969
11.8246715727 -3.50106405037 6.48554800656
11.9014675324 -3.63850340479 8.36867174939
11.8448350881 -3.50634746772 8.32846902744
12.1708874503 -4.93683221972 6.1937960167
11.8558886988 -3.03856198662 7.7176616672
12.3379832593 -4.42407333622 7.59464835221
11.8786032056 -3.73600614114 8.27201586187
13.0060598188 -3.37774559895 7.54746403438
11.9362053747 -3.40148942258 8.52586076962
11.5476872446 -4.49741712812 7.27347445328
12.5647656459 -4.39512346626 7.23503796123
11.9774438155 -4.22010035546 7.89996522638
11.7198932355 -5.25498118951 6.20189093803
11.6648966378 -3.3711386112 6.94713808033
12.3803278636 -2.90150334198 7.47242391243
11.8900420386 -3.08085577287 7.40974195912
11.9561031911 -2.89059845933 8.06718899797
11.5250674617 -3.59415335584 6.52425518437
11.0018784902 -4.53493540527 6.48679968979
12.0813508977 -3.41630712504 6.62065179415
13.0461888638 -3.25286043203 7.66580729134
11.4135350385 -4.51748779814 7.11775543154
12.0603293987 -4.35497246822 5.58573888075
11.9125264866 -3.57080172452 6.35798376327
12.0006985403 -3.90095337574 5.95242207298
12.1057118373 -4.11977531089 5.82049801118
12.4815358588 -3.92020978696 6.45291373924
11.7593657439 -3.4130440429 6.70056500034
11.9302086713 -4.58051898377 7.37785879491
12.1806804396 -3.74455446422 8.5874162552
11.6663179546 -5.11696184737 6.49368594505
11.4347164199 -3.59273891771 6.92377214604
12.2058300161 -4.79929137715 6.77118034482
12.0810306601 -2.92853748174 7.55715772376
11.733409673 -5.31890615459 6.03707976984
11.925107317 -3.01430578938 7.57241255035
12.8442806595 -3.7929205848 7.17773622667
10.9536434196 -4.49645599004 6.38369515989
11.6492853189 -3.37150371222 7.78945341495
10.846175745 -4.60055581257 6.12874767806
11.7052402289 -4.08540049511 7.79554816218
11.6330556084 -4.88134291244 6.86175217351
11.3648209376 -3.71245489022 6.52049722864
12.3371818697 -4.61498148934 6.14164838329
11.1919584454 -4.01112401653 6.81808068371
11.0759908396 -4.50438629962 6.62977575675
12.6763420748 -4.18329277461 7.76858523639
12.200669559 -4.57177362482 7.35977855294
11.3930862947 -3.64327906396 6.86250234221
12.5663163726 -4.30307637956 6.64747450672
12.1036459196 -5.02869691755 5.99970988027
11.1835345601 -3.94469255146 6.62549117076
12.6551778766 -4.0692026389 8.14274146553
12.3470936969 -4.11490158417 6.14626001054
11.0975138189 -4.27710620649 6.68683110039
11.4363793469 -4.24898706921 7.32837448539
12.6457298504 -2.99134005011 7.43349325302
11.6794628702 -3.98764171958 7.83551648595
11.5536132832 -4.17129844763 7.54958611983
12.2566759099 -4.70998147908 5.9446274203
11.0510704869 -4.10352289288 6.0306886251
11.2562137184 -4.54208980351 6.90078238196
11.4460585894 -5.25417499923 6.26319947169
12.7206798539 -4.19196589921 7.5592076172
11.9176870201 -2.95412486655 7.88928962629
11.4070142217 -4.10934031509 5.46113624209
12.6784465749 -3.20079167318 7.23524308786
11.7551205001 -3.33075394901 8.12644446314
11.675455821 -3.61787652693 7.98200926315
11.6541889683 -3.40401401675 7.84972850942
12.3242684614 -4.34148038488 7.7630779051
12.4004101769 -3.44295170541 6.74435422853
11.7516238393 -5.19217668843 6.3123912116
12.2427631309 -3.82528512523 6.22533002374
12.8879523866 -3.80168713465 7.30003993461
11.179701688 -3.94990049288 6.16414744539
11.9051745169 -3.67103912631 8.34901567911
11.6015491455 -4.47484275443 7.34036253824
13.0705334659 -3.1923305496 7.72374370918
11.2669882728 -3.84559407284 6.82490672803
12.8852067974 -3.70594675185 7.25947321121
12.3501863693 -4.25935838467 6.0903232866
11.1337042075 -4.35926580943 6.78367559747
11.7394810098 -3.18938886897 7.82342095399
12.2091711123 -3.00766931495 7.30125427423
12.4306122442 -4.14905225069 8.09411614989
11.7260083378 -4.41431462139 7.50907343955
11.8254732271 -5.20999969257 6.20160059834
11.9081214029 -3.49883774571 8.45259286468
12.3103159801 -4.40015206561 7.65289202745
12.1530254466 -4.8933345361 6.57009280625
11.8101403868 -4.24793942713 5.41158011957
11.9723428409 -3.60922891372 6.30567712501
12.1477858815 -4.91232138482 6.5133851128
12.5355417967 -3.96127357059 8.41846907131
11.8158523538 -3.10340219144 8.04150176701
11.9845011 -4.14203794903 7.99884865815
12.3431086438 -4.61292368529 7.08702223602
12.1396410563 -4.93752121704 6.42343004966
11.9213050149 -3.63041197352 8.40034542014
11.6356231566 -4.5804767338 7.24894751324
11.0554736953 -4.59833114953 6.5719797525
11.7234160756 -3.23636123045 7.89256625309
12.294054451 -4.67171789944 6.04101137317
12.8270840498 -3.62574045306 7.15474363145
11.0980683472 -4.08272802649 6.51753366327
12.3806976 -4.66166993526 6.66930144909
12.5817653956 -3.42180396384 6.93424693119
11.9998891699 -2.96805185808 7.56664327667
12.5744956021 -2.98229634045 7.39816102579
11.0901186326 -4.07273885191 6.42005820271
10.9347483446 -4.87209317368 6.24731418351
11.6632012937 -5.27354003385 6.19365501052
12.307473984 -4.6609995314 6.99921499985
12.9373063228 -3.91964374695 7.89130254083
11.610101807 -3.44037985046 6.83405696319
12.0663461603 -4.32930590169 5.60927319619
11.0437382108 -4.13411517147 5.86121374431
11.7456884831 -4.85331099888 6.93710888022
11.2376346634 -4.50066729025 6.90095607656
11.181647154 -4.03609610468 5.80286634259
11.8042453201 -3.12677633019 8.03068189622
11.4156203847 -3.66900994177 7.25160268154
12.1188946552 -4.9477649035 5.71926313776
12.7062153868 -4.20771846619 7.10790066365
12.0771735571 -4.62672719343 5.47900295024
12.5722468843 -4.38790094887 6.82964095379
11.8082091875 -4.2118424967 7.78061750207
11.0563718341 -4.14591302333 6.4551764745
12.866117331 -3.24023500101 7.40503043044
11.7310289396 -4.3356997973 7.59621803015
11.4652130538 -3.82817384218 5.98990206629
11.6731472967 -3.65364566132 7.97471560494
12.5408942656 -4.23534460314 6.50684234372
11.7400497711 -3.38298111749 6.78293405843
11.7707858528 -4.28734245816 7.6824000803
12.7279297334 -4.1350850268 7.06538325557
11.7704682507 -3.16087054526 7.94929567798
11.1491041927 -4.02226015302 5.9564058199
12.5480807237 -2.97398608367 7.4083529665
11.4065930555 -3.8313780937 6.03459776231
11.9117363827 -2.9507870184 7.96240558771
13.0262882288 -2.98373957598 7.79568807548
11.6020892367 -3.61674175656 6.36581789933
12.5023693828 -4.46902867982 6.63721849169
12.1723906115 -2.83979693751 7.67302993794
12.1330562872 -4.94281116845 6.41188849452
11.906260906 -2.96857112148 8.19423561208
12.2477494485 -4.36138765353 5.87290422909
11.5953521898 -3.37332694958 7.25254568996
12.0959627622 -4.9774232036 6.40644938463
12.8172166819 -3.90292651461 7.13966807491
12.0469849334 -3.38917786792 6.66383048764
12.0960210733 -4.85629949069 6.79712446091
12.0171737519 -4.05992144504 5.78841825362
11.4973935849 -3.74182076744 7.57617523372
11.8926576864 -3.12784943871 7.27198065563
11.1061542117 -4.08029123065 5.84861475374
12.0958498721 -4.80550089592 6.91443246776
11.6678537655 -3.29648236267 7.69231201371
11.9192059879 -2.94950310997 8.1892789084
11.0598918238 -4.10207271863 5.92912545661
12.5263837615 -2.7815850304 7.67802125986
12.6745322973 -3.36784041237 7.07902284798
12.5341630194 -3.54218496538 6.77482916116
11.4871457054 -4.69034482102 7.02373052705
11.6256294535 -3.34576873636 7.23627915582
11.6447244147 -3.34721093418 7.73699276791
12.9322596484 -3.8947560221 7.6151962223
12.7792732132 -3.42014916661 7.1672159915
11.5434830008 -3.83279490102 5.94100141111
11.5137682933 -3.97813205979 7.58188845168
12.6932142912 -4.14524451518 6.94737409242
12.407025179 -4.27504209516 6.20679589902
11.6188245127 -4.35457211218 7.47646151662
11.647751343 -4.64808017481 7.17810403413
12.4726756398 -4.24862924561 7.88508533884
11.3598630359 -3.69816339446 6.66677544275
10.8763744037 -4.33892144279 5.88205778465
12.0905386544 -4.95862882524 6.49262784491
12.6430719321 -4.15805610502 6.75782874516
12.0673034307 -4.08423403768 5.80966848808
12.1830086999 -2.86872040942 7.59242334724
12.5641162663 -4.32971431847 7.53358351868
12.1611111061 -3.75644049688 6.21950820404
11.7152813751 -4.81216681074 6.99000399765
12.5081397142 -4.48157003372 6.80007165725
11.792608515 -3.17835728443 7.31643751154
12.8169447726 -3.34238285521 7.26807860847
12.9662811102 -3.83412802434 7.64984514321
11.9948193744 -3.09504010468 7.23584701824
11.6291495421 -3.97686624967 7.77631339753
10.8685426657 -4.36909162583 5.90721556126
12.0447404396 -4.11504283303 5.76180293948
11.7051655774 -3.69033178137 8.02555033227
11.2198117297 -4.63873105746 6.78121930026
11.3227044897 -4.27911860797 7.14445199004
12.9611819022 -3.86697839202 7.71958659746
12.3762046754 -3.94977904792 8.42707542442
12.4676868397 -3.89582625515 6.44112926512
11.8656769742 -3.01911656484 7.8185032173
12.3546477017 -3.38081546654 6.78916840931
12.1556529572 -4.35757583777 7.75758052532
11.2157732839 -5.03473436489 6.43590190328
11.0789382141 -4.22467435741 6.632170117
11.7755736742 -3.46717849036 6.57235123207
11.2863946538 -3.78797130768 6.75315151042
12.9982116429 -3.76124989479 7.64141354617
12.8677874651 -4.00130463398 7.84144571115
12.8223689223 -3.07118308168 7.49260832279
11.3039317178 -5.07936765585 6.44696289653
11.7599296749 -4.75855423774 7.08017062898
11.8232766817 -3.23260737401 7.0664319174
12.1801284249 -2.85317604417 7.62775613017
11.7176057225 -3.31217221623 7.96978572011
12.0761225579 -2.76752905102 8.09549097693
11.6127446002 -3.39403714179 7.65144675547
12.5908947537 -3.12756748658 7.24165333884
11.3211884099 -3.89697567984 7.14388994853
11.3110618775 -3.90009955499 7.11728751874
12.2036540374 -3.80919809493 6.20372068596
11.0624963797 -4.12890116504 6.43221514855
12.2112541005 -4.03067469718 6.01263864829
12.1156714099 -4.96959451677 5.76230085959
11.8456545295 -3.49771212051 8.3401525981
11.8555434619 -3.26353937436 6.96132283622
13.0636421068 -3.67433947264 7.78378364687
11.9268887556 -4.17490019268 7.92370870472
12.7793195379 -4.0843357483 7.83700018639
12.4732264243 -4.07109083506 6.37507366371
11.9122682768 -4.26429489278 7.80781117416
11.8476611135 -3.15700824028 7.25801064326
12.4246325934 -4.22385213576 6.24307202507
12.8853997176 -3.34958837534 7.36158562686
12.5866796197 -4.29114079797 6.70666794775
12.6361591289 -3.58586442373 6.86395426417
11.4955165842 -3.61835959195 7.50029739768
13.1600081436 -3.34811675026 7.87163906327
11.3508991889 -3.88246491087 7.23851316487
12.3687921925 -4.66726145304 6.46317068675
12.8926573478 -3.90986222254 7.42965173587
12.3373608199 -4.5366213347 6.06492865509
11.3037843705 -3.88866469176 7.11250714764
11.6961675751 -4.24979785362 7.64881075586
12.2249279359 -3.44773664235 6.63106822959
12.4844890383 -3.30097503822 6.96842950244
11.0507373965 -4.0999003586 6.07279176574
11.8785068258 -4.10972574299 7.95095626526
11.1696354182 -3.9625250486 6.16147967353
10.9804534411 -5.03464851723 6.20470037138
12.149559213 -4.29924659154 7.84558277425
11.3064472436 -4.02220852309 7.15501097938
11.9265145414 -3.32483571401 6.78978383416
12.1669117708 -4.57994439659 5.6715293107
11.8086403399 -3.37909873022 8.29200750568
11.8533108019 -3.96433322947 8.05953257257
12.1781209843 -4.90824960154 5.95436363664
12.52209714 -4.46730314576 6.80944096826
13.1244037875 -3.4350572062 7.80257530224
11.8634256951 -3.83522805716 5.95079378373
11.7906813247 -3.75382731283 6.05810784342
11.8859680462 -5.08140578543 6.46600169712
11.7015072797 -4.01485790078 7.84830766042
12.5783242501 -4.20150304098 6.61243690162
11.4422733956 -4.37096937467 7.26158778569
11.6687412002 -4.46053279439 7.4116493028
12.834717167 -3.71412000461 7.14810717088
12.2971127547 -4.77091650647 6.29663209257
10.988700592 -4.18236030337 6.01869766187
11.5320164356 -3.80096605108 6.01122555936
11.7705911747 -5.1559431862 6.3883188162
10.9806082267 -4.58774607174 6.43597042767
11.6299659416 -4.88891873596 6.85678211253
11.8116783453 -3.42171283722 8.28267907744
12.4050421862 -3.08334442545 7.1991802251
11.6156716556 -3.97368371537 7.75256440434
11.9648139438 -2.88541683691 8.10844431022
12.1605425234 -4.69480512425 5.66771471578
12.0617827762 -2.84860309212 7.80957894237
11.6792427122 -3.39002575601 6.86196249284
11.3593270268 -3.69532779993 6.82739078664
12.312215222 -4.68116488884 6.90970911736
10.8712122919 -4.53728052063 6.17030365926
11.5263351623 -3.92874686043 7.63486430702
11.4874397853 -4.03829246543 7.51628331653
11.3260621797 -4.85669257151 6.71800192702
11.6703851772 -3.52759327677 6.51155357307
12.2655534168 -4.30969125893 5.92653804903
12.5006182768 -4.45005709443 7.25095802315
11.9435124693 -3.66461028749 6.21424321998
12.8329475833 -4.05668282234 7.58808379886
12.2595560856 -4.72434650317 5.97112419054
12.5321011028 -4.1150081496 6.4736985391
12.1747480359 -4.84728821838 5.78927005542
12.800561666 -3.98365526184 7.14807261593
11.7360168374 -3.22584141856 7.93656791998
13.1393432488 -3.25616432181 7.83164717703
12.2995674182 -4.21473360635 6.01105788512
11.4465511713 -5.29471241115 6.19868236571
12.1047115188 -4.9835388765 5.74877681928
12.4071690635 -4.15660787241 6.22636325508
11.5321226633 -3.45874924534 7.10520511485
12.1419068473 -3.11565332793 7.1211614199
11.6431479262 -3.65049879258 6.26569618699
12.4825582424 -2.6980687408 7.81256455149
12.3324522098 -4.51954734551 6.05635684953
11.6394065296 -3.31677367868 7.4807979
12.2367523558 -4.34202520887 5.85989831839
11.8859457567 -3.25653885138 8.45765957175
11.9678293492 -3.39489880329 6.64517817833
11.6603088895 -4.08065301311 7.75622998261
12.8588072108 -3.29988408113 7.35434883807
13.0195167126 -3.41594069863 7.56884761247
11.6525545401 -3.32249223282 7.71342263877
12.2322199432 -4.32071851446 5.86460493179
11.6679227362 -4.13610690086 7.718591446
12.2191774924 -3.26763161719 6.880305847
11.849017927 -3.30998208541 8.37412700909
11.7332256368 -3.24049637351 7.25581240071
11.3599107538 -3.68848360245 6.89608919702
11.2755942332 -3.83600060172 6.89355662034
11.6645437644 -3.28482459368 7.60802872847
11.5910258193 -4.81041167947 6.9468555516
12.3614053444 -2.80865976904 7.62972947033
13.072563278 -3.09710384685 7.78004221538
12.817118107 -3.97831990177 8.13597410338
11.5984817798 -3.61526574449 7.80607360535
12.2085289664 -4.01016148142 8.28547375445
11.4591390123 -4.46099631492 7.21208925122
11.6853813202 -3.2530356799 7.64768070075
11.9039570003 -3.29851670582 8.48824342443
12.8280249929 -4.05991169082 7.47855246859
12.1679545002 -3.73045620001 6.25362045462
11.8019309891 -4.85240236083 6.94421377981
12.4084404024 -4.40996732005 7.55549307026
11.4109413427 -3.88544102554 7.39398041194
12.3405623896 -4.44117956569 6.05414501881
12.7267632646 -3.94083462651 8.37682226168
11.2181431431 -3.89410610866 6.26835415072
12.1170279329 -4.11955050994 5.83179323305
11.736561664 -3.40673183735 8.10398476286
12.3591296862 -3.74315894964 6.41423295889
11.7530891487 -3.26410882782 7.11090785278
12.9281842797 -3.08869679185 7.60571492369
11.5574969408 -3.43038678081 7.19029417672
11.5985595444 -4.0020495681 7.70981155265
12.5523455309 -3.20478783175 7.1271353156
11.7743260889 -4.29413927352 7.67054959593
11.7449123761 -4.14940061411 5.48299261672
11.7717736658 -3.19756471196 8.01306036656
10.8458878343 -4.51384200088 6.08031074068
10.8424180748 -4.63315908791 6.11851715324
11.9735008101 -4.77569845368 7.06361292669
12.0525931589 -4.35265420626 5.57690620018
11.883143729 -4.8351673791 6.96635574242
12.5962538602 -3.99226293435 8.35019648229
11.9042350518 -4.22796226191 7.84242889913
12.8996898174 -3.47120904741 7.33063511655
11.4521105145 -3.71022370678 6.30730091255
11.1887509821 -3.92219874775 6.30298523129
11.1870109806 -4.43107035066 6.85717764007
12.2641016154 -3.23659037492 6.93445196394
12.1112371232 -4.99474506251 6.24387217798
12.7035759534 -3.18428405097 7.27435324053
11.353581663 -5.14337379127 6.39649407778
11.9683226441 -4.86274569151 6.88405219936
11.1521228136 -4.00118198551 6.59989271924
11.889036148 -4.80372957719 7.02598479545
11.6151542741 -4.80364089845 6.9700259296
11.539374217 -3.55277698627 6.61801013716
11.9996568033 -4.03654125674 8.13217632644
12.4023371655 -3.97036102793 6.30338348083
11.5210180334 -4.69975403715 7.03410353294
12.3958229732 -4.64155848489 6.62309920287
11.6514084479 -3.97628436125 5.68878560464
11.3766849941 -3.96692277327 5.7457414457
11.7275499689 -4.69754167558 7.15610912042
12.3936948003 -4.55435034132 7.15656670604
12.2279718089 -4.59145792581 7.30008837358
11.4987839119 -3.69980854966 6.27681936959
12.0324509461 -2.86927457758 7.80971542246
12.5511436248 -4.41393669093 6.78102890532
12.5673439828 -3.93095121185 8.47226758628
12.1446012865 -3.21270702248 6.95145647996
12.7219506358 -4.1902141706 7.18907984294
11.6982313136 -3.35472570193 7.96024986499
12.9127951335 -3.83501605282 7.40710798947
12.6979756313 -3.15014367858 7.30688326462
12.3986698063 -3.99771113363 6.28394813922
11.8002265651 -5.16252323515 6.34167678376
12.0530647848 -5.00672600985 6.41566964222
11.0381980293 -4.6064445707 6.53089331688
12.798312404 -3.41695638432 7.19682265054
11.8530806804 -3.11515175038 7.38369184721
12.8541660797 -3.90785831832 8.29416087132
11.0571177479 -4.14007493172 5.76014008948
12.3209906769 -4.12939416538 6.09208176572
10.9199567227 -4.34295951949 6.17714580631
11.8054484806 -4.23173172178 5.43217562176
11.6751090301 -3.43351411294 7.95135607164
10.8518178181 -4.41427803748 5.96478642107
12.2948069602 -2.68768000924 7.91275850911
11.5732320957 -3.8314642061 7.74951968861
11.3865878176 -3.66232293059 6.70229690126
10.8714892947 -4.34987152911 5.84749917809
11.3433393647 -3.79381812324 6.24830155823
12.3154110564 -2.6806514794 7.91274599718
12.5644715139 -2.92755982902 7.47099276931
11.4090870068 -4.06521181941 5.53748875946
12.7273581221 -3.98761365497 6.92746023242
12.8503688675 -3.27006675946 7.36501299868
11.592020058 -3.82255492333 7.77668883176
10.9589661834 -4.22246468884 5.89575034828
11.5533733595 -5.32396252415 6.14688614032
12.4696631535 -4.38319869503 6.37992708678
11.6052952035 -4.58353954878 7.22590512948
11.9976693429 -5.12945058437 6.08846904489
11.0456728965 -4.41716449553 6.58157044972
13.0739871489 -3.73587880863 8.1133666203
11.8101073098 -3.84289962398 8.1132189421
11.078701919 -4.60163759668 6.60524193086
10.301137181 -3.23861957224 6.65312573241
11.4296643654 -2.32438906199 8.4386560293
10.8031074768 -3.00185753424 5.08308044477
11.1592948308 -5.98097527192 7.68400991235
10.7649475432 -4.0011609582 6.21701111032
10.9065716774 -2.98504803502 5.95352202285
12.5118736504 -3.63885469434 8.65725149172
10.7382132798 -5.53338382404 7.96277985522
11.4455151091 -2.85528394934 6.19487618793
10.3636369738 -2.35172514399 8.03389254302
13.9145157505 -2.02757045677 5.74258515823
10.2595333244 -5.04984759994 6.06403651514
13.1387769264 -3.2266292494 5.89604038695
12.7334812072 -4.50737213817 8.78904671916
13.3738922724 -5.55086012818 7.2083206501
10.6624393144 -2.52243647035 8.86349455131
12.9382890324 -2.932968209 5.71138144295
11.4735868783 -2.0229038808 6.44746208851
13.2499657311 -4.05639974145 8.44414590206
13.8188864569 -2.6148263741 8.6620876875
11.4502575925 -5.46620481749 8.05585743562
13.4981212254 -4.31751662346 6.50068307668
12.5963224067 -2.16278799312 5.49767067895
13.2474709126 -2.82470667561 7.80670269429
13.1921893972 -3.68084073615 8.30955343888
12.3504704543 -2.5077935455 8.76525037617
10.2037848012 -3.52830116308 8.91104407111
11.4667239105 -5.91168854816 6.57384398155
12.3438264321 -4.97544673133 8.61708825881
10.0295853826 -4.81467812508 8.24508518678
10.9918955261 -5.27233778706 6.22327361011
10.7744135879 -3.06038385376 8.45371623184
10.9154621178 -2.57501622997 8.4199644544
11.5989572529 -4.40801441471 8.52222565856
11.1042785871 -3.26493857313 8.79276997947
12.2768386308 -4.99971317374 7.18194662117
12.2876625042 -4.07569508144 5.83285123264
10.0856528488 -2.65979219197 5.01203691945
12.738418786 -4.85480654162 6.22269068252
10.1987169263 -3.73065610226 5.79193925932
10.5776109076 -3.85913336554 6.51842238598
11.2833612323 -5.00129415251 6.73344557627
12.9759783708 -2.13599284254 7.04634842834
11.3056648699 -5.25675926456 7.95578911197
10.7970915399 -5.34993501616 7.35822173903
11.971773399 -5.77497925302 5.00146607115
10.0187558663 -4.9284247147 8.32031433775
13.2363841138 -4.82097402461 8.74898199347
11.7200909747 -3.65392230618 7.99257039701
10.4540688388 -5.26433937683 8.40482872929
11.0392689422 -3.02357190199 6.80403792683
11.2924109454 -2.21467401692 7.57257631577
10.3220056121 -4.85761967627 5.93654558999
13.7768926723 -3.44781696588 5.85732531078
11.6497730641 -5.50530470523 5.02884755791
12.1314349163 -4.37535473134 6.39673835598
13.3334529289 -2.88959753705 8.4207332512
12.6903452195 -2.29494574886 8.36009158804
11.6466295965 -3.09156540463 5.82088058476
10.2985184537 -5.99270819972 6.75635376155
11.0686732716 -4.51908540263 7.80839872145
10.135949353 -3.2685358921 6.54367722938
10.1537511017 -4.03003069125 8.00332937394
10.6293421292 -3.15795116076 8.60488529743
10.890081937 -2.31326972911 6.00125039436
11.4958130958 -3.28569096107 5.6822717407
11.7018110673 -3.38375410924 6.24095562909
10.3258717049 -2.94117246986 7.85692136701
10.8791197891 -4.57189086848 5.84007416665
12.114354376 -2.57480725164 7.66263136817
13.0955437945 -3.4441312201 6.59512563276
11.0381932795 -5.6290829174 7.77599361393
11.3604650204 -5.87009831604 6.42988389968
13.5723684988 -3.45424162094 6.69730524101
10.2247977279 -3.00570058973 7.24663858386
10.642459317 -3.37711910099 8.14553543572
10.8065985296 -2.24219846209 7.25645609311
13.9778857906 -5.00343750938 7.53147621035
12.0920939533 -5.35676130751 5.51395208121
10.3535737162 -2.44473571436 8.85535419374
10.1568178363 -4.23087987314 7.38703126919
12.2392200954 -3.63785514212 7.66916808418
10.8493324003 -5.95644267233 7.84476692003
13.9693840734 -2.39704818834 8.86002698063
11.1397644949 -2.95387268168 7.66874674919
10.2007410015 -4.18153757749 5.87513362802
12.9190915393 -4.9669405954 7.21050016372
12.2606930818 -3.68953085128 8.62867685694
10.4207213983 -3.19402247775 6.92899726928
10.4537366789 -2.26818022294 6.39106546734
12.4740504465 -4.90552106135 5.47405823131
13.3302501982 -5.6092026703 7.88378451064
13.8892503544 -5.07049681971 5.25161227682
11.3751614765 -5.58406956309 8.9845455074
10.4248509612 -5.32512343901 5.16070530212
13.996275486 -3.59764531574 5.32044728365
10.0445462359 -4.29693268544 6.07881034125
13.3440204389 -2.845433385 5.98443269185
10.0127356481 -4.95607537838 5.60699321235
10.9202562513 -5.11228126005 7.67023901576
11.7334162199 -2.3854293179 8.26010746546
11.9155919281 -4.10438187723 7.02572972409
13.8932607535 -4.31810638943 6.72499389881
10.6363632349 -2.40431918867 8.17168665365
12.7005056454 -2.87140233229 6.51809519337
13.3059142496 -5.17703814726 5.00239541197
12.6884479006 -3.00204978661 7.29202427118
12.1510243352 -3.81653111766 5.15169035606
12.2426102691 -2.54542507697 5.07958718408
11.0320767038 -3.92712739452 6.58859484869
12.7376315199 -5.3813564041 7.44740033322
12.001597886 -2.86529771414 7.54464889791
12.7401470072 -4.85363245931 8.66089953857
11.4365971882 -3.53429071343 6.33504342013
13.4048675306 -3.07916632268 7.30126742163
13.7265023586 -2.34939263752 6.46733240318
11.0542179645 -4.70707276986 5.97938176701
11.1446136346 -5.57217053496 8.05747936911
11.4909960162 -2.56952416528 5.03620664687
11.2184127776 -2.15850983555 8.47787839071
13.9778030038 -2.11640122971 8.5538271938
12.5781551397 -2.13637072738 5.58106941342
11.8356807122 -3.71753472821 8.12583933979
12.3440354036 -5.2021706277 8.48244069547
12.0307501302 -3.42750928766 5.04210529008
13.9420879398 -5.89365400276 8.94585229713
12.5029870201 -4.1257191385 7.57841480218
11.899854405 -2.25055350653 8.2587708327
10.0526804897 -4.28519011749 6.28498909676
13.1172623228 -4.72080514135 6.3989372206
13.7778049136 -3.76475480635 6.33257147285
10.9542504493 -5.96384197504 5.82489080488
10.8011728275 -3.01709615253 8.83389608342
11.5412094865 -5.74859292466 5.08296995403
13.9414749409 -2.46601819791 8.50902431129
12.8663162675 -3.97190610985 6.29389901043
11.4030707253 -4.05069727848 5.79796763793
11.6966477254 -2.21628664075 7.08072644413
12.0732290319 -4.06169343571 7.02348458574
13.5703571303 -3.40381646871 7.94567269827
13.7061877808 -2.96667630167 7.8261130006
13.3539758333 -4.34163250439 7.44780229572
11.0530552914 -2.50875202347 8.27733723138
11.9001672906 -3.40754150532 8.83213222824
11.2689915986 -3.69487715818 8.50311691553
12.0539660846 -5.67802075335 5.79526189322
10.3502217547 -4.5333196314 5.25172630342
12.9525724887 -5.31346960572 5.05198101422
10.2016778353 -5.85021864261 8.30228181434
10.1512652548 -4.50809994994 8.36939494893
13.3117861946 -4.8865874069 5.05640597546
13.4011265061 -3.3916334264 8.66283687055
12.7686685665 -2.97200406133 7.47345316396
12.7059818641 -3.51265241813 5.47418097877
10.9925943542 -3.09860989624 8.7365661689
10.6359132669 -5.24678821441 6.25600868267
12.2502696489 -2.47204170752 5.63300906294
13.810760082 -4.06717698585 5.21060212301
12.0724608753 -4.58272535724 6.60092278265
13.5956728345 -5.87794782826 6.17681354743
12.1884164888 -4.58648178432 6.97412518075
12.7414315079 -4.15648238334 5.36072255946
12.2135814003 -5.76975097548 6.38707226323
13.0094094438 -4.99445631774 5.91948261595
10.7558390377 -5.91438049842 7.30723477882
10.7289443654 -3.07541963728 5.27450337265
10.4776951294 -3.49577907591 7.46268584756
11.1361605471 -5.39595621599 8.38150796974
12.4747387863 -4.09875752088 5.540066528
11.3360283394 -3.53409429739 5.5862787605
12.2347934813 -2.5411380891 6.62517380766
12.1527167946 -4.35236805584 5.4871750577
12.1874612656 -5.66850600778 8.57412072435
13.7091796205 -2.32116039489 8.49191149843
10.9863017134 -3.58983760191 7.98718426833
11.7367532963 -3.21364426274 7.53464442533
10.2825136523 -5.80715302389 8.6067570179
13.6126715137 -3.12059821618 7.09967011315
13.1865468972 -2.346083922 8.01198446832
12.7181416164 -5.80973937506 6.01282342516
'@

Write-BundleFile 'samples/cylinder_B_rigid.pcd' @'
VERSION 0.7
FIELDS x y z
SIZE 8 8 8
TYPE F F F
COUNT 1 1 1
WIDTH 1780
HEIGHT 1
POINTS 1780
DATA ascii
14.3708926716 1.47272494493 6.61406893444
14.5016218066 1.93456556925 6.95642864576
14.3371526375 1.23864314143 7.35765708043
14.3301515332 1.27413260927 7.37846543031
14.3146346299 1.62841704219 7.78977393145
15.668730762 1.84932117162 7.84526690401
14.2784373299 1.42619174257 7.1659312075
14.59612268 1.68678155134 6.50469785376
14.5901648205 3.3928519138 8.76499771651
15.6679039764 2.88700602114 9.04746233525
15.455123973 2.1619335473 8.95495247128
14.3652103932 3.0547489911 8.71017562652
14.3550523915 2.23577088979 8.76576827407
14.796182193 2.11770985374 9.18740630186
15.2265194932 1.07263231041 7.75132355161
15.2004655444 1.64878769665 8.52950359416
15.2420736002 2.75055775524 7.82910875134
15.6524626593 1.62213809495 7.08564215476
15.6672201905 2.6216394871 8.84135363529
14.2985912586 1.31950826754 7.27917033728
14.6629342991 1.76543978973 6.54277358787
15.6677523728 2.82049281899 9.02358795197
15.1426807684 2.02748977139 9.06015360359
15.0272515143 1.4832584081 8.37414475568
15.6180722208 2.51983546674 8.1445869702
14.3494220386 3.15657779634 8.88495736543
15.1765217498 3.09686283265 8.24499497509
15.5871070704 1.74778601496 6.99414001162
15.6637269653 1.48307850694 7.43938047237
14.2919586113 1.97908092269 8.14434086864
15.6780406892 1.19989413101 6.79886816869
14.4262232936 2.26142671707 8.97892676495
15.5459926533 2.4068996962 7.77542278969
15.6495724959 1.77500445302 7.91815544489
14.9953536208 3.32023035876 8.4876535758
14.4225512367 1.83868827408 6.96915914938
14.6785257533 1.6022367884 8.42380066482
14.7285082211 1.23660441227 7.97337939664
15.6111637498 2.24237282481 7.73164250384
15.1837032287 1.95892147011 6.7216031698
15.5277548629 1.43022090443 7.85203082057
14.7305438622 1.85737462818 8.80544556549
15.1890913215 1.11415676383 7.82360361443
14.2762860253 2.14797582412 8.22171941503
14.5014795584 0.89187919433 7.2670236231
15.6550958727 2.64925448289 8.49820848212
14.7044695107 1.38526256006 8.14987341816
14.9214280061 1.48369065865 8.37191417501
14.9094461813 1.78576558609 6.45385653579
14.6605943882 1.79849998223 8.67902972846
15.6202084203 1.01149155044 7.04271033155
14.417601497 1.83900915044 6.98333755577
14.558111717 3.2521616295 8.62246681612
15.5937483738 1.97779960543 7.33125957461
15.131841664 0.70432341642 7.30762472342
15.2687990419 1.08697703594 7.74053894251
14.3538821411 1.45591868819 6.64041649153
15.1591962706 1.25047939857 8.02530793337
14.6799871883 1.75778918671 6.51669299067
15.1595795939 2.74434524309 7.76301669553
14.5234488301 3.36835063708 8.82146547329
15.5614591698 2.43841140087 7.83600574985
14.3085447215 2.18592802014 8.50559976406
15.1033455355 0.899103914485 7.5782182957
14.2869910521 2.12142342622 8.29431727503
14.3768840616 1.50407085801 6.64458965347
14.6081728757 2.26192246665 9.24031609742
14.5984987993 0.735070214576 7.19416785465
14.2919716671 1.35968327259 6.83185066556
15.4662598456 2.59778057816 7.85763534213
15.6338538501 1.5754793673 6.91687197766
15.2865020331 3.36944799197 8.68316687149
14.719990159 2.08952752441 6.92688340371
15.6295657906 1.78115851618 7.17594943365
15.0895368573 3.07246755042 8.17395352529
15.0009727492 3.52803637419 8.76438390606
15.6698346381 2.8123596598 9.0420122825
15.6072488457 2.09856060435 8.51206879228
15.3454047825 2.97036927664 8.20378218028
15.2043555541 3.11775092221 8.28737919605
15.056007429 1.63017206235 8.56336852812
15.0273385533 3.50194651127 8.7344395792
15.0796011062 2.39722759735 7.27398459568
14.3286812831 2.13245986107 8.52097549959
15.4348234432 2.40574677975 9.30870183897
15.5704271556 2.53081683627 9.20172415798
14.5001485949 2.30684841362 7.44984942957
15.5093370577 1.98872874499 7.12906872713
14.7372208392 2.72509182229 7.76170069216
15.665991118 1.76902232805 7.80319575406
14.3401843726 2.07930343853 8.50678428886
14.2780051001 2.41249087497 8.57255627082
15.662712339 2.95759341383 8.99282694583
15.17779457 2.83825896365 7.90175438728
15.6639380688 1.99676454023 7.68058858782
14.9729823145 1.90600184974 6.59997155502
15.1705559824 1.09284463945 7.80700486214
14.326833488 2.37696700294 8.85723851506
15.1323997343 1.22201506788 7.99913467331
14.7288072802 2.21734074283 7.0910406909
15.1448619906 2.42885785729 7.33757917597
14.5511135812 2.04082307955 8.88008945615
15.2362759396 1.68504335223 8.56138358448
14.2919130635 2.22883823276 8.47719388997
15.6616834473 2.34531646155 8.56009732856
14.3525896425 2.7035819833 8.29089867062
14.8976579907 2.11488289304 9.20946466297
14.3742849468 2.17291976454 7.52219278898
15.5559037726 2.19038910825 7.50507593189
15.0566937676 2.8491752878 7.87450213129
14.9862912095 0.89697086084 7.59394995964
14.6698640996 2.31474228763 9.37084282595
14.44291635 2.09974240411 7.27602453683
15.6534813167 1.76015302787 7.286433193
14.4124157225 1.66812492533 8.14634214702
15.4247984853 2.9881278399 8.32230703999
14.4797553515 2.81281506779 8.15882224771
15.1824060238 0.882388205248 7.51871638332
14.3106978247 1.36193010645 6.70053511154
15.6684887974 1.50626530594 7.40245803168
14.2737297427 2.02927502024 8.00151277601
15.490142426 1.3773676639 7.85155671752
15.5505167025 1.83926854819 7.00320863527
14.7000594652 2.01843887652 8.99824291639
15.117192564 0.852480817653 7.51409059588
15.6767488918 1.79321531886 7.60591642493
14.3805135274 2.05686149096 8.58449861247
15.6644444926 1.6528442847 7.61166842891
14.6974182864 1.72975744777 6.46509566002
14.5384013685 1.16304579248 7.69280751133
15.5363255678 2.74149840699 8.19859378899
15.4161295468 1.96753678028 6.94834465411
14.8865070428 3.26921589875 8.42909068871
15.6155245252 2.60830052839 9.15291328173
15.1676024266 1.05648522388 7.76678543295
15.4242273481 1.67484417223 6.56367334522
14.4625425562 2.60897396176 7.89755028626
14.6500441169 1.60986980614 8.41199421324
14.2791498076 1.39028703817 7.03012288102
15.659441023 2.71321706364 8.67803652148
14.5395756435 3.29819367266 8.70692246035
15.2063747585 2.30493262304 7.19914768481
15.6058685911 1.71441805599 7.01006863835
14.3241880127 1.21309644281 7.26543722463
15.6698201055 1.53286785066 7.08737010905
14.6932979187 1.66437923193 8.51688545983
14.4047578195 2.53132605582 9.29555452453
14.2768585343 1.7748728375 7.73088933104
14.4030283424 1.86616414642 7.03987326378
14.533632237 1.0468971322 7.53302924905
15.2097019096 0.983865420736 7.63938074322
15.1803735647 1.38703454827 8.1901002905
15.6437170309 2.32130037073 8.64229794607
14.9387798093 1.97542861633 6.69798913084
14.2769880694 2.54969342109 8.78972138502
14.3499378506 1.92248384538 7.2827613337
15.2072036234 1.86037608992 8.80632322125
15.1525289657 1.69889886685 8.6184080217
14.9130714608 2.40743659959 7.2783345572
15.536880228 2.12799116333 8.75852023377
14.4483733989 1.619903401 8.15682689183
14.2810017327 2.32064168748 8.14388820557
14.5557031664 2.54356126876 7.6758474765
15.5864236873 3.14529742923 8.87504189724
15.1668211445 2.10267215921 9.15348948313
15.441203508 2.74871814415 8.03141159562
15.3129248455 0.865950699395 7.41802099102
15.5699277508 1.35216238435 7.65259628392
15.3129423581 1.22508155281 7.88594537425
14.4356393371 1.22650764611 7.60374207457
14.5223541595 0.798326604084 7.17645192521
14.3216856617 1.49365320506 6.82239426875
14.4551674767 1.82876480834 6.89429023646
14.5958292528 1.84346250795 8.6750360727
14.2787511517 2.62593862544 8.90840311302
14.701671733 1.30425764726 8.04329336944
15.6201793207 1.66070525585 7.90425017442
15.6526831174 1.90218748272 8.0525420519
15.2876208258 1.34034072107 8.05952814516
15.1806739259 1.90134219365 6.64855081236
14.4032245854 2.07710032872 7.33569304522
15.0170306133 2.21892881771 7.02503462531
15.666158534 1.48715494345 7.02767173915
15.2372737357 2.43422461036 7.38830705511
15.6685637978 2.28719375159 8.1793476705
14.4345350457 2.02314529748 7.18617193084
14.3899085616 1.81195999697 7.01450921511
15.5569106115 1.5337498254 7.91003922127
15.6660274748 2.60718536933 8.61483601415
14.5822403264 2.17111672263 9.09669827857
14.2789926801 1.89230652006 7.88989159111
15.3296388233 1.9416544322 6.80891467244
15.6280202078 1.99845744758 8.31346573606
14.3720250435 2.97033631836 8.58203261782
14.372841333 2.44848362391 7.89470986127
15.5841691829 2.86084503918 8.47678256895
14.2732606676 2.52858469982 8.52630612325
14.8896005776 2.30943979276 7.15156791294
15.1246084651 1.25191427488 8.04453091539
14.722458982 1.61204677036 8.4666305853
14.7300067696 1.2599365813 8.00748411629
14.2876881975 3.03961453754 8.99383162607
14.8976997793 1.65703462656 8.59418822902
15.0054025391 2.37429656225 7.2273396655
14.6697589761 3.15722458067 8.3836466521
15.6663438922 1.90309360254 7.58207555044
14.6901084401 1.45318513223 8.23370295693
14.6261193399 1.8939124768 8.75910700674
15.4047290129 2.67170130469 7.87565502467
15.6608807092 2.4937599334 8.35975455745
15.6727782294 2.32273987626 8.36980664106
14.6822451613 2.30342995431 7.23436317911
15.2468861795 2.09022692701 9.09503204925
15.6750355692 1.60248475825 7.3427551508
14.3697789479 1.31183390078 7.5657595959
14.9986265016 2.3196280236 9.48352858357
14.7881474391 2.37136502113 9.51683889164
14.2724761009 2.82274598011 9.12069777095
14.4502512015 1.38562597952 7.84973287586
15.4715268223 2.85072985818 8.21288367761
14.6393299935 2.45265876791 7.4720841971
14.5797894333 1.34362358725 7.98274126598
14.4460606464 2.31036923946 7.54743013527
14.5355689807 1.74061341336 8.46170356324
15.5017707324 3.29758605488 8.86699493261
14.5900320385 3.05211538632 8.31990303719
14.273356479 2.17137161811 8.17606994959
14.903114797 2.6090322198 7.55305696674
15.2330802896 2.24684231403 9.3102173851
14.602151888 0.754802867667 7.21340537105
14.5032739231 2.86956654901 8.18795567544
14.3905795305 2.24502214706 8.87808969718
15.6704652753 2.47965198447 8.55788308773
14.6739811599 2.27560728469 9.32178877215
14.2764568404 2.12280756132 8.19789951024
14.3853700722 2.31263372023 8.95699980993
14.6515664738 3.17111617815 8.42261579727
15.1799710415 2.96302699346 8.06880513411
15.4302346788 2.36416360833 7.48921165622
15.6339658772 1.34013552084 7.40769931473
15.5556454539 1.7081579842 8.16291357558
15.6388771533 2.76088710741 9.26556169087
14.5314148 2.01654356973 7.01229478205
15.5961059406 2.24274968671 8.73971100902
15.6700532904 2.44233733484 8.59264244703
14.3381216247 3.06737647251 8.82922971095
14.3692531831 3.19162905668 8.89055628067
15.6733267178 1.82768621986 7.58886433137
14.866525691 1.75143727904 6.41431138467
15.5181869666 1.99586100088 7.14770015519
15.4260840354 1.1392934035 7.63936145765
15.6640301071 2.49401482379 8.48286025049
15.4385029744 1.93133107616 6.93024854407
15.4829088358 0.951147846758 7.31662259842
15.1554113931 0.793030451666 7.4066890198
15.494630949 0.990989589005 7.35425248271
14.3390629182 1.70230739012 7.98074100693
14.3187296897 2.65402183043 9.1852243571
15.53772477 1.13586378063 7.44729254977
14.2720659234 2.39645900098 8.48882031985
15.3996861601 2.05365565793 8.88680546032
14.5824758197 1.43563009313 8.10520005841
14.6072860746 2.20141503241 9.16655795833
14.6677440357 0.807936915796 7.34723077665
14.6419339152 2.48966617066 7.52310573176
14.47244164 3.07242345178 8.50222423785
14.8892385032 3.41973453748 8.62648904197
14.8487960685 1.98480419151 9.02484215692
15.6783215461 1.43183154878 7.04480346498
15.5534877088 2.20957756245 7.51546860645
14.8343717076 1.90882734115 6.63541502519
14.3591186359 1.53008139796 7.82762841138
15.6095861048 2.00907401272 7.41334013941
14.2866168905 1.13199833572 6.90427370138
15.4397843458 1.11353103443 7.58801653497
14.3022666204 2.43670467427 8.82576355379
15.5739625851 2.31688359869 8.9101145086
15.2227604393 0.741880930971 7.31200067393
15.4511652488 2.56784158636 9.49609662958
14.9088160647 0.778864717699 7.42430675416
15.4391139432 0.914480394126 7.3276075526
14.7102288313 1.36599702107 8.13362937034
14.4436210534 2.44605146403 7.72875391932
15.3709627899 1.22990733503 7.82513853725
15.642038951 1.53375195238 6.89007200153
15.5749703959 2.8271808798 8.40281130868
15.6715132126 2.42517854629 8.44384236705
15.0209957058 1.79527705907 8.79178169638
15.600532516 1.46921250553 6.6623255763
14.2902319629 1.77092413814 7.33735156503
14.2809467903 2.75294837929 8.69205983359
15.5874108489 1.67034599282 8.0239690717
14.3177838465 1.92698681637 7.4009038603
15.2080910365 1.10388008737 7.80935435468
14.8635872467 2.28223807226 9.41761844484
15.5592755086 2.99330085816 8.58280919047
15.3937222707 1.18345771621 7.74380899509
15.4259940949 3.06555802846 8.42811830161
14.7510514448 2.06482613882 9.08878444935
15.571300602 1.93804939955 7.20204962478
15.149371542 0.671853351043 7.25402291332
14.8045136461 2.32277253573 7.19586917778
14.3907141967 1.17235321851 7.43481187835
14.9988531726 1.26321382765 8.0852137755
14.9744822022 2.79268463075 7.78103336828
14.3165414342 2.51143295936 8.17629120367
14.5251570074 2.88463484375 8.17766646855
14.6507397435 2.24742144724 9.2647107051
14.2810461832 1.51511907234 7.14386890241
15.3496236619 2.03034684913 8.92031996029
14.7669378391 2.25770420222 7.12370153799
15.6648982222 2.46076608533 8.67963177906
15.627180697 1.09418191741 7.1449136519
15.2418078757 0.892136978026 7.4960872049
15.4462237722 2.53065999939 9.45557257775
14.5817223639 3.22217749743 8.55904186992
14.4075822669 1.4305579094 7.81909166276
14.7716427258 1.17383187657 7.9106558831
15.4428336423 2.14486766626 7.22837977955
14.2858151974 1.49580769023 7.04108903013
15.3042848397 1.90368522756 8.78652918493
14.3944624409 2.18445668481 7.49402817254
15.6259002113 2.62246544056 8.32884453614
15.6703704831 1.95304049941 7.77031376145
14.3235569518 2.1370233418 7.63499063016
15.6467163424 2.4841846985 8.22424433929
14.7030037881 2.45833444254 7.43070958425
14.2797311941 1.6131211526 7.47040728557
14.4367976813 1.79699630851 8.3738103271
14.5044847221 1.9526510558 8.69979294412
15.6061854965 3.1585285677 8.95211496855
15.1354761495 1.53600733582 8.40767264348
15.5827294782 1.17693844564 7.38496721771
15.6118366186 2.56331951742 8.19699223193
15.6647138827 1.34427999533 7.21023199435
14.9132364463 2.11219773947 6.88651506863
14.8323702501 2.05609217884 9.11934213567
15.6281055819 2.84588317446 8.63161323968
14.8128352268 2.73617575093 7.7437110736
14.3203853705 1.25795731997 7.2796804529
14.2752769269 2.13096895486 8.16276489645
14.3179262676 1.08484504191 7.05275901277
15.6753691453 1.17060225342 6.89371304926
14.8629354076 2.45422421545 7.35064437081
14.95934743 2.20702827131 7.00725486604
15.6541889384 1.40703893788 7.41593164098
15.3042654288 1.75452392138 6.52998083147
15.3195158631 0.960769672993 7.52535034806
14.3203085364 2.58291298601 8.26112816639
15.2057683582 3.3295461555 8.57207753944
14.3896073827 1.97046265179 8.49729319766
15.6740898913 1.52128666961 7.15985896747
14.2841719649 1.38673064328 6.94716524561
14.6990139573 1.3922223911 8.1566917186
14.5945094017 1.17731274834 7.7809685828
15.3011300651 1.0601659821 7.68109273121
15.6501824374 2.87366999341 8.78881345541
15.1543909391 1.3909629799 8.20577244523
15.6385578303 2.46629116314 8.86535580751
15.5574782727 1.89508186973 8.38372032844
15.6274788187 2.43859715216 8.07017666444
14.6916557708 3.16749016663 8.38232023095
14.4756516713 2.45366439552 9.3292707573
14.8228103642 0.717170040816 7.32071159177
15.4563688436 1.30060219168 7.81384370091
14.9488344775 0.800869770042 7.46270632446
15.4275867704 2.22915563517 9.07896284809
15.103188416 2.38463836943 7.26226950524
15.3905046614 2.33967165297 7.40632344419
14.2738264381 1.83794057318 7.63416723353
14.5644399772 3.352905836 8.73657286887
14.5221711235 1.74711916044 6.67839655354
15.3308154021 3.20653559281 8.50689454957
15.6518525299 1.42961546209 7.45107297397
14.324649035 1.71050762446 7.94823440409
15.1443485226 1.61146932784 8.51190074268
15.4219724032 0.772259718568 7.16659064743
14.5378407326 1.47861992346 8.1152586433
14.8523134793 3.19689023603 8.33901496861
15.4379453446 2.9272857737 8.2686416705
14.4274126477 2.14675785803 8.82624792585
14.445879221 3.05228569765 8.53195769097
15.5943627176 2.64185185215 9.2726855282
15.2157224691 2.93000390333 8.04039076243
15.6577279769 1.48830966179 7.46665217082
14.3648876469 2.44242742107 7.90610817831
14.4857156876 2.63894798014 7.90227094706
15.656107595 1.81123378299 7.90788854902
15.387797412 1.99667001522 6.94593325213
15.087601196 2.74365117378 7.73098212846
15.604708595 1.6930150066 7.99650396646
15.6773413934 1.23101602921 7.01882847351
15.2796016383 2.43125405678 9.51502054202
15.2246028289 2.09419462244 6.92721278948
15.0635630674 1.41826286225 8.27648447821
15.5562505367 2.66736173481 9.42361571274
14.6768942965 2.16409461146 7.06127145595
15.5703730632 1.22364437297 7.49164849525
15.5287396688 2.12387115725 7.34099007951
15.6384132937 1.76646279509 7.93718299655
15.6529904674 1.41538999665 6.79707989233
15.3080857684 2.17742207516 7.10883158036
15.531066023 3.15930536071 8.75280566442
15.1868819377 2.6534101903 7.65445073609
14.9486821903 3.4296184889 8.63260047091
14.3309331801 2.26947052105 8.7176282257
15.5789149041 2.11832269847 7.45442269214
14.6920947693 2.37899086105 7.33876653459
15.425919731 1.61309731207 6.47886358541
15.2015052908 0.969491138131 7.62673293718
14.2992478961 2.30905521422 7.98917347774
15.4264352127 1.6184143263 6.48611492965
14.4935994444 3.09855667364 8.5048585394
14.4205220486 2.44963129394 7.7747983929
15.1309112088 1.80801267992 6.50584062422
14.3588962635 1.7151498551 8.0891092435
14.7743152734 3.00641731515 8.11561222079
14.2765924442 1.29831392706 6.96338541445
14.2735085623 1.77936531662 7.61506328703
15.1748022361 1.90839501691 6.65394297701
14.7055925273 1.65814807109 8.51406840232
15.4796152191 1.44645848414 7.97194166128
14.2795752988 2.68503050863 9.00761602985
14.275478815 1.8703412839 7.62919674012
14.3203517798 2.38305117688 8.84610163549
14.821795003 3.10552517181 8.23272251816
14.778274032 1.1349257013 7.85919491456
14.2848989699 1.46594777064 7.36279764053
15.6080070301 1.89999572201 7.25391816294
14.641552056 2.22907355901 7.17496748359
15.6045947032 2.31742581026 8.81292871315
14.2790942634 1.2634248539 6.78916634294
14.2916131172 1.43164123326 6.92153133356
14.2776924561 2.83490361092 9.24758494138
14.320152298 1.97321077659 7.45038244831
15.0706245131 1.6859631054 8.63512875709
15.4279920171 1.61292625102 8.27448278303
15.5929284538 2.63495444972 8.21621142708
15.5997117198 2.07189640152 7.45688770918
15.4750835622 1.73438215439 6.71477666039
14.9131970587 1.8386764156 8.84778409616
14.812464299 2.12652891422 9.19545883481
14.8041456398 2.54287049964 7.49158140688
14.2932683281 2.17183584885 7.85727054684
14.2817294405 1.18145236251 6.93879458083
15.4456822167 1.13298367898 7.61310231622
15.5088534933 1.96605530392 7.08858961812
15.6668400332 2.00341065507 8.04587807097
15.4917588401 2.93287703152 8.35540201564
15.6728715843 2.50911849939 8.59693351484
15.2514909829 1.429101228 8.20885643839
15.4159330587 2.16630094979 9.01886412463
14.3570395413 1.28651386551 7.47246699404
15.509255726 3.16146888384 8.70151175647
15.5898882112 2.74652589055 8.33211851047
14.6919864011 2.68307489389 7.73766406975
15.6460661832 2.53525489428 8.28605496822
15.3486615609 0.976151128573 7.52042869773
15.6520676962 2.51104747902 8.30548175674
15.6216396372 1.64593244842 6.95208312481
14.6880442778 2.62240345928 7.66065737791
15.5083566787 2.20317381795 7.40816114631
15.6212026189 2.37132853721 7.93935697783
14.6899477151 1.75871299781 6.51112893853
15.6652321448 2.30759979778 8.13877222931
14.3498566594 1.08563080284 7.18509687559
14.2846138244 2.76294132801 8.66991818491
15.5544537044 1.3426945561 7.68394373993
15.1243627516 0.770796831765 7.39740853963
15.4531507564 1.28904279936 7.80988754617
14.3795394937 2.06770157142 7.37300135575
15.3487476528 1.73817153195 8.53454811879
15.3640606454 1.74276280581 8.51843651483
15.134306045 2.60449070944 7.5704210657
14.58348614 2.0505543058 7.00222036905
15.6540643592 2.79417092998 9.22099974042
14.3058715311 1.51448739171 7.57732382713
15.651867928 2.60485796404 8.97201305034
15.1393318766 1.5730830845 8.45479704712
15.5125748407 2.11371673398 8.78817090075
15.6370990053 2.08543981155 8.38420616798
14.3069311674 2.48104227061 8.89927591614
14.3078728776 2.87684600455 8.68678826569
14.8998478295 0.968334364987 7.6891586947
14.6982188874 1.18247826208 7.87838864084
14.3268441008 3.10832724729 8.90513260786
15.024357587 2.7130277306 7.68518258544
14.743127543 1.1963104559 7.92552307626
15.6404761422 2.86889339749 8.72106346808
14.3065288252 1.91275463782 7.44184976012
14.5903890088 1.27431943327 7.90192468099
15.5594232003 2.03979952289 8.5819998653
14.4060196223 2.98082829206 8.51322107735
15.5600572357 2.63012622185 8.10844477901
14.3431486743 2.99184522858 8.68934149348
15.3710215639 1.08195946538 7.64271848861
14.9441761831 2.84217663522 7.85415214909
14.2882585244 1.7657493129 7.82835894898
15.4735305312 2.08607172519 7.18657125367
15.5292773312 2.76089591227 8.2149905465
15.4853846165 1.8940640335 8.55511552101
14.2886084692 2.00634239586 7.68047832737
14.3675528242 2.08186753815 8.59782190996
14.6470434213 3.2830221416 8.57113312986
14.7064686413 1.21548177937 7.92561125467
15.3864923836 1.7035229245 6.55258804739
15.3762074957 0.964492449794 7.47242044668
14.75140764 0.778368750685 7.37295606229
15.6745535543 1.52354445307 7.18905728918
15.5953371394 1.95416224976 7.29328468976
14.9022006359 3.0436733608 8.12685787169
15.3378917876 0.890702521038 7.4172994512
14.4061852132 2.26736587205 7.56914828651
14.9391409263 1.85694880486 8.87111811505
15.1412850127 2.3786368748 9.5336946549
14.5894588189 1.89600450156 8.732473243
15.0669232737 3.10223485983 8.20640281368
15.1171970446 1.13659375156 7.88525672339
15.3702015558 1.31257127838 7.9415926629
14.4394322576 1.65087552669 8.18979760415
15.3160363357 2.05350125538 8.97450355426
14.4049778768 2.20155475157 7.48972317716
15.6736539062 1.67883554991 7.60590404297
14.7175089287 1.2320608924 7.96174507297
15.2202709229 1.87785736289 8.81970406396
15.6407281995 1.98138963074 8.22689331935
15.5269156245 2.31148685788 7.59954599117
14.322746566 2.11018250897 7.59781347972
15.0429362356 1.99699638812 9.05700548924
15.6159844605 2.00062768029 8.36348719879
14.8459977474 3.4091071289 8.62275037928
15.1952293319 2.70874154169 7.73313504245
14.3961513725 3.08704424548 8.66993791706
14.8074738835 1.91780566263 6.65343808039
15.3664217851 1.88638717682 8.70552256514
14.3136689449 1.46336762812 6.80704900662
14.6383017676 1.73008909606 6.51123593014
15.1174260765 1.16144525463 7.92456398302
15.6659221629 2.8311369973 8.86889414464
15.3826622363 1.75780859291 8.52034946179
14.3182212061 2.79039914056 8.516109506
15.5960828174 1.04149866965 7.16365450069
14.2759854677 2.34048460884 8.49885992205
14.4975259262 1.29007570394 7.79649612773
14.8878720048 1.66117409242 8.60651652443
15.2142479122 3.1307569254 8.31055509705
14.3077001342 1.57937516099 6.98652333063
15.3190607505 2.02949787711 6.91838095226
14.9464279089 3.40621778736 8.60469607619
14.7659629061 1.23460765554 7.9877741624
14.3231186061 2.7249438572 8.42259670091
15.6083144285 2.36421256107 8.86847789992
15.3174804019 2.02668517228 8.95158338052
14.9614532675 1.14268409669 7.91804200455
14.7762641064 2.32805409705 9.45032293196
15.2872852546 2.51567250516 7.54281481669
14.2850932968 2.38478429396 8.62904498514
15.6713236463 2.00318742498 7.99768050369
15.480152839 3.0724843572 8.53567492238
15.6607714459 2.66081832189 8.54792077825
14.6739672744 1.95577923645 8.89766706271
14.2759256768 1.6969690009 7.52965486715
15.5614588702 2.58001685806 8.05119009402
15.0415940328 2.44232520941 7.31724785877
14.5999353433 0.946934541642 7.47486563379
15.3774784751 2.13720186338 9.02961786504
15.4883938362 2.06607957204 7.1896843139
15.6252806063 1.73013954389 7.09721467439
15.0968551534 1.07354778595 7.81016163674
14.5094822672 1.97140999165 8.73150260556
14.5323460662 1.05995910406 7.53978314647
14.6617441724 2.27588772468 7.22063273911
15.566399765 1.21572372407 7.47232146011
15.5848829211 2.85115982328 8.46456485342
15.5548785652 1.98153282609 8.52430604727
15.0964431009 2.42155356492 9.6033781887
14.2715789838 2.00620879958 7.87479108406
14.2762592517 1.87049603482 7.7619044626
14.8288159719 2.3808378109 9.54343665378
14.2801176062 1.22097154099 6.91782753206
15.2864927559 1.98385216771 8.91215258891
15.2289571654 1.43892688378 8.23092997991
14.3510425439 1.36375395872 7.56053335977
14.3333925065 1.31073078616 7.43028104494
14.8525095472 1.79800161127 6.48326651555
14.543934841 2.82616682071 8.07165636486
14.4488618066 2.39400794645 7.6483987143
15.1194279884 2.1260095547 6.9178568162
14.3405112605 2.56169524322 8.14777973935
14.3406114039 2.19834050724 8.66594683273
15.1377429817 1.94203444927 6.68075789284
14.327720857 1.84303100702 8.12959506118
14.2762967332 1.69787166021 7.58514673776
14.6688055838 2.35431685619 9.4274368408
14.2756251907 2.71211791078 8.83466124533
15.3215829193 2.20584608113 7.1561453625
15.0649608412 1.3853671102 8.22707185768
15.1336572219 1.8413595595 6.54957716922
15.6691217912 2.35737156577 8.2385197341
14.2756343553 1.92435597663 7.8409347974
14.268438212 2.67838274564 8.75832465781
14.2803307699 2.90151109847 8.9059044083
14.3211336802 1.91325129096 7.38254113434
15.0348445277 3.0772614537 8.17003473082
14.4863342774 1.68120885471 6.64524114658
15.5560199075 2.64864001111 9.3820769188
15.6392493016 3.10081333883 9.01235775326
15.6682304432 2.41807721935 8.38964356046
15.1922404439 3.3308038178 8.56496868136
14.3754962891 2.12818481587 8.68106363769
14.2877998917 1.79817009354 7.87499895908
15.6729952442 2.25410863481 8.16792501092
15.5435597994 2.32302655491 7.6517228661
14.9997640559 1.07976142348 7.83392519366
15.4435185765 1.47084970522 8.05550869374
14.6786231327 1.84860844062 6.64380737448
14.4482502683 0.906829753261 7.19409090661
14.7898291497 2.17015041414 6.99794731946
14.2708444242 2.80481645764 8.88074253143
14.3961035396 1.99579139636 8.54833277159
14.8544459102 1.75003333231 8.7145996026
15.1482311762 1.10724271153 7.83754085936
14.3454874265 1.03681692219 7.10253598158
15.6535827463 1.35472218444 6.72670849837
15.0199630669 2.66551634623 7.61937424506
14.2771087349 1.75077482145 7.62375194149
14.4255180797 2.08714001086 8.74611490727
15.4507890818 1.96443469876 8.70127514637
15.318911664 2.63350972863 7.72771330971
14.388453585 2.44016850404 7.83954355084
14.5868813563 2.29822027289 9.2692095871
14.4934409906 2.75750765305 8.04133069797
14.4190464161 2.32401545262 7.6213082971
15.6116014411 1.45473550069 6.68035420925
14.5740344103 2.20240956072 7.21999620014
14.7527476954 2.11839186598 6.94593783215
14.340726389 2.00732180121 7.42315103242
15.4995385361 2.9091324634 8.34059831834
14.5052470704 2.11618666602 8.91973023656
14.9398247323 2.18822088595 9.31349342562
15.5190865658 2.12895834372 8.79726018438
14.7258076971 0.726477581214 7.28777927195
14.8270211529 2.42121633848 7.31724497604
14.8642832687 2.87727872779 7.91386839063
15.6753619533 1.75475131784 7.52851641447
14.9472285442 1.06199782893 7.8073612755
14.8223026085 0.915751945246 7.58382744229
15.6191548851 1.9014487709 7.31389735545
14.6237059208 3.25690652053 8.55384390506
15.679172311 1.33140857139 7.0332953429
15.650227264 1.15857412678 7.09116717568
14.6953726424 1.1791147734 7.86915755609
14.2777474827 2.83331528208 8.80561931508
15.5569738832 1.81254656528 6.98084251268
15.0832939685 2.69901333844 7.67738202497
15.6281905363 1.93031364661 7.36963648537
15.6762944528 1.41986004216 7.07930681738
14.3001834255 1.35054761559 6.76248089206
15.0194327356 0.795941351534 7.45428946639
15.4663833699 0.954990092852 7.33478962519
14.558517169 1.70249163761 6.5615242572
14.6048695376 1.94017054653 6.8310301549
15.5211149763 1.46617664248 7.91930940114
14.5085867699 3.07314815211 8.44617659611
15.6200896722 2.76918420391 8.48405963345
14.6751158302 1.97688044342 8.92171538862
15.0074703345 2.18397871613 9.30168256174
14.37793449 1.67674413799 8.07749560285
15.4363628693 2.57401647806 7.79068936096
14.8823186533 2.21896252523 9.34522235352
14.2797969117 1.82306270628 7.73259494686
14.7432277033 3.19227992912 8.38241193572
14.6025315575 1.96657304913 8.84192249196
14.5496420633 2.28925280708 9.21449671604
14.6287870113 2.20244960434 9.18755706071
14.8306494347 2.33145220114 9.48486728471
15.5818404736 1.8303499792 7.08170309063
14.5582791277 1.46401554311 8.12229817048
15.5924608432 2.13830289129 8.61443431774
15.4284805072 0.80463169287 7.18810267039
14.3500903569 2.60199428228 8.16591299694
15.1652783491 2.60550533938 7.58870031088
15.5465921305 1.18534981898 7.49483965445
15.2955722272 1.73958440492 6.50713143166
14.4212922844 2.66258850051 8.06634226053
14.8351359987 2.22118795434 7.05364595082
15.5551040637 0.961959915701 7.17201024633
15.0197029378 0.829003540828 7.49837882235
15.3419892262 1.22345298403 7.86043252353
15.0695296598 0.989297052676 7.70300315067
14.6459211229 1.8653351404 8.74512677581
14.2876620067 2.56017345248 8.91606554118
15.6690621899 2.65095570586 8.77290348742
14.4991200295 2.51214756811 9.44497047929
15.1281082637 1.96815094608 6.71356963416
14.2787936123 1.49308351693 7.31581017543
15.5002226438 2.41638224009 9.21576034793
14.3087248383 2.55111739425 8.26648148744
14.3883261447 2.50281240371 7.91800805361
14.3273312761 1.3567904388 7.47693072355
15.2706512159 2.47775527426 7.46665881065
14.801570596 1.22958528984 7.99250612791
14.6321679621 3.33460124087 8.65322823052
14.8568691121 1.13077025507 7.89124738766
15.6081754712 1.38245735779 7.58800547144
14.2841420789 2.11592991771 8.25790538667
15.5186596717 2.3732228496 9.12190397496
15.6737698461 1.97429444046 7.78930771691
14.7861940885 1.33984864072 8.13741976515
15.1655251423 2.314533804 7.19333935803
14.9449538292 3.02053163259 8.09046798864
15.344048036 1.33565976808 7.99889256353
15.3190280137 1.95863024444 8.84703261463
15.1496442979 1.03243702329 7.73665720321
15.544493648 1.4606620615 7.87303517013
15.6735243813 2.34871808557 8.41260345261
15.6529892989 1.35555612715 6.71542629793
15.3514259919 2.44029594802 9.45370860583
14.4858114997 2.23670668646 9.05101187643
14.4577512874 1.35276899703 7.82117093475
14.5792053114 2.30468542014 9.26314283782
15.0399773199 1.08881229431 7.84534160714
15.6096350312 1.9818694485 7.3850707667
15.3347479062 2.88954115621 8.08773782179
14.6788064339 2.37718964605 7.33766302479
14.9765090094 2.10472217617 6.86566559737
15.5327047678 1.44025929763 7.85706738991
14.2730215577 2.32151944843 8.36019856221
14.779952274 1.47415556388 8.31790001054
15.500432203 1.33560361431 7.78884765884
14.3883110829 2.6782897571 8.1613364231
14.3659544762 1.74769452596 6.98465686246
14.2831626609 1.36161083902 6.89054907811
14.5865605926 3.09114250329 8.37313385854
15.6333283081 1.98971306199 7.4936360275
14.4660438974 1.83042956879 8.47101153371
15.6596803376 1.46171984064 7.43011332379
15.0383106075 1.60775754718 8.53469936401
15.6613579285 1.27079196295 7.16655519514
14.3540613665 3.11393555023 8.82453823275
15.0629456121 1.97153555559 6.7023017362
15.0629017805 1.90546975932 8.92824845658
14.9075702066 3.24572209355 8.39494729768
15.3055233843 2.71056289588 7.81569505609
15.0759720849 1.91469110132 8.93771936661
15.2708497074 1.81197638614 8.70369507577
15.058581316 0.89606552167 7.58373757004
15.6591732348 2.15302926268 7.87109490537
14.5508877665 1.87191315763 8.65441881094
14.3366384404 1.79795383685 8.10934413145
15.6740522449 1.65918442015 7.4287870273
15.6609188881 2.84794054634 9.21759100848
14.3376976892 1.57655285264 7.82137641103
15.6530227106 1.50353047618 7.5157950236
15.6717372892 2.24101121977 8.22800797026
15.5968184227 1.18007568661 7.3437945008
14.7149164293 2.32232384159 9.41397319566
15.6633606997 2.62643762725 8.87161031111
15.6713828898 2.60806314315 8.73324789217
15.1372992071 2.15334826381 6.9657500934
14.2820881431 1.58252648626 7.16655881351
14.5928259833 2.10452089907 7.06408824847
15.4344130774 1.9749013776 6.97981379192
14.780927694 0.747117807809 7.34762143995
14.318235026 2.42878108896 8.89355343172
14.9050712763 3.34806737533 8.52707571316
14.2732201162 1.83142694905 7.68179687028
14.4564096089 1.88603730644 6.96780677642
15.5178920401 2.1020173142 7.28333786605
15.5319626429 2.35811382968 9.07040197393
14.2796773121 1.87174008141 7.60135917412
14.807232021 2.27933704566 7.13449744914
14.2799407669 1.5038388482 7.26498205205
15.4140084033 2.65445540751 7.86372853045
15.5522648077 2.22832464975 8.84569456786
14.8333937329 1.05958995249 7.78349202705
15.3854047864 1.22273134914 7.80897027066
15.1448030233 2.22737862613 7.06132594945
14.7501268553 1.33969346738 8.11432208805
15.4441806475 2.5510208189 9.49219212723
14.3251445432 1.65271696674 7.85808494367
15.6602121878 1.53801474925 7.5217676365
15.2971142209 2.10061151021 9.06089596023
14.7256230915 2.08420450782 9.10759497992
14.2869867498 1.37265843478 7.18528320423
15.562324657 1.44769666818 7.79833745663
14.5661154474 0.777224921102 7.20651974502
14.4405166109 2.02206240513 7.18173589324
15.5800907269 1.8900070329 7.1639996809
15.3649463181 2.89347832317 8.12070128241
15.6755629557 1.53985399879 7.20188608983
15.240798641 2.37432125849 9.47220809524
14.2853575079 1.88086760576 7.52610653609
14.5631010999 2.3852139642 9.35476705133
15.5962195873 2.49835737375 8.03260095174
15.6154049769 2.9977168242 8.75570094191
15.5868026131 1.00204282593 7.14996995537
15.5406259466 1.69527865937 8.17066207901
15.6714763879 1.67822515842 7.61212077726
15.6714716458 1.55588175712 7.47779586883
14.7089884729 1.13881667451 7.82253331235
15.0646066006 1.51246588788 8.40623893818
15.6101654239 2.12191522962 8.54873202801
15.6596600508 1.73917962605 7.28739904661
14.293189478 1.78761236922 7.87250970858
15.2326794966 1.65545950159 8.51686888467
14.3923873385 1.9728900616 8.50574723629
15.6771502717 1.47221549992 7.14155559548
14.7056034593 2.10540473447 6.95922264166
14.319149096 1.66360915961 7.03826848114
14.5873419817 1.02994506413 7.57048163497
14.6697176466 1.57889917126 8.38793299151
15.6692523016 2.49981346988 8.45914453331
15.5300668954 2.59291860437 7.97718490059
14.3198040314 1.43047508714 7.54960032151
15.4353560456 3.03892115169 8.40963649281
14.2783995144 1.96699636546 7.94958420739
14.3402938621 1.48134112757 6.73321502557
15.1839608612 1.63531150869 8.52211435372
14.2858962811 1.88510492525 7.55028245202
14.2814465248 2.33352786686 8.53577104127
14.7466400633 0.774308256515 7.36825356969
15.4162146457 1.87196575074 8.62637848413
15.6490875559 2.37060259719 8.06268876734
15.3971709385 2.03712500865 8.87546139628
15.5802873915 0.977637949465 7.12654423993
15.2996004134 1.03040310452 7.63201427775
14.274649853 2.70021852481 8.82669005466
14.6813718658 2.18666291031 9.21234521427
14.5641594774 2.02672965144 8.88114626891
15.6248842542 1.29828918499 7.39193994559
14.4032157433 3.15835095164 8.75454065166
14.2811712986 1.29166695561 6.9881920645
14.7583350702 1.52721880635 8.37124012424
14.2842200948 2.82394294013 8.7653409478
15.2729846196 0.85935815563 7.42808474998
14.756253751 2.18407333594 9.25185771784
14.4182800194 2.87459239958 8.3589113519
15.0406272304 1.96734565789 9.00963525932
15.595508925 1.31167796518 7.52405515276
14.6856654776 1.99201907902 6.82123003792
14.5388743347 2.33179093428 7.43257623679
15.1412453188 1.79041293261 8.74112647017
14.2899422035 1.23579242382 7.1054963561
14.5004222455 2.22561256162 7.33636123077
14.3087550235 2.25641768292 8.62580297365
15.5638573398 1.25634536522 7.53932689513
14.9218430028 1.85838535996 8.87573813446
15.6288174289 2.7892889832 8.55238230955
15.6653944131 1.27529922964 7.15915901646
15.6406854063 1.38130789522 7.44886478634
14.8791096597 2.34395603008 9.51906388735
15.6479296251 2.54761947463 8.33507372455
14.3406433592 2.86589352613 8.53823188132
14.2809887247 2.96713420279 8.94665140834
15.4827609647 1.80322344568 6.82785109564
14.2883820621 2.52808690699 8.88879126766
15.6689432799 2.8176996082 8.97337768585
14.3948578228 1.09868372215 7.3351512333
14.8659034749 2.76053439027 7.76480629615
14.4921869 1.85947487383 8.55439999841
15.5925249856 1.90732322494 8.32380292341
15.312556233 2.69673745146 7.80503211448
15.4304213163 1.61116600261 8.26052611963
15.1289539298 2.1288105933 6.92319999805
15.2739537834 1.41637587247 8.17615265939
15.6784119522 1.53468253623 7.37125068978
15.0276295154 0.946629593438 7.65542031013
14.6808298969 1.8262305941 6.61287569955
14.5008447863 2.27844773065 9.13171521601
15.3789883155 1.34017568403 7.96775288474
14.3715098191 1.58984889368 7.94202574982
14.746074432 1.19698647884 7.92407245261
15.3861181475 2.06134987451 7.02688239261
14.3174557826 2.17083971253 8.54514922804
14.3026225005 2.06732538069 7.65214897532
15.6058133228 2.65018148531 8.2712754391
14.2900179434 2.5377539168 8.88051848255
14.9453243714 0.63626636228 7.24209375973
15.4499836117 2.02727023022 8.78687133139
14.9822745837 0.641903917938 7.25191319019
15.2508316026 2.52769235018 7.53004884705
14.4318742461 1.27959149915 7.67377206832
15.6600088103 1.32239399388 7.23581353067
14.2868204017 1.93418980874 7.60457663405
15.5586235793 1.08973068839 7.33607459913
15.6427731673 1.39155173426 6.70704005285
14.6642429611 2.01724974464 6.88295452656
14.8383989455 2.50266720196 7.42398767766
14.9315424464 1.94252522641 8.98260385105
15.6017243877 2.6316854088 9.23548460614
15.1074952224 1.32835223481 8.13811369656
15.6709543577 1.65689351507 7.25297106063
15.2721659645 0.773506068162 7.31831715556
15.1359493307 1.13802491424 7.88172613259
15.3270033199 1.9693208399 8.86404099895
15.675320358 1.15198452229 6.84516529476
14.3101429524 2.05482897187 7.59879104739
15.5367064305 1.76109288069 8.27392756592
14.41537716 2.02823996909 7.23124376616
15.5332031801 3.01667375177 8.56410537341
14.7189265374 1.55787748575 8.38412150412
15.5927215393 2.0203824146 8.4751503224
14.2829569643 1.61019499507 7.26748965256
14.5769929901 3.22835083534 8.57120627197
15.0936694661 3.11888540669 8.23783730143
15.6701011706 2.74892122377 8.87668119225
14.8595621079 2.49911367164 7.40881963422
14.3667920487 2.24697644978 8.80511626426
15.6499510161 2.42972100044 8.17383170911
14.3738414691 1.02518221693 7.19222402272
14.2826260323 1.62034432524 7.25146732759
14.2760465643 2.63109659603 8.89123310052
14.6925759087 3.34861930132 8.62379679848
15.5167784308 2.66270576478 8.04118083406
14.516347015 0.826976859062 7.20240759436
14.8238222132 2.15067815203 6.95888291029
14.6690723573 1.71228493935 6.4612673638
15.6333190436 2.10545025133 7.6527555456
14.6847894386 3.12975480205 8.33529270288
15.6047306681 2.55744659185 9.13526231867
14.3062007331 2.92928726692 8.76540879823
15.3120770935 3.2858287848 8.59225727646
15.6716215284 2.14522163649 8.0604596503
15.2037000381 2.04443551303 6.85401323711
14.9505852618 1.86925739205 8.89024409767
15.2099663759 2.05391491455 9.06164323618
14.4590891384 2.50153783074 7.77089024005
14.2749780402 2.25649544025 8.11903008863
14.3305928809 1.30882686088 7.4166539928
14.3414006019 1.98426111702 8.37506505414
15.4253765621 2.23729753048 7.32296539688
14.8640599749 2.0695778536 6.83292141408
15.6222875905 2.67940162466 9.21260549544
14.8737014921 2.27662047445 7.11426535196
14.3643312314 1.59672120043 6.79883934052
15.5904257134 2.47096071137 9.05674483094
14.4029707742 1.82412590355 8.33528193923
14.4621412775 2.87207591218 8.25808629277
15.3071407464 1.97585620447 8.89111006104
15.187442948 2.31236526103 7.20264766125
14.9309579915 1.73597487822 8.70848857767
14.5116434962 2.19123614909 7.27657976994
14.344262711 1.90688297485 8.28858063683
15.3521376365 2.15047420177 9.07386077982
15.0335615439 2.52809195476 7.44212110482
15.5258470429 2.24745954927 8.93584559415
15.3410847293 1.16268429133 7.7676356858
15.5537438934 2.62606212865 9.36351754715
15.320991333 2.14501682189 9.09634102026
15.5413221305 1.69155933479 8.17277871476
14.3161437495 2.25448250118 7.84362114277
14.4077602308 0.902756243698 7.10822372677
15.6565131308 2.89938307332 8.84252364903
15.6055595521 1.77220921378 8.10859135648
14.4482972136 2.94235343912 8.38278203329
15.1572760076 2.431854434 7.34415592724
15.6665425031 2.24702304563 8.39161942852
14.6528904282 1.84761109283 6.65480532492
15.6737538034 1.23109439468 6.94915935125
15.1628822631 1.18677870869 7.93750507337
14.7226175389 3.37731368014 8.63648103535
15.378769631 2.29467250245 7.33236135639
14.2716776671 2.07455183576 7.9504081625
15.5363215338 1.91466906105 8.46301977061
14.375267043 2.49997388676 9.1796227143
14.5447858998 2.03821414145 8.87037693194
14.3671800976 2.47047358637 9.11776525658
14.9893790005 2.34152528624 9.51742346582
15.3293632548 1.69121885002 6.46633056021
15.6566810416 1.68853767372 7.19678162476
14.2673816973 2.71319823959 8.92248689974
14.5220281472 1.87120467414 6.8396475632
15.0528328385 2.31060527404 9.47061782435
14.3057075411 2.00978784908 8.24480353498
15.316548519 2.17854312673 9.14550259085
14.313785893 2.2638254569 7.86478670863
15.5941785466 2.91336096881 8.58498076983
14.3229079611 1.686567966 7.0632206076
14.3378361804 2.44087319401 7.99286601382
15.5777113637 2.26961609387 8.82939334458
15.6730167275 1.54448760247 7.13359547541
14.3238437671 2.44497559173 8.06044882008
14.5288630453 2.31345054749 7.42380610776
15.4177219837 2.11951134351 7.14704958284
14.4684722455 2.20114159063 8.98583659639
15.6145619603 2.26204556623 7.77834468106
14.2970726025 1.12738352912 6.98632009165
14.5538668645 1.81175585879 8.57936399746
14.3056087193 1.66824499616 7.79042276272
15.0490628595 3.5096560588 8.75540601024
14.2700364448 2.79892918118 8.9222965904
15.3280260827 2.53502201527 7.60382854648
15.4029335774 2.50020357365 7.64372559447
14.2898286164 2.6161185514 8.99104949618
14.6349548703 3.16764779364 8.4307040157
15.3359527928 2.22009567296 7.19130702982
14.8322807335 0.905738662087 7.58244873253
14.3283201208 2.94602254429 8.69869273716
15.3941004658 1.68119129699 8.39664072554
14.8629182519 2.29778420087 7.1490083439
14.8384411439 3.14239127904 8.27486459686
14.9383562921 2.16669153492 6.95032552672
15.2317720107 1.40674757979 8.19148466715
14.4760516753 0.945833701164 7.30047202634
15.6757046127 1.73335254426 7.43502222487
15.2624397853 0.848440479286 7.42819483561
15.0604625481 2.05310247213 6.81311897125
14.6371183028 1.70758582692 8.52598623859
14.2951291259 2.68796805222 9.1486947927
14.5816743574 2.44055189267 7.51607267395
15.036671153 2.02789522968 9.09227861154
14.5874442431 1.79231106083 8.59513107978
15.6459207597 2.59962101214 8.98965793425
14.5698194483 1.63988961613 6.4613626468
14.5236275045 2.49773628298 9.46095471965
15.3612382626 2.1959492499 7.18516857756
15.2765849885 3.23191328224 8.48889895695
14.2794798222 1.86828506228 7.80926447592
14.771667737 1.35969364023 8.16171353316
15.5310213091 2.10263791378 7.32354351096
14.3697551254 2.60484018825 9.3110352142
14.3625513306 0.999974840708 7.12595693786
15.6721825742 1.78316098088 7.76703249412
14.306233194 1.81462276548 7.32325764765
14.3757239244 2.16117782556 8.7113530914
15.6669716238 2.77151355012 8.83665118089
14.6113355418 2.98556276364 8.20966877302
15.1592008122 3.16255585848 8.32664798344
15.5133782312 2.46771847007 9.24811225536
14.694212107 1.9358329802 6.74609485186
14.8651654235 2.46228497096 7.36552405904
15.398993468 1.02819644172 7.54007363315
15.677960921 1.60104991776 7.27791384987
15.6673224759 2.06971789397 7.83045617131
15.4567069769 2.48752473584 7.70477121708
15.1273168957 2.74939961101 7.75980921176
15.2642879242 1.14421947658 7.81983497911
14.3272815338 1.04115927951 7.04682024156
14.5982621051 2.48947650129 9.53763116047
15.0945078992 2.98253626049 8.05078666068
14.6022343915 1.97473229153 8.85758318475
15.6490717251 2.01755474748 7.61222900971
14.9973228781 1.63933775528 8.57976612359
15.1319524016 3.28115113626 8.4711330929
14.5612616646 2.83202968319 8.06301019301
14.5313829863 1.52180234902 8.16085053879
15.0664726711 1.90597111137 6.61435725485
14.7046533761 1.95789496275 8.91741664885
14.611005231 2.56499204706 7.65687030559
14.3495772412 1.68317245884 6.95796054876
14.8303042924 3.06364996429 8.17314463427
15.439934852 0.932504918006 7.35039987872
15.5787733576 1.07163922068 7.25349739766
15.6752345725 1.65125845324 7.49271126247
14.6148344953 1.95147534897 6.82641860055
14.7352402733 3.38515935302 8.64701420625
14.3329792962 1.15811652173 7.22231998747
14.3303857922 2.23013307006 8.6774980964
15.6050189937 1.82475839967 8.16696839854
14.9074918593 2.83535159104 7.84916087303
15.6480165641 1.64664271915 7.75130770805
14.2906087967 1.94434518948 7.57447812946
14.6147107757 2.46839213212 7.52313828162
15.6645883495 2.71204541164 9.03079365539
14.4066978801 1.11067008716 7.38910867743
14.6373258377 2.88495159821 8.04506699413
14.5865022074 2.51534658891 7.61434401431
15.0684412589 1.68758912434 8.63414211544
15.315338572 1.90902572746 6.75281319421
15.0814617336 2.98165923688 8.05652592454
14.3222667297 1.27611945178 7.33737230443
15.3176985142 2.77691274945 7.91475329375
14.292614501 1.42608338564 7.37236061986
15.6172515241 3.15380800064 8.99493655041
14.5854247357 0.8429713172 7.31123508128
14.5328108245 3.38621824612 8.83739700879
15.6497628018 2.32392959833 8.61783673829
15.2584146549 3.33006495698 8.60753072181
15.6495382777 1.90884695095 7.46712883097
14.3508645184 1.87289137957 7.21002316728
15.1215631853 3.32779353354 8.52526722474
14.5301950852 2.45664856803 9.41270497153
15.6118963556 1.93754421277 7.32797838713
14.3037635345 2.56307564563 9.01759536519
15.0296750854 1.54872568608 8.45045704023
14.5305020311 1.88706220225 6.84063040382
14.4470526812 1.28977913657 7.71289038933
15.4852377181 1.60410187725 6.56539254931
15.4137586202 0.862889090538 7.29485843696
14.2767059591 2.05009229706 7.87830369047
14.3864356585 1.32245134098 7.61623580132
14.4083453553 1.90912802365 7.09150355137
15.6678510942 2.10525103192 7.91274244219
15.3887832852 0.914085935862 7.39666214265
14.9633648567 1.94845726122 8.98986301178
15.6621453848 2.03209191329 7.70475728425
15.0033476484 1.73234456953 8.70032819594
14.4533280898 3.05922240602 8.51465646908
15.3157983989 1.55671164949 8.32877685584
14.3875196633 2.15454912318 8.74968401833
15.2113229397 1.16803348441 7.88599947623
14.3827755448 2.86638441845 8.41867594889
14.4163179354 1.9196476244 7.0921284176
14.3143911391 2.0608586357 7.60327141059
14.2772006668 2.39460433931 8.25059287358
15.1608107947 2.27039067663 7.13596679091
14.3419465889 2.82622505611 8.48215253584
14.5129415511 2.02216454254 7.05362368388
15.3854753993 2.0613016255 8.9223795164
15.1053141733 3.46274335966 8.69864478512
14.2751810683 2.68360073227 8.93071513865
14.6542582132 1.77384090112 8.63813118978
15.3698863117 1.47341865034 8.15828330796
15.6268283047 1.65423643693 7.84661404442
14.3327632984 1.75298982162 7.10460815314
14.4718395522 0.952622716649 7.30982606362
14.2938562763 1.26594713402 7.17604872853
14.3789996324 1.49745885454 6.61856924586
15.5797737094 2.0970011885 7.44696259797
14.9835676586 2.90774226624 7.94419918796
15.427846068 1.78621400791 8.49729536146
14.34339045 2.24065845313 8.73475052963
15.6074652361 2.04404847324 8.45845619665
15.573033217 1.45940470375 7.78074782209
15.6283392179 2.32364189843 7.90441667856
15.5295284481 2.58306808815 7.96559490873
14.6119575835 1.6842092582 6.48229238405
14.5805889631 2.25333361646 7.26850812605
14.4699531645 1.66076160697 8.25357034368
14.709772808 1.30176696063 8.04333712269
15.0777322741 1.16557325986 7.9331673574
14.2807670421 1.3378459937 6.95015191337
15.2996742715 2.35246192528 9.39822301677
14.9284505652 1.12872194747 7.89379107377
15.5242743973 2.5189018766 9.3029149033
14.4882744447 2.17740559392 7.29550489059
15.3507235858 1.49580589004 8.20302273492
15.381353842 2.33299643213 7.3962959467
14.4661882043 3.31450585099 8.83811441319
15.4006364609 2.33405435271 7.41459530539
15.4191481989 2.39550776237 9.31927951942
15.645075625 2.02531532668 8.25971924328
15.0083282941 3.45270965845 8.66807944744
14.6217310947 3.36714048506 8.70640177456
15.1242964868 1.942191068 8.95812914587
14.3678058073 1.88693154936 8.32992517697
14.5853141119 2.34966954091 7.39553760354
15.4674135866 2.69930075711 8.0064150551
15.450672117 0.915898295755 7.31128332372
14.4751715438 1.92430447341 6.98204532509
15.6073305616 2.87861764117 8.5901385268
14.5032963899 2.93796429198 8.28056510178
14.5680802138 3.07530047591 8.38297219735
14.2764752902 2.5795140765 8.8251283666
14.5199876816 2.39643869875 9.32201186462
14.7204339928 2.70292659315 7.74663273129
14.4570433948 2.21013368705 8.96846286396
14.7245737855 1.71854568361 6.43624470061
14.4175797474 1.15920782077 7.48004725654
15.6334118337 2.95297260154 8.80929445864
15.1140192977 0.788401821505 7.42407263985
14.6914273242 2.17116253903 9.19188342732
15.5991583631 1.20131216892 7.37280928866
14.519421097 1.64693940135 8.30659276313
14.3579441005 2.11399282782 8.60892319037
14.9671889656 3.1017329524 8.20086970147
15.3414561301 3.00944469455 8.25138186552
15.5886181114 2.48970127544 9.09751465397
15.6587106211 2.05449217587 8.16083917993
14.8832523707 0.768503024498 7.41148916942
14.4773294749 2.48626914034 9.37620101135
14.2765032586 2.91087696042 8.9279988212
14.5281763831 0.989224069041 7.44861374211
14.7123669221 2.9857044927 8.13194658231
14.2706982309 2.64984178023 8.79063531867
14.3824175733 1.90723440298 8.40515295443
14.5519273382 2.51714232128 9.51682552329
14.521475522 2.31925330043 9.21049592969
14.5322258311 2.36899011993 7.48554800656
14.6765382793 2.30605274741 9.36867174939
14.5514751314 2.37698676238 9.32846902744
15.67675006 1.43552348441 7.1937960167
14.2771773178 2.75607269999 8.7176616672
15.4994576206 1.94484663619 8.59464835221
14.7173436897 2.2145952189 9.27201586187
15.3980792232 3.18212204756 8.54746403438
14.5607548727 2.51575860859 9.52586076962
14.914702433 1.40818242713 8.27347445328
15.6624753366 2.10513877606 8.23503796123
15.0889963355 1.88903265135 8.89996522638
15.5102603038 0.909314676119 7.20189093803
14.3264024845 2.37572803847 7.94713808033
14.6117288117 3.18256631963 8.47242391243
14.3299618628 2.74307243893 8.40974195912
14.2674109362 2.93451251522 9.06718899797
14.3500522346 2.11356689696 7.52425518437
14.5028984778 1.04799869622 7.48679968979
14.6852703441 2.59180257307 7.62065179415
15.3544465005 3.30582665818 8.66580729134
14.8200525318 1.31101740045 8.11775543154
15.2366029746 1.83182437569 6.58573888075
14.644369997 2.36664168693 7.35798376327
14.9143656316 2.15717383461 6.95242207298
15.1303930764 2.04652585702 6.82049801118
15.3088061489 2.43284032617 7.45291373924
14.4269684069 2.399539222 7.70056500034
15.2695136567 1.57352315351 8.37785879491
14.9629958106 2.39060307293 9.5874162552
15.3840824611 0.986766606627 7.49368594505
14.2772692352 2.06001368595 7.92377214604
15.6213294223 1.56616428662 7.77118034482
14.3898238509 2.97991428009 8.55715772376
15.5597070405 0.86660501135 7.03707976984
14.3176016174 2.81727283237 8.57241255035
15.520547713 2.75370141986 8.17773622667
14.4412121006 1.04944032624 7.38369515989
14.3141955206 2.36598962952 8.78945341495
14.4186586234 0.901530168381 7.12874767806
14.790780946 1.83153111768 8.79554816218
15.2150094698 1.15420910365 7.86175217351
14.2940770533 1.92240994413 7.52049722864
15.6143546663 1.79238275837 7.14164838329
14.3372145189 1.58003026119 7.81808068371
14.543410215 1.11717013068 7.62977575675
15.623102469 2.34129829568 8.76858523639
15.4795306153 1.74416444466 8.35977855294
14.2747143765 1.99458550852 7.86250234221
15.6080041994 2.17935444918 7.64747450672
15.678815433 1.32169782648 6.99970988027
14.2903049807 1.62781725337 7.62549117076
15.5372081945 2.41931531924 9.14274146553
15.3196037509 2.1964867719 7.14626001054
14.4229974874 1.31112954499 7.68683110039
14.6757455246 1.53859154557 8.32837448539
14.877378988 3.27166644624 8.43349325302
14.7110977263 1.89375518795 8.83551648595
14.7220574642 1.67138658303 8.54958611983
15.6077578784 1.66803369322 6.9446274203
14.2809744416 1.42120953551 7.0306886251
14.7097003769 1.19622341828 7.90078238196
15.2917770832 0.744235465935 7.26319947169
15.6636479137 2.36122638284 8.5592076172
14.275273763 2.86069122723 8.88928962629
14.567856107 1.63199068944 6.46113624209
15.0301815172 3.12472511943 8.23524308786
14.3737879909 2.46247986868 9.12644446314
14.4841309115 2.18569425543 8.98200926315
14.3377740307 2.34307632471 8.84972850942
15.4385553608 2.00229753737 8.7630779051
14.9553932049 2.76368159273 7.74435422853
15.4975120932 0.978515252028 7.3123912116
15.0612759339 2.36390630246 7.22533002374
15.5606194642 2.77315204707 8.30003993461
14.2904054687 1.62135169348 7.16414744539
14.699179526 2.28239500324 9.34901567911
14.9439193156 1.45875001049 8.34036253824
15.3371949818 3.36874653914 8.72374370918
14.2967680773 1.75721299908 7.82490672803
15.500492967 2.84770782145 8.25947321121
15.4094890692 2.08335869677 7.0903232866
14.5015299443 1.2676255474 7.78367559747
14.2757854214 2.56555351168 8.82342095399
14.5397238986 2.99446756945 8.30125427423
15.4067590324 2.21984426904 9.09411614989
15.0063684589 1.58225648778 8.50907343955
15.5670886341 1.00901929273 7.20160059834
14.5973115596 2.42126514778 9.45259286468
15.46295532 1.94714611089 8.65289202745
15.6362062014 1.4593414438 7.57009280625
14.9726566142 1.76562065679 6.41158011957
14.7152444401 2.37225046867 7.30567712501
15.6435256512 1.44105540754 7.5133851128
15.3766506447 2.43283377271 9.41846907131
14.2845454427 2.68022501172 9.04150176701
15.0473722182 1.95544784108 8.99884865815
15.6178275201 1.79760774596 8.08702223602
15.6522922837 1.41606509193 7.42343004966
14.6874337535 2.32449957949 9.40034542014
15.0349733321 1.39527763926 8.24894751324
14.5839309949 1.02996546086 7.5719797525
14.2914234221 2.51843729593 8.89256625309
15.6143577311 1.721115694 7.04101137317
15.4056826276 2.87638365973 8.15474363145
14.3058039064 1.4662064579 7.51753366327
15.677252047 1.78154997142 7.66930144909
15.0869688306 2.89027068016 7.93424693119
14.3491419886 2.89935179848 8.56664327667
14.8151974269 3.23575599829 8.39816102579
14.2934299547 1.46934761873 7.42005820271
14.6535005546 0.738966607724 7.24731418351
15.4763603277 0.860231088357 7.19365501052
15.6185541933 1.73776973209 7.99921499985
15.6712950714 2.70911703036 8.89130254083
14.3246850641 2.28744512298 7.83405696319
15.2258597639 1.85589837585 6.60927319619
14.2936513666 1.39241812434 6.86121374431
15.2877101436 1.24468874047 7.93710888022
14.6698415504 1.2179554185 7.90095607656
14.3441186352 1.55391013303 6.80286634259
14.2894509822 2.65459281953 9.03068189622
14.3082253779 1.98773889848 8.25160268154
15.6419757497 1.39535481918 6.71926313776
15.6616662252 2.33993232066 8.10790066365
15.4144746016 1.62567869749 6.47900295024
15.6640600599 2.11541604891 7.82964095379
14.9492738461 1.79318810268 8.78061750207
14.3108486889 1.39067174267 7.4551764745
15.2034537315 3.20690071555 8.40503043044
14.9627886558 1.64787887554 8.59621803015
14.4440291271 1.89104390522 6.98990206629
14.5039401266 2.1558219395 8.97471560494
15.5467757216 2.2178895055 7.50684234372
14.3933976001 2.41178206573 7.78293405843
14.965173286 1.71043571315 8.6824000803
15.6349958946 2.41089585235 8.06538325557
14.28319496 2.6070095441 8.94929567798
14.3098383811 1.54523015221 6.9564058199
14.7891397157 3.22638573372 8.4083529665
14.3993018657 1.85301702647 7.03459776231
14.2685165366 2.85974718917 8.96240558771
15.1757356438 3.50802581777 8.79568807548
14.4250382148 2.14219716823 7.36581789933
15.657529113 2.00854266269 7.63721849169
14.408849457 3.10584899026 8.67302993794
15.6502516624 1.40786883457 7.41188849452
14.2749202957 2.8422758688 9.19423561208
15.3896874227 1.9401414169 6.87290422909
14.2723636288 2.33189858321 8.25254568996
15.6416688413 1.35786625709 7.40644938463
15.5655766122 2.64974873007 8.13966807491
14.6414938992 2.59260192076 7.66383048764
15.5684128373 1.45432617165 7.79712446091
15.0236865951 2.04059250423 6.78841825362
14.4173878197 1.97926340897 8.57617523372
14.3604840653 2.70724449742 8.27198065563
14.3107662164 1.47303980687 6.84861475374
15.5375339278 1.49466250127 7.91443246776
14.2835756592 2.43695028188 8.69231201371
14.2736859644 2.86528979145 9.1892789084
14.2871193393 1.42770254891 6.92912545661
14.6554286137 3.36642238851 8.67802125986
15.1281610503 2.98937145698 8.07902284798
15.1219262958 2.76562900421 7.77482916116
14.9832635115 1.21795689997 8.02373052705
14.2797890118 2.37216061866 8.23627915582
14.2958629996 2.3825685193 8.73699276791
15.652215783 2.72587556617 8.6151962223
15.2432001435 3.01111708364 8.1672159915
14.5091351658 1.93473306308 6.94100141111
14.5734358414 1.80104956865 8.58188845168
15.6135078778 2.38179873459 7.94737409242
15.4642290932 2.1052712239 7.20679589902
14.8848857978 1.5649503376 8.47646151662
15.0855410683 1.34879944865 8.17810403413
15.5005076639 2.16602883191 8.88508533884
14.2814811292 1.93078671766 7.66677544275
14.2843617225 1.12808886902 6.88205778465
15.6259766946 1.36954556086 7.49262784491
15.5813437581 2.34125416138 7.75782874516
15.0783076708 2.05157544329 6.80966848808
14.4348064372 3.08924940562 8.59242334724
15.6223736489 2.15681694316 8.53358351868
14.9546102466 2.36929770026 7.21950820404
15.2386036342 1.25904099365 7.99000399765
15.6697126372 2.00205082018 7.80007165725
14.3114032025 2.60648762128 8.31643751154
15.2261267471 3.09582389988 8.26807860847
15.6426084774 2.7947298591 8.64984514321
14.421957567 2.79519034143 8.23584701824
14.6645229331 1.87188442278 8.77631339753
14.2963855873 1.09933121372 6.90721556126
15.0789907033 2.01339424496 6.76180293948
14.5516313023 2.14599374192 9.02555033227
14.7392073166 1.09725889308 7.78121930026
14.6034860101 1.44580983744 8.14445199004
15.6584296765 2.76549224224 8.71958659746
15.2428486151 2.34555571627 9.42707542442
15.2830245872 2.44387042271 7.44112926512
14.2732015003 2.77747661644 8.8185032173
14.8813584327 2.78545241791 7.78916840931
15.314064039 1.88744039717 8.75758052532
14.9756481936 0.779563063362 7.43590190328
14.3764787071 1.34162798706 7.632170117
14.4726327093 2.36625248453 7.57235123207
14.2773446686 1.81483002682 7.75315151042
15.6239230039 2.87207088162 8.64141354617
15.6653722938 2.60203625359 8.84144571115
15.0663184086 3.31500426739 8.49260832279
15.0728411555 0.797383509283 7.44696289653
15.2417018219 1.32874163849 8.08017062898
14.3686490499 2.58185996153 8.0664319174
14.4231062584 3.09988091971 8.62775613017
14.3326776722 2.45456905169 8.96978572011
14.2884764768 3.10512016654 9.09549097693
14.2987429717 2.32593718509 8.65144675547
14.9161686486 3.13003253607 8.24165333884
14.3710113784 1.74911012041 8.14388994853
14.3648403368 1.74049481425 8.11728751874
15.0204061661 2.35304463493 7.20372068596
14.3054289951 1.40792109921 7.43221514855
15.160491082 2.18133015494 7.01263864829
15.6526207615 1.37602595346 6.76230085959
14.5469014806 2.38435713716 9.3401525981
14.413055737 2.57676291391 7.96132283622
15.6234141301 2.98085648788 8.78378364687
15.0213957473 1.89442053363 8.92370870472
15.6451937119 2.48239694181 8.83700018639
15.3935023019 2.30769761142 7.37507366371
15.0638570783 1.81440674623 8.80781117416
14.3423095329 2.65680033367 8.25801064326
15.4472666029 2.15667848903 7.24307202507
15.2849833026 3.13151570413 8.36158562686
15.6169917988 2.20117973264 7.70666794775
15.2295579605 2.79258315403 7.86395426417
14.3411765447 2.0764129152 8.50029739768
15.5027040139 3.29887652732 8.87163906327
14.3858819281 1.77864246804 8.23851316487
15.6711582554 1.76989366389 7.46317068675
15.6298311002 2.6898829908 8.42965173587
15.5670746253 1.85487230616 7.06492865509
14.3521266075 1.74519367527 8.11250714764
14.8830493998 1.69516639728 8.64881075586
14.8185904147 2.65367291526 7.63106822959
14.9364046754 2.92759030122 7.96842950244
14.2785169653 1.42389179461 7.07279176574
14.943436986 1.91702477096 8.95095626526
14.2900320839 1.60520951958 7.16147967353
14.7882619254 0.637219035389 7.20470037138
15.2739128411 1.93018751389 8.84558277425
14.435065389 1.64049312939 8.15501097938
14.506650375 2.57091662349 7.78978383416
15.457601427 1.71723007111 6.6715293107
14.4456518727 2.45638285111 9.29200750568
14.8353892669 2.01752110427 9.05953257257
15.6652107472 1.46265528012 6.95436363664
15.6721897912 2.0218553029 7.80944096826
15.526975145 3.20811736664 8.80257530224
14.7653088742 2.12642103608 6.95079378373
14.6581356356 2.14719894576 7.05810784342
15.537424295 1.14800146091 7.46600169712
14.7451177865 1.88542980402 8.84830766042
15.5560926725 2.26748234265 7.61243690162
14.7542597142 1.44505040659 8.26158778569
14.9887496284 1.51080565589 8.4116493028
15.4652453316 2.81064558937 8.14810717088
15.6768260456 1.64399613391 7.29663209257
14.2790341079 1.32070293778 7.01869766187
14.4807444318 1.95313208546 7.01122555936
15.4906836586 1.01883892963 7.3883188162
14.5179258631 0.993084505921 7.43597042767
15.2171346216 1.14630828894 7.85678211253
14.4738598578 2.42429701051 9.28267907744
14.7414512353 3.05276235123 8.1991802251
14.6518673795 1.86626135308 8.75256440434
14.2712095779 2.94390915003 9.10844431022
15.5220431232 1.62193672477 6.66771471578
14.3261257165 3.03190019498 8.80957894237
14.3492534051 2.36937433766 7.86196249284
14.2793383577 1.9327197031 7.82739078664
15.6345324162 1.72458575052 7.90970911736
14.4002964663 0.967054381001 7.17030365926
14.5535528708 1.84796962398 8.63486430702
14.5888844024 1.73722318925 8.51628331653
14.9556989237 0.988044590965 7.71800192702
14.4254560584 2.25449860342 7.51155357307
15.3725749181 1.99207079867 6.92653804903
15.6446537403 2.02258588894 8.25095802315
14.7258091029 2.31071450529 7.21424321998
15.6711509551 2.53686595221 8.58808379886
15.6187442569 1.65834097345 6.97112419054
15.4669496413 2.30836590392 7.4736985391
15.6256325972 1.50914438703 6.78927005542
15.6011737642 2.57540249306 8.14807261593
14.2950882374 2.53443775751 8.93656791998
15.4306046664 3.35957225249 8.83164717703
15.3421858331 2.0882498587 7.01105788512
15.3167019102 0.712262392924 7.19868236571
15.6523347067 1.35829229749 6.74877681928
15.3926688556 2.19964186672 7.22636325508
14.2737240393 2.22562962767 8.10520511485
14.5515263637 2.86779582746 8.1211614199
14.4781536716 2.1401718995 7.26569618699
14.5699969049 3.40638574519 8.81256455149
15.5528340142 1.86549400823 7.05635684953
14.2732093042 2.40358081349 8.4807979
15.3692149271 1.94890025444 6.85989831839
14.43302189 2.60073497235 9.45765957175
14.5819416533 2.54014369328 7.64517817833
14.7521387231 1.80811867539 8.75622998261
15.2337330756 3.15499106386 8.35434883807
15.4319071935 3.15985947676 8.56884761247
14.2871370135 2.40698536241 8.71342263877
15.352712225 1.96311921415 6.86460493179
14.7917599221 1.76858053027 8.718591446
14.705015467 2.79357151763 7.880305847
14.4359673618 2.53584145908 9.37412700909
14.3017351873 2.52108198862 8.25581240071
14.2756610381 1.93852152147 7.89608919702
14.2978133046 1.77005841626 7.89355662034
14.2738854977 2.44422767525 8.60802872847
15.1386236177 1.18524045201 7.9468555516
14.5404772325 3.24502593243 8.62972947033
15.2811809764 3.44578338909 8.78004221538
15.6111251913 2.58966961983 9.13597410338
14.4212731143 2.14118901367 8.80607360535
15.1459073276 2.19601117956 9.28547375445
14.8221692389 1.38358825999 8.21208925122
14.2712357197 2.48214491996 8.64768070075
14.472764776 2.57821724231 9.48824342443
15.669186228 2.53131641755 8.47855246859
14.9443328185 2.39412490709 7.25362045462
15.3319339959 1.27944929254 7.94421377981
15.5470106414 1.99871591228 8.55549307026
14.435481618 1.81260992657 8.39398041194
15.511863282 1.93278949865 7.05414501881
15.5165095862 2.56482951786 9.37682226168
14.2872420855 1.68903294485 7.26835415072
15.1392655903 2.05355316462 6.83179323305
14.4049943873 2.39076354739 9.10398476286
15.1042118182 2.49970907762 7.41423295889
14.3318381436 2.51430562364 8.11090785278
15.1611553596 3.36509990231 8.60571492369
14.2767595125 2.26356469393 8.19029417672
14.6554113335 1.83332374024 8.70981155265
14.93221294 3.04522920478 8.1271353156
14.9721049509 1.70716738136 8.67054959593
14.8610953016 1.80459070398 6.48299261672
14.3064409907 2.57858793195 9.01306036656
14.3659514023 0.970387389331 7.08031074068
14.4353982621 0.873301138182 7.11851715324
15.4220977889 1.44434375363 8.06361292669
15.2290412988 1.82898803934 6.57690620018
15.3861557658 1.34231862843 7.96635574242
15.4437368765 2.44490575781 9.35019648229
15.0354739437 1.83846896644 8.84242889913
15.3699625969 3.04334363168 8.33063511655
14.3622165894 1.9770126184 7.30730091255
14.2808447456 1.64888111376 7.30298523129
14.5874217349 1.24272364986 7.85717764007
14.7219930956 2.84547041621 7.93445196394
15.6643114661 1.35332044145 7.24387217798
15.0401964198 3.1530745283 8.27435324053
15.1511023155 0.776476733753 7.39649407778
15.4706553398 1.3719131022 7.88405219936
14.2994852358 1.56383696815 7.59989271924
15.3718208947 1.37091176504 8.02598479545
15.1537343051 1.20523277393 7.9700259296
14.3364011943 2.15516420823 7.61801013716
14.995592264 2.04860407419 9.13217632644
15.2761082016 2.3449857632 7.30338348083
15.0159230528 1.23096544793 8.03410353294
15.6771219351 1.80671404145 7.62309920287
14.6818907364 1.88581844191 6.68878560464
14.45752234 1.72701215085 6.7457414457
15.1790008893 1.3577169928 8.15610912042
15.6226505482 1.87485109035 8.15656670604
15.5131781654 1.74501704204 8.30008837358
14.3930694134 2.01355006188 7.27681936959
14.3152852237 2.99769273546 8.80971542246
15.6630165741 2.08191801069 7.78102890532
15.3836171705 2.47621916193 9.47226758628
14.6124069396 2.79216359159 7.95145647996
15.6635994403 2.36338996556 8.18907984294
14.3430068102 2.40896778226 8.96024986499
15.6005665807 2.76165392805 8.40710798947
15.0150768048 3.17686388681 8.30688326462
15.2897405885 2.32099335123 7.28394813922
15.5182580689 1.03153559469 7.34167678376
15.6252521165 1.30857749503 7.41566964222
14.5750882493 1.0130514995 7.53089331688
15.2564247064 3.02518106537 8.19682265054
14.3212929837 2.69340145538 8.38369184721
15.5979760897 2.66818387426 9.29416087132
14.3079093651 1.39577076972 6.76014008948
15.3075942727 2.16915226996 7.09208176572
14.3215006896 1.15124964858 7.17714580631
14.9591127807 1.77568387055 6.43217562176
14.3722812102 2.33225231312 8.95135607164
14.3104174092 1.05323739309 6.96478642107
14.4142440319 3.30103152241 8.91275850911
14.5320126199 1.95379615556 8.74951968861
14.2810661334 1.97549220483 7.70229690126
14.2870996097 1.11641528115 6.84749917809
14.3262158437 1.84463763662 7.24830155823
14.4263930483 3.31909614006 8.91274599718
14.77409162 3.2732644063 8.47099276931
14.5428002519 1.6683750913 6.53748875946
15.5452931747 2.52794949085 7.92746023242
15.2089704095 3.17362137995 8.36501299868
14.5415776354 1.97225891059 8.77668883176
14.2796336559 1.2707816264 6.89575034828
15.4194430943 0.753624187981 7.14688614032
15.5795490737 2.05707715765 7.37992708678
15.012683314 1.37448531693 8.22590512948
15.6554239468 1.17735388119 7.08846904489
14.4664891416 1.16825798878 7.58157044972
15.6688924201 2.93812669838 9.1133666203
14.7275056995 2.08804626409 9.1132189421
14.6044236243 1.04139065707 7.60524193086
13.1605369549 1.65589561446 7.65312573241
13.5056592861 3.0666690223 9.4386560293
13.4168622079 2.1481636361 6.08308044477
15.5033387456 -0.0079039843033 8.68400991235
13.9912485504 1.32954049737 7.21701111032
13.4890555014 2.22416053381 6.95352202285
15.1626852961 2.67518257609 9.65725149172
14.8972462854 0.093583485334 8.96277985522
13.839568175 2.65362485834 7.19487618793
12.6735555761 2.39976192477 9.03389254302
15.3041786308 4.80675978358 6.74258515823
14.2235473788 0.188828237685 7.06403651514
15.4122796019 3.38274191447 6.89604038695
15.8647184318 2.11788379106 9.78904671916
17.0060440513 1.674747979 8.2083206501
13.0147394524 2.44469251816 9.86349455131
15.0749547682 3.49518816303 6.71138144295
13.3581705539 3.33325779828 7.44746208851
16.0029610296 2.78946502235 9.44414590206
15.5834489973 4.28138131369 9.6620876875
15.4234374051 0.577983142393 9.05585743562
16.3585379855 2.73177443469 7.50068307668
14.3366180812 3.90136260176 6.49767067895
15.2555710455 3.76848608656 8.80670269429
15.7296830217 3.05347601001 9.30955343888
14.3496919952 3.47792301169 9.76525037617
13.2583476634 1.36636845644 9.91104407111
15.7061466719 0.233305953696 7.57384398155
15.837792959 1.50944341811 9.61708825881
13.8981681786 0.236881478121 9.24508518678
14.9412170989 0.45492305294 7.22327361011
13.4294388008 2.08420662985 9.45371623184
13.2479873626 2.55596058435 9.4199644544
14.9014123492 1.51038239087 9.52222565856
13.8158327178 2.12099374507 9.79276997947
15.7991507767 1.44958518978 8.18194662117
15.2485642989 2.19173148367 6.83285123264
12.6386949074 1.98628252554 6.01203691945
16.0789117361 1.84428504696 7.22269068252
13.3767756686 1.20220995879 6.79193925932
13.7561597217 1.32923277771 7.51842238598
15.0092163024 0.84708758221 7.73344557627
14.6226399824 4.15245641537 8.04634842834
15.1815758799 0.657213803698 8.95578911197
14.8330975897 0.275256431806 8.35822173903
16.0254737801 0.647787093313 6.00146607115
13.9583848658 0.139775784954 9.32031433775
16.4548588865 2.17258041257 9.74898199347
14.5414786498 2.18401138277 8.99257039701
14.5082214575 0.135805057927 9.40482872929
13.6180077645 2.27379888814 7.80403792683
13.3299960083 3.07094748818 8.57257631577
14.1569468286 0.37966515255 6.93654558999
16.054133008 3.59283693167 6.85732531078
15.6059311602 0.66760040642 6.02884755791
15.3055439937 1.8586304789 7.39673835598
15.3632910029 3.76886267055 9.4207332512
14.4914481963 3.85305528171 9.36009158804
14.1426664848 2.58723677364 6.82088058476
14.8251892261 -0.538174539806 7.75635376155
14.546480515 1.10103992022 8.80839872145
13.0471385514 1.532110289 7.54367722938
13.5221565356 0.936669992999 9.00332937394
13.3729961708 1.91873960284 9.60488529743
13.0693771878 2.7489729926 7.00125039436
14.1400860598 2.34142446729 6.6822717407
14.3634241916 2.38802515576 7.24095562909
13.0002167659 1.9076574312 8.85692136701
14.4275372087 0.944287153626 6.84007416665
14.2022795947 3.28168024521 8.66263136817
15.509491642 3.18342801575 7.59512563276
15.1939712934 0.198942805406 8.77599361393
15.5963858479 0.202108827872 7.42988389968
15.895202741 3.46394711844 7.69730524101
12.9588049512 1.79511904354 8.24663858386
13.5160760094 1.75220190018 9.14553543572
12.9599060351 2.75502865357 8.25645609311
17.1555807785 2.47607092316 8.53147621035
15.8681592089 1.05353998784 6.51395208121
12.7218331125 2.31962756992 9.85535419374
13.6461491078 0.778633159446 8.38703126919
14.9450253029 2.51097207844 8.66916808418
15.2417358808 -0.175959028439 8.84476692003
15.571461314 4.54583011066 9.86002698063
13.6558296253 2.39010385057 8.66874674919
13.6512543414 0.844495464056 6.87513362802
16.2906043928 1.86435763767 8.21050016372
14.9933930361 2.48282904307 9.62867685694
13.2287464575 1.76376938149 7.92899726928
12.694722241 2.52079779952 7.39106546734
15.8991441221 1.64391991432 6.47405823131
17.0066094165 1.60189083643 8.88378451064
17.1256029267 2.36904513157 6.25161227682
15.4349847456 0.438705779472 9.9845455074
14.5217472666 0.069733588451 6.16070530212
16.3194541922 3.60632850627 6.32044728365
13.5967457697 0.658104339401 7.07881034125
15.3449760819 3.81041634983 6.98443269185
13.9703260734 0.114120185323 6.60699321235
14.7873221986 0.538986345709 8.67023901576
13.7844119494 3.20190235658 9.26010746546
14.9697258001 1.94372233567 8.02572972409
16.6734590803 2.97043800233 7.72499389881
12.9224977349 2.52294288362 9.17168665365
14.8484004346 3.4002964867 7.51809519337
16.7256958677 1.93120222624 6.00239541197
14.9178675226 3.28899298178 8.29202427118
14.9829463585 2.31535615398 6.15169035606
14.2866003405 3.38268964136 6.07958718408
14.159101641 1.55014035459 7.58859484869
16.3969458251 1.42463078962 8.44740033322
14.2883168588 2.98218679949 8.54464889791
16.0795770064 1.84626561089 9.66089953857
14.2433944257 2.10768156983 7.33504342013
15.5348674623 3.6611691777 8.30126742163
15.3492666244 4.43677921059 7.46733240318
14.6487402783 0.942638067496 6.97938176701
15.2442482138 0.308653983231 9.05747936911
13.7028368518 2.90863801944 6.03620664687
13.2370974668 3.07087619786 9.47787839071
15.4083197639 4.77434362974 9.5538271938
14.3061680442 3.91139837603 6.58106941342
14.6719951402 2.20332390201 9.12583933979
15.9751695377 1.32907866417 9.48244069547
14.6517672895 2.55226181632 6.04210529008
17.6658296094 1.74571962292 9.94585229713
15.4502545367 2.28221961213 8.57841480218
13.8352856844 3.41000093182 9.2587708327
13.596114875 0.672375147358 7.28498909676
16.2994071122 2.18023234908 7.3989372206
16.246665701 3.34107992776 7.33257147285
15.3297373972 -0.118354516502 6.82489080488
13.4245442696 2.13486159551 9.83389608342
15.666740178 0.408221395181 6.08296995403
15.590983018 4.47403397588 9.50902431129
15.6464095101 2.62454959356 7.29389901043
14.5292267848 1.67628891036 6.79796763793
13.6527782978 3.31430230849 8.08072644413
15.06938368 2.07310574458 8.02348458574
15.8630849065 3.50287251229 8.94567269827
15.7066662002 3.93307558013 8.8261130006
16.2583807775 2.62534134084 8.44780229572
13.3174208619 2.69198196374 9.27733723138
14.5357281761 2.48913088499 9.83213222824
14.2071510646 1.87840867858 9.50311691553
16.0322280796 0.774716080039 6.79526189322
13.9831472641 0.654911190072 6.25172630342
16.5269726805 1.60875372228 6.05198101422
14.6618632358 -0.483347537738 9.30228181434
13.8094986095 0.55458240083 9.36939494893
16.5545935884 2.16597897623 6.05640597546
15.7209901367 3.41015516164 9.66283687055
14.9635466306 3.36146035336 8.47345316396
15.2408357982 2.89312181566 6.47418097877
13.6262628553 2.18581553054 9.7365661689
14.6423631357 0.259827029779 7.25600868267
14.2482872311 3.4457443054 6.63300906294
16.4559225685 3.12027055027 6.21060212301
15.3840935989 1.65785579542 7.60092278265
17.3805489932 1.5485773415 7.17681354743
15.4786773228 1.72504012566 7.97412518075
15.6586938278 2.40203285387 6.36072255946
16.2148091299 0.798288155582 7.38707226323
16.3791571545 1.89711198495 6.91948261595
15.1418518737 -0.19905492536 8.30723477882
13.4023409413 2.04471953089 6.27450337265
13.4567213128 1.5580255701 8.46268584756
15.1308763374 0.443819653967 9.38150796974
15.4114497709 2.28658787201 6.540066528
14.1632143262 2.04697503333 6.5862787605
14.2777830957 3.38137182926 7.62517380766
15.3085749287 1.88980930228 6.4871750577
16.1327432357 0.863080083583 9.57412072435
15.3183905038 4.4487709559 9.49191149843
13.9185377155 1.79094889197 8.98718426833
14.2882929674 2.54459341052 8.53464442533
14.7001525931 -0.400142958822 9.6067570179
15.7253708653 3.75394606419 8.09967011315
14.91741444 4.11263952046 9.01198446832
16.6406817792 1.07180702222 7.01282342516
'@

Write-BundleFile 'samples/cylinder_partial.pcd' @'
VERSION 0.7
FIELDS x y z
SIZE 8 8 8
TYPE F F F
COUNT 1 1 1
WIDTH 1680
HEIGHT 1
POINTS 1680
DATA ascii
11.8682262879 -4.30696218953 5.40660986962
12.1464063471 -4.12803175372 5.86216625494
11.7195871077 -3.90758524878 5.80206629056
11.7421705727 -3.89227315845 5.83017549005
11.957097629 -3.63973871603 6.25902949532
11.2736389563 -3.848798158 6.90658908798
11.8422830953 -3.98662401904 5.74349515205
12.0023570136 -4.42376056469 5.46640025073
13.0280446563 -3.06674150218 7.73048402677
13.0391255697 -3.78034926124 8.05149743701
11.7661779727 -3.14957990217 7.61059662302
12.8174397476 -3.04983127031 7.50781262758
12.3128707794 -3.07255603883 7.18992507863
12.1128452356 -2.92379081913 7.52162143631
11.2827661727 -3.84610344753 6.22657271448
11.641658387 -3.37752790387 6.99434033918
12.7401674076 -3.93494009111 6.96017103998
12.2395423676 -4.83401633666 6.13313429445
11.7237420641 -3.25241395313 7.92153276495
11.7749960184 -3.9366299397 5.77690786926
12.0547763845 -4.42125889231 5.53805304872
11.8150016278 -3.14783305398 8.14908599399
11.9002817395 -3.05295916935 7.49817080807
11.6350234592 -3.44700115576 6.76035109307
12.7539421923 -4.13241742014 7.21306627334
12.8792153486 -2.9421407973 7.66662800793
12.93379895 -3.64201426294 7.37321040443
12.2670412141 -4.76125040144 6.08015061113
11.0837679957 -4.08812462261 6.4550886587
12.1711915729 -3.4160985984 6.64617296959
12.0262900624 -5.12940353127 5.80275520639
12.312519773 -2.96805933651 7.35943806855
12.6423065485 -4.24056518935 6.87467808034
11.2926962558 -3.79649089229 6.8838745027
13.0320532778 -3.39458730387 7.59160352678
12.0876808528 -4.10247300015 5.81446692657
11.8464396543 -3.36055320694 6.75393258205
11.6109154116 -3.63908587984 6.30215471447
12.5814120233 -4.36037426917 6.80647557832
12.252309449 -4.55423830011 5.85035029456
11.2615516851 -3.81911409102 6.58735040928
11.9812656921 -3.14083030117 7.13416781786
11.3294874996 -3.79964407141 6.28127766681
12.274952201 -3.35845745924 6.76901944117
11.4740369815 -4.01446818325 5.62519173104
12.8650400617 -4.00813239767 7.53900448465
11.7079360275 -3.53043971929 6.48141018675
11.6834141148 -3.43377422698 6.7292485391
12.0941493894 -4.57132090914 5.53549784789
11.9698556964 -3.20233497737 7.00597517574
10.8958904633 -4.31658049319 5.92656629678
12.0874641909 -4.09372748251 5.82376029813
12.9402327676 -3.14479491536 7.56894127042
12.4097878759 -4.57365074004 6.41294855871
11.1170041174 -4.10094254024 5.73460590212
11.2650714424 -3.85749034268 6.24113511184
11.8586971209 -4.28861131999 5.41371351539
11.427845422 -3.67356249418 6.46545750376
12.0520257175 -4.4419923488 5.51982937639
12.7183877814 -3.91862239798 6.88983200124
13.0081785175 -3.0164154947 7.74764555471
12.6707568697 -4.21874138268 6.93292200549
12.2916109447 -3.20973263799 6.98447477552
11.2483078505 -3.93464367383 5.99255465714
12.2571005243 -3.32361910792 6.8087814893
11.8863546839 -4.29066154047 5.44267215651
12.2632127407 -2.85791306222 7.57641275868
11.3533348983 -4.07894034845 5.52725806312
11.8018857193 -4.17685726753 5.49178711109
12.7233914075 -4.09645983349 6.97748549933
12.1946335419 -4.88564784483 5.9806493889
13.1265465109 -3.45423091804 7.81587438504
12.2553095806 -4.20997956921 5.94793790253
12.3157514682 -4.72315529636 6.2420052536
12.900836846 -3.63206158636 7.29315819622
13.1581723446 -3.23182695611 7.8697479002
11.817826316 -3.13429810386 8.15200835494
11.5570843611 -3.43425255902 7.38433848283
12.9040896454 -3.78461240851 7.33523436194
12.9533005418 -3.63411395945 7.41760879763
11.7086196175 -3.33803491194 6.96111960161
13.1468071717 -3.26380483434 7.84440913365
12.4917860124 -4.16527190563 6.39081749131
12.2557865696 -3.20975062126 6.97578384905
11.9286716809 -2.93538370763 7.94566111558
11.8649089546 -3.01476335419 8.00814406018
12.370052567 -3.83183868474 6.356443816
12.3729428858 -4.57946308612 6.23943347806
12.6386307395 -3.71701124568 6.79105884788
11.2468737554 -3.86995791159 6.82904862311
12.2222469202 -3.22173013142 6.94395064406
12.4325041904 -3.15043371174 7.12251737293
13.0630165433 -3.74938995971 8.02050025982
12.7781388482 -3.84820244218 7.02977476019
12.4802923322 -4.52154860312 6.71317057113
12.1784025148 -4.51129275879 5.69754768024
11.3274035221 -3.8067114589 6.25396372489
12.4029192612 -3.0071194725 7.30764395428
11.4255627141 -3.68577160846 6.42641974356
12.3329645461 -4.11492555126 6.1159329581
12.5247129966 -4.16450366606 6.46253808868
12.1478453094 -3.06061755768 7.22418123495
11.6419299466 -3.36099413182 7.04319448722
12.3205269188 -3.21752859931 6.97999733466
11.589999875 -3.41696494579 7.5919801331
12.6064460202 -3.30059010259 7.07062996961
12.0704990209 -2.92851932793 7.56295791695
12.2886005287 -3.76248116475 6.32399519134
12.516965295 -4.41326947326 6.60192953702
12.7584431644 -3.79478117198 6.98811013153
11.3042584293 -3.90930173203 5.96687417759
12.2752902433 -2.79147789327 7.70095684752
12.2440984476 -3.92343889032 6.13921113106
12.3245350182 -4.72091966049 6.3313246829
11.9605007307 -3.46273316688 6.53716667095
12.9421921569 -3.78103669091 7.44739932654
12.6719871063 -3.40415528034 7.05250242523
11.1951234799 -3.98162916019 5.97225513475
11.8029011326 -4.25207931233 5.40674166396
11.0743174443 -4.11420159233 6.45543167017
12.2044462741 -3.48611260413 6.56812229301
11.2673712025 -3.81607389852 6.54297764268
12.3034668492 -4.69951005565 6.10457228887
12.0880535935 -3.02024501577 7.32750366222
11.2132651712 -3.97419165245 5.93289087374
12.3862928749 -4.65351825978 6.6072588242
12.1999458155 -3.19128434589 6.99640531851
11.1690363551 -3.9886353049 6.65663041008
12.0369667103 -4.47850455311 5.47580759981
11.6265881227 -3.76710141249 6.03709823523
12.8397423262 -3.97533605676 7.30085635241
12.323067845 -4.59374542659 6.07428324571
12.9835362504 -3.37880524061 7.50767455985
11.8452822445 -3.05102527611 8.04781408786
11.3075238644 -3.82860448898 6.20857136995
12.1501117623 -4.82934399318 5.68924702392
12.5522648613 -3.55181478339 6.78090747229
11.8603350266 -3.36151012056 6.74231082681
11.8210661886 -4.0631224222 5.63620204312
12.9134134252 -3.93972366307 7.70396901333
12.9668491457 -3.08922506146 7.64269128277
12.4653493979 -4.28423349818 6.3294163318
12.2585333486 -4.78234893354 6.08745000969
11.7065814097 -3.95760483584 5.72661432302
12.206022117 -4.88909675507 6.11595874071
11.8786196852 -3.30801041506 6.84754453491
12.4790135852 -2.77216565571 7.6861322591
12.0517663406 -3.6517522768 6.27477576377
12.1051460346 -4.05469592886 5.86935709271
11.5585180462 -3.86109897567 5.87726363506
11.239782881 -3.91236538794 6.10723776142
11.4971853904 -3.579181292 6.64472913233
11.6215172794 -3.36318847963 7.60980792191
12.2137386941 -4.43748276814 5.78761366394
12.5152455294 -3.02072890688 7.32753069533
12.1371896186 -3.90624758474 6.05125419574
11.7638094755 -3.21254022622 7.27585447133
11.6988577921 -3.31903033011 7.06022272348
12.4690709854 -4.07919086575 6.36220886888
11.666278634 -3.27572447587 7.50895109082
11.9231931149 -3.46693252155 6.53061881363
12.3791031472 -3.38949370462 6.79241573097
12.5165334095 -3.70817929988 6.61958996471
13.110947464 -3.64323433778 7.95859980351
11.931745463 -2.99878309852 7.60111030624
12.8031481561 -3.97153587925 7.15373975527
11.1047190174 -4.05379619342 5.94257793525
11.1679204778 -3.94314696401 6.44572142967
11.3182362288 -3.77548387583 6.41677454946
11.6908876719 -3.79408631658 5.98338520479
11.4126431671 -4.07327361723 5.52726580344
11.8813029604 -4.17938725849 5.54844144788
12.0817598121 -4.15451834731 5.76588827223
12.0169626442 -3.19185403311 7.00853168305
12.5604729587 -2.95139527234 7.44143400162
11.6605537641 -3.59276918501 6.37281107107
11.2809810313 -3.79886245963 6.79108470237
11.356840442 -3.71802447211 7.03768647015
11.4037521487 -3.66902185165 6.57456391313
12.2157400464 -4.59747557504 5.7765277531
12.229605598 -3.88131572675 6.16333978867
12.3727205074 -4.28081459132 6.1313563258
12.1755611266 -4.92303864201 6.05606683928
12.5516014321 -4.19076309959 6.52049638453
12.6690996451 -4.2707344051 7.19427260641
12.1987078744 -3.97440542317 6.0429584375
12.0710464338 -4.0707125129 5.82835950879
11.288890771 -3.78962041316 6.69286295657
12.8616341451 -4.01279546566 7.62835806893
12.2169000481 -2.93692725098 7.43306769364
12.1212901126 -3.55853035127 6.43462217098
12.2800222406 -4.60379097951 5.94126133399
11.4664879358 -3.55515357233 7.22386740145
12.7665788284 -3.12787306313 7.38604072508
12.4533282463 -3.54021279334 6.69569933432
12.9380525961 -3.87359941761 7.56303841511
12.5038204243 -3.16490778297 7.13995101774
12.4068336326 -4.14436915377 6.22917435338
11.4476307213 -3.65682186362 6.46697744288
11.8373328323 -3.34378059575 6.79794020035
11.6243800902 -3.61841727369 6.33529061814
12.8106847794 -2.87507482107 7.67868174636
11.7969590916 -3.29844352226 6.94934981574
12.4650005688 -4.15293393128 6.3319869668
12.8915851849 -3.3214691891 7.38627161614
12.4273969199 -4.59205012782 6.61049793814
11.7534795206 -3.47656998983 6.56343416704
12.0374233697 -3.15136940547 7.0967935537
12.743548014 -4.0305139494 7.00260646213
12.7797889126 -4.12058613474 7.38973917388
11.5179674137 -3.53890171749 7.49027995732
12.381046841 -4.01189420946 6.24059210366
11.8769259642 -3.04164325808 7.58331376778
12.2684018601 -4.80585262083 6.34551257472
11.7569122851 -3.79510978588 5.98062008373
12.1477718672 -2.78045224041 7.86647227524
12.2673442461 -2.72666471514 7.85353032954
12.6796246418 -2.81947490208 7.66859116434
11.7827172986 -3.65054070068 6.22119031426
12.877144398 -3.89229229381 7.33072674709
12.4663682816 -3.85650247778 6.45881185531
11.7226482268 -3.60392422543 6.32112738585
12.370653601 -3.76134003638 6.41435906291
11.9727940416 -3.30649137346 6.80701597362
13.1596410082 -3.53499675975 7.97832486428
12.8221755901 -3.33486340008 7.28335721425
12.2894887733 -3.38114391237 6.74800114406
12.5876588926 -3.91173038175 6.63433248212
11.9787385205 -2.911730304 7.79226203236
11.3636907005 -4.07078303644 5.55172245297
12.7082489929 -3.39014742363 7.09936136935
12.3109165447 -3.01723388921 7.27669774996
11.6062877856 -3.42774873063 7.68961331412
12.2505332779 -2.82165790307 7.65178073459
12.2599453842 -3.37297882365 6.74178879157
12.3525608002 -2.96823891905 7.35820396625
12.8972675101 -3.29361270724 7.41647697664
12.8539001959 -3.749601005 7.19713547606
12.5687702665 -4.2812747652 6.61449685011
11.0640330287 -4.10130722175 6.33313787964
11.3965347082 -3.63465943014 6.93513047265
11.8957892657 -2.98603643116 8.21306472418
12.1983648484 -4.09974203389 5.93901422069
11.6588918056 -3.29609097764 7.5914642408
11.611678195 -3.40175021281 7.67836449034
12.8238323885 -2.97886894508 7.5910150738
12.8983714643 -2.94452380353 7.69122770528
12.3971083128 -4.63525895685 6.60025351175
12.0682178329 -4.57524246656 5.48443834528
12.3821178586 -4.57585023079 6.25707439848
11.1825732486 -3.93630218216 6.26283873679
12.7935577295 -4.09816872592 7.49337426718
12.3095872937 -4.62477603584 6.05360899354
11.0225592428 -4.13289701449 5.98867992024
11.15688667 -4.04712615792 5.84984720985
11.0358382873 -4.11054685882 6.03674768163
11.9963735085 -3.53676081575 6.42583320876
12.5698424501 -2.80855520467 7.64920203217
11.0769784497 -4.06196512533 6.19056893959
12.4245137578 -3.19416605338 7.05662181837
11.7477642637 -3.18552586782 7.49136686973
11.7767586127 -3.53208066373 6.44512069598
12.2275476527 -2.89996941676 7.49856242694
11.3749643405 -4.00444363706 5.68020107057
12.4879372282 -3.82792878128 6.51072542018
12.8294071368 -3.19395242002 7.3936815097
13.0749999902 -3.26129036039 7.70627858363
12.0128744221 -3.0311935791 7.36855222469
12.1598160066 -4.95644111291 6.05877439769
12.5279625288 -4.40094717432 6.61438288996
12.1582009114 -4.42996519302 5.69608001297
11.8897185757 -3.63481804102 6.25028514752
12.4392689772 -4.5471234684 6.48922437981
11.6645350609 -4.15237327396 5.44059355043
11.1560159613 -3.96801736802 6.22308555902
12.4431995566 -3.0148809323 7.30762178553
11.7337539208 -3.19006245524 7.72026986461
11.0878100039 -4.10954764467 5.78500665712
12.0107029487 -2.82519859097 8.15151965118
11.2677684074 -4.00119191363 5.78058547841
11.0384056259 -4.12360561288 5.95967261451
11.6945803406 -3.53983883715 6.46233874791
12.4523862015 -3.65164347145 6.59485664813
11.2794014882 -3.82003629094 6.40351125566
12.1759335586 -4.91658439088 5.94949153733
12.9124656461 -3.90317749708 7.49312537979
12.7639928153 -4.14806754497 7.44593729501
11.8242829692 -3.19552268568 7.17597371021
12.1068770133 -4.98005629561 5.74270244529
12.0498547456 -3.86924691625 6.00956835334
12.6384309927 -3.05957085566 7.35015955881
11.3341614995 -3.72258382643 6.84849532702
12.141605782 -3.83162279126 6.12616845257
11.3125239374 -3.80774047024 6.27171989762
12.1844614211 -2.80062838912 7.76877385461
13.0042348437 -3.7736615907 7.67878143056
11.2348985107 -3.86951995257 6.33695083917
12.9892537362 -3.71917367318 7.55298065433
12.0978188361 -2.97646146267 7.42192187443
12.3730975874 -4.61342795315 6.29441295514
11.088056797 -4.13622853125 5.69076450307
12.4036736023 -4.08215717199 6.24798451397
11.6689655483 -3.88044052914 5.83573649992
11.5171436788 -3.61568041357 6.46078203586
12.7119207698 -3.80585667024 6.88025535717
12.4917209493 -3.36759829315 6.90392067809
12.7180137154 -3.40212389382 7.10305824075
12.2412744542 -2.8512571179 7.59560301717
11.895815939 -3.99271326177 5.76767798588
11.7720986176 -3.15885278536 7.48068983491
12.3610138845 -4.10978198257 6.16297156352
11.6459401653 -3.34667517458 7.7307373103
10.9373871313 -4.25391781156 6.03448098933
11.1657112874 -4.0022709536 5.9822684116
11.9930185218 -2.84878180992 8.10524200689
12.9229224582 -3.19044926583 7.51809586809
11.8195686847 -3.65652038285 6.21145999658
11.557870002 -3.68547544498 6.24384672341
12.4391733564 -4.45279055085 6.35036316205
11.8844574402 -4.05046073234 5.69015144485
11.7279455946 -3.23668260398 7.31869258848
12.294499428 -3.7841317207 6.31472056346
12.8220942965 -4.04344605914 7.39020527596
12.4721783221 -4.53105544744 6.77942746539
12.2684576136 -3.68886737527 6.37501457718
12.7562672472 -4.14676663185 7.27340976789
12.474739353 -3.90313401038 6.44559549138
11.9537539216 -3.80809962864 6.03088251442
12.0320127099 -3.33353952753 6.75147093213
12.108015834 -3.15675039837 7.05358734526
13.1318609208 -3.62827645749 8.02711391365
11.6110426319 -3.4435523956 6.841634456
11.0478107889 -4.10552941099 6.19809361552
12.7755908773 -4.09560545782 7.26628519189
10.9883031106 -4.22927958504 6.24949086068
12.2912842571 -4.31392954383 5.96981134173
12.0620551327 -2.97059245505 7.45810513231
12.9589902285 -3.8656483842 7.69224121777
12.6528022773 -3.75816679912 6.79932958562
11.7332529794 -3.94891424043 5.75730446962
12.2648424749 -3.39194857583 6.72195900522
11.6306636335 -4.08235289808 5.526791949
10.8503205314 -4.4223245096 5.98058354011
12.4901429256 -4.01372861676 6.4210933007
12.3557556612 -4.26287371522 6.10180215592
11.0671163615 -4.09825712372 6.38823058053
12.16086772 -4.74973092932 5.66328975122
11.1561698548 -3.99327985163 6.06152521331
12.5339560162 -3.31804559075 6.99361076578
13.0809332046 -3.46528373505 7.70241049241
12.1464782422 -3.24578868481 6.90112082941
12.2101575229 -4.88347333128 6.17450609833
11.8185152135 -4.11015474277 5.57992002032
11.7139835949 -3.5249414472 6.48745447275
11.6189679297 -3.72699019595 6.11504817649
11.2281066701 -3.89609476157 6.20122024792
12.9965534326 -3.8284581581 7.83095592455
11.5141273965 -3.56717589888 6.64864705271
11.7193065108 -3.22766738049 7.81631391427
11.5021171349 -3.50587634739 7.17133630403
12.7119493287 -4.19408742881 7.1335814631
12.8993516317 -3.33016917714 7.39426258573
12.4153867878 -2.77204897179 7.68981997902
11.2656890506 -4.04858318448 5.66050996188
11.2532383994 -3.83368767182 6.46478438009
11.263695737 -3.98306779439 5.82625166448
11.8292687494 -3.07373496788 7.71125891096
12.48939294 -4.18499758872 6.38245800407
12.5396089322 -4.29611302213 6.53545926351
12.0903944009 -3.69942685927 6.23659037033
13.0032391544 -3.07521422096 7.68847174693
12.0345680268 -4.29975818401 5.59681526715
13.041427718 -3.59290367411 7.63871146516
11.0838476147 -4.07738744477 6.42026446577
12.0044662698 -3.54933468296 6.40344948472
11.65134929 -3.38011706752 6.9462154159
10.9682130326 -4.21761467655 5.78006811055
11.8155109438 -3.51405664842 6.45938557196
12.9357894749 -3.41726670509 7.4080853299
12.9091701472 -3.82785751497 7.39099169025
12.2435845147 -3.05990469585 7.20688168611
12.8153406297 -3.17250689829 7.40190405008
11.897778925 -2.9759353992 8.12417500886
12.8437881208 -3.78813895554 7.17145325667
11.0976731464 -4.07194179926 6.47034586849
12.4500636102 -3.5317618315 6.6989928716
12.5703908812 -3.55515196455 6.80184082291
11.2937673424 -3.80607101236 6.90601326066
12.3316434898 -4.5689777186 6.07513973815
12.7035101108 -3.8946205728 6.84992158219
11.3228901544 -3.74186331921 6.85458711249
10.897860531 -4.34327012671 6.08217309683
12.0589292592 -2.79558824361 8.02937325563
12.3433541083 -4.4580631864 6.05853812093
11.5783465741 -3.51186144459 6.67818896983
11.9638557209 -2.87925810006 8.2066820276
12.2952592491 -4.11576169238 6.06395206713
11.092679851 -4.0381987848 6.27927437197
12.4643029803 -4.47298270219 6.4476146554
11.3047199362 -3.78556801847 6.88458595717
12.114382762 -5.00190341281 5.84656572842
12.4147261213 -4.4098781856 6.2411408698
13.0889405402 -3.64052083136 7.85541412998
12.6702076034 -4.00014280184 6.78352643958
13.0900698887 -3.28498975122 7.72695126329
12.3374483048 -3.09298100274 7.16837187213
12.4868377758 -4.47079199089 6.54518636621
12.4248868199 -3.95587891504 6.34827976697
12.1141757111 -4.88016905917 5.60478611069
11.2360762674 -3.91850601881 6.08920177712
12.3715640633 -3.47870488834 6.68584169545
12.1176553301 -4.87614704427 5.61204358137
12.8454764918 -3.19781894631 7.41090089378
12.4548799846 -3.61838447653 6.62262092576
12.1475137054 -4.65382419382 5.62857548271
12.0009992682 -3.47711553017 6.50586765709
12.8116674142 -3.51925629755 7.15925976471
11.7665185068 -4.1044665898 5.5498516222
12.0551056266 -3.71406901955 6.19822270311
12.2188618996 -4.59031930259 5.78153944548
11.8706422888 -3.31291803719 6.84652868154
11.3193186203 -3.74072932697 6.64664783742
12.595837453 -2.89291225334 7.53395704407
12.1098406261 -3.70010186926 6.24756604755
12.4080137177 -3.01059496893 7.30089449395
12.8756365784 -3.46960240256 7.29188950139
11.5321254938 -3.71825061104 6.19442454604
11.8649400464 -3.87775326296 5.89462839989
12.3724352294 -4.63673247596 6.33184370059
12.3319315974 -4.0354707856 6.16187643828
11.6911705767 -3.25293895695 7.68022982007
11.746350292 -4.20122120595 5.41658257059
11.8452319445 -4.12242536129 5.58327191244
12.686772727 -2.74826272649 7.75862699638
12.1692757337 -3.80192744754 6.18003405
11.7345987163 -3.29679999213 7.03821943726
11.4634413023 -3.55346957207 6.89707770629
12.8063308628 -4.04755173823 7.29665638739
12.4710428341 -4.50205473085 6.53859639711
12.2062430692 -4.78634703454 5.83369773943
11.8989966683 -3.14639132346 7.20195038319
12.1117113242 -2.92463544853 7.53722580154
12.5352429888 -3.90589930546 6.54380757989
12.2891495039 -3.55958156162 6.53666811476
11.6953609408 -4.12773400971 5.48357278432
11.1632253797 -3.95190103256 6.249817729
12.360000955 -4.60028726825 6.20027038244
11.3654785157 -3.72909187973 7.10962257401
12.9355891824 -3.82766732813 7.4696804181
11.6215022474 -3.40345481958 7.72932857761
11.479909686 -3.57454121138 6.69967800654
11.8020306524 -3.10708459768 7.63634389228
11.7436271702 -3.84831068298 5.90629878997
13.0806429262 -3.64272244569 7.81090403128
12.8727402441 -3.965764925 7.41729879333
12.6086108712 -3.71577292657 6.74849662264
12.7864451241 -4.10717573843 7.33619082314
11.1450664335 -3.99855054469 6.07615630643
12.7786156696 -4.11824354269 7.3475971185
12.2282264227 -4.83727569079 6.02461867598
12.5716698076 -3.76067719873 6.66957257212
12.5025942858 -4.41014131929 6.5194248939
12.6665000434 -4.254299688 7.00892150563
12.0532623883 -4.44893893785 5.51852107056
12.6726604606 -4.26579697685 7.16445279166
11.6251136913 -4.01666647986 5.62264911013
12.6444314458 -3.07149405318 7.33994830441
11.1813181336 -3.92166888638 6.45112779532
11.1606354253 -4.04657689887 5.82202670686
11.2497086417 -3.83440372209 6.4541925707
12.2250866344 -3.85330594744 6.17894229046
11.5988395193 -3.38981627976 7.09234009373
11.5900852771 -3.40241539672 7.09109611403
12.627441451 -4.01990902584 6.69416869826
12.2198026195 -4.12254485012 5.95822821112
11.8802663306 -3.01653828516 8.21867752222
11.8901339733 -3.76288163246 6.06519606502
11.7706504688 -3.16674542284 7.96717123223
11.6310448985 -3.41597130464 6.89099661605
11.6831057952 -3.25578672668 7.50847975751
11.5003600438 -3.51476270655 7.32032200939
12.4685892183 -2.97373565559 7.37825139346
12.7115331781 -3.06039640484 7.40527735223
11.3848993732 -3.83720467893 6.03647909048
11.588935865 -3.6907533873 6.20760913799
12.8501094464 -2.92933429298 7.65613990408
12.6710574811 -3.89019062733 6.79340891573
11.5816099173 -3.67124480153 6.2564901339
12.9836680298 -3.84053760488 7.77271697503
12.1332273275 -3.80784496441 6.14599891076
11.6780700583 -3.65467184854 6.23879086899
11.5873500636 -3.38525199634 7.36741288029
12.772764506 -3.17467160847 7.35022787686
12.7845889404 -4.06056954585 7.20302341325
12.7804114847 -3.05891761287 7.46202372454
11.1920894969 -3.92617225739 6.2131043375
12.7352133368 -3.74933262343 6.94624398984
12.0441978815 -3.60238291539 6.33913150744
12.4173502291 -4.50450859402 6.30516022403
12.8474818964 -3.95876707137 7.31841108788
11.5799475417 -3.39145856416 7.23864517671
12.1900689267 -3.66675398175 6.34467085126
12.2182589736 -3.17713509183 7.01181545396
12.9649160554 -3.20220865711 7.56365977994
11.6057954801 -3.66478597518 6.25637638262
12.1544424712 -4.80265440352 5.68189168852
11.117817034 -4.03082719298 6.05089818711
11.3288863836 -4.00342302735 5.70346562785
12.2146400487 -4.87706963891 6.19953610987
12.3968726824 -4.5950974329 6.37591814041
12.8500254193 -3.56679376605 7.20908490816
11.1018010933 -4.05939048476 5.96496781805
12.3449998906 -3.73972487541 6.40259480099
11.8986085651 -3.13745721587 7.23367613257
12.1105977776 -2.76736359724 7.96907386811
12.0501049839 -3.15744727478 7.06984745886
12.9145276756 -3.59980383465 7.32252719251
11.3826371801 -3.75369765228 6.30862337587
11.3295106185 -3.74874986612 6.51664477643
11.9442281645 -3.44325974558 6.56319227209
11.8091319585 -3.12527212081 7.51558917382
12.3049699328 -3.78807424198 6.32084137932
11.164294022 -3.99202706275 6.67448327809
11.612063141 -3.64357268575 6.2893420402
11.7663603347 -3.2065306179 7.29752864847
11.4310974736 -3.61020589966 7.17338955637
12.5757056875 -4.32012254927 6.70504095163
12.2527563872 -3.71025230822 6.33760768882
11.9340343232 -3.03968595898 7.44948671857
11.4878430009 -3.52318809064 7.24678609989
13.0626165837 -3.2447518925 7.69042974627
12.7054680761 -3.95849527838 6.86281197001
12.83715126 -3.07781948354 7.49956778427
12.161263956 -4.40710041826 5.70587053577
11.6737747694 -3.29067331787 7.28118114047
11.8642145601 -4.18656503301 5.52241800212
12.0329126999 -4.43049615772 5.4955040102
11.3974488241 -3.72839683152 6.34472202059
12.9927730693 -3.84300858407 7.88949224187
11.5852608651 -3.40241525739 7.10637339815
12.6601380581 -3.16105418174 7.25323684471
10.9492889907 -4.24053046687 6.00047356838
12.3903582857 -3.19331129884 7.03951951838
11.7133430311 -3.69630192407 6.15473710383
11.8036634522 -3.28703200028 6.95496667145
12.9636386451 -3.62665563574 7.44136363765
11.9340910611 -4.07878627795 5.6928135527
12.3293804071 -4.53099914157 6.05077408749
13.0748759486 -3.30147779282 7.6982377987
11.5962445185 -3.63857080643 6.32116224297
12.619907091 -3.21929115965 7.16365586517
11.7144847436 -3.21925971298 7.740151277
11.7927957559 -3.13571240162 7.48616578356
11.4621802307 -3.71241928454 6.28617404205
12.2458434239 -2.76528740392 7.78743876698
12.6125092857 -4.13555613974 6.67531230432
12.4148763614 -3.12269936631 7.14948395731
11.3467190277 -3.75990463124 7.08816515446
13.0132349227 -3.71205840814 7.65040724204
12.878457816 -3.99483377814 7.58334465218
12.0594268949 -3.07471512087 7.22562532518
12.0050030332 -3.76796673533 6.10589136664
12.7546890302 -4.09858909918 7.14444735889
12.5136981734 -4.11645913696 6.42894036711
11.4794606183 -3.91224790722 5.81007169138
11.8149810938 -3.09653250539 7.61330225754
12.4109317917 -4.51932166965 6.30498318253
12.2816595745 -4.76398177444 6.16486519876
11.3557226185 -3.79541690673 6.22348706803
12.1178691111 -3.13920439114 7.0843799737
11.5663659057 -3.85858571511 5.88878058035
12.3614235126 -4.01498001538 6.21698733357
11.0899529198 -4.05231283447 6.26372189084
12.9326817071 -3.88152580075 7.55072619217
11.5592997452 -3.41804826526 7.29834251831
12.1601725063 -2.72102944732 8.02105368737
12.1915968669 -3.55531024848 6.47141260383
12.1086340347 -3.62973621931 6.33870960297
12.2573370641 -2.7182462902 7.88648648703
11.7190593624 -4.13691228753 5.48709502675
11.7877467518 -3.15850592092 7.43071011831
11.4994682152 -3.56040713074 6.71194868057
11.7911947755 -3.79305381657 5.99776369022
11.7631069945 -3.86339753088 5.8826765944
12.0936411841 -4.52886993625 5.54912027817
12.6844274706 -3.46966040944 7.00874223965
12.4214808397 -3.70047774201 6.51865206025
12.3382902683 -4.39874739757 6.04024473394
12.5212680784 -3.38584804472 6.91064194128
12.2933007852 -3.1270156412 7.10358408929
12.2311576739 -4.55150943496 5.80494501554
12.0828630037 -3.44296139099 6.58508310189
12.0055754583 -3.73719431587 6.14333736586
12.2994150352 -2.75623921318 7.75523695695
12.6121435949 -2.98693887572 7.43041484507
12.4364404437 -4.39089455511 6.28854479594
11.5579391654 -3.54317102334 6.63206732451
12.168590966 -4.62879950238 5.67280869264
12.7087234939 -4.22184081503 7.25890731341
12.1410454082 -3.58245703244 6.41518445671
12.5946112589 -3.02523407785 7.3596791208
12.7269678098 -2.9332352862 7.5597839131
12.1323392155 -3.84494629037 6.10965202693
12.8925997393 -3.60525443754 7.28072373977
11.9938938798 -4.31068947538 5.53926498934
11.9504781156 -2.90689458931 8.17370517747
13.1225360647 -3.65938051456 8.06676865289
12.7518275047 -4.15968371742 7.39887269959
13.0785699228 -3.46057431021 7.69454861995
12.2442471381 -3.13020348672 7.08990205181
12.0638007023 -3.57419198778 6.38435299905
12.6560484681 -4.29365619584 7.17764113788
12.5908457607 -4.30941186186 6.75255184523
11.4071142656 -3.76833226666 6.21300696446
11.3652680614 -3.68814747651 6.69645748047
12.1053227052 -4.36692707253 5.64604060057
11.49638364 -4.04364944518 5.57118390448
12.3105575111 -4.19436334472 6.04489600061
12.6699711757 -2.95068702244 7.49826835191
12.1602346677 -3.21725529442 6.94781447175
11.8703796849 -3.21756751053 7.05815469687
11.3484400125 -3.78530339221 6.27309560153
11.5967132546 -4.06441875324 5.54493143551
12.0785186439 -5.04813232339 5.77456774884
12.642149261 -3.9267163204 6.72691332393
12.0368020666 -3.7136564593 6.19344045717
12.2085123293 -3.10668900687 7.12589853082
11.652680211 -3.30102834987 7.35076248
12.692841682 -4.04838625804 6.85998118989
12.4486206078 -3.57486729277 6.65713185645
12.2914412053 -2.83532347056 7.6063762418
12.6427214598 -3.47233738654 6.94754328939
12.3783489379 -3.71205458102 6.46556916463
12.1053640238 -4.98809820009 5.75532559041
12.3094152165 -3.99089922742 6.17062610502
12.2758367697 -4.21071676412 5.9796855472
12.1885066426 -3.82081932795 6.18096630065
12.9243793726 -3.84572030384 7.45290695134
12.2056213262 -3.02541708114 7.27344757761
12.0961334514 -2.87232566684 7.67653179597
11.6857154638 -3.25065173776 7.52471292382
11.3068965979 -4.04972176963 5.61717070876
12.4657850401 -4.01843781998 6.37690014228
12.7444870194 -3.67761276338 6.98547058042
12.3587376165 -4.68786248694 6.53402458488
11.4202894983 -3.77729897235 6.17282786559
11.384504941 -3.89139217948 5.92477981669
12.3807284371 -4.62700034013 6.38328709565
12.9481884125 -3.20411649526 7.53557014526
10.918335178 -4.34170452315 6.16177090535
10.9246114369 -4.29428896128 6.05888707051
11.5878209287 -3.69641139312 6.19976588781
12.6873323145 -2.99018535247 7.46037164166
12.2907467542 -4.72056188998 6.08048877251
12.6741067373 -3.9262295568 6.79506171164
12.4049363766 -4.60489087468 6.43647113217
12.1564931326 -4.95550638802 6.08524364841
11.7958399191 -4.21763375875 5.44271221915
11.2284345315 -3.99857888556 5.83896866239
11.0384824199 -4.12312867147 5.99678268254
12.0110548665 -4.37703018148 5.50324000061
12.1551348605 -4.2309240087 5.79827568695
11.2905231394 -3.7769771652 6.64330827765
12.8312221015 -3.23600212294 7.36241429314
12.9062740093 -3.93327402581 7.55150135259
12.0715317604 -3.06182691739 7.25231351234
12.0626217941 -2.89077346971 7.68735858569
11.9734718284 -3.4922078905 6.48680030613
12.6958983181 -4.11059211546 6.91371380078
12.1390787361 -2.84481698696 7.69576617274
12.0791616215 -3.65118559249 6.29994080892
12.9192123332 -3.34875748378 7.41491051951
12.0884016314 -3.09456126736 7.17719318674
12.2969672842 -2.85937170099 7.55706162547
12.221452717 -2.89226714512 7.51835400153
12.2272207285 -2.75175056962 7.82452467495
12.314227784 -4.69867702607 6.17088304406
11.801018057 -3.51459008511 6.46221428676
11.6023869939 -3.37064101816 7.45802906346
10.9812353473 -4.20809859217 5.8142571879
12.5454992996 -3.37534789076 6.94174800818
12.6342505786 -4.02853780105 6.71517109437
11.0969019994 -4.03411645612 6.24951176965
12.1484824512 -4.7583059917 5.63999673091
12.5811494776 -3.44652914282 6.91394109914
12.3453434348 -4.18093329425 6.11493945121
10.9537685969 -4.23003417779 5.94106264803
11.2480424998 -3.97217778766 5.88318857517
11.2973123622 -3.79279116503 6.4096988331
11.319356175 -3.85660704454 6.10635403676
12.0142589729 -3.16242601024 7.07834219793
12.5199568757 -2.95273736086 7.41876764939
11.7048204503 -3.29960123628 7.91188743479
12.4442460964 -2.70886156202 7.7983676315
12.2444080786 -4.52672197243 5.83655551709
11.8823339631 -3.89977897013 5.87341023484
11.8755402837 -2.99744194031 7.92152135521
12.515678301 -3.31376820137 6.98083289526
12.4866885074 -3.5269307061 6.73668640612
11.7921182156 -3.83236352845 5.93222437532
12.5874997998 -4.16579894615 6.6002000047
11.5799144506 -3.64356459662 6.3322987618
12.9946551181 -3.14840198979 7.63884431448
11.4995006067 -3.7105265161 6.23302415958
11.136325926 -3.98576976529 6.443456172
12.254232188 -3.34340730272 6.78142711109
11.8312333111 -3.05575185043 7.85013034214
12.4867212112 -4.51789597744 6.79990498896
11.6516613219 -3.55281405668 6.47324669689
12.4607780532 -4.26289757782 6.32026339807
12.8428676789 -3.60807365216 7.18311962553
11.3622409629 -3.71200828392 6.55461455008
11.7506237937 -3.20169399875 7.38913369767
11.303031804 -3.84615551749 6.17304694523
11.2637941556 -3.80540212394 6.62186726546
11.5353835493 -3.51241508922 7.52882713962
12.0781145057 -5.04997499691 5.76520135106
12.0143724794 -2.8412804005 8.02195835818
12.2828273034 -2.94191930466 7.41037069992
11.7612790419 -3.66945751745 6.18894690579
12.2973882908 -2.83864080999 7.60450987982
11.3934142155 -3.76609387116 6.23583736812
12.4227005075 -4.56673608678 6.45977515876
12.8515052157 -3.84632500793 7.21931268933
12.4244834544 -3.94963821205 6.34232723627
12.2982575401 -4.35442860719 5.96420837811
11.262543506 -3.8162303858 6.59771494547
12.3793564959 -3.27038375913 6.93784331665
11.7343179424 -3.44249709709 6.65109977592
11.2338665258 -3.85261422713 6.48589735618
12.5906387338 -3.38361209655 6.97885131722
12.0337442007 -4.08322691444 5.77472047197
11.8043572058 -4.14107365798 5.52939176957
12.8458208609 -3.30138209696 7.33514552174
12.44506237 -4.54952113773 6.55371571366
12.04475294 -3.28399639708 6.83814502259
11.0798271052 -4.09337348495 6.43474529061
11.7039255085 -3.35337496791 6.92704254545
10.9625102408 -4.25216608297 6.17605424866
12.8526759623 -2.9807077887 7.60928817201
12.2324540475 -4.49780840717 5.816174809
11.8694605231 -3.12037435369 7.32946009608
12.972316185 -3.40888215689 7.47890370154
12.7358159322 -3.98507103852 6.94848930329
11.8683623913 -3.11635216587 7.34386400677
11.6958056852 -3.27991663008 7.20764464295
11.2692195088 -3.92586892446 5.98133827597
12.5699425775 -4.39810105027 6.9064850379
12.0469147123 -3.19544775172 6.99814880928
12.0543799625 -3.45827753288 6.55298859664
12.3032270174 -4.75831974995 6.42988204918
11.8857637591 -3.02266956312 8.25676666499
11.9219101495 -3.63024095234 6.26175794931
11.1183613375 -4.04130798206 6.50323704879
11.4594183617 -3.62643823285 7.36606014489
11.0302500777 -4.13260745948 6.18335425734
12.2645755466 -2.77424923638 7.74403809093
11.7373040937 -3.23383027217 7.9381536597
11.6831143826 -3.3216960548 7.86307640704
12.3574911656 -4.3818642977 6.08978423954
11.9372640201 -3.97388135339 5.81096652962
12.2526399771 -4.08841444913 6.02526960557
12.3347464706 -4.59057676989 6.10403002293
11.299491943 -4.02373844805 5.67996615928
12.4356044972 -2.98168100608 7.3528890888
13.0345543043 -3.32748475849 7.61095604475
12.0863347549 -3.67391721216 6.2658588999
12.1161363019 -4.11077898989 5.84063918947
12.4466627208 -4.49286855341 6.39349490604
11.8077564072 -3.08865168968 7.8175364853
12.109818135 -3.7173350991 6.23110715237
12.3784676951 -4.11927298423 6.1876357516
11.8882053634 -3.92845022323 5.84520346993
12.7366353412 -4.04555915657 6.98983841625
11.7071182306 -3.22673578071 7.62352792855
11.4661945042 -3.77266362514 6.12426913309
11.2648410501 -3.82889264169 6.39375672869
12.4051321776 -4.32747161506 6.18677105523
11.6646636721 -3.56129080504 6.4494434302
12.0074465229 -2.82526095624 8.13653669429
11.9693299651 -3.60430842343 6.31875111255
11.1220235791 -4.03893929123 6.53196805459
11.8503460677 -3.06978974811 7.58556998093
12.1186196802 -2.9584127008 7.43520017907
11.8078448137 -3.98441028287 5.73682532979
11.2337063433 -3.85479161159 6.58053200972
11.3878269937 -4.06574842217 5.54794655785
12.1969673286 -3.98024351507 6.04196719361
12.348912751 -4.64943436586 6.25277617695
12.8642912564 -3.84952846812 7.25125012954
12.2243558283 -4.86634200513 6.21378321044
12.0498833069 -2.81606362531 7.9604071504
12.1156871009 -3.75787574133 6.18435567771
12.3503374897 -2.77956878041 7.69733736275
12.7261757436 -4.15858785617 7.11327570152
13.0412060739 -3.75698415827 7.82806799098
10.9401220159 -4.24593590072 5.96461139711
11.4044670507 -3.63003161787 6.92694665205
11.1680086509 -3.98851142499 6.67647187627
11.1037178464 -4.06717814384 6.52652227102
11.5590512563 -3.72760858232 6.15451700887
11.6341054766 -3.43294972979 6.80615058568
11.5694937352 -3.41059040007 7.41872158811
12.3180660073 -4.73482026903 6.3277821214
12.0559526236 -3.57942239272 6.38058923132
11.6262781765 -3.38927385795 7.0005210298
12.1472498246 -3.2418642644 6.90867997275
12.1879841735 -4.91569410158 6.14869567898
12.2631537032 -4.18594487833 5.97430424956
11.9844132793 -4.04717034606 5.76606250694
11.5328032547 -3.85259242994 5.90820817865
11.8354541393 -3.37968976834 6.71703041599
12.7979496747 -4.10258493105 7.47465533681
12.7476937691 -4.09698140059 7.08245452791
11.8377959631 -3.78534238386 6.01142530815
12.9761657781 -3.73998885608 7.53285466932
12.1658052974 -3.52155134681 6.50811464369
11.8732089262 -4.2340583634 5.48638262105
11.6432540283 -3.37959590298 6.97744737813
12.1174659372 -3.74580743321 6.20366542885
12.3850179002 -3.17570002426 7.06302298207
11.3282351829 -4.00443033236 5.69663849006
11.626589736 -3.34289770824 7.2435464205
12.6890346537 -4.2409379717 7.1134153323
11.7404021628 -3.19041982501 7.47388665981
10.9325135822 -4.26057710763 5.93512268622
11.2109618218 -3.92808607044 6.15673753048
12.605313612 -2.99142113174 7.41928690324
12.1949959457 -2.88767931732 7.54019892936
12.1358061317 -3.06216146138 7.22094858455
11.0556182291 -4.10907079857 6.29448349233
12.8791640999 -3.02978676544 7.5898895776
11.7609457723 -4.09471792078 5.56630936012
11.7738211798 -3.40842960902 6.70612489654
12.6805205261 -3.0158991796 7.43152621901
11.1265282413 -4.04703501167 5.93402120502
12.1672226678 -2.87948515419 7.58498578598
12.7075428231 -3.27208798296 7.20398273782
11.9173566518 -3.07012346954 7.40565762348
11.1104232996 -4.02391439911 6.36102713047
12.1936543809 -4.26110328184 5.8277411176
12.3857862498 -3.85325828394 6.3634599665
11.7595863451 -3.24516195727 7.17998139726
11.7264857848 -4.03490725253 5.62128648067
12.3221805892 -3.89851524949 6.24357700991
12.3344509199 -3.13691513989 7.09457460301
11.1180939189 -4.01073166094 6.32359315176
11.9070592783 -3.13080968717 7.23166651981
12.9252512437 -3.91204561189 7.61358295067
10.957957822 -4.25642766696 6.17662431311
11.0795348251 -4.07589883631 6.3829283884
12.2151808022 -2.73795107144 7.86500101757
12.7966286225 -4.09097870689 7.38017294299
12.704401797 -3.15033823088 7.30552127191
12.7675376248 -2.90431409889 7.61413486594
12.2499521473 -4.72913481013 5.94436821232
12.5008883168 -2.96919787731 7.38588262672
13.0002639186 -3.83297119825 7.9744899151
11.6235417889 -3.94277498446 5.73864606771
12.6729414512 -3.76932581206 6.83606751965
12.0554659245 -3.24089372562 6.91250713566
11.468012733 -3.54301787307 7.15806079543
12.7292826657 -3.99708464693 6.93757135325
11.4597549113 -3.56384929106 6.88967575109
12.3424371159 -4.40056715348 6.0468679918
11.4581096663 -3.59641078799 6.68025268053
11.0613622855 -4.13451777211 6.46350796344
11.3144853872 -3.87883142184 6.04250198539
12.0919606003 -4.38632104496 5.61596446978
12.303896137 -2.8972441867 7.48601984136
11.3393284183 -3.73406434012 6.55023571194
11.922808673 -3.5710874177 6.35631784708
11.5808779683 -3.67374761358 6.25736321749
12.3706619174 -4.51825109897 6.15661166944
12.2814892125 -3.18926683775 7.00494011339
12.2266851132 -3.68017560712 6.35358515699
12.8241731856 -4.03364417004 7.34670404616
12.5056539048 -2.97574708815 7.38663698425
11.166935168 -4.11497356497 5.60489197807
11.6908306561 -3.24938232138 7.43533292697
11.1537848394 -4.11428142186 5.62426174772
12.6098642521 -4.11738844776 6.66202835801
11.7237950881 -3.75010153411 6.05204639236
10.9945093502 -4.21090707966 6.24486486466
12.146849477 -3.71292901687 6.26181834909
11.0260055165 -4.13141788579 6.10925951303
12.090572614 -5.02864747276 5.76545837279
12.2046873198 -4.21997142168 5.87884868349
12.5156499977 -3.95992949527 6.48713001887
11.9530124976 -3.07008078769 7.34447223887
11.8811030322 -2.99919553203 8.10077376501
11.5020637828 -3.60248739456 6.56087533645
12.2819892025 -4.79098848777 6.28152129886
11.0760243218 -4.11180529775 5.82145883037
11.3734661527 -3.75777880125 6.31265038496
11.751928849 -3.18978093982 7.40638587628
10.8318862053 -4.45310053971 5.94483408475
12.2188170329 -3.71220990647 6.31357968045
11.4487875885 -3.56637888708 7.02145498634
12.2018024522 -3.943377901 6.07193550062
13.004058986 -3.75486384273 7.66653448071
11.8060032691 -3.39457629745 6.71939795572
11.535162205 -3.45184626798 7.30927961249
11.9520781756 -3.92012883635 5.89372619712
12.9265884447 -3.18145365197 7.52786616516
12.9294366114 -3.59641557918 7.35757698775
12.9589395362 -3.889283034 7.87858815232
12.5172553383 -3.97680447108 6.47859631406
12.3167380214 -3.05434024484 7.22495010535
12.7266921564 -4.18695792752 7.21955802881
11.5845465624 -4.01974848867 5.60442828336
11.9588511931 -3.92688583067 5.88632366075
12.5641390347 -2.95940442981 7.43130132815
13.0079085494 -3.18641038811 7.63647276424
12.7837315245 -4.04266705572 7.15004889928
11.4312638485 -4.05696544007 5.55657377678
12.3027100891 -4.23158346403 6.01692542804
12.0245562754 -4.47050085867 5.45979583993
12.5151401351 -4.45619672836 6.71210669594
12.8764756153 -3.35541727394 7.34441066475
11.8341550011 -3.05871546746 8.00169746287
12.7429115719 -3.01357740209 7.48102447236
13.0839988146 -3.52689690694 7.72487310944
12.5934152313 -4.37276167791 7.06416719458
12.3069416103 -4.48982689329 5.98356575583
11.9009240123 -3.12713454671 7.25474202669
11.8775347296 -3.05990298529 7.53355336449
12.4863614945 -3.62908014112 6.64970459155
12.3413147999 -3.40553354507 6.74610696543
11.7625060923 -3.87035070935 5.87207758731
12.1648512287 -3.30157944127 6.81468289253
12.4889386812 -4.37895428554 6.44765864643
12.2603981663 -4.32231658412 5.90342410552
11.8743530995 -3.01711406299 8.12736942094
12.3842071571 -4.16050180662 6.18719682345
11.9424585868 -4.19589555594 5.5843629701
11.8016604505 -3.10538891445 7.90101327483
12.0560287949 -3.34666358503 6.73052138394
12.7084180604 -3.33860003328 7.14056994074
11.7694762474 -3.1714729255 7.41957203328
12.4643329905 -4.27124945265 6.33124990121
11.8299389436 -3.23383581467 7.06908643993
12.3015651988 -3.93768943723 6.19074052859
12.1182603968 -3.35368424295 6.72238089917
11.8416184568 -3.06781046824 7.63840269495
12.5609798757 -4.04087470382 6.55154903927
11.7484708892 -3.16895612788 7.67488210204
11.2611269586 -3.8514836929 6.32282979532
11.9406276826 -2.91689807619 8.14865863216
11.8604310193 -3.05151124316 7.63869660995
11.4026229155 -3.62738510378 6.92551241238
12.3375767194 -3.56727271592 6.56771333866
11.5036334157 -4.0819451054 5.50217400795
13.0180853362 -3.80745744812 7.88162988159
11.3704172433 -3.67332144544 6.96355244166
12.7493304206 -3.26250175673 7.25397066321
12.5299513688 -4.1672440976 6.47058870887
11.5165887764 -3.51963056501 7.44439441013
12.1044795459 -4.34942414255 5.6462936443
10.8798468528 -4.39115633007 6.04941811103
11.387770912 -3.72702246008 6.3799716783
13.0291808851 -3.18804479236 7.66193899531
12.5084971543 -4.33041089786 6.46238474916
12.2325604651 -3.50928486413 6.55223654616
11.5384055627 -3.45542455626 7.21882808305
12.4667382594 -2.83216849847 7.58835616289
12.14813931 -3.06454739457 7.2147684923
12.4508094905 -2.86639367087 7.53211641341
12.1652185233 -2.75763697027 7.89529467389
12.1304741432 -4.80594418257 5.59944441041
12.2839574323 -4.77851639193 6.24076508994
12.6154752656 -2.93248702695 7.48492054782
12.1095836211 -4.20236899308 5.75893861165
12.1162711397 -2.79378570839 7.86870579596
12.1865089993 -3.36434639326 6.7316541653
11.8834930877 -3.02145796813 7.68461413726
12.3433779357 -3.55427114091 6.58548597351
12.9756931544 -3.82720766903 7.6658006441
11.9973301299 -4.03407117414 5.79454296746
12.4493257527 -3.477479903 6.75247979268
11.6996661035 -3.24011152003 7.64944950523
12.2180384909 -4.87550704158 6.15732195977
12.4514896313 -3.43762939726 6.79832348434
12.3739986611 -3.85636834367 6.34815408498
12.4163202796 -4.47469775964 6.27356673234
12.2664308079 -2.97429246316 7.34540899322
12.5955506985 -4.34138885702 6.85016178558
11.6598184313 -4.1109066242 5.49637043979
12.0101483857 -3.24068620278 6.92165037155
11.9823637049 -3.63528454773 6.27555041963
13.1538781558 -3.2647015934 7.86840782863
12.6661161667 -2.9288292243 7.52403106012
12.6364373951 -4.12901374075 6.73594619533
12.6395413977 -4.16724625197 6.77090755997
12.5527014251 -2.90956856263 7.49506265601
12.8942817944 -3.28289271892 7.4169565079
12.448980257 -4.38153760638 6.32306630553
11.3747877521 -3.89158501749 5.92033110626
12.7518507261 -3.05518345072 7.44769280118
11.530327832 -3.47962508689 6.99619291458
12.3945591989 -4.13613192494 6.21858448615
12.9002113115 -3.45094540481 7.33934661258
12.3292543398 -4.28573187488 6.04036370722
11.478721763 -3.58300765364 6.67125189679
11.512970653 -3.98722162253 5.66536101019
12.3390190452 -4.71706244122 6.45087331352
11.1269589953 -4.04406573361 5.92401919709
12.2804334062 -4.430954962 5.92647526383
11.9227312485 -3.29184515624 6.85977404383
12.5954323983 -2.81761533822 7.63216723218
12.454837445 -3.81305609551 6.4731224359
11.9554478106 -3.01967380415 7.48632065728
11.9888118907 -3.23837412802 6.93088468063
11.7791068177 -3.15579802018 7.97054367383
11.9747001743 -4.43925967037 5.40977008161
12.4289935182 -2.70538044343 7.80819917258
12.4424468526 -4.40515637463 6.31580646658
13.0413541322 -3.56243468195 7.62176478231
12.1063793486 -3.60577288023 6.37103232969
11.6689882972 -3.53435550956 6.49362243647
12.4520138858 -4.48798007227 6.4287608841
12.5308732071 -2.75112805929 7.72095860388
11.5718440611 -4.05530457252 5.54636087109
11.2341034722 -3.89396993695 6.82361279352
12.0740151766 -3.8800476331 6.02377795837
12.263375222 -3.11387442732 7.1265295573
12.9621140815 -3.88203501121 7.84973992266
12.7835999546 -3.40729631845 7.18392448804
12.9687431169 -3.58324620867 7.45323085862
11.8916497203 -2.98119542713 7.97531334556
12.1594157672 -4.31058292133 5.75552074978
12.4941686507 -4.0071017859 6.4362198837
11.1389831368 -3.99027247389 6.13235971895
12.2627119219 -4.8201061038 6.29069346955
12.5310864156 -4.45464052395 6.85473333342
12.6517504724 -4.1810775764 6.82498686731
12.7134112565 -3.90230618369 6.88302217368
11.3020458832 -3.80948923294 6.31794489591
11.6031157188 -4.08953339738 5.50474881029
12.4023961626 -2.67708457374 7.87330841711
12.8488435606 -3.70715547946 7.17097337331
12.0935257516 -3.08394497584 7.19029900151
12.4761346607 -4.51733942819 6.65982122694
11.7423069388 -3.32105739788 6.95933155493
13.0344324179 -3.4800366004 7.59530741005
12.687948863 -3.48090295279 7.009569568
11.8430041683 -3.48594600171 6.50815673954
12.1939952367 -4.55191955544 5.72881399535
12.0502196031 -3.07033302409 7.24832612762
12.5300931077 -3.73933571091 6.62900797082
11.9946545328 -4.09877421571 5.726632186
12.8517522237 -3.50850037455 7.23495114248
11.0484415045 -4.11000216851 5.98330083486
10.9899233067 -4.18442408808 6.06105650039
11.1227546496 -4.06467453916 6.60288161088
12.1643996055 -4.23424290943 5.79991619003
13.0334622366 -3.18847828983 7.67662826199
11.6718415825 -3.98749768127 5.67578879546
12.3145232338 -3.11561796124 7.12264040272
11.3999841969 -3.63992208033 7.02799842741
12.7254465567 -3.73538279409 6.93233799649
11.21884484 -3.89694325586 6.71426963757
12.1531913437 -3.72877487116 6.24597142663
12.4726975223 -3.82002671468 6.49705209502
11.8001081197 -3.13426598691 8.07353511977
11.6284367237 -3.91310676108 5.78153173103
12.7263524041 -3.51224970381 7.03247267729
12.4992048996 -3.75679379248 6.57374234585
11.7366168305 -3.29814238184 7.03836020796
12.2559102674 -4.6267231383 5.88536822854
12.8431627453 -3.69938166665 7.17401055131
11.7445078936 -3.91405484625 5.80262811447
12.7795432208 -3.93484236457 7.0474350528
11.8396243894 -3.87756308206 5.88553405246
13.1368018659 -3.62586667488 8.06267191282
11.4213337171 -4.0088574952 5.6526639521
13.0185037395 -3.01055783237 7.76877093507
11.6107932814 -3.37862347894 7.60137592397
13.0951104188 -3.47926295325 7.74000394708
12.4107163777 -4.6042009477 6.51474478762
12.1078087242 -3.94911623485 5.98026054124
13.0611788733 -3.44046708727 7.64869562674
12.402527003 -2.73628544794 7.75955521229
12.3975358234 -4.60280695224 6.40239515955
12.518924096 -2.90049673742 7.49151596813
11.6727889635 -3.40463764489 6.84333840293
12.1208341153 -4.20193222873 5.76644826337
11.7260689127 -3.73183453308 6.08674787665
12.1308500652 -4.8881460379 5.68156224498
11.0285293462 -4.14046091819 5.90291193258
12.2167497754 -3.55316722973 6.49520752866
11.7593492097 -3.77170678978 6.02383960033
12.1300954301 -4.02620445463 5.92486157938
12.5560956479 -4.41976162974 6.93148482737
11.0786562185 -4.07622254874 5.98213408033
11.9424062648 -3.07116783762 7.36145025978
12.4987832668 -4.49725065415 6.74094328569
11.7949685529 -3.25052107356 7.08359837844
12.8214511976 -3.18151030086 7.39242122884
11.5140287768 -3.50902791729 6.86032920868
12.2574867788 -3.09291865971 7.14990790731
11.3486877954 -3.76399496293 6.35394225767
12.7039819964 -3.22766545701 7.23271338896
12.1358924099 -4.0282305774 5.93194415287
12.221279305 -3.71204684107 6.32135562808
12.4243587242 -3.32387290454 6.8948722536
12.4322817377 -4.29533626558 6.26209371757
12.6805536252 -3.18413551631 7.25069489867
12.1996789146 -4.07219438878 5.96752888731
11.7638894981 -3.16098267131 7.51076227217
13.1391325047 -3.32723617908 7.82034494015
12.5954960823 -2.93467501951 7.48121653783
11.9570997323 -3.22618578613 6.96666426927
11.4256514946 -3.61810170509 6.73231764858
11.2613019577 -3.83727619904 6.76046847936
12.0372825304 -4.00875157505 5.85252747678
11.5183851185 -3.97915550075 5.67255104947
11.7438246128 -3.99533792805 5.68289126316
11.8835276022 -4.30428127634 5.42112656777
12.4735118705 -4.48250908113 6.53475820287
12.7803167311 -3.7154707137 7.04465854093
11.5660837451 -3.42135624786 7.1243969754
12.3181543085 -3.08676709493 7.16877881668
11.5280665593 -3.4639634764 7.32072615879
11.2268577319 -3.86745361668 6.58235009965
12.6429997414 -4.2892117095 6.96970454663
12.7412420793 -4.1040882662 7.0706468091
12.0022624909 -4.44131362134 5.45256278965
12.3424655405 -3.96079395593 6.22436250968
11.9423935685 -3.41455466756 6.61783317364
11.6562459427 -3.59489698995 6.37416256567
11.4203648363 -3.72050939642 6.34157013987
11.7892909816 -4.11130761949 5.5601032016
11.9988548083 -2.86654509217 7.92299265504
11.4683220586 -3.72290815965 6.25514208259
11.9118230947 -2.94776698397 8.039314544
12.2922248673 -3.92101879545 6.1936076199
11.4527144233 -3.59066384277 6.76529951714
12.5308357027 -4.29716435046 6.52504880873
12.9737553298 -2.99260164705 7.72539297973
12.5391917811 -4.30081178017 6.54247582756
11.9358026278 -2.92704537061 7.94042417009
11.4473157517 -3.59169512329 7.22073317564
13.1133532639 -3.29408847043 7.77443924085
13.0133697496 -3.1133546755 7.68702267423
11.8594748029 -3.110656529 7.38524481851
12.1013147448 -3.33805975934 6.74689828721
12.3997366364 -3.88731248087 6.35375849553
12.7837042173 -4.0121363466 7.12466325859
11.0293357169 -4.13482869126 5.95517075938
12.1399865391 -4.10563678464 5.86966812922
12.963229342 -3.84891872415 7.66347269959
12.7489677389 -3.33521300086 7.19198382072
12.8333692401 -3.29271636554 7.3335964573
12.5331901204 -2.99918122338 7.3643015195
12.3694678809 -2.78726821834 7.66922167649
12.6227545908 -3.72101282699 6.76899435324
12.2743199102 -2.98323704494 7.33665154949
12.0317442761 -4.50662545498 5.45763827181
11.6550391132 -3.86194089941 5.86614920964
13.0284539624 -3.77507644906 7.86461010619
11.1766146972 -4.02936317465 5.84442577904
12.1821726679 -2.90374206084 7.52347121891
11.0410070002 -4.11435656655 6.21249351454
11.9209384861 -3.39684213201 6.6592753102
12.2393370673 -3.16752503346 7.03180584984
12.8946021329 -3.55355374704 7.2983202177
12.9264916877 -3.75293123471 7.38306961467
11.8170705885 -3.07922462897 7.93321390119
11.4167236999 -3.65908464803 7.19786896646
11.2723253554 -4.00310406626 5.75921805374
12.4344771152 -2.74433376038 7.73597532328
12.7335519272 -2.91824205397 7.57696243775
11.5255357659 -3.91021123066 5.79393021586
12.7909103742 -3.48874969019 7.1513984856
12.5764227288 -3.01161469574 7.37058670327
12.1106524722 -3.2971192372 6.80851537303
12.4324184852 -2.6801956379 7.86183414196
12.3225283199 -2.85643041653 7.56243078475
12.4087029908 -3.81848311982 6.41318531009
12.2679911547 -2.7930240069 7.69644848062
12.3487195472 -2.7928405617 7.67361687726
10.9895542523 -4.24468753324 6.31010914394
12.6408893661 -3.04378933073 7.36754777338
11.5964129138 -3.36963366527 7.29569120512
12.1993491889 -2.85960328588 7.60183845623
13.0493430255 -3.62319492139 7.67525618483
12.4290598451 -2.67682015415 7.8692746969
11.641245589 -3.49081299608 6.6274107191
11.4452515544 -3.61265374505 7.26933746967
11.8463743759 -3.14071379455 7.31131453765
10.9706072471 -4.20651969403 5.90999273549
12.4107574927 -3.50365447039 6.69101021112
12.9015815008 -3.25061476696 7.4473266307
12.6313611197 -3.22565485582 7.1630278224
12.7484571676 -2.83798541588 7.67906188245
12.2525613472 -3.75988480378 6.29632180471
11.5673623669 -3.8817534015 5.84042274302
12.5533463811 -3.78327775612 6.62843620522
13.108468785 -3.51720414879 7.79638755686
11.6216617824 -3.56921010683 6.455131782
12.1881821489 -4.67026728634 5.71738203401
12.4141699979 -3.92762620598 6.34633887203
12.3170097934 -4.27675013541 6.03556006491
12.2917160887 -4.46458305992 5.94368074745
12.5684133876 -4.20559459488 6.58504973606
12.4242132907 -3.66525156416 6.55270811706
11.5542299653 -3.477700553 6.88535815411
12.2066710022 -2.71071626441 7.95541758215
11.1251857815 -4.01869327568 6.07823427993
12.2220793451 -3.52880154439 6.53109640571
11.2223972602 -3.87940784243 6.66179496417
12.7713612892 -3.14712464162 7.37766123656
10.8922180881 -4.31115813547 5.81096717157
12.6750250132 -3.13024692566 7.30451426883
12.8397610447 -3.96777708757 7.28468395501
11.5838719338 -3.92629945151 5.75799840536
12.3987300552 -3.04217100393 7.25460365866
11.5003135934 -4.07344082973 5.51811665655
11.9437456377 -3.15958901746 7.13273384164
11.3381595786 -3.77994318697 6.33361555165
12.1399299303 -3.76052987882 6.20054317511
12.311450483 -4.71794242945 6.21500407135
11.9238695499 -3.63251015431 6.26155236934
11.5974649779 -3.80774493025 5.97712757292
11.6741355763 -3.28317181564 7.67950558532
11.4956575476 -3.50938355674 7.03959991725
12.183577152 -3.56514939729 6.45990365641
12.5409525355 -4.4087831058 6.72231226914
10.9059493874 -4.36318672815 6.13936720612
11.960694653 -3.72503408611 6.14397404973
11.8387369843 -3.04530875216 7.89580353499
12.4293558358 -4.39655420087 6.27823361805
11.7443274308 -3.74124192753 6.07424640763
11.8091008749 -3.41919839662 6.66277659427
12.9838916908 -3.37193688601 7.50967852639
12.0096531273 -3.12075643 7.16679969647
11.8730297096 -3.29299605725 6.87886729369
12.2319595693 -4.82130479976 6.02254376597
11.8394926515 -4.06113071944 5.65046334695
11.5931474617 -3.68062164291 6.23228130109
11.0458477837 -4.14717204668 5.77909256272
11.5914624526 -3.41859906208 7.60333534548
12.7032120297 -2.94318042772 7.52923670868
11.9684269648 -4.44086079255 5.40741279415
12.9196522734 -3.56456816899 7.34496642103
12.4446668646 -2.8633191092 7.53568769522
12.2528263007 -2.98237464177 7.34089141621
12.3804281202 -3.01619986789 7.29023240173
11.6851636274 -3.26414002354 7.40427086359
12.6909356544 -3.81901588842 6.83925627347
11.0236185299 -4.13982332045 6.00858158006
12.4681295057 -4.1843745045 6.33936878266
12.8713972146 -3.9467249091 7.39451242666
11.959405302 -3.97569419346 5.82875703944
12.2461036679 -2.81039777384 7.67921274967
11.6587207356 -3.4562323484 6.7034690791
13.1423395141 -3.4666444066 7.85601930171
12.0364514111 -3.61051172065 6.33759890447
12.8877990561 -3.89492775469 7.37169529647
12.3911934023 -4.50194103502 6.21741971197
11.6992721018 -3.70289250201 6.13534606056
12.5252917429 -2.99987783447 7.35609950342
12.7850501269 -3.33136464281 7.23632591437
11.8407263488 -3.06277872462 7.70780068716
11.7040692774 -3.36297798058 6.88814403221
10.9671921647 -4.21416407599 5.99032570645
12.362117731 -2.72609857139 7.78465101242
11.6310925343 -3.33255909456 7.31798679545
11.1336474961 -4.00251582608 6.49432165797
12.094262738 -4.62338430086 5.50899198578
12.4246868073 -3.98164229714 6.32557037021
11.1091127752 -4.03779666102 6.45607317636
11.9912868595 -2.86426009063 8.00517570506
12.5920380177 -2.87615755043 7.55587143367
11.9070258053 -3.07571886944 7.39392109305
11.3644409691 -3.68962701681 6.97344468856
11.0730723056 -4.09480856359 6.39775839542
12.2753396743 -2.77649177751 7.72867770582
11.5786863847 -3.52507928925 6.64133059387
11.5339942239 -3.84973026497 5.90748498387
12.4945181798 -2.96870950534 7.38497927111
12.268433518 -4.7761414814 6.11519494222
12.8666705243 -3.86601442146 7.28109409897
11.8614102651 -3.79833564943 6.00339437398
11.1992257721 -3.95757266227 6.77850540788
12.7897412683 -3.77498797396 7.05256819455
12.7232536544 -3.13636790035 7.34211888402
12.9554606913 -3.36511955075 7.45642066185
11.8650288561 -3.84979665593 5.93758328454
11.3384463561 -4.06023512549 5.57853524941
10.9732352653 -4.20865943522 5.86521136411
11.326653094 -3.74320313978 6.88990656538
11.7509228248 -3.22527110124 8.0085515275
12.3581654367 -3.57066095809 6.57559699437
12.2003042259 -4.64862872202 5.74058575888
11.8213841784 -4.15900121757 5.52620468656
11.3468857092 -3.74122801983 6.44932034108
11.6195092989 -3.67240441782 6.23215476711
11.9175697366 -4.19054563581 5.56437089909
12.5755740772 -2.88586419704 7.53903667563
12.1735122987 -3.36055543306 6.72547861498
12.0923912675 -5.02343153902 5.77658964354
12.684246993 -4.25346237775 7.14523278781
12.1178722287 -4.86944808136 5.6063210808
12.5515795202 -4.42910095759 6.86366127289
11.8559785393 -3.19496383867 7.14899409893
11.8156644804 -3.8375337287 5.92846720686
13.0069361352 -3.56302670694 7.53533699212
11.7632878877 -3.30492339692 6.96323566327
12.1199930407 -4.0931748167 5.85425165688
12.2298193118 -2.9914416025 7.32689076284
12.5298625232 -4.39624993548 6.6083498272
12.432378331 -3.60810657327 6.60646147994
11.7996001397 -3.25284578859 7.04873564943
12.7012959495 -4.21267708198 7.12392369141
12.5488163497 -2.93011950003 7.46058840629
11.9121719496 -4.10042073231 5.6599216769
12.9439174936 -3.3518874353 7.45696723102
12.0968711812 -4.05792420364 5.85990027171
12.7034616665 -2.90091277299 7.57620466274
13.1806160993 -3.31319454074 7.92391351265
12.2696531661 -3.86561636423 6.21473889567
12.4789094375 -4.52116784934 6.67905896879
12.8476544773 -3.07968853918 7.51309615933
11.0698822769 -4.10248018989 6.38611821206
12.6910447896 -2.7780934994 7.72679629122
12.298155746 -4.61423824826 6.00193175202
12.3866958073 -3.3266532963 6.86843687009
11.0622298917 -4.10230137722 6.34655608054
12.8000189407 -4.04659331547 7.23360713897
12.5489769154 -3.74455035618 6.65115758872
11.2311610697 -3.85537474292 6.58575661388
12.2665667233 -4.42576861023 5.89569971562
12.1455585429 -3.20706934509 6.96365014925
12.6085136436 -3.31286824297 7.06085675745
11.8693187665 -4.16378777527 5.55528863087
11.2891061 -3.78304404128 6.66399305242
12.4464549728 -3.08388164622 7.21438104054
12.7047761177 -2.77968781903 7.73388317543
11.8439406925 -4.11600257839 5.58466138155
13.0150073594 -3.1418650055 7.67430862587
12.8583068847 -3.71171921933 7.20265156781
12.7230841562 -3.8889092199 6.89784920229
11.4973087937 -3.65078949402 6.39542394109
12.409683099 -3.3358100757 6.87747981894
12.4124792584 -3.06388709831 7.22150878673
12.9099514389 -3.94215945465 7.65367697613
12.8996230223 -3.73082823496 7.29946601838
12.1474579054 -4.13807057191 5.85237810659
11.9978752627 -3.24517954368 6.9242688058
12.6671040241 -4.24303687169 7.01779912652
12.4239219365 -4.48596676018 6.32574715718
11.7474134763 -3.36477685364 6.82471932789
11.5257574274 -3.5738977749 6.58849185619
11.7267107021 -3.19559562686 7.58894938955
12.1448006212 -3.67847001022 6.30173749885
11.6627377448 -4.16375227077 5.42309277794
11.0981758052 -4.04748359082 6.39250398638
12.6230488096 -4.29292106806 6.84775812809
12.2842890816 -4.44120053209 5.92620484182
12.8386250544 -3.13184955119 7.45527347277
11.5674234017 -3.42348417274 7.43835722031
12.4535512497 -4.13256951492 6.31358755023
11.386485555 -3.70473574424 6.47350347681
12.4975431515 -4.49825455755 6.81485082479
12.5493468359 -3.28237141238 7.0383137439
12.9463477231 -3.65892037382 7.39976979192
12.9414050809 -3.89444018519 7.69722665289
12.6612423842 -3.34421175368 7.08545965321
12.0123380086 -3.14589946769 7.1052985184
11.6424789007 -4.15793616772 5.43225098065
12.2607715188 -4.47098008204 5.87815130326
12.210733996 -2.97283000351 7.3675434772
11.5235893015 -3.75949629314 6.11697853436
11.7766564205 -3.51264094375 6.48028702263
12.9421118349 -3.90330428666 7.74996366452
12.0312931523 -2.84478267014 7.91928778314
12.568504663 -4.19257921415 6.57381302005
12.6532830132 -2.98745697397 7.44508474521
12.6912847595 -3.7600518232 6.8652235473
11.71166765 -3.25469915634 7.29564790472
11.2389940725 -4.00292855661 5.80636906346
11.7736130893 -3.75685497225 6.03887479204
12.4049341855 -3.75190545292 6.45997517955
12.0752272505 -3.63811304163 6.30826188228
12.9714408889 -3.84853932522 7.70583404371
11.7216734133 -3.25113159276 7.90709320973
13.0417945657 -3.42915549069 7.60865022866
11.203172327 -4.01041653825 5.85093395185
11.4261101654 -3.651135529 6.565938183
12.5349207176 -3.43372292878 6.86686143411
12.8451051949 -3.10791222513 7.48135829223
12.4477413135 -2.94334126038 7.41992613066
12.8489748615 -2.81679264765 7.7772655983
12.3774178239 -3.11731818377 7.14131242998
12.9035928116 -3.50500578619 7.32740219254
12.0177970788 -3.45145085541 6.56310115289
12.0140444584 -3.46484916124 6.5397951211
12.4540964581 -4.1756168934 6.31087287574
11.8264853469 -3.84922843945 5.92061647797
12.3795849181 -4.3676214039 6.13905780438
12.0909414103 -5.02895749173 5.80904714438
12.3546862595 -2.78315284392 7.68235275361
12.5307583466 -3.50586140224 6.80286935485
13.0368523916 -3.7622888686 7.84860865627
11.8832678598 -3.11655239328 7.30832942817
11.7110109427 -3.2475282324 7.81565454468
12.5200733242 -4.31955609708 6.50307171595
11.8146546006 -3.19238334111 7.20859039744
12.5789701515 -3.31842752108 7.02577314463
12.4490834188 -4.44600022913 6.36535800967
12.9832605276 -3.64445706254 7.49446947466
12.560460194 -4.38883392588 6.77708256607
12.7649342776 -3.90215872674 6.99578202907
12.2201065727 -3.22650345223 6.93911821904
13.1607228195 -3.5336064025 7.98261125043
12.0328471643 -3.39821991971 6.64108958125
12.355546834 -4.68920235106 6.48229563686
12.8660104886 -3.9893955223 7.48936280236
12.3214397549 -4.68462955875 6.16089806877
12.0205052362 -3.46033026053 6.53298209934
11.8259396095 -3.26151171819 6.99744165912
12.6040891478 -3.82768989048 6.68841763308
12.7853567217 -3.67733425949 7.05885457767
11.8413911511 -4.03706102939 5.67929397866
11.9325822968 -3.09042850486 7.31504396044
11.9492449573 -3.97907966243 5.82058814924
11.2311053477 -4.11055305469 5.53744973585
11.7638016091 -3.19845682907 7.35869983849
11.9385836196 -3.46472195212 6.53605766759
12.5287601973 -3.62993621269 6.70183358818
12.1879066361 -4.79705806758 5.79234982395
12.4250028441 -2.78417126661 7.65768790064
12.037670402 -3.00954030916 7.40369500036
12.1571667041 -4.9505246233 5.98915807578
12.5098207084 -4.48723520986 6.82692460093
13.1169814679 -3.60382765863 7.90717761753
12.2817814486 -4.21281803321 5.98908768029
12.282760176 -4.11246998653 6.05185796456
11.084757117 -4.05036211851 6.2079204151
11.9927862815 -3.11871993801 7.17954933196
12.5633919309 -4.35068901233 6.70866802809
11.7262336107 -3.47272547229 6.59485934747
11.6695909785 -3.41977655582 6.78681315942
12.850484856 -3.9243469008 7.26714784854
12.2839807969 -4.79258056568 6.31553990909
11.779023055 -4.0745691081 5.59837783638
12.1573781069 -4.08953139487 5.90295201354
11.0551179339 -4.09159081755 6.07167035335
11.5305148945 -3.91589767605 5.78540338975
11.3323627029 -3.78134318081 6.32615823686
12.3981164216 -2.80021450023 7.64588294094
12.8349405109 -3.45804885152 7.23054863463
12.0129332787 -3.15894744573 7.08495491216
12.7524691715 -2.81820170409 7.71367037804
12.1568890756 -4.87032663614 5.77467167443
12.8041503585 -2.9868943551 7.55848828811
12.4060821561 -3.55724023419 6.63521342996
12.1448238086 -3.59212778044 6.41199722316
11.2896723211 -3.80030653024 6.84133840759
11.5435225842 -4.04329127774 5.57137048199
12.0321476882 -3.20580143076 6.97348141254
11.9552269608 -3.28741566619 6.855253008
11.3724938672 -3.82959699569 6.08085032218
12.3372505509 -3.77803670945 6.36173021448
12.3230235348 -4.56913118182 6.05634044963
11.4448515343 -3.59761765813 7.21458939552
12.3893668574 -4.03937231787 6.23846596355
12.8307202662 -4.05974630966 7.59092418395
12.2335397483 -4.82509130268 6.04327842204
12.5487570075 -4.32678071139 6.59326691904
12.1483001637 -4.94350726049 5.85890302315
12.7760475559 -4.0966856995 7.22692222807
12.5034570183 -2.9443850265 7.42234600338
13.1677872345 -3.48378197373 7.95541797465
12.3733474676 -4.4910883498 6.14366970893
11.0101148121 -4.18865606791 5.73144492139
12.0804074414 -5.04060495979 5.79395330408
12.4551588803 -4.40666869509 6.35480433112
12.3227972629 -3.41509199932 6.72320719742
12.7091157022 -3.44308840436 7.06237848106
12.2691501089 -3.93719837175 6.15601495228
13.0335612119 -3.03427232748 7.76633536721
12.3195366547 -4.67472271648 6.1552740719
12.428357254 -3.1990879802 7.05515865494
12.2967902304 -4.60468977117 5.99048001223
12.5142844687 -2.68218508183 7.83029488963
12.5151428569 -3.73516563105 6.60304719652
11.9442074176 -3.17319288465 7.08399956791
12.9834688161 -3.61309442735 7.48617732341
13.0474579924 -3.64248746732 7.69220209778
12.4285777296 -3.07319944233 7.21463652918
12.2994179216 -4.58959587953 5.99569226055
11.9058837167 -3.20401918337 7.05225637426
12.6750909787 -3.63634790239 6.89637454932
12.4746630323 -2.73334057695 7.74660180664
12.4983526521 -3.31926599956 6.95876701947
12.1496070168 -3.5509597161 6.45821154687
12.0449529693 -3.56924013564 6.38728890943
12.452786359 -3.12610116804 7.15808550555
11.4002406555 -3.71814243833 6.37734733064
12.9359487615 -3.13427385955 7.56655338693
13.1711490287 -3.3923596155 7.91283208879
11.8396241589 -3.06163258485 8.02583398987
12.2417009326 -3.07013755107 7.18954807056
11.9990633271 -2.91761598232 7.72420599266
11.6641457265 -3.51399478151 6.55163530977
12.4762488864 -3.10023700396 7.20011109042
12.4905134838 -2.67676235204 7.8512403946
12.8147852251 -4.0818159811 7.49765293827
12.4667343394 -4.10742787372 6.3453525312
11.3377062255 -3.74161371775 6.48664173758
11.5770928735 -3.40013607459 7.32370071096
12.0417413611 -3.32080482457 6.77133868306
12.3408231084 -4.62547318632 6.16460386773
11.9468489652 -2.90313526214 8.1051640778
12.0004380596 -3.91277259485 5.92752133
12.2982279702 -4.46262033922 5.95609511382
12.3947255819 -2.88725524998 7.49612504735
12.5592787051 -4.09425862341 6.53482570465
12.4935060562 -3.40628964722 6.86302873605
13.089345836 -3.41989474199 7.73191099346
12.3439957418 -3.36913257312 6.79961719162
11.9920505548 -3.18569978554 7.04293072846
12.8550319152 -3.58094473893 7.21662058699
11.7944119879 -3.26313092252 7.0429059308
12.1002759188 -4.53086189829 5.55195579302
12.526616322 -2.9062041805 7.49856485241
11.5528347251 -4.0854499124 5.5051493867
11.4792020807 -4.08583673476 5.5035709387
11.3683305441 -3.67834487182 6.6775273973
12.1845718302 -4.67048626294 5.70821503208
11.3354144743 -3.73467613678 6.55253916166
11.9445681453 -2.91080431944 7.99457988505
11.8430053623 -3.16858263983 7.23435448615
12.9556171246 -3.72947189577 7.46042553734
12.170404364 -3.89132693582 6.09273061771
11.9762179378 -3.89604293998 5.93293161659
11.660083244 -3.68060244648 6.194014379
12.7086687969 -3.60903513808 6.95786359018
11.0000430805 -4.20743602254 6.2715240732
12.9386402652 -3.54638349177 7.38583396471
11.1495854243 -4.04994085054 5.83223262076
11.2828022659 -3.79334063855 6.55089333779
11.92066373 -3.74608568623 6.10099152728
11.3635334698 -3.69625898368 6.59797864451
11.4039396159 -3.70533681572 6.40609007616
12.2784168685 -3.70116544889 6.37447402159
11.987525901 -2.98780474174 7.50839332349
12.5070285777 -4.26987496019 6.43598070471
11.4897746493 -3.65067993565 6.4187185949
12.39465754 -4.64313752087 6.62457106941
12.1283425897 -4.340610097 5.69276068227
12.0221046222 -4.2417520442 5.62020017561
11.4805842086 -3.59821563256 6.60764808672
11.3993846508 -3.65031194219 7.06158323384
11.4636972042 -3.54845333326 7.01820919018
12.1914664959 -3.91498910777 6.09422054167
12.783852572 -2.98653793314 7.54194270412
12.5302331932 -4.45598966269 6.81567038258
12.01205459 -2.8316660816 8.06257673488
12.6676474845 -3.56202812115 6.92567835007
12.7032731089 -4.22534783175 7.21856492672
12.4189531712 -2.95185643768 7.39420347053
12.8879817997 -3.94521834026 7.48439536216
12.9472729955 -3.51505058374 7.41364359382
12.4960930826 -4.29165906612 6.41647642441
11.0344335441 -4.12466999094 6.06357169669
11.0634559128 -4.09348627415 6.31304979322
11.5262007121 -3.87414987427 5.87191529177
12.9113517334 -3.72257875268 7.32932837332
12.6013918405 -3.24172160208 7.1228937362
11.9078683983 -2.96373726854 8.15352219806
11.8232713993 -4.21652135468 5.46123880013
12.4106221331 -4.41804186072 6.2249362276
11.6702908669 -4.00874030457 5.63896391247
12.0964689149 -4.60645175489 5.52604844617
12.3672221843 -2.96699998037 7.35999667272
11.6129370796 -4.1314320196 5.45257514267
12.9655773041 -2.93572576123 7.75973930555
12.1015373579 -3.13193972709 7.0920676332
12.1722346013 -3.65474503148 6.34420910913
11.6546485572 -4.18603532072 5.39677578492
12.0925642873 -3.92193511589 5.98802053866
12.9764078361 -2.93832880952 7.76990384438
12.9718797638 -3.30669666974 7.51514116129
11.9896017312 -4.38743095598 5.46958861546
12.7157080689 -4.14643584696 7.02799607155
12.988025853 -3.59081329186 7.49551527466
12.1095524944 -3.12010591968 7.1223298332
11.7492265682 -4.14525061884 5.49380473311
10.9592825099 -4.22881355623 5.75713764187
12.4498877552 -4.51795311141 6.46967866585
11.5769849723 -3.53521251086 6.60964851079
10.9243012104 -4.29688745963 6.0719468297
11.6484187221 -3.81654059497 5.94996048789
13.0705801063 -3.74064565824 8.11775989923
12.1202389779 -2.95557470789 7.4412647418
11.5345669846 -3.83537869729 5.94034731173
10.301137181 -3.23861957224 6.65312573241
11.4296643654 -2.32438906199 8.4386560293
10.8031074768 -3.00185753424 5.08308044477
11.1592948308 -5.98097527192 7.68400991235
10.7649475432 -4.0011609582 6.21701111032
10.9065716774 -2.98504803502 5.95352202285
12.5118736504 -3.63885469434 8.65725149172
10.7382132798 -5.53338382404 7.96277985522
11.4455151091 -2.85528394934 6.19487618793
10.3636369738 -2.35172514399 8.03389254302
13.9145157505 -2.02757045677 5.74258515823
10.2595333244 -5.04984759994 6.06403651514
13.1387769264 -3.2266292494 5.89604038695
12.7334812072 -4.50737213817 8.78904671916
13.3738922724 -5.55086012818 7.2083206501
10.6624393144 -2.52243647035 8.86349455131
12.9382890324 -2.932968209 5.71138144295
11.4735868783 -2.0229038808 6.44746208851
13.2499657311 -4.05639974145 8.44414590206
13.8188864569 -2.6148263741 8.6620876875
11.4502575925 -5.46620481749 8.05585743562
13.4981212254 -4.31751662346 6.50068307668
12.5963224067 -2.16278799312 5.49767067895
13.2474709126 -2.82470667561 7.80670269429
13.1921893972 -3.68084073615 8.30955343888
12.3504704543 -2.5077935455 8.76525037617
10.2037848012 -3.52830116308 8.91104407111
11.4667239105 -5.91168854816 6.57384398155
12.3438264321 -4.97544673133 8.61708825881
10.0295853826 -4.81467812508 8.24508518678
10.9918955261 -5.27233778706 6.22327361011
10.7744135879 -3.06038385376 8.45371623184
10.9154621178 -2.57501622997 8.4199644544
11.5989572529 -4.40801441471 8.52222565856
11.1042785871 -3.26493857313 8.79276997947
12.2768386308 -4.99971317374 7.18194662117
12.2876625042 -4.07569508144 5.83285123264
10.0856528488 -2.65979219197 5.01203691945
12.738418786 -4.85480654162 6.22269068252
10.1987169263 -3.73065610226 5.79193925932
10.5776109076 -3.85913336554 6.51842238598
11.2833612323 -5.00129415251 6.73344557627
12.9759783708 -2.13599284254 7.04634842834
11.3056648699 -5.25675926456 7.95578911197
10.7970915399 -5.34993501616 7.35822173903
11.971773399 -5.77497925302 5.00146607115
10.0187558663 -4.9284247147 8.32031433775
13.2363841138 -4.82097402461 8.74898199347
11.7200909747 -3.65392230618 7.99257039701
10.4540688388 -5.26433937683 8.40482872929
11.0392689422 -3.02357190199 6.80403792683
11.2924109454 -2.21467401692 7.57257631577
10.3220056121 -4.85761967627 5.93654558999
13.7768926723 -3.44781696588 5.85732531078
11.6497730641 -5.50530470523 5.02884755791
12.1314349163 -4.37535473134 6.39673835598
13.3334529289 -2.88959753705 8.4207332512
12.6903452195 -2.29494574886 8.36009158804
11.6466295965 -3.09156540463 5.82088058476
10.2985184537 -5.99270819972 6.75635376155
11.0686732716 -4.51908540263 7.80839872145
10.135949353 -3.2685358921 6.54367722938
10.1537511017 -4.03003069125 8.00332937394
10.6293421292 -3.15795116076 8.60488529743
10.890081937 -2.31326972911 6.00125039436
11.4958130958 -3.28569096107 5.6822717407
11.7018110673 -3.38375410924 6.24095562909
10.3258717049 -2.94117246986 7.85692136701
10.8791197891 -4.57189086848 5.84007416665
12.114354376 -2.57480725164 7.66263136817
13.0955437945 -3.4441312201 6.59512563276
11.0381932795 -5.6290829174 7.77599361393
11.3604650204 -5.87009831604 6.42988389968
13.5723684988 -3.45424162094 6.69730524101
10.2247977279 -3.00570058973 7.24663858386
10.642459317 -3.37711910099 8.14553543572
10.8065985296 -2.24219846209 7.25645609311
13.9778857906 -5.00343750938 7.53147621035
12.0920939533 -5.35676130751 5.51395208121
10.3535737162 -2.44473571436 8.85535419374
'@

Write-BundleFile 'samples/sample_A.pcd' @'
# .PCD v0.7
VERSION 0.7
FIELDS x y z
SIZE 8 8 8
TYPE F F F
COUNT 1 1 1
WIDTH 1400
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 1400
DATA ascii
0.318566321525 -0.494219847033 -0.158449013831
1.75369209037 0.405364792877 0.0975713695779
0.0696251454785 -0.0836364826559 0.00104485075467
-0.241393547281 0.0471595746946 -0.163324527823
1.39616557831 -0.144297718145 0.163226177594
-1.52053041328 0.549066411105 0.0103832983503
-1.06602144451 -0.253102014817 0.161675949563
-0.419585379453 -0.219934823596 -0.0313736760516
1.93684816091 0.178420088566 0.112576937111
0.655614952279 -0.542539897681 -0.161813831771
-1.15610685132 0.0597908185358 0.290957189551
0.279161755009 0.0907049686998 0.0484837803184
1.28772963027 -0.225837833961 -0.172131382969
1.32018554901 -0.181410147384 0.275768491366
1.76208200001 0.24030502976 0.252463246622
0.878788306097 -0.390752993848 0.0397365459758
0.106916801105 0.0851274760224 0.0649233513021
-1.78761116621 -0.193945580775 0.0145177863744
-0.244524333446 -0.537737462643 0.248454467027
-1.77164668782 -0.171685458362 -0.0504998893301
1.15727740297 -0.0839636102396 0.100655037845
1.70607827008 0.373681846912 0.0586430930018
-1.78665590224 -0.111466105925 -0.0301948442598
1.18692270811 0.169588156245 -0.05007909923
-0.369134792084 0.370892151522 0.23356663531
-1.73261829174 -0.0731904011023 0.351140030861
-0.447449940501 0.13964549434 0.188681057629
-0.488489104981 -0.184347493634 0.17091475557
1.2621610473 0.561996603683 0.278507526385
0.0646437047455 -0.0388546487528 0.033679216758
-0.355015574278 -0.438414552582 -0.121228083293
0.0995908977534 0.149451360621 0.0268870237008
1.70867621761 -0.314496309171 0.183774969099
1.82579063054 0.0221242010295 0.349517699234
-1.20105136516 -0.284744851163 0.264376631236
0.486267872676 0.154596128863 0.155925976257
1.72107370817 -0.183302516891 -0.0515037692505
-0.0389559106409 0.122772015341 -0.235656048066
0.357073445136 -0.527177868456 -0.145711817659
0.633736777923 0.388942563129 -0.100291119124
-0.338070198005 -0.51485264064 -0.0772541724476
-0.659799636537 0.220990069497 -0.104655267469
0.763753229849 0.529778225645 0.096306089431
-1.36948303753 0.131141592188 -0.0131169608724
1.34855758025 0.528473996113 0.134470981754
0.679189342148 0.482727795548 -0.0683559750216
-1.11464955053 -0.257777967967 -0.0309287139665
-1.7731500626 0.379493447255 0.0131958029622
-1.66001486817 0.151173850244 0.159050565497
-0.822404995595 0.352259617357 -0.229211856195
-0.357633326773 0.0947388925915 -0.0989705038118
1.34275676969 0.304983335307 -0.0754145691653
-0.206218900924 0.465434062199 -0.0619104817286
1.287361161 0.233991616917 0.314837035421
-1.6335564711 -0.0202927716936 -0.0187133060771
-0.711856742328 -0.545870556084 0.282907616391
1.06424023368 0.112466462052 -0.0154420251814
-0.826033168712 0.375127610541 -0.225851347358
1.03915796438 -0.420441414775 0.192528139139
-1.20586500268 -0.127327969357 -0.0461262504268
1.37363395582 0.102997673249 -0.107733679694
-1.21780625185 0.28949713651 0.235809391272
0.814714291727 0.56278040107 0.302304784753
0.497062721453 0.497381759712 -0.101621389082
-0.17332636816 -0.486990617685 0.119031161796
0.910905776015 0.356395109138 0.0408438856933
0.652389038392 -0.119318464825 0.0332924145586
-0.694973711422 -0.442650565709 -0.192674961827
-0.265517109634 0.093999012925 0.167850656173
0.888179744597 -0.519971633025 -0.171701260311
1.84759067073 -0.498999658133 -0.141452078985
0.411251910666 0.528085928111 -0.164287395948
-0.612440366787 0.448857378801 -0.225792344609
-1.22500905088 0.591420624511 0.0225748623068
1.48824609289 0.454872313271 -0.0236202930551
-1.39276894695 -0.257471297595 0.0947978717411
-0.0818939193173 -0.535180908813 -0.00285011098465
-1.48554727263 -0.466458098841 0.248388973126
-1.71461436542 -0.168209260152 0.0597823095096
-0.155279172103 -0.0532765869788 -0.0253004835013
-0.638372596879 -0.572103431326 0.211268042114
0.779772977749 -0.542436284472 -0.0590093231535
0.318550219157 -0.434317531361 0.0591341400268
1.11703159249 0.311763569055 -0.00175467867714
-1.67018115561 -0.0313000781501 0.0885398623017
0.864727129015 0.434147206196 0.189422435432
1.21602959973 0.217278591816 -0.1280132846
-1.14217729495 -0.183243283064 0.0900513528942
1.18896817179 -0.513026246699 0.0889514366384
-0.138497354394 0.306363864513 -0.237358555111
-1.17065386613 -0.448563540455 0.19736604331
1.40513486034 0.348967448507 0.10822551667
0.267784301099 0.368319413395 -0.225128081684
-0.238812835986 -0.0743968885741 -0.239133877501
-1.9546026568 0.132916777001 0.362343999214
0.944638208677 0.321865268573 0.00458926836865
-1.91640564304 -0.267775786238 0.236971926724
1.24284464117 -0.27231427472 0.287993929067
-0.0129234267146 0.554150133279 0.0941222109656
0.685011616317 0.538930512392 -0.00878968924015
0.572516277912 -0.337339809092 0.0651085192123
-1.48476939758 0.52191073105 0.300629477798
-0.378911259297 0.40405547433 -0.150619576932
1.35545210422 0.581977211334 -0.0436037804703
1.98701177738 0.103740439784 0.225273503913
1.55503698839 0.317306500543 0.313725862796
-1.40709348422 -0.185132280112 0.0627283576581
0.889665533105 -0.402336650261 0.164637371158
-0.0621259665168 -0.344087064971 -0.0977138427232
-1.75857957758 -0.582160077178 0.239535159998
-0.201706474303 0.460353143009 0.0485244420928
-1.18215627645 -0.323836035818 0.202963591541
0.977200857865 0.174625954763 0.0571918811299
-0.972985083161 0.429547562367 -0.0330026474681
-0.646820975294 -0.323658934801 -0.151209653901
-0.745277663779 -0.404709378945 -0.0867121967467
-1.36846697638 0.210238944413 0.289099829835
1.4437996356 0.0462174192527 0.221543345821
1.08691007501 0.576352478881 -0.0512934445774
1.47899688503 0.122140094578 0.301494510658
1.76078946166 -0.434565275531 0.198031502592
1.6880126824 -0.0998314625973 0.261664550499
-1.91969826854 0.159654123223 -0.028241124675
1.85589305023 0.200279722524 0.416914219414
-1.99905809023 -0.362052578794 0.307483685969
-0.470667766556 0.347944039773 -0.230700849861
1.77438023395 -0.0484951122858 0.122826697132
-1.78897858782 0.219921081154 -0.0307112662002
0.486551523537 0.119953879157 -0.160959355011
-1.77517594289 0.364421820839 -0.0674366649798
0.427390768936 0.1138654724 0.191504948297
1.28422739964 0.329138116705 -0.0355416407489
-0.74112762599 -0.582619040241 0.163354623756
-0.00889096960086 0.54084896447 0.246052496684
-1.29197363335 0.0130077352329 0.051953302667
0.120319962961 0.478118944797 0.0375436771208
0.681032853688 -0.532843720599 -0.0344080309298
1.07436523197 0.263127127004 0.102659677317
1.38183816751 0.595075016768 0.0139822398281
-1.55934630442 0.23540243804 0.176842703983
0.625486254957 -0.229173550557 -0.187447799038
-1.16563127461 -0.210666449 0.209004133831
0.572928070013 0.221241177007 -0.161189671896
-0.0897266081124 0.53091207845 -0.123486711619
-0.00576260023785 0.339405120357 -0.0353120503353
0.770821650605 -0.153549446315 0.070551852652
-0.0793270057825 -0.300109101695 0.0445281048392
-0.0351769330388 0.384429287808 -0.0919740878006
1.53432306993 0.0117857989831 0.253835327998
-1.40740440002 0.15993053844 -0.00799495248758
1.050508639 0.207171730367 0.0317133639186
1.85297213285 -0.0894585180415 0.284455071531
-0.844597272671 -0.404555922626 -0.01275078057
1.11223805915 -0.518191319164 0.21099155876
-1.79011471626 -0.165623234282 0.0796203942209
-1.40756421859 0.557324411267 -0.148294296506
-1.66362852389 0.529042986578 -0.0210757833077
1.76242635947 -0.219204824946 0.28059282064
-1.12229611773 0.170756575251 -0.183447164482
1.33125079584 0.345590618823 -0.0521487273805
0.952088807516 0.388251983081 -0.184185171064
-0.441481781734 -0.140523187815 -0.182825806958
0.553564189426 -0.389457378925 0.0625399556278
-1.36330213447 -0.226062979679 0.262516260205
-1.63159829489 0.313537961916 0.0985573780312
-1.52295342256 -0.395038370718 0.304909539499
-0.945869007923 -0.243357593431 0.146099074085
0.0524726718414 0.496242033857 -0.0793540875553
1.59186904983 -0.153969661473 0.234110912753
-1.31693862478 0.204118327257 -0.187842935316
-0.933906826703 0.036435121908 0.0462921766447
0.277366574393 -0.378881959284 0.0409841828257
-0.598245991743 -0.0598911235629 0.215828482731
0.87791033309 0.0737917156538 0.0572558447829
1.11661662719 0.320323329447 -0.0289405046989
1.38057684213 0.486331696953 0.197995965727
0.717891645878 0.496882242243 -0.0994928372675
-0.543999601314 0.179103668597 -0.00136486429811
1.58257727341 0.02111806729 0.122286541426
-0.0872288046319 0.172152571604 -0.219042362589
-1.06594564337 -0.024438614277 0.0579847781201
-0.687409173061 -0.367046335742 0.0874971322326
0.533465719994 -0.241757874913 -0.124224465805
-1.38738209644 0.172840245256 -0.108955559712
0.650280228403 0.362165276419 0.267424520193
1.71315289449 -0.246555160343 0.265068607546
0.769295443533 0.595862488243 -0.0540059504069
0.601782831108 0.225889369363 -0.133767154395
-0.409857568924 0.202639535287 -0.241476577925
-0.391013909031 -0.418136909281 -0.21652190811
1.30797841081 -0.0886848776848 0.315494446904
-1.34596904613 -0.489236597642 0.0945432560725
-1.94928133501 -0.529747457419 0.293984031811
-1.07540087601 0.146009625981 0.161788228853
0.801253540894 -0.308787256069 -0.220456960513
0.704642318458 0.0839629531008 -0.203502514646
0.208674442567 0.313626610936 0.111881300044
-1.11798549921 -0.418139299605 0.0146032491247
1.40437291599 -0.048928605122 0.207539116358
1.01630474507 -0.0623330615108 0.217081801452
0.197115515081 0.425969102956 0.00406540251645
-0.956670176454 -0.423532498138 0.180556811004
-1.57267982738 0.28911040032 0.166825864408
-1.63282707768 0.381200426946 -0.124666450833
1.13582239851 0.49300996521 -0.0823337954021
0.0188959820347 -0.460022047511 -0.227233571588
-0.624827481396 -0.553698278555 -0.0947193202026
-1.09251162952 0.242766339103 0.204045680109
-1.7367614581 0.498228649394 0.150310265871
1.96199244616 0.569293565785 0.318793054359
-1.18423447526 0.105130008596 0.283191198747
1.29643784291 -0.443978238597 0.0716064250112
1.70511603422 0.240644367811 0.00200920724486
0.888268859022 0.289661676411 0.0596879361908
-0.231692670188 -0.493034876791 -0.0302027936244
0.831417621748 -0.0258504838684 -0.207414962187
-0.577770509327 0.307378284045 0.162285218934
0.303142224324 0.355305952793 -0.0296782338319
1.59211333131 -0.186754508367 0.212056044061
0.799411157475 0.446897258604 0.0119284403128
-0.613037291607 -0.367154334482 0.0762774342687
-1.34244780299 -0.0516671807944 0.128562544096
0.0814546733305 -0.561755285865 0.0694931364967
1.38873460301 -0.387393157585 0.0377934583734
-1.74961483881 0.565267092451 -0.0556256870653
1.6315758415 -0.240214247127 0.240273089934
0.953053061132 -0.553564650233 0.218772431404
1.11076842561 0.334157273375 -0.0148930902263
-1.23668673047 -0.333838928004 -0.0389014876357
1.57613771462 -0.436785950108 0.112389256519
0.691094719494 -0.366148741305 0.187181073265
1.48649037367 0.578125091413 0.0201873902549
-0.125013662412 0.57173687545 0.233465372742
1.32046313697 -0.145122523309 0.121071965268
-0.621273792894 -0.0736136218374 0.102282119387
-0.567020196303 -0.250350013582 0.269909499831
1.97590709587 0.248309810717 0.157425300467
-1.54601483353 -0.0536681542466 0.00308192482825
-1.36763990326 0.409565498472 0.226020263425
0.876999855727 0.417354098877 -0.156216961025
-0.649370098376 -0.35913676129 0.11881938366
-0.489263453794 -0.437479179948 -0.107992072597
-1.53152729684 0.47537417125 0.0815052830053
-0.359187716613 0.149220507079 0.139689742726
1.41870331364 -0.475544197689 -0.178791384986
-1.93973894078 -0.37041341477 0.377887072221
0.9408426067 0.000487751862698 0.285503134125
1.9634711197 0.167130219362 0.404948216894
-1.08974283637 0.149130666037 -0.118590928711
0.286673422029 -0.141587260379 0.236804262008
1.28377308155 0.508962187541 0.109968894847
1.35238164794 -0.455643334113 -0.00538491462083
-1.86582077099 -0.418530111119 0.345009570499
0.592280297621 -0.062464480096 -0.223014358785
0.176786125624 0.0583520937225 -0.10743188851
-0.461717741275 -0.576522477417 0.203625113341
0.0589280318598 0.282662803367 0.192366511263
1.24468012364 -0.346561555683 -0.152920489393
1.10068570881 0.218688823304 -0.0863757117316
1.70711583465 0.476381409684 0.149090144433
0.554728615612 -0.225170075377 -0.230715176185
1.94291648589 0.472965111879 0.142466242188
0.347501216303 0.0573872080458 0.129114313975
1.33181978348 0.107687620248 0.0409007773335
-0.545826184651 -0.478188167162 0.0397260623989
0.446926051405 -0.480101637678 -0.0239472654974
-1.60084306822 0.248837569645 0.124095955634
1.89469008071 -0.353654468594 0.238579561026
1.90219495471 0.149537561627 -0.0073216307007
0.145951991792 -0.21417005004 0.189884590872
0.935025412792 -0.423921519964 -0.0373174554449
-1.48666849265 -0.105760211713 -0.0590397021574
0.828313680825 -0.398305842428 0.0461922861486
0.968630817926 0.0179518383696 0.155541122232
-0.121024153548 -0.409491963754 0.233683892699
1.48231221148 -0.291449950728 0.153129505125
-1.33209071798 -0.324770775144 0.233188227256
0.254510448502 -0.299165086676 0.12525727001
-0.538110959915 0.494668720942 0.246826860852
1.86209500596 -0.159552247236 0.271108362936
-1.44365274219 -0.351893344314 0.304566745757
-0.599076041629 -0.53466153694 -0.199610382272
-0.710602860928 0.502493451549 -0.0483713670688
1.79891291081 -0.328731481786 0.119679178184
-1.48253300748 -0.0745496043186 0.310375972464
-1.24940546983 -0.513453062471 0.173575141586
-1.11205987912 -0.283421009387 -0.124441135639
0.629255880965 0.538454991174 0.0294468551836
-0.631244121979 0.292925587715 0.034967090384
0.0545808075022 -0.33655118927 -0.0883574192672
-1.75842738118 -0.341322137887 0.131776366688
1.91955774088 0.514431530003 0.272771538977
1.33212659216 0.178257738072 0.339985488963
-1.55468493802 -0.477520924512 0.266986616383
1.79544327635 0.518764373186 0.141552883703
1.02540784019 0.221137839687 0.238682376438
0.650094797916 0.161655123778 -0.0420178732793
0.220846304666 -0.0774133081086 -0.11670907102
-1.11803417056 0.1817964874 -0.0503112232933
1.18353319911 -0.071248404629 0.133703646845
0.803412726133 -0.317094240631 0.00886353909202
0.987384833788 0.387582560112 0.135472704473
-1.97943395526 -0.587087962842 0.444849917916
0.51975878913 0.408193504764 0.208740362374
-0.251171123146 -0.266107952828 -0.098304896075
1.51304912235 -0.175786026166 0.140573454138
-1.94386207278 -0.235999042246 0.351764746527
-1.20007027172 -0.368327248554 -0.0652217576876
1.92647387275 0.00263182770961 0.144286637056
-0.975221912744 0.0972023201062 0.188353948194
1.61207091516 0.413861221281 0.115935355309
0.0109633690799 0.578282868859 -0.0780253571664
-0.889031942757 -0.0372427931192 -0.115721726679
0.0789900509765 0.20591287266 0.146143801513
-0.507231619169 0.493633858434 0.100613996766
-1.96468039347 -0.46231449085 0.206267718989
1.71963121234 -0.490988321617 -0.0555830781814
-0.700250280942 -0.232653151179 0.111500142198
-0.393147640928 -0.513944915939 0.214748176489
0.0914457142059 0.515944187523 -0.0845656400518
1.90160868572 -0.0249750996435 -0.0319374727627
-1.5856239031 0.571722456531 -0.122096439099
-1.78159275726 -0.0578823447297 0.165212546426
-1.1585747985 0.447420532045 -0.20435736514
-0.488599025584 -0.346261138668 0.16789006797
0.549780447052 0.173935214367 -0.0291852817524
-1.69336844885 -0.132939850826 -0.00455502226202
-0.725079626539 0.542388847991 0.100100082385
0.336359303568 0.084742913042 -0.212157965286
-0.843689506342 -0.0657603637087 0.247900375636
-1.71736285087 -0.514279239996 0.416670397225
-1.16021220623 -0.471687463462 0.00138203855334
1.09005397025 -0.134451244809 0.178054290631
1.67420381438 0.351197530868 0.0679897695505
-0.710216120965 0.429209936423 -0.138233744455
0.554207207961 0.117169849353 0.255682470521
0.855552473695 -0.0939018560824 -0.151477608388
-1.66208990861 -0.0673275635346 0.140695000939
0.436992779457 0.136996298442 0.0142619209495
1.45197459994 -0.236578950958 -0.0835129956357
-1.3697147142 -0.182874444572 0.17090560169
1.60722894409 0.418258424799 0.062906081177
0.339285361172 -0.192050423826 0.247698280273
1.4026059113 -0.314953804413 0.238128802211
-0.292578360097 0.351197152506 0.0995680941271
-1.06439286097 0.167478675106 -0.00524530215492
1.93326105985 0.0805157679175 0.0396351405299
-1.10118085509 -0.591105725239 0.181603596995
-0.0245824713693 0.231229253963 0.0163130143072
-0.650958979132 -0.497368590806 0.0411914091778
0.820594211737 0.306375990699 -0.0268143338216
-1.20104468348 -0.418573476036 -0.13411646738
-1.59662344358 0.468294347819 0.106628382161
-1.53932126678 0.535474536142 -0.146809210889
0.801330676106 0.558920779941 0.024080079258
-1.53165042428 -0.281224570325 0.185026398697
1.65573920614 0.533368732805 0.0359068314913
-0.606672467861 0.0410368370717 -0.2009785093
1.07324592762 -0.112107262401 -0.0210055941441
0.531143588193 -0.018910618222 -0.191624122611
-0.0430630643563 0.236681014371 -0.188609215912
0.644975222427 0.310204917463 0.144204034328
-1.73955945309 0.565904019073 0.0455359866601
1.77992978655 -0.392315653641 0.0762301768565
-1.59952113352 0.53253836903 0.280746441197
1.24155907332 0.228791464616 0.309224547431
0.23924296144 0.490680651363 -0.0298059329786
-0.354337620732 -0.184126185452 -0.220355277659
1.27763417796 0.137227340661 0.0252112351094
1.74584367975 0.385152296368 0.0738530313878
-0.900127868905 -0.462133612744 0.0802728650085
1.29018409184 0.309229649831 -0.148410615615
1.46373495201 0.481463193438 -0.00668600556495
0.486830997321 0.589423251571 0.200237110096
1.53992068501 0.225112807333 0.196122937703
-1.2426607987 0.297434082969 0.0772186513668
-1.18849187235 0.105685681735 -0.131029554227
0.572120575581 -0.595282316213 0.0584154815497
-0.250757830026 0.052635262844 -0.115066879614
-0.221750209083 0.423836708391 -0.0377737375382
-1.01179884398 -0.410400482663 0.294664849076
-0.0724244342067 0.460885371972 0.0844155302565
-0.563843019982 -0.0205681345189 -0.0978417770959
1.16816792237 -0.256552960825 -0.0978532507928
1.38891200491 -0.527998798798 0.224243367347
-0.758397343374 -0.59410211189 0.265584596795
1.9227503004 -0.251507412032 -0.0132302520554
1.27533042211 -0.0645300705681 0.100798762565
-1.89987138271 0.441056631454 0.274610689973
-0.676961501778 0.512974226997 0.208173632815
-0.280751634038 0.274315490812 0.0429957945428
-0.404557672238 0.293076418144 0.0622144073754
-1.59027737784 -0.222679068223 0.173808630831
1.55086401413 0.398200677407 0.197903276072
-0.0449823607823 -0.399254871915 0.147807918662
-0.170156961688 -0.526591673425 0.0902158098725
0.185432067557 0.340516107926 -0.132673302683
0.721550279588 -0.527409549872 -0.171648711363
0.693609620082 0.0791048566531 -0.221096398542
0.999689318159 0.0512598226258 0.203835978216
-1.90918341811 0.564526477851 -0.0717356143856
0.14291372761 -0.500566861053 0.135407926745
-0.935690973974 0.172358321042 0.1701085934
0.573540959201 0.525330517283 0.0254353390624
-1.25954384438 -0.198006133115 -0.117491593507
-0.179503984319 -0.0414896489615 0.0309825706671
-1.63891057606 0.228927472965 0.140093540911
-0.514731876983 -0.0961621897968 -0.00552619577458
1.7213066813 -0.0860868386068 0.373464955024
-1.54690752973 0.256006678216 -0.094195060542
0.746894485655 0.106755074197 0.181014526969
-1.64567194464 -0.334159734682 0.108161529235
-1.86045213124 -0.370931991902 0.410005412354
-0.81447048471 -0.397135649965 -0.139521524427
0.978299211614 0.374125504026 0.204749424043
-1.42753772636 -0.335789253518 0.187394683254
-1.53226135838 -0.439377921389 0.37878745997
-0.0791882322265 0.413921528466 0.0150405942642
1.71391015955 0.255074003209 0.311750470258
0.630051417112 0.14888240108 -0.0863295169557
1.51182850312 -0.0679942296143 0.248522698706
-1.03163740231 0.544462350669 0.253013733632
-1.83250893737 0.378756095968 -0.0967289900318
0.120957277724 -0.500983114181 -0.114328241723
1.50758941502 0.132540654336 -0.107132256153
-0.600356004929 0.457595300813 -0.085726701555
0.673865688396 0.43286453913 0.116820331093
-1.9218811057 0.4281397262 0.205152777351
0.530402424954 0.124235224783 -0.181524998254
0.999961840562 0.592787088976 0.230883810806
0.961593440058 0.277729607941 -0.134017529555
-0.433738306338 -0.0256042082858 -0.229083317969
0.843157664153 -0.074054039974 0.249259568735
-1.45568785084 0.276925486143 0.15365618595
0.140434376892 -0.371413184528 0.174763665385
-1.33851667816 0.101933623737 0.0917342067898
1.01619374658 -0.0588385608501 -0.190761247839
0.266612203283 0.200175394494 0.169236950233
-1.72336663346 0.0850045784945 -0.00431810897225
1.95306294852 -0.278030650415 0.0241204563054
0.612297168709 0.23649496037 -0.0489469304018
0.370531314531 -0.539994582817 0.0760526060422
-0.933276167234 -0.274238221358 0.00461710760169
0.314715464033 -0.576470281141 -0.15837011563
-1.85747882821 -0.170237332895 0.108296711011
-0.0624518934052 0.112358355678 0.148724323697
1.94009984072 -0.0506665654399 0.200111178703
-1.06172422663 -0.458873207684 0.11354596963
-0.812671148035 0.115809987642 -0.0310829312853
-1.97295933036 0.318139072455 0.312018453889
0.640964546452 0.447959764262 -0.0656036945145
0.121664646336 -0.256442673367 -0.0699245530317
1.46663997683 0.519409163274 0.135932142189
1.38107306037 0.0173894918323 -0.109484731903
0.858030177579 -0.105182418999 0.0460910265357
-1.38647682211 -0.155310508058 0.306777735686
1.56347317394 -0.176783825869 0.149694361918
-1.72614725445 -0.427576943159 0.139109402328
-0.129166442219 -0.200736012376 0.148140866804
1.49027202641 -0.0585987716245 -0.11089155452
-0.317564455157 -0.407599435413 0.13194414178
-1.51143421046 0.204002131552 0.100207123212
-1.14039913368 0.258540273785 -0.149666675984
0.00745708286728 -0.325243397598 -0.169379618533
0.802283350463 -0.582677094106 -0.104558075314
-0.993728314853 0.229374218344 0.100271922231
0.846060291263 0.157785686106 0.0648728051551
-1.52231105392 0.326873425047 0.323265912303
-0.31434936242 -0.460528817192 0.0930181052152
-0.942414721384 -0.300628590479 -0.190493620728
-1.17360819353 -0.188723705515 -0.0761254258299
-1.14139871801 0.528535188491 0.013630339984
0.304276176633 -0.421077362055 0.0338227207038
-0.795353320533 0.0267640245572 0.250714369896
0.275361788538 -0.128807373344 0.140595385225
-0.706588949725 -0.216677170559 0.27092755149
-0.611025798512 -0.497285110848 0.0759514874753
1.46701297122 0.131268109533 0.0174169641453
0.209128468093 0.227199093297 -0.0136554319111
1.17917434436 -0.270989617423 -0.0662105751898
0.0810907699476 0.571722704285 0.212132420436
-1.36759859372 -0.36827164948 0.351813245501
0.469949676914 -0.451363420977 -0.167267083095
0.657874477482 -0.336071020158 -0.216738895565
1.63021324793 0.442418588761 0.103368937268
-0.0696157421673 -0.379863797397 0.0599631560112
-1.23319295471 0.59647835786 0.188570508367
0.958640482819 -0.0270663351128 0.157412223726
-0.0745797820593 0.570521070275 -0.157797339919
1.51313444534 0.527510847063 0.128988619905
1.70204764842 0.200452289093 -0.0800045746026
0.656493991699 -0.541622680783 0.0289648573171
1.9774837621 -0.490562068661 0.244489509903
0.846931133983 -0.00210756488476 -0.0594457780285
-0.314477075579 -0.075950946461 -0.00362801311657
0.925220532518 0.213437051805 0.207934193716
0.245640036019 -0.35679607417 0.236608036507
-0.0582782410786 -0.0381931690106 0.187219736661
0.709348518461 -0.21937420128 0.187371171332
1.62122868914 0.0551605995955 0.185493603148
-1.33294609448 -0.0312787261081 -0.130118435309
-0.122482104303 -0.0200206054749 -0.24889298141
0.203994703048 -0.519462155092 -0.121475236117
0.0563327573651 0.0649658686509 -0.0448957142203
1.43317103955 -0.0739741844136 -0.0819980703964
1.76681934957 -0.575893647533 0.114179971099
1.85127933153 0.475425195404 -0.00443233503492
-0.566557070098 -0.072118264769 0.160610867213
1.6598666012 -0.56110852508 0.238323205084
1.40945966959 0.221118306766 0.195390918554
-1.22586367206 0.557991456493 0.161526493982
-0.945861875055 0.280872120617 -0.0574840347016
-1.54046611854 0.0118670143113 0.0346332323752
0.0235482094911 0.541189545711 0.0709979331862
0.935038190702 -0.525134992497 -0.12434898062
-1.05273058047 -0.290218423462 0.103549000432
1.6742659409 -0.0965653922678 0.328933564787
0.290624379895 -0.521184864843 0.222468179839
1.25149649353 -0.416798095289 -0.0563585325318
0.697987471114 -0.45928468012 0.0750570257624
0.983882430635 0.582478860889 -0.0831326738495
0.675596178523 -0.404144301781 0.217152029596
-1.82252492492 0.383702152611 -0.0955004512032
-0.161915413341 -0.0769979856032 0.170364132488
1.84060684191 0.574854464078 0.0752576382803
-1.29221869658 0.024382765027 0.225539916122
-0.608244963658 -0.311561538173 -0.102648774978
1.97068079307 0.564204661939 0.0348512355191
0.0829423463829 -0.167321437332 -0.206626362812
1.09638128957 0.010570392087 0.105713864451
-1.53593093418 0.029802501811 0.293061761737
-1.48750578573 0.162731634863 -0.0617057741802
-0.924877652916 0.306617704559 0.207069579472
-1.70580500354 0.148621696472 0.0549111733379
1.97103622171 0.49555726069 0.1169555318
1.39828081569 0.45529921951 0.0303347201017
1.48643935288 0.392341475082 -0.0794631550103
1.28776874576 0.599378882001 -0.0407847437738
1.95279822467 0.143794777502 0.385332319011
-1.50198733515 0.169951593008 0.0595644142718
0.297266603106 0.573467959654 0.235071978806
-1.05509291821 -0.0446574945892 -0.0260741315059
-1.08644588331 0.225561410657 -0.17522380068
0.633786959564 -0.557492570436 -0.109819896165
0.128595650956 0.192761535236 0.242698590169
1.69569757067 0.231016364384 0.327398952274
0.602640629935 -0.488193575063 0.184089740827
0.576675074926 -0.0633139672462 0.122812138435
0.541308689357 0.145288944523 0.18460400721
0.895258283635 0.385407403207 -0.0142524912902
0.860190546477 -0.350696515738 -0.111611952196
-0.475166950296 0.432472829212 -0.0944192335235
0.71197102175 0.46075829274 0.11332468088
-0.616106465367 -0.596718035613 0.205717506317
-0.746323016276 0.0668598478908 0.204984552382
1.96185812527 -0.0594525073284 0.119800905451
-1.24704982341 0.257636764313 -0.0117718520318
0.646471833138 -0.31658883146 0.253108987371
-0.147545437069 -0.347687427634 -0.0555871299725
0.287363776572 -0.313082798693 0.190051180415
-1.33622155834 0.386195935678 -0.0651289191158
-1.68206684023 0.232975595756 0.158968346334
-0.585451116265 0.432731443372 0.023564314487
-0.619395389747 0.471958899936 -0.171056106393
-0.705505410113 0.582699417895 -0.0153497536755
0.395946386279 0.338528786909 0.145644669218
-1.42519138391 0.590727712668 -0.1433908615
-1.5734604698 -0.303763056529 0.284047415337
-1.8029672917 0.483452387289 0.0720949854198
-0.134812132398 0.33706002481 0.0845145671511
-0.719986584358 0.448105575485 -0.199955723653
1.40185720033 -0.278020034904 0.134404852577
1.84949898864 0.393151653711 -0.0170747344589
0.737885763718 -0.0570241629162 0.246222541483
0.101956088372 -0.102274775649 0.201174706426
1.52302620762 0.476125001936 0.164446140524
0.210488453457 -0.174372236384 -0.0390580456603
1.29354170552 0.480477723088 0.32427017228
-1.64433944317 0.00449186230165 0.229052386282
0.907486342923 -0.28103736062 0.120499990094
0.20863018165 0.113714142834 0.244639561125
-0.545939745505 0.52937705111 0.0104860545262
0.377763790034 -0.057927640313 0.0949561015632
0.425488371381 -0.20830522678 0.252178143318
0.947276694115 -0.0372252484458 -0.092083144042
1.96738656492 -0.0953543813137 0.127623083402
-0.821235848551 -0.358196097645 0.15688922324
1.0714428787 0.50901392486 0.0702025857841
1.10082248096 0.338928500271 0.202293364403
-0.588020402549 -0.25138605429 -0.110167934563
1.87472077863 -0.584406816643 0.124102679393
-1.75342572069 0.348612618796 -0.0244953893265
1.39692894072 -0.294346612861 -0.134989431765
-0.0974412843013 -0.51840752207 0.0930448045456
1.74633759918 0.0709570186593 0.144865721147
1.10207401955 0.226951419219 0.189711872637
-0.785150166545 -0.274412575523 -0.0669982955644
-0.137923802602 0.0647517318232 0.17164531082
0.72377121711 0.39659112935 -0.0907924226538
1.62505732343 -0.4922175646 -0.00412820004884
0.384878715073 -0.198891114829 0.0625755867804
-0.374177542044 0.510069441593 0.0372468967665
-0.917473222765 -0.00709246764119 0.137607316339
1.99997646895 -0.422874494543 0.15086685199
-0.753831774084 0.184096628665 -0.142315917662
-1.13017286863 -0.0128140711153 0.292159179663
-0.432225542577 0.404826269235 -0.087174624239
-0.314444858209 -0.101202136966 0.138250890309
-0.459351363657 -0.198934535268 -0.190372221667
-1.70234183091 0.290194316147 0.20449944412
-1.26168866686 -0.313915108614 0.203758588248
-0.86469428646 0.278820867904 0.235220094874
0.199378001676 0.270801210225 -0.0125888797308
1.37491547335 -0.236223157946 0.105004993653
-1.79756419449 -0.0495971866781 -0.0266432482671
-0.699561202202 -0.177093287808 -0.12660803624
-1.40457343375 0.308470928949 0.0248307499869
0.549205847586 -0.475698636539 -0.175446394592
0.156259929473 0.124940012755 0.244955082126
0.342988557742 0.323329295364 -0.0699726143749
0.350989818545 0.32456245784 -0.0253673979167
1.32971821125 0.223800799177 0.0202399064856
-0.0703055098807 -0.0268247727374 -0.238491067896
0.832737011609 0.0351258731882 0.20245579641
-1.10647101403 -0.00827190302725 -0.0556832125354
0.114441904172 -0.222077701519 0.0613770570422
1.90459762433 -0.0957034268846 0.275056535915
1.45713258897 -0.508893703933 -0.0169851484584
0.375544896289 -0.442695519206 -0.114744569119
-1.18889927653 0.423240679166 0.134396072471
-0.457651561861 0.511699617999 0.117034359248
1.16672208678 -0.324303189655 -0.136866994583
-0.592812127322 -0.0672815308044 0.148427103634
-1.38476609539 0.205916103643 0.180528252595
0.734714412343 -5.43142538713e-05 0.188174378115
0.561183547969 -0.342318316549 -0.0419633446639
-0.878314562783 -0.130744007375 0.104926284372
0.886521015531 -0.500904814648 0.167987393785
0.937779871793 0.0915004734747 -0.133797933318
-1.37542893868 0.329198691205 0.281713067149
-0.113988721854 0.136510666517 -0.09743864131
-0.339420885598 -0.358358256833 -0.0682303734309
1.27780977683 0.134595033294 0.266802493481
0.390334903274 0.576994054716 -0.121167679078
-0.54982328785 -0.226244420638 0.0706936677176
-0.576033753513 0.197225359986 0.126008604417
-1.31137437602 -0.282923051014 0.0345678447871
1.45559342066 -0.315913328527 0.105916375383
-0.948556846448 0.278982278609 -0.115555074103
-0.168427465207 0.267595311411 -0.160875743195
1.79048009165 0.107834233869 0.322103010501
0.529102084478 -0.588527409123 0.0517506764911
1.40481512113 -0.519134539263 -0.0518524112655
-1.03541110283 0.0876903906439 0.166351210334
-1.50863190924 0.497099008214 0.0392458154835
1.91361908825 0.310686828803 0.154213814208
1.84084886621 0.34734530159 -0.00216607710203
-1.77018537865 0.405022535551 0.173982392951
0.816813543582 -0.19635348122 -0.0860939628462
1.06477746859 0.451694296609 -0.158031347519
1.69331919507 0.152527156889 0.128595535741
1.69610562167 -0.118991710477 0.257723052789
-0.962626740542 -0.0905055448505 0.0453824549941
-0.171573743722 -0.075934979296 0.0775940366902
-1.71686372112 0.257686050728 -0.00414145680147
-0.170006198326 0.341091809199 0.0468887459729
1.2058374333 0.231858216146 -0.166906588399
0.945911742596 0.256539327905 0.295531097752
-0.365785819553 0.285796329487 -0.0928250553776
-0.323697486213 0.352562290036 -0.0628649126848
1.98945820427 -0.561774951704 0.15839940011
0.39062876934 0.404779816138 -0.190167287726
1.96098574747 -0.405663026196 -0.0404926304028
-1.30256113483 0.409758672026 -0.120701093152
-0.225914559991 -0.280800595377 0.156314183075
0.695315337145 0.424453763129 0.0572296705021
0.314071490856 0.401143389709 0.20793624435
0.133355275851 -0.280929391888 0.103969851394
-0.762115784011 0.521381619668 0.185258890209
-1.88564170019 -0.112441108131 0.0493178503154
0.00964464254352 -0.504126840875 0.128966970878
0.664131186658 -0.461307405531 0.1717630888
1.82323681218 -0.391283560857 0.283101902415
-1.82018125944 0.456441321219 0.218698039316
0.701096977367 -0.197492011891 -0.115596765402
1.15701510965 -0.347748899621 -0.119942367982
-0.123741361359 -0.149963257891 0.157643293286
0.941382814248 0.0205657842 0.1575921614
-0.476544983879 0.335211328622 -0.159153856604
-1.03677095242 0.0639339982586 0.162054690058
0.315326665993 -0.120389610028 0.180815715827
-1.32742058194 -0.267814386619 0.0588092517814
-0.708180228997 0.200483044309 0.150754746633
0.0979868999221 -0.584420124063 -0.177987026457
0.19475097717 -0.00756081021394 0.194993333772
0.765968901587 0.243518796823 0.154245546494
0.970593591568 -0.358907532499 -0.205672698863
0.452139943495 -0.102840271819 -0.0433680517452
1.4154747049 0.0641356987203 -0.118869335757
-0.422735110459 0.324759104921 -0.0395323234447
-0.153741118864 -0.113599712316 -0.128036499768
0.797690910819 -0.385254370783 0.148627067523
-0.947643354913 0.246026784546 0.167675540897
1.86442628554 0.0535364346955 0.0763702848836
-0.866974555811 -0.242719000433 0.172876445253
-1.79365758463 -0.455075289345 0.0965423591852
0.97084679712 -0.0924989318412 0.00223923022093
-1.13995240558 -0.154872218485 0.146542477621
1.30001894724 -0.381134930876 0.0254388918042
-0.608088557967 -0.58561374407 0.0299528168347
0.64390310194 0.101609404633 0.21088085769
-1.13364730218 0.288277747082 -0.20115628971
-0.599783456318 0.0705166931371 0.195859586006
0.584878094386 -0.368494541678 -0.0787665495859
0.583241128707 -0.084296106664 -0.0271906755246
0.225991189199 -0.392739031375 0.197340753857
1.11726284482 0.173765328891 -0.00781599820121
-1.10656364295 0.32491500178 0.171147933004
0.751654144389 0.559096756526 0.118865540693
0.864870898937 0.0347561759569 0.200387220919
0.479883442151 0.355092380434 0.14294562093
1.68633847289 -0.371627700349 -0.0869343711816
-0.0235122247752 0.346233783097 0.0831979199703
1.8821940785 -0.390888872281 0.154574345414
-0.396114566153 0.476305067187 0.0957571179767
1.87308340211 -0.567950310165 0.0188801603851
0.930016002041 -0.34411669523 0.0966444682116
1.5029960369 -0.519391463144 -0.0192691617581
-0.980818157709 -0.592718228893 -0.14075899413
-0.258442065846 0.530449628151 -0.0993190676757
-1.80046884109 -0.503395243995 0.324356347121
1.90251170004 0.161909820261 0.130946416227
-1.4962350426 0.260368119756 -0.0384249260001
-0.349739019681 -0.513690503266 0.0945811116668
1.72248212269 -0.126781266175 0.150713055667
1.04506390898 0.271337035194 -0.154646476855
-1.04364325018 0.532437309028 -0.111225953922
0.744574561835 -0.210981495569 -0.142690502869
-0.00983544737341 0.526048316981 -0.200729453888
-0.196518648624 -0.0753241014036 0.0784710183896
-1.67816488841 -0.538036883537 0.144382037656
0.633120624301 -0.44733570954 -0.187957797789
-0.159229187631 -0.123775125545 -0.0413209771207
1.32278223024 0.498454533016 -0.0578990275221
0.0937531373703 -0.293234652974 -0.0176310883064
0.851413391552 0.324925080795 -0.0985099105732
0.766627569653 0.276594077544 0.260243464676
-0.617753881654 0.0401206118477 -0.0974666520254
1.83152686102 0.27497122972 0.358803877793
-1.22528579688 -0.0388660718689 0.0917546059855
-0.824345660212 -0.583776798879 0.102706154355
-0.872132558703 -0.118876641712 -0.180146956904
-0.913177541299 -0.596886148695 -0.0405492741014
0.890230675761 0.569610069664 0.267402107731
-1.38619978537 -0.0800209599424 0.190159810636
-1.02214611951 -0.292816527614 0.148592476455
0.845455551599 -0.526375016209 -0.0312562533355
0.468585512713 0.381357358886 -0.143106938161
0.935015323428 0.422262101828 0.198348674577
-1.91442281827 -0.158112515607 0.292959735227
-1.00995263986 -0.245351241575 0.0605491148363
-0.151122869463 -0.0534564469564 0.158055744106
1.85821035641 0.433634502786 0.365738359782
1.39724252594 0.017783634038 0.0534820610493
-0.451799815312 -0.203504728912 -0.104104879118
0.304896358793 0.041751329932 -0.0749113970152
-1.4255428603 0.0582672223658 -0.0424072237203
0.419024468558 0.491824887101 0.0469139723886
0.366901603507 -0.166264661785 -0.00348995606765
1.07846078136 -0.546885708999 0.0439910665549
-1.45210536142 0.0218125583766 0.181011738011
-0.297180744298 -0.359234637207 0.166318929759
-1.24859572592 -0.405200159935 0.243912830011
-1.67631498862 0.198517965698 0.0787856240575
1.98980847218 0.320213353813 0.0789827642328
1.95333229599 0.458316287776 0.175421324412
-1.19202706652 -0.103653661311 0.10428655158
-1.43433911678 -0.544797872666 -0.0176724051545
1.04101050127 -0.19980159755 -0.0140160439821
0.391299986878 0.181744877404 -0.0963528569092
-1.0591968108 0.150444591609 -0.159678290139
-1.50645693089 -0.147849733033 0.23759642444
1.62994253063 0.41103649021 0.245799295279
-0.842934327427 0.0701990053115 0.0742586614131
0.369763146442 -0.0463110391846 0.102817243073
0.539172855713 -0.185882344339 -0.196286454602
1.05114248612 0.00556767748742 0.21543492179
1.49535455522 -0.502986736471 0.168929713742
0.794762844206 -0.0659852806574 -0.139329762493
-0.14888875676 0.350208881771 -0.0096389807402
-1.94140229288 -0.4130517392 -0.0216622128422
1.20893677619 0.501382409042 0.307538956839
-0.821227371657 -0.0468697621997 -0.177432871812
-1.80279298351 -0.304946762534 0.120086421959
-1.64098265198 0.180919553878 0.238786672124
0.253678652337 -0.122887836168 -0.025992629684
-0.13302961798 -0.266154004389 0.237277037442
0.676300005434 0.299747146923 -0.0672446845823
-1.15590071193 -0.235593636165 0.25237306642
-0.276052066571 0.439319218092 0.152087423118
-1.40500724313 -0.555462801522 -0.0390010414568
1.15241436717 -0.289907593661 -0.0178073403017
0.0428232545932 -0.22818275088 -0.106940255705
-0.351822711317 -0.281265252932 0.132453961501
-0.836244014734 -0.13482753272 -0.132872829074
-0.726488376557 -0.529123882405 -0.188238995908
0.503253211386 -0.429210321255 0.168695356173
1.10049240499 0.277099431487 0.0780901272588
0.541992042505 0.290885641248 0.119833175284
1.7193870914 0.468801545952 0.301691974499
0.905521172648 -0.223168039745 -0.216235055077
0.112001272759 0.554986923262 -0.0916540300949
1.60202014945 0.331924255832 0.338396436024
-0.749170482236 0.515652408488 0.224465120426
1.80412061549 0.379682423448 0.147318813668
-0.703046736706 0.404280853813 0.171526291505
1.25094835279 0.595939226033 0.120454773929
-1.15983461329 -0.0663742420313 0.157203133505
0.355720425391 -0.504273678081 -0.0510720511994
1.04095806697 -0.203326297247 -0.0289513474705
1.54909284175 0.383386175538 0.0905276194787
1.59726755449 0.429688051472 -0.0239725123927
0.752486676368 0.18653113228 -0.0948380342473
-1.82976498155 -0.452405510297 -0.0420611053199
-0.377546068651 0.0149976499184 -0.0146552871498
-0.199672513363 -0.0869477952712 -0.0646141499682
1.57189428151 -0.315055844517 -0.0685495327554
0.132715419487 0.171047692759 -0.0595798799582
1.05698735617 0.0315502621667 -0.188104347388
0.321282581045 -0.267984649411 -0.210563201855
-0.237501037746 0.512459799886 -0.00458690189315
0.117599262293 0.273361479488 0.164977872142
1.18179426191 0.363740126859 -0.077905816582
-0.833820992359 -0.240110443521 -0.202115383294
1.82355716031 -0.0922998231552 -0.0280797806413
0.168714686821 0.0280208964376 0.059551441944
-0.466406646545 0.212783999615 -0.0564161324061
0.317431899786 -0.574901796299 0.192464243549
0.643881078273 -0.140503069289 -0.0741895895621
-1.03856098201 0.262883709272 0.168034163938
-1.7008152411 0.206621528686 0.0597528329939
-0.52435401176 -0.187168820431 0.0194449759069
-0.560785236592 -0.547848838182 -0.150801360012
-0.392257993671 -0.437669025876 0.059899509326
0.0606983732084 -0.225939814249 0.177883430778
1.4274549338 0.311196390261 0.335527770636
-1.13053042805 -0.019335779601 0.248738562153
0.945814336922 -0.388229949422 0.103352236961
-0.284867458768 0.366243347379 -0.177834810603
-0.31794828655 -0.282433364878 0.228738237925
1.4409839119 -0.592657888742 0.0813845324539
0.463806650713 -0.0220812938955 -0.0573315995676
-1.36841205164 -0.565524182747 0.276550025548
1.72434358507 0.450736104227 0.401701820446
-1.96264507368 -0.0405793896248 0.033316430575
-0.0538517191287 -0.516193520948 0.138854944517
0.423328852869 0.268271228959 0.16023207715
-0.673041850284 -0.468808670794 0.0640807370299
-1.69996764242 -0.0355741474779 0.383334257031
1.06098431028 0.515909454678 -0.104586595886
1.25721099893 -0.0524566646636 -0.152924966616
-1.16574080675 0.235459980264 -0.123824968289
0.2587746904 0.577241119406 -0.0944376191808
-0.391097399617 -0.472828618438 0.0306943582749
-0.712773506322 -0.0880916509392 0.0993829241365
1.67917813863 0.0650038214756 0.306060016889
-0.535867662484 -0.465990923487 0.00116048297058
-0.327792445911 -0.184994530467 -0.23867673259
-1.27256465123 0.5795089159 0.0681127046208
1.46734365559 0.209414502455 -0.0937140788209
0.636673335599 0.0415931279936 0.117319607281
-1.26326843654 0.309134013159 -0.169739939151
-1.78412328228 0.417283608673 0.00350154522094
0.483926794196 0.287913039069 -0.215954454851
1.79290576661 -0.256305332569 0.0438469338735
0.45527049862 -0.307392523216 -0.101967282615
-0.655545952506 -0.0697584389988 -0.215113949871
-1.77961350667 -0.0766796365977 -0.0865973075375
-1.27202661201 0.129759242087 0.214959226028
-1.50572817602 0.261717251628 -0.0565699930961
-1.98602018345 -0.129377943826 0.0172910127462
-1.06646393136 0.025508181294 -0.0121039561147
1.88853356093 -0.515486656176 0.00928042692098
1.94850924671 -0.364147774199 0.212978329896
-1.69969789869 0.271238776606 0.298097444906
-0.196637994552 0.112393936437 0.173388192997
0.469415552621 0.00094596790399 0.0384723305699
1.55250125688 -0.165416746333 0.142780877051
-0.175691717464 -0.244366332577 -0.116887917218
0.911036964095 -0.195967990533 0.120975790809
0.572724738088 -0.285937964139 -0.206187571236
-0.769564003967 0.558316048616 0.13991938305
-1.46520457861 0.44452775195 -0.0247256009588
-0.279824757038 0.420081714099 -0.0974392433222
-1.23313048726 -0.517236990988 0.249487893974
-1.99817068596 0.166727363998 -0.0174345629141
-0.869809083806 0.228111237351 -0.058654653094
-0.811922106057 0.187926262793 0.0974349016686
0.608688963714 -0.513596565716 -0.0966374035328
-1.79920744133 -0.38320153856 0.108976123472
0.602088218324 -0.562820795527 0.211598201532
-0.970438645877 -0.428359864757 0.244422007445
-1.80915796827 -0.457232031641 0.102934702865
0.103603726645 0.440050066259 -0.067503705393
-0.181345467393 0.353898233208 -0.0240252694493
0.146208439068 -0.596888006252 -0.0920463058451
1.00472215628 -0.232936916109 -0.147682297158
0.45998695852 -0.246974760608 -0.069673202865
0.359372700308 -0.34652748048 0.15637584803
-1.11958636576 -0.312671856527 0.182110192164
-0.965943292388 0.455196246529 -0.052590496206
0.746870308841 -0.0928151875655 0.24127061397
-1.62583145046 0.0160606027128 0.0490812254024
-0.125091490172 -0.0835408726187 -0.0783473368595
1.73843709761 0.448000054248 0.314610313641
-0.345810086452 -0.0549010076753 -0.0353367445956
-1.8745098747 -0.299225713619 -0.0399158636714
-0.554997965382 -0.42597049351 -0.13939683005
1.76365907577 0.264005080855 0.375946389435
-0.307643089294 0.420842284399 -0.0651851330346
1.10262159721 -0.519386410349 -0.206155172491
1.2450066793 -0.0887535631286 0.0141783506294
1.80345995005 -0.437539042432 -0.0920240418998
-0.179064217867 0.462622406047 0.0172556193278
0.77238099966 -0.329956700166 0.017040646057
-1.26881786041 0.59443548079 0.00585781284906
1.05866109627 0.0956140437658 0.138005449816
-1.55453380742 -0.36041516331 0.17507195038
-1.40880511281 -0.403992162133 0.102127342586
1.9162228281 0.342468759654 0.373066706096
-1.41432372861 0.175445305111 0.00593176842932
1.21561150051 0.110388832343 -0.175046045461
1.3876671936 0.239105414987 0.00624633858636
1.36617342704 0.596067418918 0.30569128795
0.143704945314 -0.099604541075 0.112911117929
0.0988953196853 -0.506587711571 -0.0507643394837
0.602165658098 -0.0389445406217 0.216296016001
1.73416137033 0.272374154972 0.288357525864
-0.367000261681 -0.330641834579 -0.194811163937
1.41155900338 0.578504001001 0.205996308446
0.618915175564 0.432675094652 0.114359709555
-0.439341258145 0.282211086451 -0.239019571565
-0.998523203758 -0.388721004656 0.0230597621424
0.716189380141 0.147834242165 -0.177774953835
-0.991510225275 -0.408523457334 0.182687537513
-1.62516113676 0.12065159337 -0.0282965066074
0.433626692296 0.310220857769 -0.226620939584
0.303288830185 -0.396947545714 0.0583625433498
0.949472392351 -0.278008530865 -0.0410958310039
1.5511822469 0.12290649935 0.132226674244
-1.86708013877 0.201487487777 -0.0527188113565
-0.85612295701 -0.516321446188 0.0693876678291
-1.73531533876 -0.0137759549949 0.0903940369085
-0.459651717661 0.520971278422 0.140007608157
-1.37663026669 -0.514474601104 0.163815183286
-1.13937517356 -0.377652638178 -0.114236400573
-0.640976886545 -0.249358202295 0.0147420982539
0.520162120904 -0.0549003651857 0.0456263805906
-0.0396997765921 -0.133243788739 0.151491119006
1.30745771 -0.161806176774 0.118213701738
0.993533916343 0.254060682946 0.273442042438
0.979596563408 0.430744906935 0.141286356279
0.149418561106 0.430867559853 -0.0723812355163
1.60637677195 -0.277279113525 -0.0552007293831
1.32279124145 -0.403415115683 0.0842451206012
-0.129482336789 0.0220881713583 0.208395025165
-0.091771364995 0.182432365369 0.00405549861489
-1.80595239746 -0.361279157432 0.0850275399135
-1.16547328488 -0.46625604681 0.137084998382
0.997064827876 -0.320493577707 0.132990452009
-0.502798906691 0.588126984913 0.0697352498106
-1.45363320735 0.216736794446 0.295546300934
1.73552593291 -0.133907401686 0.0360573596829
-0.243988077278 -0.58809385292 0.15595840694
0.578638089296 -0.0203387734343 0.069742392561
1.78777872098 -0.326156861024 -0.128645509787
-0.463881445664 -0.510739040114 0.0447870279658
0.654836677961 -0.594924199338 -0.20965100898
-1.46202901767 0.418284491084 0.0493620892586
1.56343008016 -0.497675130902 0.111511600571
1.67667440984 0.367933659616 0.279941786928
-1.04058734792 -0.138513154093 0.277315495363
-0.890869520178 -0.393422419427 0.124280459377
-0.260469444022 0.326865542131 0.103017257578
0.12004633804 -0.114831549557 0.104723313
1.10719288597 0.4400390543 0.0624480657002
-0.44736063097 -0.0208915617907 -0.129552803311
0.799837405393 -0.0756012426049 0.27099286086
-0.263415580075 -0.236059781792 0.181489512051
-0.834027236227 -0.105585278364 -0.0582568179259
-1.65423951411 -0.452259115329 -0.0255726988615
1.82906880741 0.218278806679 0.384018228404
0.483372086251 0.267268351707 -0.20026042925
1.6035692665 0.369350853123 0.141508834139
-1.04586407591 0.573139063059 0.0636059826914
1.88911654856 -0.329722408883 0.101605126654
-1.54121324128 -0.459438463966 0.150112081941
-0.750377139661 0.0862806792635 -0.00759710694394
0.0153263142216 0.563753462067 0.121080079723
-0.816807104431 0.300870538579 0.158475036475
1.12716824972 0.298736198058 0.0351601424109
1.70394821523 -0.0166974289895 -0.115893211199
1.72356528028 -0.0653506306183 0.0435692702445
1.68663746962 0.204375670841 0.156803150964
-0.371329842192 0.156069832833 0.0913003878604
0.646316676141 -0.0441930037376 0.244715480108
1.2621543463 -0.142333079046 -0.056520678614
-0.325170104118 0.193440822206 0.0161345869043
-0.840320016358 -0.0420817710075 0.270132019289
-0.96750796092 0.58793092401 -0.117783068108
0.364361618575 -0.306901721576 0.219732716098
-1.55134578401 0.151396751449 -0.140537288841
0.769779738344 0.567542587131 0.298155497103
-0.747602524982 -0.213149520772 0.191565997366
-0.96951044111 0.00240801597186 -0.139375477008
1.29781247371 -0.224011760228 -0.0508902533041
-0.762377423466 -0.46300969176 0.295660918939
1.84714403273 -0.53208912054 0.0907910830279
0.894461783145 0.0227405911154 -0.0152636068449
-0.0140991631305 0.165956981527 0.107493578684
0.63043909343 -0.11239036895 -0.0362362952855
0.157194705192 -0.32064586792 0.165882071835
0.806854321978 0.441213703918 0.257819145533
-1.50092483047 -0.260896755038 -0.116436279791
-1.89220903926 0.327363860171 0.275390805082
1.81438186909 -0.404667244762 0.252149113884
0.330171396414 0.205996361282 -0.212071048
0.310158690388 -0.213693360225 0.196217132761
-1.65906760271 0.295744182604 0.133282526283
1.66041885433 0.33407000865 0.0268237545088
1.99580354319 0.167629429049 0.108231042413
1.55823528542 -0.340609217555 0.144912926606
-0.522921883058 0.192489027652 0.0983081258923
1.35857081502 -0.574009101813 0.261115816501
0.747944722435 -0.150316274672 0.244315370274
-1.8372514148 -0.472321797011 0.316460823004
0.723267759698 -0.428501574194 0.207435102857
1.04004692566 -0.591784499223 -0.23053879495
-0.842209229769 0.421827838977 -0.0836661825361
-1.29956818549 -0.318513183745 0.311450354635
-0.567161066476 0.304907059823 0.017024087509
0.610605915697 -0.114546888841 0.109536383656
-0.702131536231 0.250700740173 -0.00770322143545
1.81866300954 -0.474134622174 -0.127437804514
0.61216613393 0.436900649493 0.237780217055
-0.728518892962 0.168024236486 0.0806507179621
1.7707302405 0.23359016337 0.126566839892
1.53038266002 -0.412745515101 0.160923240612
-0.996245430667 -0.418837579128 0.145715266478
-0.373309152416 -0.393285739973 0.0256307751279
1.73099364507 0.169842082407 0.0217136060646
-1.78925178159 -0.0514441654303 0.0738827628052
1.16363614435 -0.298703761863 0.225155761858
1.88748250871 0.274723975227 0.252553144037
0.703851306474 -0.55800563084 -0.23649942944
1.81133508282 -0.139967702615 0.0838577993925
1.99329777498 0.359799605769 0.451386180115
1.12299954308 0.468992104648 0.17313537174
1.22623580935 0.036267022531 0.282676906006
-0.238048967667 0.174674568624 -0.213353404189
-1.34571310069 0.557198913556 0.133011229481
-0.680321592826 0.535414673144 -0.0112572326166
-0.749760986559 0.336023239937 0.0812891789359
-0.428984784263 -0.0188007337001 0.0509651421709
0.942419989638 -0.438360580765 0.0685540289062
-0.32888623884 0.26934580345 0.177992380143
1.94332322327 0.130877344546 0.235964739804
1.32205956861 0.45173420834 0.0211136464441
-0.486034157703 -0.0808196182236 0.0430737275374
-1.55952139437 -0.388087055474 0.142828434355
0.33620388973 0.153828391109 -0.138679087505
1.49225893201 0.457130005722 0.0356506423243
-1.48461303839 -0.490026274766 0.214843452992
-1.76201583246 -0.523233759004 0.0423945147883
1.96875215206 -0.492286288901 -0.105410236644
-1.87646266468 -0.580505015082 0.209589888451
1.76094813099 -0.522105771207 -0.0656159231352
0.166983094497 0.200182516796 -0.162386740929
1.47064088241 -0.407842584224 -0.0273843031422
0.533359149347 -0.293908375638 -0.109126176621
0.772427934631 0.528710726365 0.0393444487438
-0.237363740078 -0.323009735822 -0.126886593089
0.909826377322 0.357734499637 -0.0357378592385
-1.53207337319 0.252642865369 -0.164595369805
0.642669635281 -0.331099740272 -0.0194180454383
-0.880575811769 0.394003853597 0.172792333633
1.36047794506 0.247939169745 0.0592949579912
-1.84305498526 -0.107217240477 0.131480652259
-1.80805584968 -0.455267646028 0.446030696495
0.353714650812 0.111762188751 0.152032164884
-1.44113006433 -0.325982663096 -0.0440387586513
-1.56302789066 -0.571295215959 0.0471242033766
0.524830617511 -0.296803723433 -0.217977322603
-0.53744360434 0.405470748675 -0.0937422943822
1.33963230751 -0.391613519792 -0.0764373073612
1.86193244031 -0.19703533244 0.0828707018338
-0.389299100593 0.358029683438 0.0555358101129
0.630490856458 0.0989134585265 -0.0169073176301
-1.77083461442 -0.170095687562 0.0558772054623
0.515553522174 -0.363482839878 0.205325334506
-1.8273261444 -0.194594657319 0.334305929652
-1.81021883601 -0.00515275268089 0.321636252665
0.0552393486343 0.246826240168 -0.205416337695
-0.0791431232809 -0.0344257475503 -0.148588796463
-1.69172259542 0.264564145943 0.19403506221
0.374876669607 0.034838362041 0.15070860783
-0.221785131816 0.387624134557 -0.0448497873105
1.36811854018 0.0281682177292 0.276985779151
-1.51064593176 -0.276951359928 0.150529207573
-1.76499818475 0.567303548337 0.0286633430215
1.38923964068 0.00127119584931 -0.0906156247791
-0.806667100903 0.0141520750059 -0.109351702278
0.445425392066 0.436672239695 -0.110737599066
-1.2177859064 0.37594856806 0.207336270762
0.297592942375 0.193634624917 0.183877469164
-0.699338407737 -0.474695683628 0.0680049768932
0.676528975567 -0.190632373875 -0.136953705687
-1.36741589847 -0.122646031421 -0.102807789331
0.349683147891 0.437148842838 -0.174982981193
-1.18452716912 0.500317120441 -0.12179047828
-0.0807990030432 0.0777180834643 -0.241670954297
1.1494733803 -0.45971495849 -0.175850023888
-0.162571555646 0.37933507573 -0.0852692923428
0.149574041616 0.475598905651 0.129595253118
-1.61140832972 0.393426777406 -0.00803239404264
-0.620602740334 -0.275035681559 -0.217971202047
-0.697217707908 -0.295813109837 -0.0694828384119
-0.356319681602 -0.366493623651 0.103522837678
0.324559889385 0.523273165293 -0.166073618747
0.860497692081 0.302024889818 -0.118083907963
1.49859060063 -0.518794913715 0.151272919241
-1.59286888217 -0.376886793863 0.0998101486713
1.65615647055 -0.425872820147 0.193853431714
-1.1150290229 0.401513114709 -0.124459037706
-0.501922968422 -0.0592351333592 0.0418169706184
1.93563166473 0.33139795959 0.268730983635
1.79169271573 -0.240599489168 0.187067878927
0.0643466821506 -0.297779684028 -0.0365765179511
-1.13937238298 -0.321318688184 0.172108231147
-1.43320548158 -0.0733683626577 0.0244552995516
0.567230410646 0.114729074419 0.148069638472
0.870948710963 -0.415848537821 0.000628094641793
-0.693018676685 -0.212871581759 -0.0675913232945
-1.48268790549 0.419494018376 -0.100633745115
-1.97992285772 -0.510658507618 0.229835665539
-0.990339292335 -0.327490225985 -0.163090482181
-0.266217607809 -0.538644595191 0.206251489423
-1.91616716701 0.224570286114 0.208557486753
-0.802435131565 -0.375747726742 0.176685120388
0.414799910798 0.154602866665 -0.139048888177
-0.578157081495 0.285196495218 -0.0461679083067
-0.142467684688 -0.246189409236 0.0597432003418
-0.409376444922 -0.493768004621 0.0653039000085
0.549284148761 0.508262256998 0.211936628457
1.14390386167 -0.441922981939 -0.0352779727688
1.10928822639 -0.214843795343 -0.169226134621
1.68870218371 0.397296086906 0.283164299052
1.54589561789 0.249544951992 0.124999284042
1.16469900651 -0.252943307714 -0.105201163958
-1.71200631808 0.201278542859 -0.0855312113353
1.09690915831 0.481139614355 0.0677229295862
-0.622649928253 -0.554510413527 -0.203991190602
-0.283562648168 -0.265940075798 -0.00604099321898
0.112268814242 -0.461291491565 0.111835640462
1.75895765001 0.0875667085811 -0.0654941209383
1.64001971305 -0.498515142183 -0.103321057216
0.682614125963 -0.553053659676 -0.237947891348
-1.19339183462 -0.17578957635 -0.0184083084862
1.73116558549 -0.399917626971 0.300425196616
-0.229828160486 -0.43429803369 -0.163391166123
-1.08098570607 0.215275209662 -0.0689709564674
-0.343031981551 0.22624912075 0.0912774497557
1.0666888377 0.231059963665 0.152044993825
-1.89849829359 0.436430362279 -0.11737152544
1.55663683651 -0.525774984489 0.138906222132
-0.969819977609 0.555446709858 -0.0453573549837
-1.18949682694 -0.170676039376 -0.0599707589987
0.344425813528 -0.556781367834 0.0557721825544
0.753520990155 -0.584113770706 0.0911375156437
-1.10168257437 0.371326570319 0.01160210825
-0.604726335026 0.405417903486 0.00473656661242
1.92359608178 0.420195954471 0.418478959598
1.47262329407 0.297229450877 0.339523729663
1.48049048819 -0.358331211421 0.0910517993531
0.75441435232 -0.440665175834 0.230625587925
-0.141465293361 0.461419862964 -0.111748749254
0.735399005794 0.375639332351 -0.125277054972
0.886671458628 -0.493250077632 -0.0163840381166
-0.215158641046 -0.590437483022 -0.118353751566
1.8477363583 -0.534727913259 -0.114571473614
0.393965832873 -0.138427561471 0.185283471553
0.187376227494 0.554596999864 0.185886874729
0.0278704463592 -0.0919314319379 0.119229321586
-1.76353404994 0.418172925509 0.060829638912
1.12876344575 0.0242579802909 -0.0227245905736
0.3532209906 0.060971101931 -0.0253069681967
0.0580804783596 0.419412482319 0.116585641142
-1.22631597462 -0.434681812496 0.266779652824
-1.97367138312 -0.307752683207 0.0710534443228
0.610314834811 -0.0815614899213 -0.049977440073
0.624466242681 -0.360153253449 0.158750016707
-0.959430332739 0.0942846237976 -0.0357101469149
1.36103885234 0.577371639671 0.0794243601088
-1.19133623259 0.369982738431 0.0201412633484
-1.00183753808 -0.00152763779782 -0.169174498808
-0.320587245014 0.185527857107 0.094617184133
0.259879380233 0.587490032934 0.145441769558
-1.93350030163 0.310405271879 0.361310839826
1.89307670243 -0.23533027967 0.0743219414052
1.47320363441 0.44805719905 0.368293109396
-0.18500721115 -0.506014678472 -0.0486900410044
-1.83802995191 -0.435045989601 -0.0333513442007
1.79151749178 -0.0522240999137 0.157110841798
0.89382399432 0.174819083747 -0.0490573119819
-0.298729316796 -0.55021900377 0.157017648847
0.504344394562 0.0844775555761 0.0627535174809
-0.149130196608 -0.126825525397 0.195961009884
-0.672621361487 0.376128563414 0.126483818166
-1.1133266944 0.541045809151 0.239573033252
-1.95318247298 -0.320974861152 0.209735223189
1.33894702401 0.552553754143 0.0378309931533
-1.74632893214 -0.572109487097 0.113306432924
-0.580217214251 -0.38514465642 0.148023736815
-0.469201717043 0.276338391826 0.169241804339
1.84193920252 -0.148541063835 0.176172074924
0.352863092317 -0.519087040575 -0.164258803166
0.260124504153 -0.208940517374 0.00926355729721
-0.159024496134 0.537536206099 -0.0270214707573
1.1838813801 -0.0205176449125 -0.152888515056
1.58091612876 0.314795372564 0.222767147012
-0.959228174429 0.26190983807 0.224482081014
-1.28739794018 0.519794168543 0.247754713954
-1.2417408279 0.180214588578 0.207381055411
0.394000573407 -0.468257617368 0.0678521799712
-0.838382967724 0.472527187883 -0.00799323489336
-0.526162558191 0.209012280789 0.0640786510524
1.80635267451 0.451012409266 0.156863478976
-1.19649279925 -0.597079475679 0.191868255077
-1.92079998597 -0.497185329106 0.101250053569
-0.531018624568 0.156025759574 0.00338389741757
0.461871559283 -0.300670401253 0.116933815333
-1.58015443815 -0.37204915866 0.27442072109
0.868878603399 0.0196733868056 -0.161777572926
1.8420133724 -0.491652090898 0.299627888938
1.89621632396 0.395214640526 0.126977859117
-0.518951827344 -0.248309388243 0.193583616663
-1.69346210834 -0.591913734164 0.0514053825071
-1.14459679465 -0.505577429445 0.161167100157
1.36017102409 -0.522277956217 0.283777710034
-1.2538070835 -0.258935969199 -0.0548442219231
1.69203016494 0.164112471274 0.129866215713
-0.276682359646 -0.154788446894 -0.0653895269087
-0.787191300997 0.319792998398 0.0875406620424
-0.0603467920711 0.39132428035 -0.190998079587
1.56941039056 0.358170492346 0.366377703612
0.934359871629 -0.506522652194 0.190586673534
1.46406855752 0.510983934597 0.182993835934
0.645973108593 -0.27153806371 0.231326075151
-0.171677871073 -0.150024418535 0.0407051591135
-0.931893089148 -0.503911265506 0.0731675235613
0.918293372844 -0.523081829232 -0.213063643907
1.5225251629 -0.362542735385 0.0796482510977
-0.278254279571 0.076157748027 0.078368277069
1.84359386758 0.0295940249416 0.312537649456
-0.466197937648 -0.531510021198 0.253423333144
1.36740128705 0.565536015987 -0.0259626270107
0.589738948651 0.399451810694 -0.106671417823
0.0371596145372 0.207566341314 -0.189319938759
1.04507092767 0.379671434642 0.294625850389
0.0950763362901 -0.372500112732 -0.0903146245325
1.71055889897 -0.340707779151 -0.0487999022839
-0.852053358244 0.084846947873 0.223892302326
1.7328941941 0.385693363062 0.312408264301
-0.963305014458 0.0744680219048 0.215277578885
-0.514438735367 0.28958757546 -0.134816876211
1.68418363375 -0.207863681802 0.177734761083
-0.681930451568 -0.0305731997186 0.128534004713
-0.320935573166 0.0897882141528 -0.00119379164562
0.423578760769 0.54981195604 0.186847895718
-0.279611030246 0.197988399941 -0.215574490795
1.71162195194 0.076418235343 0.245023592343
1.2093107697 0.136235479396 0.00129814264159
-1.03940948328 -0.267432012579 -0.0254735113254
1.49415063247 -0.170421755589 0.230824936635
1.48666110093 -0.198313842573 0.189100179067
1.01346178984 -0.284192144783 -0.0357836549486
1.75156962133 0.273964169162 0.365305400097
-1.58501755347 -0.158586106255 0.0947178740399
1.88060637684 0.0180558176141 0.300786082841
1.85527938667 0.258576047709 0.0721075779756
1.82463583611 0.329036023349 0.330813188426
0.857281270525 -0.41768594227 -0.058756053229
-0.240074310019 0.129296632423 0.144406447475
0.771301287605 -0.481252025951 -0.208543133299
1.26542386603 0.0896660533836 0.278232955394
0.77985461211 0.535074053949 -0.0635943264896
0.423589918165 -0.158264810625 -0.0281691642651
0.422008612639 -0.484191496301 -0.17242203007
-0.473135247523 -0.0230832457427 -0.138293531932
-1.36571907977 -0.58435523116 0.289476598618
0.909786593672 -0.217799456809 -0.205508683456
0.322954150973 0.0257171032923 -0.230858233948
1.14075241916 -0.0472675445162 0.259889143522
-0.140145726723 -0.0266596754912 0.0476055923192
-0.358388828516 -0.181324631397 -0.0224924245906
-1.57856284293 0.122373157698 0.3260534511
-1.86638792974 -0.347661620247 0.411576875579
1.29764282177 -0.420258100872 -0.118900264127
1.87481540274 0.301079860255 0.340674799797
1.55982211816 0.0225826634351 0.244758591757
1.0928387778 0.47001707616 0.233281712311
1.96514851903 0.32111781425 0.297626622958
-1.62198979233 -0.352303246769 0.383966099259
0.394560147583 -0.479209362958 -0.203917982234
-0.481867396206 -0.356774186556 -0.0782425464479
0.44514632419 0.35714578595 -0.0557526747395
0.376354898214 -0.278826063458 0.165174885743
-0.319949875452 0.179144494063 0.205579922103
1.49898684222 -0.177105635854 0.207473820372
1.33926051214 -0.0286435483294 0.179341756656
-0.710768808431 -0.235266120196 0.181404596143
-0.26087282661 -0.474629830307 0.133544252546
1.09628241243 -0.290975927666 -0.0434910219608
-1.09767330768 0.349705773826 -0.0434162543358
0.603233666961 0.0623475455627 -0.0832696865012
0.16593571303 0.538338170975 0.0869491291454
0.252467055198 -0.0776534769481 -0.149138229978
0.778791536448 -0.0993665698846 0.199030523232
-1.24293274474 -0.278638430861 0.336963499149
-0.284642548947 0.0595151853791 0.0315830207823
0.036443836974 0.551077054508 -0.0414098759238
1.87629201815 0.496065398577 0.382317171147
0.396703631144 0.153314829421 0.0239740711872
1.30253870661 0.206364542949 0.0429930425723
0.241815676143 0.0765166382824 -0.218230959519
-0.222378028498 0.331947561606 -0.0872442529907
-1.37722981917 0.354025130258 0.127971435042
0.565219020781 -0.591567362428 0.127140786124
-0.983314251381 -0.25374112006 0.239913480939
1.08049626704 0.339292104111 0.199112350286
1.54235822848 0.537184101465 0.359623359464
-0.901121837007 0.581366965978 0.174197001733
-1.85425383598 -0.301586020363 0.353268145614
-1.14844371001 0.535966892185 0.208996696641
1.8456313457 0.544941947787 0.124510840297
1.25463269758 -0.15226222908 0.220977590005
1.58244333551 -0.0656789294461 0.251548071123
-1.55316803782 -0.0358642962815 -0.0170761610763
-0.0550629408604 -0.473168994184 -0.0786200861471
-1.86362919182 0.0287155679647 0.312785101253
-0.253391109936 0.223661423223 0.0201804870714
1.74773255377 -0.593733805965 0.17713270182
-1.96270232389 0.446831494281 -0.0981194295268
1.85516453805 -0.46906074345 0.239577003421
-1.09855170247 -0.0175064576374 -0.00502924589294
1.22941062281 0.303509307102 0.0995952130605
0.537063153358 0.348454732065 0.0873326223171
0.604980523046 -0.44936394101 0.0787697232864
-1.30904561359 -0.293222102706 0.272143883971
-1.15096886151 -0.438059331251 0.312869914197
1.91854857808 0.0888982389503 0.255583256755
1.41209736239 0.138773718181 0.222448979752
-0.705739257351 0.193498377147 0.132666101935
1.13092301805 0.410504846496 0.283828908036
-0.370445709462 0.186003254134 -0.0899853060901
-1.59743696219 -0.0204278586749 0.247963854886
1.69430087435 -0.117377706042 -0.0661229785068
-1.87733455431 -0.489642073008 0.172410766484
-1.26950005583 0.574545089395 -0.168438152827
0.0665627835965 0.0805511292542 0.00112557295763
1.18902412333 -0.449231499725 -0.114319441483
-1.73617026615 0.216643988735 0.205476528279
-1.20239379406 0.264891776791 0.0764298607922
0.20416603293 0.204919062311 -0.0405540842105
1.77818387212 -0.30216765513 0.250316504043
-1.49833263572 0.0245995507019 0.177264558732
1.59485459572 -0.0561806808652 0.257861510446
0.336095570598 -0.131937056277 -0.181400756917
-1.49095219234 0.0101611834687 0.242304171384
0.0545950313754 -0.323315640729 -0.00321291581314
-0.646735363063 0.273506809928 -0.236096948104
-1.54750317725 0.504106702606 -0.0668121438446
1.29482002669 0.196025075007 -0.157834627711
-0.00420100096052 -0.586767625047 -0.00184360288526
-1.54625028216 0.350405509661 -0.0336943470738
-1.47712907163 -0.574148648542 0.150293476789
-1.35033316938 -0.464656422099 0.187842565254
0.661655630332 -0.370623710395 -0.069050155759
1.54866902612 -0.200863199142 0.224529336602
-1.15644075745 0.231174666804 -0.111992566688
0.264742028838 -0.516670508183 -0.183024045489
1.2426099765 -0.527905580249 0.0486962228402
1.38545088205 0.510033011599 -0.0331699410106
1.92514721292 0.531288330785 0.0848766175552
-1.86529440746 0.51931903392 0.32553747181
1.69790195124 0.48207163883 -0.0616960286714
-1.03289620434 0.229068184132 0.0864711520095
1.59446071483 -0.32855955005 -0.01623048792
1.50753333771 0.0398834826559 0.136295013543
1.47078613645 0.0848712205617 0.177021961142
'@

Write-BundleFile 'samples/sample_B_changed.pcd' @'
# .PCD v0.7
VERSION 0.7
FIELDS x y z
SIZE 8 8 8
TYPE F F F
COUNT 1 1 1
WIDTH 1365
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 1365
DATA ascii
3.04301343359 -2.65368101696 0.588630754305
4.14724727541 -1.72892358659 0.691383769735
3.71769253201 -1.95828833842 0.726718013071
3.90080775258 -0.954795519739 1.1234395011
1.8914577697 -2.78923018855 0.422181513255
3.63082359965 -1.08862872217 1.27958436006
4.03352598265 -0.913807086987 0.999443244223
2.40372730934 -2.60469345629 0.495688058581
4.4590246947 -1.54284123981 0.932974838745
2.89846594186 -1.55754852 0.60973628422
2.54419367947 -2.85565894669 0.492909865262
4.11049651164 -0.814443170144 1.30458890696
3.18478571155 -1.29235736598 0.73914841434
3.04874455729 -1.8848377267 0.450540770372
1.57272083158 -2.9460071879 0.660213460678
3.75761066297 -1.04618710471 1.09175448092
1.84217380057 -3.15814619878 0.415011837071
4.36147282458 -1.42503307967 1.21058555763
2.85709808997 -1.45085509102 1.02681128661
3.41933853629 -1.0298034793 1.09416884855
3.86141303686 -1.7501656424 0.82656034232
1.71003868541 -2.38033565083 0.852639522023
2.65021197106 -1.65820418402 0.665454772328
3.92620106554 -0.611223833549 1.23712668734
4.72499984762 -0.71553393233 1.46276563238
4.25422496961 -0.840665532526 1.51520223503
2.19325788588 -2.74188872606 0.407309432988
4.15574619536 -1.6622933043 0.969251182267
3.35606476018 -2.03920331588 0.525881497063
2.15795565525 -3.29617474844 0.344421049978
2.76233966075 -1.60365807476 0.907697226917
2.45977420318 -2.7752567117 0.539055515492
3.87389166246 -1.15560425984 1.09541396799
2.16482603983 -2.02034369405 0.639710454661
2.87684216384 -2.3273030992 0.345627283193
2.84910009401 -2.46734046629 0.35216405313
1.99076190616 -2.53356439013 0.760368641852
4.32719929185 -1.05734943145 1.31005962286
3.71560421342 -0.76141937086 1.16435405198
4.31136193568 -1.01733163768 1.41749817314
4.86963192529 -1.21473723754 1.1952411885
4.61094878329 -1.04285901959 1.35314388049
1.57438211268 -2.74211325102 0.324186015881
4.56713449987 -0.800606971217 1.63905864868
1.8351638331 -3.30139153304 0.426106132971
2.60903908494 -1.71547536191 0.551412825577
4.64513338257 -0.899320042441 1.26597336046
1.64195760221 -2.62524165348 0.374190272176
3.51074581584 -1.37535694942 0.76202288164
1.56465122064 -2.49870422608 0.394968494599
3.47631682918 -1.5618469576 1.06501507695
4.02255580432 -0.835034173632 1.13868756708
2.96633946198 -2.69809249706 0.517121368835
2.87222558296 -1.52244110055 1.16068658574
2.16484986227 -2.53161096701 0.494512619867
3.00770218387 -1.40711692684 0.980539773771
4.06346316108 -1.78742429546 0.693561430816
3.89890569883 -1.05751345504 1.19077216414
3.94075915468 -0.611556532311 1.3001495239
1.82071594033 -2.57426390693 0.622174212055
3.83175024436 -1.53643759035 0.648428676555
2.40466758937 -2.68770942345 0.588204804926
3.51827539618 -1.25478122289 0.817923482024
2.8044482868 -1.41808015475 0.803324864584
2.98932239326 -1.54560334769 0.835867767517
3.90832248093 -1.5107247147 0.94318407219
3.31941159039 -2.0775520133 0.666055072327
2.93723900599 -1.50579326923 0.79337725632
4.42090955797 -1.04526529081 1.34872692583
1.98259693647 -2.46542207932 0.464391738866
3.91193038559 -1.08075763426 1.10119767841
4.73659249302 -0.953300127196 1.416637889
2.77182025378 -2.55380883099 0.395589247921
4.40417143118 -1.64089647545 1.023441793
1.87695015949 -2.94822364584 0.338414360162
1.73864442436 -2.12200256218 0.476882884622
1.55512049687 -2.33856699662 0.521291569651
4.74284417923 -1.09480964136 1.34604375303
2.1985666662 -2.22475576423 0.377074987802
4.04962364704 -0.790074896062 1.14060464646
3.71859246674 -0.914660348765 0.945888826629
2.92880198248 -2.06872693183 0.430131781807
3.87776661583 -1.79688492828 0.80142293903
2.25796403572 -2.83136512307 0.584270725467
1.71395789854 -2.52546386299 0.561549310384
2.23409310187 -3.05895711254 0.525360017284
2.59779133528 -2.56216418939 0.571992948377
2.93974674445 -1.38242434324 0.864914849022
4.56643011904 -1.12339846496 1.28632148989
2.02331988175 -2.30735161826 0.338545462937
2.4356190826 -2.3132196388 0.582728184806
3.65092712991 -1.93388801337 0.719963650472
2.76541655696 -2.26708312245 0.782213894927
3.8558024082 -1.28297160729 1.03645216798
3.89460902267 -0.93739748798 1.10172786668
4.01019040084 -0.76790617175 1.42829726743
3.46862276974 -1.00316980348 1.00507380899
2.65857912928 -1.97408154723 0.682364145241
4.45032971373 -0.956050593119 1.24434381041
3.02088894583 -1.63272727924 0.591368363685
2.36759545204 -2.43516344431 0.540549140518
2.87684712702 -2.48196518821 0.536913551134
3.76770835276 -1.6234051141 0.679390263591
1.98815766697 -2.40230895428 0.382255211635
3.50559740726 -1.29244241565 1.27396513256
4.71975359451 -1.13520931839 1.31066568946
3.45086959963 -0.923058566843 1.09325266759
3.53913773034 -1.24701132219 0.85124898356
2.74502303554 -1.78095946222 0.505070692889
3.13595526031 -2.22485548818 0.314100729372
4.30301699091 -1.26914829872 1.31547260384
2.42658164088 -2.93876096751 0.343833716903
1.97583016305 -3.38786972012 0.366809884216
2.25967041201 -2.36251045074 0.692176607278
4.01897704397 -1.48157656753 0.632351383984
3.70505314162 -1.26172124158 0.762694498574
3.17938699295 -1.50702529034 1.01108432126
2.56307047676 -2.72722686474 0.350604921243
4.35297320424 -1.14138313572 1.25458537115
4.05235785224 -1.37094615851 1.16627412455
3.09951675081 -1.38749494945 0.950158513078
2.69899648442 -2.7115762549 0.537343999905
1.77737546927 -2.53904476472 0.628798463857
1.66637950051 -2.38327034926 0.382871114883
3.80413926122 -0.780645900262 1.1185962195
3.48735576116 -2.02217244393 0.387124663177
3.03486477599 -2.50344423731 0.32121161677
2.18863088615 -2.32074648678 0.760363766527
1.51989634 -2.47384904485 0.64829217397
4.42601405929 -0.43620602154 1.70503614857
2.20083292185 -2.50369356055 0.761890707711
4.50240148551 -1.42633181151 0.967066101521
4.41210571108 -0.679996456497 1.24165663477
3.73360630419 -1.12390355473 1.11694142754
3.3128326102 -2.26862795068 0.494395348616
3.87227435764 -1.2680308713 0.750714131514
2.55831797967 -1.9704468333 0.867595599569
3.22578939004 -1.36473088717 0.919982447912
4.58588220851 -1.13736465419 1.2548908541
3.56658596329 -1.04071530349 1.1077976066
2.93583776653 -2.43590841414 0.544393393966
2.16571840074 -2.63836864517 0.529175369791
3.60623498739 -2.18571532269 0.634978324261
4.54081270293 -1.32020376193 0.978259648702
1.46388122734 -2.34588904904 0.482287578909
4.65033963353 -1.16557204774 1.27104691692
4.29901905456 -1.75805845267 0.980224586824
3.88194792456 -0.936708324779 1.11791832586
2.41623679654 -2.71035688831 0.303545745124
4.72177824816 -1.28281694627 1.07301092706
3.97635053087 -1.75645947132 0.95517709871
4.03448400385 -0.568068158707 1.32469136828
2.76077937017 -1.55964556622 1.13253435833
4.34211667098 -1.22024103258 1.1225465249
2.752482275 -2.24163728823 0.669091938242
2.90679951386 -2.40876938752 0.771713547869
4.62702793154 -0.589631093204 1.44946245853
2.00165612089 -2.69995090291 0.366448237785
1.86930746958 -2.36389296523 0.7734536832
3.64192966147 -0.947477684909 0.963590276683
2.90314940006 -2.46840397756 0.577099369577
3.07219542783 -2.33930135088 0.382237723355
1.69535615099 -2.34685757697 0.626739085907
2.82738238985 -1.95232405689 0.843537682356
4.61243945898 -1.27485815813 0.758279858959
1.88924166009 -3.30416657732 0.50102735575
3.95604510599 -1.3969903695 1.23236513505
4.67251876984 -0.759379369593 1.6421499776
2.23921265758 -2.24955991913 0.435950998866
3.51987464527 -1.84195944333 0.982861499897
3.91719263989 -0.768326374841 1.33351798561
4.55202957094 -1.37094720341 0.906542978039
1.97634245376 -3.28351660286 0.471922842204
3.70362113476 -1.42067101449 0.666882975109
3.30273159059 -1.61442782331 0.715227895414
3.1861347059 -2.55530546858 0.622148409799
3.08093809213 -1.64656234554 1.03750095007
4.39657292355 -1.2904309216 0.785648157263
3.94190395407 -0.994609173831 1.01022823569
4.27493403256 -0.572613578863 1.45813204307
3.77189210661 -1.55463086154 0.59383247254
4.46456431984 -0.441040981735 1.50698386115
3.44526493905 -1.62026646267 0.969685817214
4.19627566013 -0.999237168191 1.14144237002
3.05554180993 -2.46241709419 0.488267824401
3.84545919712 -1.88441381222 0.665912648957
1.77821023313 -2.56541883917 0.569261706348
4.92835700197 -1.09954847644 1.29221022577
4.62385212072 -0.631517798309 1.24805142002
3.45052186401 -1.95229216195 0.881426196597
4.19975705063 -1.5669862911 0.789560350208
2.07881277936 -2.67787090285 0.305999988693
4.10144221249 -1.64340639447 0.848819054354
3.96429550663 -1.31402195005 1.12741049819
3.35716745684 -2.25903622947 0.789023151735
4.56025434521 -1.24836723703 1.13865231582
2.34173924408 -2.87215143142 0.530458956902
3.58668672814 -1.92529098701 0.818849430097
2.4788225365 -1.85026831915 1.01937002456
4.78589071564 -0.992686029933 1.3820990959
2.27113649291 -2.98383925774 0.559054286673
3.04118239615 -2.43109871921 0.239029606758
2.32923120489 -1.81568342636 0.71378371141
4.83400304781 -1.08468809113 1.17052118407
2.07266057218 -2.80964979202 0.652490420917
2.5201175179 -2.93580013743 0.429852747115
2.48279662036 -2.56874925224 0.273414080182
3.37621121007 -1.07735621773 1.11537574073
2.5212342311 -1.95663421746 0.734508332054
3.44466430639 -1.97283547054 0.564743209598
2.00972825038 -3.07828563212 0.331464849384
4.42422252141 -0.479558288629 1.63400166124
4.16148611858 -1.07523197512 1.43715570673
2.25772766686 -3.11952915312 0.454502102489
4.31944130421 -0.489971820636 1.48718897718
3.88878277138 -1.17236124528 1.28756892313
3.61877518359 -1.30489420972 0.923259138389
3.41965124721 -1.68304698227 0.669611947047
2.19868191569 -2.27085503537 0.502534409961
4.18876049171 -1.24896764986 1.12738557586
4.03156809443 -1.58339790483 0.837620573906
3.75525277709 -1.03082709489 1.24352740566
1.99033805062 -3.50951273629 0.476126522822
3.37235847292 -1.30731926107 1.20596263158
3.15839777137 -2.08839661873 0.507799931603
4.51448025419 -1.14325551537 1.17521478446
1.80403182018 -3.19930389871 0.523603924902
2.46556781766 -2.70349595762 0.276299756311
4.7358619571 -0.787220403503 1.33951902446
2.36961278529 -2.35290700151 0.722903851748
4.23621459771 -0.656162981436 1.38355420847
2.85713464087 -1.34743049473 0.885068192598
2.51175578272 -2.27233497858 0.420795042129
3.14213094062 -1.67070175892 0.973451755018
2.50030301755 -1.77192558591 0.893935534362
1.92056557474 -3.31109299966 0.307399043524
4.86450464887 -1.17060982336 0.935964503174
2.78598793442 -2.40318953583 0.602804144618
3.20317935884 -2.47711967889 0.670501222472
2.95873136524 -1.34444571674 0.876379633047
4.72825981086 -0.746188593067 1.16432212166
1.58884517928 -2.22188076535 0.463339864387
1.82078319039 -2.90268933754 0.455796803205
2.00189136242 -2.03829286962 0.44669350529
3.02461591435 -2.39051226002 0.664276303492
3.5318127639 -1.3573671474 0.915348803863
1.93206900362 -2.83538675493 0.296661592329
2.29737402869 -1.85807227757 0.85881231659
3.4111503581 -1.46242961276 0.667588201225
2.5743718463 -2.42143357521 0.750854216719
2.1542516378 -3.29970014665 0.538476121302
2.56148696839 -2.78341020577 0.309785492797
4.15367745799 -1.3649461428 1.12312943219
4.32234254842 -0.64609196102 1.33289293025
2.37155058927 -1.82981779162 0.606754932802
3.576921899 -1.51608762118 1.15443982481
3.93405717051 -1.3269349392 0.783205094999
1.92101071508 -2.83256536569 0.458682092824
3.46545619674 -1.46493021908 0.914910057308
4.49689162637 -1.12583552621 0.936409847083
2.22441181311 -2.76527248035 0.514953379635
4.22834850825 -0.633263383241 1.33592420703
3.59254755278 -1.85337371896 0.987507946342
4.5131734522 -1.34550826942 1.18842595616
2.75728439663 -1.75387289966 0.893980996101
2.25119425689 -2.27032164658 0.551067868612
4.69151086461 -0.683456807663 1.27371623919
2.6852773903 -2.91223660545 0.445081176321
3.04105401207 -1.65527039427 0.840149289344
2.98347277531 -2.53525119979 0.457867369159
3.66741538795 -1.11299015563 1.02838901802
2.49341110604 -2.71079424623 0.196027717883
1.64845054266 -2.39877780263 0.631530370302
1.64699268275 -2.21156852804 0.439234002964
3.50069987081 -0.964701731428 1.15860681731
2.15530407486 -2.93167364439 0.454703139302
4.1966910085 -0.512542928094 1.36343715734
2.6870585569 -2.02315769925 0.438199448888
4.12170946368 -1.27405150669 0.946712839886
3.62941690938 -1.43684127687 0.696076101035
3.01781712692 -1.57490149286 0.652092751829
3.52964330237 -1.28037689932 1.14286391833
1.47408245888 -2.38266562139 0.576514004553
4.85622124018 -1.12231564916 1.10432722333
1.61174628579 -2.4281733398 0.811089522878
4.05804521158 -1.07647680626 1.40552032049
3.09307102505 -1.30344887635 0.952228251675
3.02358526987 -2.03552093459 0.401543715238
4.134876896 -1.00162732141 1.12472963255
4.35900107121 -0.584434781145 1.36715835198
2.76479116473 -2.66525517768 0.446407943996
4.03645471375 -0.798169339815 1.03089592052
4.07411899247 -0.638460692581 1.26099309523
3.23636267359 -1.19247388131 1.25409211538
4.29492609368 -0.865202063703 1.37272414962
2.03280384562 -2.31151984044 0.629019817741
2.18654064233 -2.33030315295 0.385953527195
4.01686754254 -1.93196528835 0.729802522429
2.96560159388 -1.85318864238 0.604677247035
2.76625643691 -1.60438365871 0.811954292446
2.6500773912 -2.78117203966 0.632189206574
2.86586205755 -1.54653348986 0.971118000557
2.76103202925 -2.08703248116 0.520142461305
4.28264700668 -1.29196833712 0.848945761317
4.63071405416 -1.49956378804 1.09776393502
2.9621401204 -2.75919138662 0.60156284375
4.8825201541 -0.904300801423 1.10671923452
4.25694432818 -1.17915365596 1.12176701081
1.42777002589 -2.65810881437 0.70200587945
2.35622322828 -1.89808009425 0.957790154887
2.81173629326 -1.77830746043 0.818555339804
2.70231573266 -1.84192244991 0.813120581141
2.07294780953 -2.91769097652 0.451172646817
4.19904306008 -0.736119232533 1.43773494401
3.40932894864 -2.17304807615 0.732924564636
3.38518338314 -2.30935731279 0.606287380532
3.13838768724 -1.397200876 0.793534644858
4.0889350524 -1.70289192293 0.580815445856
3.69875756366 -1.26388515579 0.742432478475
3.97011840963 -1.29337219476 1.19022870235
1.33687664251 -2.4283899389 0.429508337024
3.61985569261 -2.13567592254 0.730772546839
2.355184216 -2.26945393704 0.742175981553
3.33968560769 -1.11604356662 1.09388932665
2.31391582896 -2.59269214774 0.274652337818
3.08296007963 -1.9426637646 0.720816128566
1.76034742361 -2.60760856314 0.567707014847
2.84818782046 -2.15283197777 0.588865284429
4.63199863672 -1.06183727167 1.46713276206
1.81124856197 -2.4378666005 0.386915054584
3.73472240076 -1.38470211022 1.12896423896
2.09456660417 -3.00042498355 0.339397198832
1.95349911828 -3.27401578669 0.548778415981
2.78808596617 -2.47807086004 0.29055262738
3.75792198744 -1.07483233582 1.29937826824
2.2712416816 -2.91375335727 0.462429842575
2.25537284193 -3.12711307687 0.574477404689
2.88709946532 -1.5545032624 0.890187854495
4.41828521795 -0.795916993703 1.52931866525
3.6094089543 -1.3064194894 0.873879500934
4.45109623501 -1.11257102126 1.31054596583
2.05595373842 -2.09190994585 0.925150407992
1.50959215783 -2.50795789576 0.359847942124
3.59625581315 -2.04246424681 0.499244919986
4.31741423099 -0.820991708094 1.05789347247
2.44319525686 -1.77062165849 0.690389991175
3.47779162058 -1.1650137851 1.16801673101
1.41628396635 -2.6501855605 0.629335607277
3.54254500379 -1.33918875411 0.75532583372
3.64364266252 -0.917512769243 1.40500001914
3.79425620104 -1.00962688685 0.954759094681
2.86431218078 -1.96267478972 0.430446778684
3.9224127217 -1.48929586562 1.1501399511
1.8775497297 -2.47708084446 0.640401277005
3.54080528416 -2.06138006146 0.811189282989
2.07504864616 -2.51077463996 0.550710289392
4.0397527641 -1.19584744417 0.798130499459
3.29556502684 -1.58017833068 1.03694800952
1.77643256452 -2.69636265467 0.366298650359
4.92364128801 -0.92220845583 1.13843330844
3.54326088724 -1.26948730773 0.934287106761
3.82339667468 -2.01207864666 0.717276353625
2.62287699863 -2.51733766873 0.436012684377
3.79503000014 -1.96997125762 0.478902282605
1.82683887668 -3.00114730497 0.346750442587
3.0861458218 -1.8173850197 0.909306467968
4.78035621014 -0.841376888306 1.3745918311
2.63501308337 -2.76693045943 0.439273573674
2.48218123369 -2.15625307585 0.569355428043
1.44484530233 -2.8024936642 0.675334447622
3.43781767092 -1.09529939826 1.00029415144
3.45011372173 -1.88603779362 0.625520289698
4.05712963632 -0.670093181784 1.40416788096
4.28624270544 -0.972721519068 0.985242850887
3.94788950929 -1.41726267274 0.95875087985
2.19786877913 -2.81241526393 0.64369731525
4.5554615265 -1.119771898 1.1951101753
2.08776103361 -3.12509476156 0.315481684233
3.2222983877 -2.07810434655 0.782945215361
4.41908518842 -0.965689319276 0.983234767948
3.19695134127 -2.32397566461 0.650833735309
1.87588962155 -2.53760859853 0.553132098075
2.13194557331 -2.18637033567 0.434197725614
3.39824344916 -1.9566718953 0.484139542188
4.18833483519 -1.72588417159 0.641350752134
2.27272673159 -2.23142018614 0.685162797307
3.77986021231 -1.24386866123 1.06528223588
1.79863782336 -2.55024850572 0.795708428953
3.23051674108 -2.34354836997 0.597754871909
2.62657841924 -2.45868931006 0.24787754525
2.37777746881 -2.55574961723 0.335801334312
1.97208884314 -2.06302934053 0.67667863887
3.69767807934 -1.94604949583 0.705053508976
2.55699071033 -2.32957909046 0.797391487708
3.50068839746 -1.79838650121 0.897534404864
2.77535002274 -2.46278912036 0.751287254337
3.01610205001 -2.52768661169 0.498867812646
4.28905566296 -0.897208630351 1.16059375397
3.22879615718 -1.51541647289 0.867147822903
4.30094477854 -1.30956121983 0.875146893372
2.92433485754 -1.4359379173 1.16220171826
2.34279424015 -2.97323057417 0.614166329011
3.84275966407 -1.7903812437 0.551688609779
3.92141441907 -1.58243981656 0.592053458078
4.23307365796 -0.620330913898 1.38651727637
3.3757530677 -2.13570238565 0.654327623251
1.8623937699 -2.13959455686 0.837150683859
3.98360531715 -1.35255840785 1.11091757857
2.79168645411 -1.36680819497 0.789766120014
4.08907141282 -0.635489656967 1.41177737298
4.43186892255 -0.6757178201 1.15253768427
4.05085042279 -1.83418449682 0.742035663731
5.07719248529 -1.15386059799 1.26915367604
3.87404633749 -1.30506708387 0.896743079913
2.9954513571 -2.0277544217 0.645284595717
3.81288927831 -1.22059847577 1.2332035224
3.61730613626 -2.01857034217 0.897337785181
3.18146730879 -1.93899485522 0.892278849151
3.90215503937 -1.64144791747 1.01124290322
4.46213409319 -0.93696376634 1.32272944853
2.15435833813 -2.50899123508 0.304332634992
3.10823950129 -1.77709848522 0.488454710977
3.67335576075 -2.00644390781 0.506020002607
3.20442892413 -1.70320036924 0.745551976513
4.38365628664 -1.0206876985 0.99042716975
4.95773428837 -1.27692407881 1.0710964423
4.38637227292 -0.42808332668 1.35303349565
2.79662922594 -2.23481492244 0.735446295425
4.86681178662 -1.37842383974 1.16329150714
4.19345509106 -0.940342742254 1.33964723191
1.89080654912 -2.15158275272 0.800883125193
2.27567819989 -2.10118051146 0.571764878805
1.96725905853 -2.66336711128 0.419359482861
2.89338065343 -1.43003678231 1.00998681301
4.25873609792 -1.6024953974 0.675195893707
2.53995740134 -2.63711593395 0.49159821025
4.59974576216 -1.07665195587 1.41194260646
3.75214192105 -2.10508057506 0.837483863138
4.44692492473 -1.3777326849 0.850044772141
4.03528368478 -1.77174226379 0.822561357649
3.62906226189 -0.800889627778 1.11318224309
3.98774623153 -1.80493641924 0.965288930334
1.51458216135 -2.4993862946 0.365071045316
3.12198566684 -2.01727070145 0.838749279512
4.31979780517 -0.396673312617 1.4575888619
2.16220613396 -2.5971035341 0.655650753425
2.901479914 -2.31774712436 0.403023095347
4.42876447909 -0.31480165882 1.4481747916
3.36191885001 -1.78599524428 0.523821966472
4.06919610456 -1.22711815882 1.11006057033
1.96661931792 -2.75742759057 0.660771519357
1.91576021655 -2.48525300777 0.39769453016
2.28356906984 -2.18309730189 0.825377104576
1.75346528015 -2.66618318028 0.446456991173
4.4726430316 -0.398441750096 1.49850088083
4.03876928578 -0.709261023317 1.26977051094
4.14421760079 -0.658732885898 1.16917856876
3.86186626131 -0.637638427588 1.22970345964
4.67762863056 -0.773697206021 1.61365215543
1.90295940697 -2.53948967213 0.506611127563
3.09597446873 -1.32411113664 1.23497448166
2.38631497207 -2.40799301589 0.459902798745
2.19418629073 -2.1691062953 0.412294044049
4.03882590492 -1.79941064163 0.605379670556
3.19203851694 -1.69337811051 1.06806421372
4.41872851566 -0.829875931802 1.53071056299
3.97963122857 -1.89162536573 0.888483457669
3.70053257041 -1.57636898912 0.976057379369
3.54783993634 -1.47307017988 1.09688165915
3.67940124309 -1.02025901291 1.08527277783
4.0940131214 -1.52482373663 0.730210782905
2.55783133397 -1.71524146628 0.703451469951
3.491176915 -1.12238793939 1.18370649399
3.07548000462 -2.65653915501 0.58025098004
2.57061976621 -2.2542703219 0.781716660514
4.80094204282 -0.801551239677 1.3039483819
2.05109899297 -2.30474021634 0.533407454076
3.91254394764 -1.77376837333 1.02168272521
3.29130887074 -2.10714336659 0.542462594131
3.62290998859 -1.94439091021 0.880447831334
1.90102228965 -2.23985881743 0.509047186068
1.72402252432 -2.63671934925 0.575963681209
2.47288143764 -1.82637281533 0.784173958256
2.4171766151 -1.73482242275 0.613634773205
2.28564307795 -1.76948448447 0.773072335547
3.314291118 -1.39930855603 1.09492253785
1.70454143722 -2.11000575169 0.488866372554
2.1381685615 -3.01297336243 0.526522043725
1.47412399662 -2.48814048049 0.55653255394
2.89105345326 -1.66981840903 0.912884216723
2.35077380991 -1.795615061 0.555173846113
4.48760288391 -1.27560753595 1.10728807892
4.43437116952 -0.4825422884 1.31226263645
3.82822698354 -1.53441314498 1.12834795491
3.34813552215 -1.90155508991 0.920495944198
4.12891685512 -0.681734212036 1.42818985796
3.47200262171 -1.79100414003 0.703414110741
3.94765722857 -0.873980656579 1.51990712109
1.89397809115 -2.80874817886 0.568144942004
4.09547635621 -1.54696968841 0.97612267723
3.30359639412 -1.70617879154 1.06108092486
2.44557746504 -1.72975440793 0.815669024992
3.53820137054 -1.67140889378 0.905441031136
3.6711197263 -1.81892398449 1.00634610189
3.9743349517 -1.26051981307 0.878703808209
4.82724772031 -0.827454129724 1.29973574763
2.76672006297 -2.579482832 0.571053073616
3.74709911615 -0.869604845563 1.24705214494
3.87668538145 -1.03077842112 1.3139151756
2.88101063472 -2.26028818071 0.422157825393
5.04904038013 -1.22716976576 1.10274069172
1.59262073179 -2.51608334029 0.433473416697
4.48667726515 -1.17597311146 0.856417096113
3.43820075399 -2.26424132916 0.629008729347
4.55115399339 -0.838853717968 1.32122385861
3.94505708653 -1.10481228298 1.26348676887
2.73908678453 -2.40471922438 0.406306117725
3.05542397658 -1.90312070602 0.895402091682
3.53415720612 -1.0752834472 0.979123203333
4.79126491238 -1.24589774612 0.959650061443
3.6282600199 -1.75452156773 0.828294779461
2.59468201592 -1.65931075622 0.873950289914
2.47734703437 -2.37385339729 0.654040837958
5.05179102938 -1.05331657804 1.2134904232
2.48490595838 -2.0276003237 0.506594295441
2.31540674343 -2.56173046891 0.741434789041
2.60891802177 -1.71417943152 0.710508633072
3.01436206546 -2.10585458849 0.764908991033
2.94969535061 -2.11723612713 0.39853303209
1.67445015411 -2.62636925737 0.632475571862
2.39047661325 -2.812756059 0.524355143995
2.34900719487 -2.18139905981 0.855412433956
3.19470062695 -1.49012080455 0.881111264879
4.44013460728 -1.24827045652 1.08894048824
1.79816436519 -2.82443024931 0.281164960564
2.74687343274 -2.26228009178 0.406851440571
1.89588745926 -2.37154647934 0.54696671702
3.92036318781 -1.76021474377 0.554573020714
3.25512294267 -1.72742788187 1.05286121156
3.27581697537 -1.34836101482 0.881731408763
3.28257958899 -1.36191090695 0.924461183857
4.12387868538 -0.908645952238 1.1630170809
3.15415901355 -1.7573331831 0.507887279733
3.8469170971 -1.39722293111 1.14362673844
2.32265763151 -2.39803366328 0.433656428106
3.42693731344 -1.92107514501 0.754781537601
4.781230029 -0.92505904372 1.41820451425
4.66732440658 -1.34581848129 0.902233087518
3.76369843674 -1.85895120666 0.579857601866
2.00100802756 -2.21586670541 0.737768896403
2.52927358966 -1.73837259408 0.926936818367
4.32145935531 -1.32469346672 0.78947372051
2.77249124692 -2.24080847864 0.719870629369
1.97762900226 -2.49976031663 0.656652938764
3.78977990892 -1.47087296635 1.09503611693
3.85266879366 -1.71470170345 0.725153007363
2.58244379828 -2.42663121067 0.590320147353
4.21291795114 -1.73593146984 0.936915372545
3.88788815432 -1.15611321041 0.883884281683
1.91311302442 -2.44926225901 0.793808982623
3.02423348716 -1.72458100721 0.682613145986
3.14467479307 -2.21618847329 0.481655374829
4.1427710803 -1.10569025602 1.34263301364
3.15885187303 -1.11898692739 0.93572320514
2.90083577205 -2.29762730328 0.603857562182
2.62536872681 -2.03287510415 0.796464846253
2.32786662364 -2.74661852232 0.37021072975
4.55256707349 -1.2607367754 1.08095196374
2.273193701 -2.07944661635 0.517871047438
2.90002570878 -1.6342995231 0.658267546429
4.56852532013 -0.862959961772 1.50517676399
3.97836441659 -1.94825192203 0.715913900147
4.63097325657 -1.36749078587 0.854624020397
2.32688203383 -2.38388463934 0.685329861889
1.69937254594 -2.30069483841 0.601541862849
4.53964477957 -0.57833580771 1.45365807524
4.4555570831 -0.526415992686 1.30761831573
1.55023775692 -2.5691060369 0.629046093689
3.96682049281 -1.44941818021 0.797222287566
3.77062298914 -0.81766997112 1.01864441302
4.45921447084 -0.803150026653 1.32253846059
4.62887488428 -1.05038546578 1.34476810113
2.48947290821 -2.41957038852 0.530489526496
3.11128692065 -1.98261045225 0.752813792328
1.67721904929 -2.56936230605 0.428658892285
2.85963624301 -1.67059054172 0.871860532191
4.0156050889 -0.892589315407 0.966919675556
3.80553962388 -1.21535395566 1.3325897768
2.73363029059 -1.75991611748 0.67937510957
2.7275397656 -1.70144636436 0.739963267792
5.12758190054 -1.16166364212 1.16902282008
3.26143853319 -1.21273468195 0.812816281031
5.00546150078 -0.981694877227 1.03697167276
1.91215860626 -2.1807559202 0.474998350955
3.19388376652 -2.19264108337 0.739226980445
3.4984334816 -1.13386339378 1.1161954795
3.21284111662 -1.42647350068 1.15386564873
3.47866088848 -1.97065801235 0.777177837966
2.28275991684 -1.92975219426 0.919750157297
1.76797134035 -2.95052878325 0.306945475681
3.51574070386 -2.20965158691 0.692010727978
4.01201865129 -1.83296857865 0.901380635932
4.89535370947 -1.18505970595 1.30232800036
1.48048851816 -2.57909544321 0.675717542132
3.87462782713 -1.5021322622 0.742598361841
4.32833705065 -1.35402224483 0.794258047863
3.19616510798 -2.04280907692 0.810674186544
3.94107391516 -1.32818355126 1.12370915796
2.61388269252 -1.7581385609 0.610336732253
2.34005181027 -2.39980672974 0.672771220152
3.52844367117 -1.7871573984 0.946414638027
2.30657574231 -2.75500834191 0.393656573243
2.5188214452 -2.11455507881 0.788607775215
3.62678610115 -2.08794815899 0.406827773335
3.36459722486 -1.77958801212 0.970224007534
3.66654460411 -1.26497779979 1.15729452351
4.18447559628 -1.42943953684 0.668387872146
3.62103980841 -1.60357206843 0.78207780161
4.28513172683 -0.916185127767 1.00134111102
2.66609419605 -1.78631115993 0.727785981788
3.14300919265 -1.91256090556 0.55760285356
4.07178452579 -1.6944815835 0.938889727026
2.30106960178 -2.22240636306 0.763007591649
4.65395528913 -0.756600316895 1.28114504891
2.66090059528 -2.52914329754 0.61522039319
2.04955068939 -3.16429942327 0.251226560083
4.02892253246 -1.32685334003 0.950310367585
2.38978790576 -2.60708762283 0.557340922306
4.46608266379 -1.35986479835 0.948181554755
3.07066658739 -2.56972797861 0.426883875144
3.65659799955 -1.45833432857 1.1297225931
2.11802965501 -2.13955448198 0.399618715658
2.68484446515 -2.16625117579 0.809570353843
3.88641943208 -1.70465051235 0.688261678459
3.71461843332 -1.52421191487 0.834404862611
3.62239036718 -2.03857692511 0.844481564396
3.98426468845 -1.05076098501 1.06953331694
2.12694074035 -2.2559070687 0.756085986811
3.46346123478 -1.03234615858 1.23269857078
3.87267139268 -1.37873066669 1.14926187919
3.37103559858 -1.33961735432 1.11824830048
4.76503869375 -1.09052730983 0.941584359126
2.97408583458 -1.60076972739 0.941370424824
4.93877549003 -1.0975581802 1.20008573952
2.59912174049 -1.72042756935 0.909863184391
5.03510368763 -1.17176908273 1.01284180943
4.15093979805 -1.56943160168 0.937717690705
4.71012703278 -1.32683616462 0.907379189449
2.77385614073 -2.70994027025 0.18119215233
2.67102068449 -1.52252362053 0.784944659248
2.07915146285 -3.29908136186 0.438944560211
4.62015182142 -0.681032522487 1.3776887055
1.85037769902 -2.43016236753 0.450997707924
3.23452008513 -2.40190990363 0.572085038983
4.65185405302 -0.995974221437 1.25139299331
3.86405006527 -0.959016858971 0.953673171166
2.04437323597 -1.95298528393 0.588215514768
3.91670669838 -1.47611314826 0.723658682987
2.86902399516 -1.34440112362 0.750655393009
3.09107996668 -1.99642552665 0.747893093239
2.19287599783 -3.17959931201 0.292863000771
3.96970627763 -1.68794604043 0.573151895633
3.14700354718 -1.95960091617 0.631254210202
3.95032010676 -0.683064525978 1.18707771822
3.45146859117 -1.95001175656 0.653319659199
3.67890934661 -1.05222679781 0.97730233115
3.64977721173 -1.28583917858 1.26506104951
2.68143053278 -2.07380116789 0.52898439381
4.50109775054 -0.736164399627 1.60687717991
2.2503196067 -2.54843928681 0.528188034964
2.89923776958 -2.71955093201 0.442010005435
2.57291892955 -2.29401981762 0.337794307599
2.83278466257 -2.71770581598 0.28655693841
3.5712229737 -1.01060276659 1.40384683713
2.14959778322 -2.70906301855 0.564598910862
2.56702713748 -2.64102410981 0.538747225894
4.19053887266 -1.69264122148 0.737771540964
3.338864866 -1.20602463023 0.865737732586
3.69419681575 -1.06179308146 1.30020104245
1.77888327289 -3.10234306667 0.50470530826
2.54579369986 -2.56303078155 0.478585363184
3.11603207802 -1.9892236549 0.838437142336
4.42659826944 -0.610817712061 1.67523041991
4.30303512236 -1.03243496604 1.13680835341
2.96067117125 -2.15282314463 0.47684690372
3.41559288974 -1.56877736438 0.769299720988
2.02873905542 -2.53362960449 0.393206934608
3.23747039155 -1.23507147817 1.06484262833
3.59253764553 -1.71322985767 0.775652357594
4.39036660242 -1.60950610657 0.854097444571
2.0353294404 -2.66905847755 0.576417547347
3.18481768934 -2.29260805899 0.703797149392
2.45711174631 -2.88773317394 0.531767303351
1.74738793816 -2.62420811999 0.492613770612
4.59262490581 -0.497280113904 1.40698507977
4.48255361932 -0.45966980893 1.53415860114
2.31628603437 -2.58155712002 0.524686021849
2.38695253251 -2.98015020676 0.201688917714
4.14923936273 -1.35764394037 0.914577460148
3.39920289897 -1.41153415741 0.819593443713
2.26168955554 -2.21422875807 0.406464468651
2.0960707222 -2.84455574304 0.555147393168
4.25546371857 -0.703216329893 1.50441557696
2.48834863643 -2.2502988194 0.641538765121
3.52500928666 -1.67088373027 0.914738686306
3.73663401355 -1.54977407587 0.635113495454
4.03900239781 -1.30231871555 1.1969139137
4.69892664125 -1.39903469489 1.0817587969
3.86909199096 -1.34594541557 0.789562109344
2.86949521262 -1.628390382 0.828890793111
1.90350203578 -3.16642499287 0.123821414329
3.86723373542 -0.899030099434 1.49198722326
2.56998451956 -2.21536227391 0.3776449673
1.95211723778 -3.07201764027 0.32310943705
1.79023879736 -2.68486643274 0.639728645634
3.4755969023 -1.73569417336 0.743596576742
3.26104530604 -2.16476001758 0.839770964087
3.55551091493 -1.18090521623 0.955147593499
2.4285908134 -2.71847669964 0.621037211043
2.71850707776 -1.70390932188 0.976424050241
2.41620869282 -2.96242374229 0.185599797574
4.29231139809 -1.35846865951 0.905975231589
3.36931479996 -1.89403406168 0.583184200447
3.09331356648 -2.25293328513 0.687527135944
2.61234092974 -2.30546732278 0.383534029952
2.93668466846 -2.50285209212 0.220986449544
3.86445211475 -1.89824261942 0.871635491379
3.91063319541 -1.02258408223 1.17964013864
3.45871055547 -1.34117625742 1.089528264
4.29317917989 -0.635809046421 1.59658349339
4.05033649029 -1.36413196351 0.691033270734
2.95131249682 -1.30209357004 0.888560254367
4.28342297878 -0.814515451175 1.55384654745
2.29753016002 -1.94324296859 0.956320255892
4.41057951166 -0.587019381448 1.44562061193
2.40023151633 -1.97478941627 0.880222977694
3.83874420116 -0.72884765082 1.36576227991
2.32072869185 -2.55939367558 0.593354197264
3.78676671881 -1.94096311526 0.611178043191
4.15094748923 -1.35387013462 0.899801457927
4.20384994533 -0.702238302207 1.33487039929
4.21129158616 -0.593854469541 1.2588933392
3.68390866812 -1.20777200553 0.908501475205
2.0156537866 -3.12380420291 0.118061514685
2.88997396396 -1.99315794982 0.652252343765
3.09194604528 -1.94591498473 0.613481941958
4.64019158257 -1.1215474551 0.950900984742
3.20073455038 -1.57863882856 0.787671608377
4.01765362447 -1.10965030137 0.841982496331
3.61243064015 -1.72367262743 0.541552811377
2.70098544742 -1.56383792072 0.869393373837
3.13257397079 -1.60896963507 1.02337687085
3.91900361315 -0.849359518737 1.08812771742
2.67615290043 -2.3500756891 0.284422881694
4.70691854942 -0.839386397908 1.12561475109
3.31890208661 -1.7112972125 0.853876367451
2.69858921276 -1.88351955599 0.662783736422
3.80519427545 -2.1158655828 0.797816252406
3.79567847058 -1.5107562586 0.786513947756
2.21850327597 -2.26109745045 0.747642012021
1.72249736108 -2.62399007193 0.472400189334
2.89618258536 -2.23382197701 0.577225942608
3.08088538389 -2.43988349917 0.287701073999
3.15382453012 -2.35653622324 0.557272941092
3.38945606118 -2.00306527284 0.846158738381
4.1568999098 -0.925256585674 1.50247338022
2.31795713588 -2.54820974441 0.699737114496
4.19035770416 -1.59501867036 0.932052769136
2.74725066291 -1.62138535838 0.649881440335
3.12344527228 -2.27568142671 0.782364393316
4.70761697817 -1.45633583705 0.958056338058
3.58114890958 -1.53343093619 0.800572206162
2.45947652544 -3.08284360634 0.476529112569
4.31059811827 -0.688306295305 1.68198557619
1.66281218302 -2.93522618271 0.299392092755
3.47273495014 -2.25779705387 0.681633642925
3.37893845611 -1.44047710303 1.08996382747
2.94920926348 -2.5368108824 0.483378596216
1.87784723193 -2.93366300084 0.680567181633
3.73014486659 -0.796496856211 1.08869822699
4.22874691601 -1.07318938761 0.891927303289
2.1263821846 -2.22791284259 0.443469350327
3.05464204077 -1.20333298498 0.928744653388
3.17525940556 -2.36866322584 0.518750652758
2.68831309619 -2.30167562433 0.639631271132
4.50539574431 -0.948722934242 1.44914716624
3.05511263166 -2.43182851266 0.459994103038
3.04477693048 -2.0136142729 0.390956565747
1.83823385284 -2.12263862536 0.712744214065
4.23924023139 -0.794106960376 1.0874805184
3.68473833619 -1.46565569165 1.02219376135
2.00302379378 -2.2100711969 0.404584014511
1.52737955925 -2.49592203572 0.475651428577
3.405712741 -1.23345461919 0.770590931139
4.7835003939 -1.00414661391 1.12585895681
3.74570031667 -1.7232640832 0.657905046681
2.71476885502 -2.12358073731 0.374864038685
1.82729928301 -2.80842209644 0.221624460774
2.11430643772 -2.50605062722 0.687882111588
1.84154118566 -2.4267984628 0.432782786142
1.69747671692 -3.0049342735 0.248132506642
2.33519928996 -2.37007059592 0.494497089006
5.0154423883 -1.12159950808 1.02624868707
4.9768960988 -1.06626458749 1.27813279909
1.69040248058 -2.6780752088 0.711208706403
2.97992051972 -1.90246871455 0.899758299333
3.57413719699 -1.55440581238 0.896754314953
4.53967779545 -1.11482358373 1.19023393143
3.2048730493 -2.02298514018 0.516549470071
4.04688627485 -1.48437603173 1.00727824111
3.82358301106 -1.59844611756 0.598977749088
2.25334449123 -1.88829622454 0.889893261242
1.76409771441 -2.28703241966 0.535466125748
2.72076810878 -1.61412405829 0.742797497387
2.5372990602 -2.96158722631 0.501139210063
1.50790540287 -2.7852971383 0.317803873534
2.36809499188 -2.09608628536 0.570250406686
2.44245787111 -2.15868616674 0.711250763156
3.99264276582 -1.78757429848 0.626770448516
2.00199773332 -3.12126621839 0.286413135686
4.02500938253 -1.9569324406 0.887051075327
2.69258225608 -2.74972656884 0.590212220566
2.03867691482 -3.17717274368 0.252573458835
3.01472840897 -1.39916327636 0.868063126206
2.84105722335 -1.63772269451 0.809443341047
3.67490779879 -2.10641252819 0.49173914872
4.13697047619 -1.34493803467 0.77326399674
3.71375334867 -1.69111648368 0.709493627784
3.69960034186 -1.91397598109 0.855320522458
2.50230824029 -2.72362973823 0.538965032701
2.15442668563 -1.98979547599 0.632654574798
3.85689137489 -1.55290566885 1.1134274887
1.89712831842 -2.71398693608 0.413625094632
3.14891411353 -1.8961647857 0.619969889257
4.32125112387 -0.645549528813 1.60550410122
2.95697069073 -2.01671617312 0.616514880237
1.88747910543 -3.04009649521 0.163172803868
3.01210178699 -2.35435584148 0.342214588653
4.45413139768 -0.789026764378 1.60241730403
2.69898339574 -1.64271466823 0.765661639795
4.38659629451 -1.47050107123 0.642966010892
4.24523664925 -1.17667441509 1.02760658201
4.89800272838 -1.07032311814 0.941663799107
2.77819684945 -1.57619866321 0.885559230343
4.01484667802 -1.61332312351 0.833131530393
1.83060483309 -2.08352627917 0.662499465453
3.98857521848 -1.20097729867 1.16020945185
2.18470643475 -2.99680918652 0.412433142054
2.32521294595 -2.9159922078 0.365707781435
4.52808680373 -0.646817164162 1.6636346832
1.96806382722 -2.46407616177 0.480808677992
4.09661398011 -0.970547799714 0.919205645949
4.16040562159 -0.859534267425 1.16949482033
3.93512914329 -0.743059931039 1.56094771563
3.37750818211 -1.83904819867 0.851425736019
3.58370087261 -2.08565757309 0.549595900169
3.70847916627 -1.58433897575 1.07533580138
4.42335268683 -0.762376203399 1.51902435704
3.10273215877 -2.15813089954 0.370202909037
3.97933792531 -0.688155679502 1.4752825542
3.43409424043 -1.19468460989 1.15265995621
2.67350678614 -1.74151703228 0.528236990772
2.6406117755 -2.64330015401 0.396992882403
3.67628903605 -1.22052374516 0.811173113501
2.66223896961 -2.72113444133 0.53626396289
1.83245713471 -2.60607963441 0.380448441668
3.35190765769 -1.24097958893 0.756810482567
3.68293022319 -1.93973661268 0.73551781646
4.12295035833 -1.45302891921 0.840824773912
4.3640513243 -0.904956037253 1.28163623203
1.59035933953 -2.67256299152 0.329220371983
2.83230771904 -2.67490139958 0.427976818697
1.82905386189 -2.81373027475 0.414533458864
2.52266198066 -1.74258273594 0.950522742296
2.41919542709 -3.00319252721 0.390413595888
2.51827787765 -2.65563881304 0.243063686474
2.84080956759 -2.34118712195 0.523401359445
3.64848399196 -1.56912337225 0.895679581418
3.25281048505 -1.98148637298 0.830952379393
4.34177559424 -1.23819560038 1.11100649649
3.84438925002 -1.18127527696 1.32303510859
3.72310642329 -1.00676368041 1.26210179413
3.05663102862 -1.3781697093 0.871310954501
4.64514743683 -1.08100298607 0.984455412807
4.49918268013 -1.38801803315 0.99902386836
3.08887462582 -1.94448343576 0.915703763909
3.01674890729 -1.7223550077 0.795937899683
1.98276349866 -3.09920754778 0.270822139882
2.55747653653 -2.83990503169 0.433335982971
4.19096680025 -1.5306205801 0.994868114715
2.44591818776 -1.68882960274 0.900216041888
1.91919251043 -2.5790385416 0.748244736273
4.66362221419 -0.945270780792 1.14815771396
3.3652608659 -2.42224316285 0.626671662321
3.67476077836 -1.52208475317 0.94355883944
4.81724722834 -0.983913657335 0.943894827441
3.14059042378 -2.44223356319 0.500896019369
4.07566750823 -1.77219494051 0.50682780375
1.78438141493 -2.33539304396 0.594097177937
4.74844943339 -1.33304853267 1.04780810585
4.3195985336 -0.722482710784 1.53130357324
2.46234400018 -2.59546374637 0.705135084915
2.7317452275 -2.62960992939 0.512597035071
2.79764584986 -1.75486177932 0.896190451656
3.3676689419 -1.85963301064 0.83303860728
3.81706291742 -0.895741697301 1.22430000519
2.85315667354 -2.00902232809 0.518999364524
3.88941285621 -1.52370837154 1.1589809327
3.13762109963 -2.19217961037 0.768827038787
2.59833075394 -2.31491640504 0.461904958778
2.15573142477 -3.03300919703 0.174767783253
4.53405763346 -0.788738369462 1.60921387596
3.41815230013 -1.25516709745 0.777420199983
4.25700728684 -0.703542517695 1.38905851093
2.02245866112 -1.99913767372 0.760312810603
4.90595714644 -1.02755118818 1.1752468111
2.25453992095 -3.04962566949 0.358213681347
2.55022813426 -2.15264416468 0.595059563127
2.87447123125 -1.4396825245 1.06131268071
2.37184482894 -2.10649782847 0.805039602173
3.91769555955 -0.974096814642 1.15470338156
4.56374537208 -0.814715299827 1.04421451434
4.61284500732 -0.906092597735 1.17619807962
4.42326910909 -0.781741385445 1.36470698512
2.81234016734 -1.93369782828 0.799235658802
3.74752819474 -1.5755550647 1.10972389499
4.2894770517 -1.17550793983 0.94883738834
2.82458020367 -1.84947261706 0.755264017891
2.5633066158 -2.41204175897 0.780105364666
2.07127256263 -1.8681694656 0.619867094617
3.68123260457 -1.90969233943 0.927801274708
1.86977522445 -2.49550672032 0.307147997668
3.47735975452 -1.09212514351 1.40233954988
2.73853967541 -2.44949092818 0.670907092881
2.42310792314 -2.27875527036 0.394171516647
4.3673892981 -1.21644493384 0.933722657724
2.88048610388 -2.68041603987 0.673900352266
4.99460633337 -1.19100825339 1.08439511272
3.89799314011 -1.27955856808 0.956780767943
3.0911870161 -1.7347043166 0.902287832217
3.76894909702 -1.51420486812 0.827564052562
3.52323093122 -2.01199824999 0.824962102553
3.58222058233 -1.14473585949 1.33025035444
2.159786816 -2.77241778277 0.196140759436
1.5026238104 -2.73545870424 0.664599037698
4.89560535804 -1.1864515317 1.26749112875
3.33292173106 -1.37922092007 0.708786403031
3.58112924706 -1.8632597916 0.926360779127
1.70373121376 -2.56817159935 0.580216970874
4.32067195216 -0.648579884135 1.28632000591
4.69038932636 -0.615414650815 1.38130228371
4.65021074604 -1.23778732039 1.13199122381
2.66981075095 -1.99498018198 0.782340346198
4.63531410764 -1.56495172065 1.10778247137
3.89258698389 -1.59470655553 1.0962429231
2.03087981845 -3.29399074577 0.433965593217
4.04017762858 -1.79170991159 0.959265381921
4.37992369937 -1.54675787309 0.580580031046
2.27231848475 -1.93164424109 0.622203511024
2.3658456458 -2.88271315561 0.611262202691
2.56455311278 -1.90481307276 0.73770027201
3.75818121182 -1.58849627425 0.954104635252
2.48923554095 -2.00820287159 0.66418425012
4.9313273367 -1.07303441968 0.900352500641
3.42931484205 -1.24766969699 1.26431014405
2.52046346942 -2.11939968673 0.708885483329
4.47178692873 -0.701260904029 1.36757551838
4.67205447468 -1.31163689577 1.11453166899
2.66376129423 -2.71549092594 0.498033276711
3.14120443985 -2.29975305024 0.546332859649
4.47601480254 -0.724558124199 1.24078285257
1.80846335999 -2.86368342387 0.373528943987
4.31276144039 -1.46137252759 1.12558160575
4.54308614929 -0.660223057917 1.52386994494
4.09168657283 -1.70715995741 0.507132323827
4.72886124039 -0.927656202347 1.20733737753
4.58097092157 -0.624695373832 1.7589695779
3.81496741518 -0.913106422334 1.33846604412
4.16155978747 -1.21140744801 1.31021190019
2.89943312306 -1.71725621795 0.561555817208
1.79513950709 -2.20676727603 0.746292655045
2.33438390866 -1.79100926468 0.766155997926
2.4020032102 -2.01138117408 0.763424159901
2.87112770433 -2.07372536977 0.687579702627
4.2170742678 -1.61801578505 0.882124069772
2.77986189252 -1.86579627206 0.927621588085
4.67408397409 -0.724969305172 1.47159464408
3.98000535325 -0.75031915914 1.24204965729
2.86300002747 -2.14646918781 0.645088229636
2.19664217229 -3.00571778263 0.372327917334
3.37113514309 -1.44423183842 0.758360424345
4.11261925564 -0.657908715639 1.29756654665
2.31974576707 -3.06740330987 0.419544755593
2.11456662659 -3.17249781331 0.185770614361
5.06235718997 -1.01182041754 0.949601571159
2.0623379363 -3.34790981876 0.289862686135
4.915955584 -1.16561871886 0.925769820039
3.20778074351 -1.49521731851 0.712948352236
4.6167229199 -1.26165397363 0.931520805106
3.79953648059 -1.66714073323 0.674720111807
3.49634181832 -1.00884516409 1.15494874288
3.20306169617 -2.10929460916 0.465211459487
3.7071813908 -1.02284112305 1.05955826672
1.82329654544 -2.40221280693 0.325504761638
3.91133726143 -1.67088299998 0.76888013295
2.26513611479 -2.08145850636 0.835560579748
4.13477154656 -0.890806441817 1.21417582061
1.80081484243 -2.95788073032 0.393310377992
2.04711919332 -3.32040623698 0.56423485948
3.41792379767 -1.58763662392 1.01101654182
2.24858757196 -2.81632732501 0.253056208961
2.30216834175 -3.09813469917 0.220472174907
3.79172004141 -1.62787380658 0.573099726912
2.52459094539 -1.76948658824 0.679776682734
4.5013570579 -1.30218383981 0.861657550938
4.8036198996 -0.939883351905 1.19842553114
2.67502576869 -1.78416685742 0.833514299403
3.64173872633 -1.37128986622 0.919301413456
1.89439826502 -2.93064101946 0.319924388803
3.83544366443 -1.8599145019 0.930818427973
1.87133593101 -3.09746487622 0.550037353454
1.77010485701 -2.9471425087 0.609171704137
3.0895190168 -1.50583274097 0.663802399493
3.15401151654 -1.8057456001 0.584532649868
1.69813260015 -2.63435405537 0.616520578089
3.48124292631 -1.63029652104 0.987828633666
2.78794059456 -1.62729717197 0.7928181899
4.27927145526 -1.13583788044 1.3359391736
2.16856349809 -2.90232612062 0.429956103674
1.45255276547 -2.38867726658 0.555678787303
4.30297031051 -0.987689148512 0.998610461016
2.546422488 -2.19245585019 0.464194595644
3.28781070185 -1.19306900308 0.908975741191
2.0084611498 -2.29663110314 0.78034608174
3.32455823652 -1.57381353706 1.05527328851
2.93193224861 -2.55731374217 0.478613828535
3.85037604224 -1.50185730407 0.719826985675
2.18284822844 -2.6050559549 0.288778889184
3.20965797588 -1.21880527208 0.828204029086
1.95135666559 -2.04987126403 0.533877509617
3.08252159775 -1.68708435198 0.539233989981
4.38859603282 -1.41459890234 0.702506940831
2.83906387714 -1.58316229356 0.767377769854
3.03486471402 -1.43161066285 1.06997074274
1.67901599636 -2.41198837963 0.497881302768
2.86661745485 -2.2496875981 0.308478794313
2.82196907979 -2.37003761101 0.417440924948
3.1405204346 -2.3041205268 0.630321491672
3.13781639932 -1.17498329518 0.860551764617
3.69948693825 -1.05525734511 0.953691673772
4.71060951353 -1.40106079531 1.06098529787
2.16222382814 -2.99805444836 0.329377660062
4.78096617528 -1.26497667368 1.16963920193
2.06635230376 -2.08070818966 0.513275754673
2.83726901154 -2.13934685444 0.647754859909
4.54757054164 -0.599761115321 1.56987263993
4.77669315459 -1.05422611342 1.26078889913
3.43032090723 -1.96160370431 0.627576146784
2.49152782867 -2.73658720405 0.522166812427
2.10392496258 -2.66031131892 0.405699437111
3.58602076174 -1.46502753073 1.05922343613
4.14482948443 -1.61293436758 0.811528469546
2.77521774477 -2.30920377256 0.449285289366
1.76337610535 -2.28252251354 0.453774382517
1.93825801619 -3.36411522025 0.308138244302
2.60536139716 -2.51616248835 0.251866995939
3.31895092013 -2.42055136417 0.684302400672
1.54398881023 -2.79398704487 0.562273218257
2.79280426209 -2.58995022359 0.587284292198
3.4332321313 -1.39978698711 0.776980046818
2.56610265539 -1.89827101457 0.670936154171
3.23693255658 -2.08057863466 0.683764568192
3.17424833793 -2.40845703215 0.538393272015
3.33544977212 -1.22070010288 1.25102467093
4.37699161438 -1.46448967197 0.834735243783
4.20873344053 -1.26469631991 0.784964643436
4.31150552978 -0.69616182088 1.54739450575
4.2830981379 -0.814299340313 1.31831628402
4.27751553364 -1.28820715781 0.842734777876
1.71311144766 -2.57253059743 0.336293362349
3.78416251003 -0.874312827009 1.24106908364
3.03430189428 -2.45655244775 0.222486816451
3.13486125561 -2.14536111779 0.583713995639
3.57111215497 -2.11466932399 0.715936496174
4.54579375942 -0.730899659968 1.13955492925
4.80445411016 -1.20008060813 0.871165088551
4.0717477937 -1.71482377568 0.502511839799
2.35567960113 -2.58194587083 0.387910741598
4.82771240055 -1.24979926505 1.2930977664
3.27540978731 -2.16921135161 0.394853860491
2.20746252421 -2.21840459359 0.506202008461
2.79244187281 -1.86776796891 0.830591892857
3.91344026533 -1.10561850109 1.22240743561
1.42166056009 -2.4947048851 0.345726248728
4.76072768627 -1.36851591335 1.06113194295
2.09091789827 -1.92334193253 0.673496500657
2.35462895992 -2.55852699435 0.352993567954
3.8122438136 -2.03002070898 0.686808583767
4.15537318151 -1.83689359928 0.806478074206
2.09869848583 -2.15246518098 0.629062840703
2.47356759575 -1.84865515837 0.752947783578
4.48812520746 -0.606369656597 1.73381462626
4.20140664902 -0.911799952527 1.51192287973
4.59765372731 -1.27091604143 1.05850931301
4.07292030161 -1.79289247257 0.983397952172
2.80556752413 -1.50152200418 0.777247203723
3.55520152781 -1.0691932122 0.943298369832
4.20370669591 -1.65232064453 0.77267218334
3.38263270541 -2.29174539002 0.384282842907
4.99143439938 -1.10562409858 0.897631682976
3.60207155261 -1.75818626431 0.962817551361
3.0186384772 -1.37792825237 1.15768253701
3.28080830374 -1.90069310366 0.832308553663
1.5446965872 -2.50809949107 0.532774522184
4.08342596339 -1.14493841606 1.00625116885
3.44371281241 -1.54914652238 0.832459589773
2.99565750897 -1.51718421749 1.01670538716
2.49325648491 -2.90609398488 0.547416112727
1.81651700118 -3.14835030618 0.237101103236
3.73393879008 -1.49754484937 0.82116508308
3.91895404286 -1.77721193319 0.915697948974
2.3782333525 -2.25134727451 0.522719153681
3.93657175825 -0.663492296693 1.34825185511
2.02835050941 -2.20692861783 0.615013009822
2.39898967743 -2.28694182057 0.358118737433
2.83501478479 -1.88580572519 0.824648169197
3.05544485664 -1.29694492288 1.14984314338
1.48219377013 -2.80693410599 0.726636887722
4.85134886797 -0.946312562751 1.18463567334
4.11141901494 -0.815824127138 1.59109288843
3.35737686918 -2.24410302539 0.484191296258
1.99880099844 -3.11967955697 0.130082057914
4.66190626684 -0.906964388762 1.29978501308
3.80468355501 -1.1568800092 0.979443109425
3.2988079543 -2.42607406498 0.62792235175
3.55206600068 -1.48552934332 0.956386692728
3.16294112209 -2.05661656146 0.847467054849
2.4403253942 -1.95891750422 0.836775967654
1.99263899225 -2.13411821999 0.89236062089
1.84436066111 -3.20511387016 0.362917976607
3.93292594668 -0.675919773575 1.29661610807
2.15841283983 -3.22873287022 0.236549977595
2.97467402456 -2.46088182998 0.6108495047
2.66369836049 -1.93517066412 0.888799817845
4.76076487697 -0.955837677778 1.29520092257
3.79055992133 -1.90522560974 0.502793158657
3.53365171828 -1.80855561249 0.746830877067
2.74773259799 -1.49274395635 0.876538704925
4.15105720924 -1.09117450392 0.885748277293
4.27402653432 -0.789552672879 1.43809907516
2.28369325473 -2.24154660358 0.817277069152
1.86710774548 -2.24963716486 0.85092814338
2.10772230069 -2.44991745317 0.705940774037
3.79850375448 -1.94426003427 0.740626378211
2.24664468861 -1.92530468894 0.709451006896
2.6563682686 -1.97047914506 0.756375453064
4.36947720306 -0.538821591573 1.47984958911
2.61326759532 -2.97388918914 0.429622717464
1.97390350434 -3.26714665215 0.210476603506
2.68298699539 -1.98536800034 0.681644125613
3.75247607704 -1.80745776491 0.860073205992
2.17387621165 -3.06144305604 0.492227339755
3.87574166939 -1.23395981306 0.816938447179
4.97140258497 -1.25336518535 1.28650272673
4.47399302345 -0.516056907373 1.45454683816
2.94188865108 -2.34825002816 0.714735198653
2.21089668481 -3.18726797283 0.186102404783
2.59848371573 -2.86659597813 0.446295583081
4.60589193536 -1.53667071999 1.14685617471
2.35691672183 -2.65958305116 0.311348097368
4.4512166647 -0.796122467377 1.32745213979
3.07162783891 -2.03694057626 0.570645230789
2.38217440917 -2.04645940892 0.754487364587
2.91050538452 -1.47294725333 0.700139568674
4.24230707335 -0.825742634379 1.58065410832
4.25497790492 -1.72289554859 0.966779544082
4.06137644327 -0.697471334116 1.4432165826
3.88435556078 -1.73261547646 1.01766158209
3.15505408055 -2.02002070693 0.693358315732
2.76457672547 -2.70979123641 0.417748201276
4.2419002581 -1.57278739322 0.591596343042
4.63337504436 -1.24570917239 1.0566946546
2.93442407092 -1.93356347221 0.781579397045
4.65786858362 -0.885292298713 1.48165767358
3.15662564967 -2.54669962703 0.681991631316
3.94610409451 -0.623797635308 1.25016797244
3.4253122008 -1.14109777168 0.933888100992
3.09927010761 -1.55077605825 0.660291706927
3.81002257545 -1.0717619306 1.39859063019
3.49858818665 -1.97517448972 0.559969560187
4.76660195017 -1.07108846046 0.992737029136
2.47605015662 -2.30824876151 0.780025215871
4.35444972408 -0.692247060405 1.58030707649
2.39353143052 -2.37392772543 0.742133262689
2.61191626349 -1.82214579623 0.607343598192
4.67107056919 -1.08669401894 1.23828006877
2.67883956719 -2.25573216197 0.693565511375
2.89017314335 -1.91388549909 0.704169849888
3.20960969828 -1.25032170801 1.21301805443
2.85219212881 -1.72277508292 0.557853980668
4.52276837918 -0.896669004389 1.40559357934
4.08047046583 -1.03023114778 1.08648477184
2.53349606517 -2.55879031776 0.385924989067
4.49849344942 -1.1881447753 1.25433895793
4.50832850338 -1.19458907419 1.20497541147
4.17776969794 -1.42409198758 0.858673955377
4.43821405187 -0.784128873132 1.59340521402
2.03637015219 -2.83545822763 0.403310643832
4.69401197825 -0.86797079485 1.47576056271
4.52260658887 -0.613294776124 1.3471308069
4.46221227015 -0.689495922984 1.59888096027
4.13354399792 -1.59671268024 0.753855714665
2.9343805435 -1.9022848288 0.869124446202
4.09969866387 -1.62658888128 0.57544248705
4.16036357814 -1.1495429558 1.33425899287
3.49578165443 -0.956583640378 1.06572734254
3.63220443108 -1.66551831275 0.769587659758
3.8243059897 -1.83834562922 0.524093659594
2.83373809311 -2.02123105324 0.504187192574
2.47333470054 -3.10028066689 0.482261186844
4.05076045738 -1.36246132917 0.70364674687
3.43568504549 -1.50417185578 0.626733842296
4.14342161353 -1.30904926801 1.23991397958
3.10575389095 -1.91719785608 0.750434620622
3.02371368067 -2.11953838081 0.580750360918
1.87755531976 -2.72893489252 0.713030757128
1.93474518296 -3.26134715459 0.556964322663
4.48416088448 -1.32805103343 0.803160443753
4.51931558795 -0.685735496096 1.60991987449
4.43445191864 -1.01951463056 1.35036058654
3.79186944931 -0.954619740559 1.38612559716
4.57802257692 -0.602919287548 1.5994459986
2.13142580547 -3.11698165432 0.588423570917
3.79863731586 -1.83672340421 0.490796175322
3.03005218858 -2.29008132812 0.439284797584
3.33706925925 -1.27335890629 0.93077029538
3.67241608005 -1.85984904536 0.891105864644
2.84221198632 -1.93699133031 0.923045917644
4.50578890427 -1.18034644212 1.23199393278
4.28815088505 -1.15117475799 1.22069745551
2.78097650233 -2.44050501453 0.66269187547
3.28265101998 -2.34102973821 0.642214072628
4.24761232169 -1.37959366361 0.868997995951
2.11355791121 -2.14239950615 0.572596630014
3.64045094874 -1.38450464809 0.839880411566
3.00887393848 -1.35959711753 1.05727663102
3.444144199 -1.65189436203 0.647675915936
3.88518874771 -1.52194447659 1.08046123454
2.38748063228 -2.8334915939 0.661835464719
2.93820585711 -1.92920987582 0.731846215219
2.89480250779 -1.36820374389 0.914728063918
4.40367413099 -0.563140680575 1.71647123431
3.42376209725 -1.47979396954 0.919860489408
4.11336118001 -0.945868417909 1.17103729509
3.3406972616 -1.51834846809 0.6367257134
2.82004659039 -1.64948452285 0.734728177718
1.8927483521 -2.3674277945 0.662872031267
4.01088012828 -1.96224483757 0.79170486163
2.57664972638 -2.63014083505 0.644403685381
3.86020159344 -1.04048225008 1.30633055516
4.11237357805 -0.709952582312 1.63098564446
2.13554292215 -1.95953290137 0.897760253348
1.91506313809 -3.19696869688 0.52322804518
1.96697087368 -2.14434549769 0.854539158431
4.34313828662 -0.436114599377 1.49288034817
4.29657036047 -1.30427217313 1.19486565806
4.50599405669 -1.07290305248 1.33088434025
1.98468342457 -2.68266861485 0.352746259223
3.44021216039 -2.13564215341 0.499511666314
1.7068808853 -2.94890165608 0.600356550466
2.86356063928 -1.7896383286 0.786605070897
4.95492951049 -1.3269508117 1.11730339578
1.36474725622 -2.53114537464 0.351552981517
4.96668335363 -1.20447302315 1.24318164271
2.33583760956 -2.42167369272 0.478167707779
3.99785449944 -0.941070647446 1.23903593986
3.41915365448 -1.28900132769 1.07914518156
3.95533279952 -1.81797485644 0.807300266657
2.34200759618 -2.85326472243 0.582296770995
2.55646214348 -2.88609447062 0.605879532348
4.68023863423 -0.777073160874 1.46872707754
4.24602714276 -1.00920145384 1.33585453
2.52452600668 -2.11053250516 0.770353464366
3.85945826113 -0.997374793227 1.42005039368
2.79032308298 -1.83505721817 0.64578673484
1.94686750788 -2.80847170699 0.587667032646
4.61820094034 -0.913134175763 1.05162945745
2.00576374085 -3.26769363723 0.287901519793
1.83764022354 -2.02433733427 0.497507499606
3.20432548055 -1.70584870414 0.795135121633
4.41531639168 -1.4111451866 0.771312979976
1.69200756984 -2.69819062068 0.599484704771
2.08451610593 -2.31204515357 0.626446546726
3.23762859367 -1.5227189964 0.833782807467
4.80477280166 -1.13253710319 1.29323021224
1.99674487414 -2.69120263585 0.563014671997
4.51029419455 -1.06187899258 1.34288827876
3.54271983975 -1.63051037921 0.619270378669
2.01300879611 -2.7249531482 0.618597730251
3.43884606257 -1.99941125019 0.646502871967
2.51372564682 -1.86438137653 0.478528225514
1.66148290451 -2.27241450743 0.498716905054
4.10834355795 -0.872534276514 0.983700264729
3.55134128906 -2.22105980578 0.541229236619
1.75624597789 -2.39562477362 0.475018695194
2.37491404669 -3.09605366855 0.333318884603
2.41062713807 -2.96311473308 0.435922476468
3.94908133424 -1.66756351414 0.714563242588
4.56014146301 -1.17690661376 1.25090421526
2.13667901814 -2.23081088298 0.454890258213
3.71846224657 -1.9445883736 0.46570146292
4.50970035308 -1.50658795544 0.904043937666
3.99384510256 -0.650383953207 1.22843595453
4.41368854474 -0.384850334297 1.4710926862
1.40928392904 -2.60447581163 0.783834194876
4.25878112999 -0.484435220067 1.26705157336
2.24137596679 -2.24759125627 0.663246946437
4.66765635379 -1.14079422184 0.998902284106
4.37959573258 -0.99032306778 1.24578317876
4.32418066601 -0.995849443515 1.28973277707
4.89831492183 -0.220752777347 1.88212394906
4.90070338757 -0.219083433591 1.88283705694
4.9030918533 -0.217414089835 1.88355016482
4.90548031904 -0.21574474608 1.8842632727
4.90786878478 -0.214075402324 1.88497638058
4.91025725051 -0.212406058568 1.88568948846
4.91264571625 -0.210736714813 1.88640259634
4.91503418199 -0.209067371057 1.88711570421
4.91742264773 -0.207398027301 1.88782881209
4.91981111346 -0.205728683546 1.88854191997
4.9221995792 -0.20405933979 1.88925502785
4.92458804494 -0.202389996034 1.88996813573
4.92697651068 -0.200720652279 1.89068124361
4.92936497641 -0.199051308523 1.89139435149
4.93175344215 -0.197381964767 1.89210745937
4.93414190789 -0.195712621012 1.89282056725
4.93653037362 -0.194043277256 1.89353367513
4.93891883936 -0.1923739335 1.89424678301
4.9413073051 -0.190704589745 1.89495989089
4.94369577084 -0.189035245989 1.89567299877
4.94608423657 -0.187365902233 1.89638610665
4.94847270231 -0.185696558478 1.89709921452
4.95086116805 -0.184027214722 1.8978123224
4.95324963379 -0.182357870966 1.89852543028
4.95563809952 -0.180688527211 1.89923853816
4.95802656526 -0.179019183455 1.89995164604
4.960415031 -0.177349839699 1.90066475392
4.96280349673 -0.175680495944 1.9013778618
4.96519196247 -0.174011152188 1.90209096968
4.96758042821 -0.172341808433 1.90280407756
4.96996889395 -0.170672464677 1.90351718544
4.97235735968 -0.169003120921 1.90423029332
4.97474582542 -0.167333777166 1.9049434012
4.97713429116 -0.16566443341 1.90565650908
4.97952275689 -0.163995089654 1.90636961696
4.98191122263 -0.162325745899 1.90708272483
4.98429968837 -0.160656402143 1.90779583271
4.98668815411 -0.158987058387 1.90850894059
4.98907661984 -0.157317714632 1.90922204847
4.99146508558 -0.155648370876 1.90993515635
4.99385355132 -0.15397902712 1.91064826423
4.99624201706 -0.152309683365 1.91136137211
4.99863048279 -0.150640339609 1.91207447999
5.00101894853 -0.148970995853 1.91278758787
5.00340741427 -0.147301652098 1.91350069575
'@

Write-BundleFile 'samples/sample_B_rigid.pcd' @'
# .PCD v0.7
VERSION 0.7
FIELDS x y z
SIZE 8 8 8
TYPE F F F
COUNT 1 1 1
WIDTH 1400
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 1400
DATA ascii
3.74836900334 -1.90899106624 0.508636721836
4.35363512635 -0.575658122448 1.39760304
3.3060218853 -1.8214945019 0.738118920411
2.9751363128 -1.83146260104 0.561277456415
4.40296428359 -1.19537305077 1.1790059711
1.65774608127 -2.25794317606 0.590829110545
2.50841974276 -2.64258396088 0.554116158437
2.9981071272 -2.17743506407 0.544599013745
4.63703901248 -0.642345447898 1.37501432608
4.04583830287 -1.75456240314 0.568734414286
2.25083499639 -2.52374449344 0.759684210713
3.3686568289 -1.60033838285 0.892124827038
4.357372882 -1.17203308222 0.820887961624
4.36777982934 -1.3118303555 1.24982842516
4.46405370922 -0.754571274836 1.48189082095
4.13689705578 -1.60718700772 0.857623599171
3.23531437764 -1.70713105601 0.864110407288
1.89440500273 -2.93951780982 0.270103969381
3.33675032376 -2.42569845689 0.727996805665
1.89199914553 -2.88719427887 0.222837324329
4.17470076383 -1.25867694862 1.08674929391
4.33388813633 -0.608323631049 1.33990280951
1.84416158722 -2.86108923088 0.258810039886
4.04117194806 -0.997093052475 1.04635074042
2.68784520345 -1.83912072178 1.0040509583
1.87377204244 -2.96509512265 0.630438046538
2.76415034502 -2.02902361726 0.863561066592
2.92689497643 -2.27597333889 0.723913901558
3.87222334454 -0.813787571373 1.49963854944
3.27581533356 -1.8060660654 0.782218317492
3.17930541359 -2.25966460887 0.401833649639
3.18962422277 -1.64911708464 0.850516950056
4.75518978187 -1.15185807227 1.21211726678
4.64915372147 -0.916195001058 1.50829133209
2.42266468051 -2.78382253545 0.603910866313
3.49766062361 -1.48490249595 1.06109650147
4.67974434591 -0.951557767908 1.04807651608
3.08875108167 -1.63413650426 0.57045256151
3.79927649636 -1.91651915752 0.517748302794
3.46685747737 -1.12682986222 0.946433149058
3.24012932579 -2.32350045459 0.418835661987
2.53842677018 -1.96484360145 0.576010503615
3.49024313999 -1.03702394741 1.20484644684
2.03006205045 -2.46273471269 0.458655089708
3.9576002577 -0.728700500527 1.37796018083
3.44716116428 -1.04800710219 1.01910010576
2.46761773294 -2.59144596632 0.366490959829
1.55920945633 -2.52093775142 0.473765135089
1.79103502279 -2.6829665863 0.552547022305
2.32643062768 -1.908743811 0.47066825255
2.85546879002 -1.88937226205 0.608638552143
4.08273998872 -0.80285442905 1.10800664416
2.75285817499 -1.55578335651 0.808398873891
4.09151018372 -1.04964861289 1.42331685162
1.91122615384 -2.71557481902 0.337624621362
2.97047697448 -2.70614515377 0.645254851877
3.97891441969 -1.12086170433 1.02849225909
2.30980281876 -1.89583618158 0.480881520659
4.28642218949 -1.60386001995 1.02368463047
2.31574425731 -2.54250496782 0.376866509496
4.22860988726 -0.916397738777 1.01513026008
2.06143589876 -2.37050408553 0.775760510002
3.51611948418 -1.0722826931 1.41510550503
3.29245237816 -1.12479134299 0.950830022503
3.35945413111 -2.29500771427 0.645539232358
3.71080409902 -1.05561928992 1.12869718837
3.7923867534 -1.53637930959 0.893313657105
2.9093842458 -2.42161474839 0.254834157284
2.93606152058 -1.95160325036 0.871909500357
4.21710000894 -1.60483140529 0.622988692994
4.96903300863 -1.06878164607 0.885803421463
3.20397269639 -1.12405907696 0.884466784782
2.43528369544 -1.72429565335 0.55760416403
1.86773254425 -2.06838163011 0.686993006691
4.10927730564 -0.636663154489 1.24214361515
2.24921341021 -2.79921257144 0.414347586142
3.45827301052 -2.22698443825 0.539969802564
2.30560947456 -3.06527501197 0.457980015619
1.93811748255 -2.89966194244 0.337487180365
3.10793685254 -1.91378317653 0.671464045849
3.04301343359 -2.65368101696 0.588630754305
4.14724727541 -1.72892358659 0.691383769735
3.71769253201 -1.95828833842 0.726718013071
3.90080775258 -0.954795519739 1.1234395011
1.8914577697 -2.78923018855 0.422181513255
3.63082359965 -1.08862872217 1.27958436006
4.03352598265 -0.913807086987 0.999443244223
2.40372730934 -2.60469345629 0.495688058581
4.4590246947 -1.54284123981 0.932974838745
2.89846594186 -1.55754852 0.60973628422
2.54419367947 -2.85565894669 0.492909865262
4.11049651164 -0.814443170144 1.30458890696
3.18478571155 -1.29235736598 0.73914841434
3.04874455729 -1.8848377267 0.450540770372
1.57272083158 -2.9460071879 0.660213460678
3.75761066297 -1.04618710471 1.09175448092
1.84217380057 -3.15814619878 0.415011837071
4.36147282458 -1.42503307967 1.21058555763
2.85709808997 -1.45085509102 1.02681128661
3.41933853629 -1.0298034793 1.09416884855
3.86141303686 -1.7501656424 0.82656034232
1.71003868541 -2.38033565083 0.852639522023
2.65021197106 -1.65820418402 0.665454772328
3.92620106554 -0.611223833549 1.23712668734
4.72499984762 -0.71553393233 1.46276563238
4.25422496961 -0.840665532526 1.51520223503
2.19325788588 -2.74188872606 0.407309432988
4.15574619536 -1.6622933043 0.969251182267
3.35606476018 -2.03920331588 0.525881497063
2.15795565525 -3.29617474844 0.344421049978
2.76233966075 -1.60365807476 0.907697226917
2.45977420318 -2.7752567117 0.539055515492
3.87389166246 -1.15560425984 1.09541396799
2.16482603983 -2.02034369405 0.639710454661
2.87684216384 -2.3273030992 0.345627283193
2.84910009401 -2.46734046629 0.35216405313
1.99076190616 -2.53356439013 0.760368641852
4.32719929185 -1.05734943145 1.31005962286
3.71560421342 -0.76141937086 1.16435405198
4.31136193568 -1.01733163768 1.41749817314
4.86963192529 -1.21473723754 1.1952411885
4.61094878329 -1.04285901959 1.35314388049
1.57438211268 -2.74211325102 0.324186015881
4.56713449987 -0.800606971217 1.63905864868
1.8351638331 -3.30139153304 0.426106132971
2.60903908494 -1.71547536191 0.551412825577
4.64513338257 -0.899320042441 1.26597336046
1.64195760221 -2.62524165348 0.374190272176
3.51074581584 -1.37535694942 0.76202288164
1.56465122064 -2.49870422608 0.394968494599
3.47631682918 -1.5618469576 1.06501507695
4.02255580432 -0.835034173632 1.13868756708
2.96633946198 -2.69809249706 0.517121368835
2.87222558296 -1.52244110055 1.16068658574
2.16484986227 -2.53161096701 0.494512619867
3.00770218387 -1.40711692684 0.980539773771
4.06346316108 -1.78742429546 0.693561430816
3.89890569883 -1.05751345504 1.19077216414
3.94075915468 -0.611556532311 1.3001495239
1.82071594033 -2.57426390693 0.622174212055
3.83175024436 -1.53643759035 0.648428676555
2.40466758937 -2.68770942345 0.588204804926
3.51827539618 -1.25478122289 0.817923482024
2.8044482868 -1.41808015475 0.803324864584
2.98932239326 -1.54560334769 0.835867767517
3.90832248093 -1.5107247147 0.94318407219
3.31941159039 -2.0775520133 0.666055072327
2.93723900599 -1.50579326923 0.79337725632
4.42090955797 -1.04526529081 1.34872692583
1.98259693647 -2.46542207932 0.464391738866
3.91193038559 -1.08075763426 1.10119767841
4.73659249302 -0.953300127196 1.416637889
2.77182025378 -2.55380883099 0.395589247921
4.40417143118 -1.64089647545 1.023441793
1.87695015949 -2.94822364584 0.338414360162
1.73864442436 -2.12200256218 0.476882884622
1.55512049687 -2.33856699662 0.521291569651
4.74284417923 -1.09480964136 1.34604375303
2.1985666662 -2.22475576423 0.377074987802
4.04962364704 -0.790074896062 1.14060464646
3.71859246674 -0.914660348765 0.945888826629
2.92880198248 -2.06872693183 0.430131781807
3.87776661583 -1.79688492828 0.80142293903
2.25796403572 -2.83136512307 0.584270725467
1.71395789854 -2.52546386299 0.561549310384
2.23409310187 -3.05895711254 0.525360017284
2.59779133528 -2.56216418939 0.571992948377
2.93974674445 -1.38242434324 0.864914849022
4.56643011904 -1.12339846496 1.28632148989
2.02331988175 -2.30735161826 0.338545462937
2.4356190826 -2.3132196388 0.582728184806
3.65092712991 -1.93388801337 0.719963650472
2.76541655696 -2.26708312245 0.782213894927
3.8558024082 -1.28297160729 1.03645216798
3.89460902267 -0.93739748798 1.10172786668
4.01019040084 -0.76790617175 1.42829726743
3.46862276974 -1.00316980348 1.00507380899
2.65857912928 -1.97408154723 0.682364145241
4.45032971373 -0.956050593119 1.24434381041
3.02088894583 -1.63272727924 0.591368363685
2.36759545204 -2.43516344431 0.540549140518
2.87684712702 -2.48196518821 0.536913551134
3.76770835276 -1.6234051141 0.679390263591
1.98815766697 -2.40230895428 0.382255211635
3.50559740726 -1.29244241565 1.27396513256
4.71975359451 -1.13520931839 1.31066568946
3.45086959963 -0.923058566843 1.09325266759
3.53913773034 -1.24701132219 0.85124898356
2.74502303554 -1.78095946222 0.505070692889
3.13595526031 -2.22485548818 0.314100729372
4.30301699091 -1.26914829872 1.31547260384
2.42658164088 -2.93876096751 0.343833716903
1.97583016305 -3.38786972012 0.366809884216
2.25967041201 -2.36251045074 0.692176607278
4.01897704397 -1.48157656753 0.632351383984
3.70505314162 -1.26172124158 0.762694498574
3.17938699295 -1.50702529034 1.01108432126
2.56307047676 -2.72722686474 0.350604921243
4.35297320424 -1.14138313572 1.25458537115
4.05235785224 -1.37094615851 1.16627412455
3.09951675081 -1.38749494945 0.950158513078
2.69899648442 -2.7115762549 0.537343999905
1.77737546927 -2.53904476472 0.628798463857
1.66637950051 -2.38327034926 0.382871114883
3.80413926122 -0.780645900262 1.1185962195
3.48735576116 -2.02217244393 0.387124663177
3.03486477599 -2.50344423731 0.32121161677
2.18863088615 -2.32074648678 0.760363766527
1.51989634 -2.47384904485 0.64829217397
4.42601405929 -0.43620602154 1.70503614857
2.20083292185 -2.50369356055 0.761890707711
4.50240148551 -1.42633181151 0.967066101521
4.41210571108 -0.679996456497 1.24165663477
3.73360630419 -1.12390355473 1.11694142754
3.3128326102 -2.26862795068 0.494395348616
3.87227435764 -1.2680308713 0.750714131514
2.55831797967 -1.9704468333 0.867595599569
3.22578939004 -1.36473088717 0.919982447912
4.58588220851 -1.13736465419 1.2548908541
3.56658596329 -1.04071530349 1.1077976066
2.93583776653 -2.43590841414 0.544393393966
2.16571840074 -2.63836864517 0.529175369791
3.60623498739 -2.18571532269 0.634978324261
4.54081270293 -1.32020376193 0.978259648702
1.46388122734 -2.34588904904 0.482287578909
4.65033963353 -1.16557204774 1.27104691692
4.29901905456 -1.75805845267 0.980224586824
3.88194792456 -0.936708324779 1.11791832586
2.41623679654 -2.71035688831 0.303545745124
4.72177824816 -1.28281694627 1.07301092706
3.97635053087 -1.75645947132 0.95517709871
4.03448400385 -0.568068158707 1.32469136828
2.76077937017 -1.55964556622 1.13253435833
4.34211667098 -1.22024103258 1.1225465249
2.752482275 -2.24163728823 0.669091938242
2.90679951386 -2.40876938752 0.771713547869
4.62702793154 -0.589631093204 1.44946245853
2.00165612089 -2.69995090291 0.366448237785
1.86930746958 -2.36389296523 0.7734536832
3.64192966147 -0.947477684909 0.963590276683
2.90314940006 -2.46840397756 0.577099369577
3.07219542783 -2.33930135088 0.382237723355
1.69535615099 -2.34685757697 0.626739085907
2.82738238985 -1.95232405689 0.843537682356
4.61243945898 -1.27485815813 0.758279858959
1.88924166009 -3.30416657732 0.50102735575
3.95604510599 -1.3969903695 1.23236513505
4.67251876984 -0.759379369593 1.6421499776
2.23921265758 -2.24955991913 0.435950998866
3.51987464527 -1.84195944333 0.982861499897
3.91719263989 -0.768326374841 1.33351798561
4.55202957094 -1.37094720341 0.906542978039
1.97634245376 -3.28351660286 0.471922842204
3.70362113476 -1.42067101449 0.666882975109
3.30273159059 -1.61442782331 0.715227895414
3.1861347059 -2.55530546858 0.622148409799
3.08093809213 -1.64656234554 1.03750095007
4.39657292355 -1.2904309216 0.785648157263
3.94190395407 -0.994609173831 1.01022823569
4.27493403256 -0.572613578863 1.45813204307
3.77189210661 -1.55463086154 0.59383247254
4.46456431984 -0.441040981735 1.50698386115
3.44526493905 -1.62026646267 0.969685817214
4.19627566013 -0.999237168191 1.14144237002
3.05554180993 -2.46241709419 0.488267824401
3.84545919712 -1.88441381222 0.665912648957
1.77821023313 -2.56541883917 0.569261706348
4.92835700197 -1.09954847644 1.29221022577
4.62385212072 -0.631517798309 1.24805142002
3.45052186401 -1.95229216195 0.881426196597
4.19975705063 -1.5669862911 0.789560350208
2.07881277936 -2.67787090285 0.305999988693
4.10144221249 -1.64340639447 0.848819054354
3.96429550663 -1.31402195005 1.12741049819
3.35716745684 -2.25903622947 0.789023151735
4.56025434521 -1.24836723703 1.13865231582
2.34173924408 -2.87215143142 0.530458956902
3.58668672814 -1.92529098701 0.818849430097
2.4788225365 -1.85026831915 1.01937002456
4.78589071564 -0.992686029933 1.3820990959
2.27113649291 -2.98383925774 0.559054286673
3.04118239615 -2.43109871921 0.239029606758
2.32923120489 -1.81568342636 0.71378371141
4.83400304781 -1.08468809113 1.17052118407
2.07266057218 -2.80964979202 0.652490420917
2.5201175179 -2.93580013743 0.429852747115
2.48279662036 -2.56874925224 0.273414080182
3.37621121007 -1.07735621773 1.11537574073
2.5212342311 -1.95663421746 0.734508332054
3.44466430639 -1.97283547054 0.564743209598
2.00972825038 -3.07828563212 0.331464849384
4.42422252141 -0.479558288629 1.63400166124
4.16148611858 -1.07523197512 1.43715570673
2.25772766686 -3.11952915312 0.454502102489
4.31944130421 -0.489971820636 1.48718897718
3.88878277138 -1.17236124528 1.28756892313
3.61877518359 -1.30489420972 0.923259138389
3.41965124721 -1.68304698227 0.669611947047
2.19868191569 -2.27085503537 0.502534409961
4.18876049171 -1.24896764986 1.12738557586
4.03156809443 -1.58339790483 0.837620573906
3.75525277709 -1.03082709489 1.24352740566
1.99033805062 -3.50951273629 0.476126522822
3.37235847292 -1.30731926107 1.20596263158
3.15839777137 -2.08839661873 0.507799931603
4.51448025419 -1.14325551537 1.17521478446
1.80403182018 -3.19930389871 0.523603924902
2.46556781766 -2.70349595762 0.276299756311
4.7358619571 -0.787220403503 1.33951902446
2.36961278529 -2.35290700151 0.722903851748
4.23621459771 -0.656162981436 1.38355420847
2.85713464087 -1.34743049473 0.885068192598
2.51175578272 -2.27233497858 0.420795042129
3.14213094062 -1.67070175892 0.973451755018
2.50030301755 -1.77192558591 0.893935534362
1.92056557474 -3.31109299966 0.307399043524
4.86450464887 -1.17060982336 0.935964503174
2.78598793442 -2.40318953583 0.602804144618
3.20317935884 -2.47711967889 0.670501222472
2.95873136524 -1.34444571674 0.876379633047
4.72825981086 -0.746188593067 1.16432212166
1.58884517928 -2.22188076535 0.463339864387
1.82078319039 -2.90268933754 0.455796803205
2.00189136242 -2.03829286962 0.44669350529
3.02461591435 -2.39051226002 0.664276303492
3.5318127639 -1.3573671474 0.915348803863
1.93206900362 -2.83538675493 0.296661592329
2.29737402869 -1.85807227757 0.85881231659
3.4111503581 -1.46242961276 0.667588201225
2.5743718463 -2.42143357521 0.750854216719
2.1542516378 -3.29970014665 0.538476121302
2.56148696839 -2.78341020577 0.309785492797
4.15367745799 -1.3649461428 1.12312943219
4.32234254842 -0.64609196102 1.33289293025
2.37155058927 -1.82981779162 0.606754932802
3.576921899 -1.51608762118 1.15443982481
3.93405717051 -1.3269349392 0.783205094999
1.92101071508 -2.83256536569 0.458682092824
3.46545619674 -1.46493021908 0.914910057308
4.49689162637 -1.12583552621 0.936409847083
2.22441181311 -2.76527248035 0.514953379635
4.22834850825 -0.633263383241 1.33592420703
3.59254755278 -1.85337371896 0.987507946342
4.5131734522 -1.34550826942 1.18842595616
2.75728439663 -1.75387289966 0.893980996101
2.25119425689 -2.27032164658 0.551067868612
4.69151086461 -0.683456807663 1.27371623919
2.6852773903 -2.91223660545 0.445081176321
3.04105401207 -1.65527039427 0.840149289344
2.98347277531 -2.53525119979 0.457867369159
3.66741538795 -1.11299015563 1.02838901802
2.49341110604 -2.71079424623 0.196027717883
1.64845054266 -2.39877780263 0.631530370302
1.64699268275 -2.21156852804 0.439234002964
3.50069987081 -0.964701731428 1.15860681731
2.15530407486 -2.93167364439 0.454703139302
4.1966910085 -0.512542928094 1.36343715734
2.6870585569 -2.02315769925 0.438199448888
4.12170946368 -1.27405150669 0.946712839886
3.62941690938 -1.43684127687 0.696076101035
3.01781712692 -1.57490149286 0.652092751829
3.52964330237 -1.28037689932 1.14286391833
1.47408245888 -2.38266562139 0.576514004553
4.85622124018 -1.12231564916 1.10432722333
1.61174628579 -2.4281733398 0.811089522878
4.05804521158 -1.07647680626 1.40552032049
3.09307102505 -1.30344887635 0.952228251675
3.02358526987 -2.03552093459 0.401543715238
4.134876896 -1.00162732141 1.12472963255
4.35900107121 -0.584434781145 1.36715835198
2.76479116473 -2.66525517768 0.446407943996
4.03645471375 -0.798169339815 1.03089592052
4.07411899247 -0.638460692581 1.26099309523
3.23636267359 -1.19247388131 1.25409211538
4.29492609368 -0.865202063703 1.37272414962
2.03280384562 -2.31151984044 0.629019817741
2.18654064233 -2.33030315295 0.385953527195
4.01686754254 -1.93196528835 0.729802522429
2.96560159388 -1.85318864238 0.604677247035
2.76625643691 -1.60438365871 0.811954292446
2.6500773912 -2.78117203966 0.632189206574
2.86586205755 -1.54653348986 0.971118000557
2.76103202925 -2.08703248116 0.520142461305
4.28264700668 -1.29196833712 0.848945761317
4.63071405416 -1.49956378804 1.09776393502
2.9621401204 -2.75919138662 0.60156284375
4.8825201541 -0.904300801423 1.10671923452
4.25694432818 -1.17915365596 1.12176701081
1.42777002589 -2.65810881437 0.70200587945
2.35622322828 -1.89808009425 0.957790154887
2.81173629326 -1.77830746043 0.818555339804
2.70231573266 -1.84192244991 0.813120581141
2.07294780953 -2.91769097652 0.451172646817
4.19904306008 -0.736119232533 1.43773494401
3.40932894864 -2.17304807615 0.732924564636
3.38518338314 -2.30935731279 0.606287380532
3.13838768724 -1.397200876 0.793534644858
4.0889350524 -1.70289192293 0.580815445856
3.69875756366 -1.26388515579 0.742432478475
3.97011840963 -1.29337219476 1.19022870235
1.33687664251 -2.4283899389 0.429508337024
3.61985569261 -2.13567592254 0.730772546839
2.355184216 -2.26945393704 0.742175981553
3.33968560769 -1.11604356662 1.09388932665
2.31391582896 -2.59269214774 0.274652337818
3.08296007963 -1.9426637646 0.720816128566
1.76034742361 -2.60760856314 0.567707014847
2.84818782046 -2.15283197777 0.588865284429
4.63199863672 -1.06183727167 1.46713276206
1.81124856197 -2.4378666005 0.386915054584
3.73472240076 -1.38470211022 1.12896423896
2.09456660417 -3.00042498355 0.339397198832
1.95349911828 -3.27401578669 0.548778415981
2.78808596617 -2.47807086004 0.29055262738
3.75792198744 -1.07483233582 1.29937826824
2.2712416816 -2.91375335727 0.462429842575
2.25537284193 -3.12711307687 0.574477404689
2.88709946532 -1.5545032624 0.890187854495
4.41828521795 -0.795916993703 1.52931866525
3.6094089543 -1.3064194894 0.873879500934
4.45109623501 -1.11257102126 1.31054596583
2.05595373842 -2.09190994585 0.925150407992
1.50959215783 -2.50795789576 0.359847942124
3.59625581315 -2.04246424681 0.499244919986
4.31741423099 -0.820991708094 1.05789347247
2.44319525686 -1.77062165849 0.690389991175
3.47779162058 -1.1650137851 1.16801673101
1.41628396635 -2.6501855605 0.629335607277
3.54254500379 -1.33918875411 0.75532583372
3.64364266252 -0.917512769243 1.40500001914
3.79425620104 -1.00962688685 0.954759094681
2.86431218078 -1.96267478972 0.430446778684
3.9224127217 -1.48929586562 1.1501399511
1.8775497297 -2.47708084446 0.640401277005
3.54080528416 -2.06138006146 0.811189282989
2.07504864616 -2.51077463996 0.550710289392
4.0397527641 -1.19584744417 0.798130499459
3.29556502684 -1.58017833068 1.03694800952
1.77643256452 -2.69636265467 0.366298650359
4.92364128801 -0.92220845583 1.13843330844
3.54326088724 -1.26948730773 0.934287106761
3.82339667468 -2.01207864666 0.717276353625
2.62287699863 -2.51733766873 0.436012684377
3.79503000014 -1.96997125762 0.478902282605
1.82683887668 -3.00114730497 0.346750442587
3.0861458218 -1.8173850197 0.909306467968
4.78035621014 -0.841376888306 1.3745918311
2.63501308337 -2.76693045943 0.439273573674
2.48218123369 -2.15625307585 0.569355428043
1.44484530233 -2.8024936642 0.675334447622
3.43781767092 -1.09529939826 1.00029415144
3.45011372173 -1.88603779362 0.625520289698
4.05712963632 -0.670093181784 1.40416788096
4.28624270544 -0.972721519068 0.985242850887
3.94788950929 -1.41726267274 0.95875087985
2.19786877913 -2.81241526393 0.64369731525
4.5554615265 -1.119771898 1.1951101753
2.08776103361 -3.12509476156 0.315481684233
3.2222983877 -2.07810434655 0.782945215361
4.41908518842 -0.965689319276 0.983234767948
3.19695134127 -2.32397566461 0.650833735309
1.87588962155 -2.53760859853 0.553132098075
2.13194557331 -2.18637033567 0.434197725614
3.39824344916 -1.9566718953 0.484139542188
4.18833483519 -1.72588417159 0.641350752134
2.27272673159 -2.23142018614 0.685162797307
3.77986021231 -1.24386866123 1.06528223588
1.79863782336 -2.55024850572 0.795708428953
3.23051674108 -2.34354836997 0.597754871909
2.62657841924 -2.45868931006 0.24787754525
2.37777746881 -2.55574961723 0.335801334312
1.97208884314 -2.06302934053 0.67667863887
3.69767807934 -1.94604949583 0.705053508976
2.55699071033 -2.32957909046 0.797391487708
3.50068839746 -1.79838650121 0.897534404864
2.77535002274 -2.46278912036 0.751287254337
3.01610205001 -2.52768661169 0.498867812646
4.28905566296 -0.897208630351 1.16059375397
3.22879615718 -1.51541647289 0.867147822903
4.30094477854 -1.30956121983 0.875146893372
2.92433485754 -1.4359379173 1.16220171826
2.34279424015 -2.97323057417 0.614166329011
3.84275966407 -1.7903812437 0.551688609779
3.92141441907 -1.58243981656 0.592053458078
4.23307365796 -0.620330913898 1.38651727637
3.3757530677 -2.13570238565 0.654327623251
1.8623937699 -2.13959455686 0.837150683859
3.98360531715 -1.35255840785 1.11091757857
2.79168645411 -1.36680819497 0.789766120014
4.08907141282 -0.635489656967 1.41177737298
4.43186892255 -0.6757178201 1.15253768427
4.05085042279 -1.83418449682 0.742035663731
5.07719248529 -1.15386059799 1.26915367604
3.87404633749 -1.30506708387 0.896743079913
2.9954513571 -2.0277544217 0.645284595717
3.81288927831 -1.22059847577 1.2332035224
3.61730613626 -2.01857034217 0.897337785181
3.18146730879 -1.93899485522 0.892278849151
3.90215503937 -1.64144791747 1.01124290322
4.46213409319 -0.93696376634 1.32272944853
2.15435833813 -2.50899123508 0.304332634992
3.10823950129 -1.77709848522 0.488454710977
3.67335576075 -2.00644390781 0.506020002607
3.20442892413 -1.70320036924 0.745551976513
4.38365628664 -1.0206876985 0.99042716975
4.95773428837 -1.27692407881 1.0710964423
4.38637227292 -0.42808332668 1.35303349565
2.79662922594 -2.23481492244 0.735446295425
4.86681178662 -1.37842383974 1.16329150714
4.19345509106 -0.940342742254 1.33964723191
1.89080654912 -2.15158275272 0.800883125193
2.27567819989 -2.10118051146 0.571764878805
1.96725905853 -2.66336711128 0.419359482861
2.89338065343 -1.43003678231 1.00998681301
4.25873609792 -1.6024953974 0.675195893707
2.53995740134 -2.63711593395 0.49159821025
4.59974576216 -1.07665195587 1.41194260646
3.75214192105 -2.10508057506 0.837483863138
4.44692492473 -1.3777326849 0.850044772141
4.03528368478 -1.77174226379 0.822561357649
3.62906226189 -0.800889627778 1.11318224309
3.98774623153 -1.80493641924 0.965288930334
1.51458216135 -2.4993862946 0.365071045316
3.12198566684 -2.01727070145 0.838749279512
4.31979780517 -0.396673312617 1.4575888619
2.16220613396 -2.5971035341 0.655650753425
2.901479914 -2.31774712436 0.403023095347
4.42876447909 -0.31480165882 1.4481747916
3.36191885001 -1.78599524428 0.523821966472
4.06919610456 -1.22711815882 1.11006057033
1.96661931792 -2.75742759057 0.660771519357
1.91576021655 -2.48525300777 0.39769453016
2.28356906984 -2.18309730189 0.825377104576
1.75346528015 -2.66618318028 0.446456991173
4.4726430316 -0.398441750096 1.49850088083
4.03876928578 -0.709261023317 1.26977051094
4.14421760079 -0.658732885898 1.16917856876
3.86186626131 -0.637638427588 1.22970345964
4.67762863056 -0.773697206021 1.61365215543
1.90295940697 -2.53948967213 0.506611127563
3.09597446873 -1.32411113664 1.23497448166
2.38631497207 -2.40799301589 0.459902798745
2.19418629073 -2.1691062953 0.412294044049
4.03882590492 -1.79941064163 0.605379670556
3.19203851694 -1.69337811051 1.06806421372
4.41872851566 -0.829875931802 1.53071056299
3.97963122857 -1.89162536573 0.888483457669
3.70053257041 -1.57636898912 0.976057379369
3.54783993634 -1.47307017988 1.09688165915
3.67940124309 -1.02025901291 1.08527277783
4.0940131214 -1.52482373663 0.730210782905
2.55783133397 -1.71524146628 0.703451469951
3.491176915 -1.12238793939 1.18370649399
3.07548000462 -2.65653915501 0.58025098004
2.57061976621 -2.2542703219 0.781716660514
4.80094204282 -0.801551239677 1.3039483819
2.05109899297 -2.30474021634 0.533407454076
3.91254394764 -1.77376837333 1.02168272521
3.29130887074 -2.10714336659 0.542462594131
3.62290998859 -1.94439091021 0.880447831334
1.90102228965 -2.23985881743 0.509047186068
1.72402252432 -2.63671934925 0.575963681209
2.47288143764 -1.82637281533 0.784173958256
2.4171766151 -1.73482242275 0.613634773205
2.28564307795 -1.76948448447 0.773072335547
3.314291118 -1.39930855603 1.09492253785
1.70454143722 -2.11000575169 0.488866372554
2.1381685615 -3.01297336243 0.526522043725
1.47412399662 -2.48814048049 0.55653255394
2.89105345326 -1.66981840903 0.912884216723
2.35077380991 -1.795615061 0.555173846113
4.48760288391 -1.27560753595 1.10728807892
4.43437116952 -0.4825422884 1.31226263645
3.82822698354 -1.53441314498 1.12834795491
3.34813552215 -1.90155508991 0.920495944198
4.12891685512 -0.681734212036 1.42818985796
3.47200262171 -1.79100414003 0.703414110741
3.94765722857 -0.873980656579 1.51990712109
1.89397809115 -2.80874817886 0.568144942004
4.09547635621 -1.54696968841 0.97612267723
3.30359639412 -1.70617879154 1.06108092486
2.44557746504 -1.72975440793 0.815669024992
3.53820137054 -1.67140889378 0.905441031136
3.6711197263 -1.81892398449 1.00634610189
3.9743349517 -1.26051981307 0.878703808209
4.82724772031 -0.827454129724 1.29973574763
2.76672006297 -2.579482832 0.571053073616
3.74709911615 -0.869604845563 1.24705214494
3.87668538145 -1.03077842112 1.3139151756
2.88101063472 -2.26028818071 0.422157825393
5.04904038013 -1.22716976576 1.10274069172
1.59262073179 -2.51608334029 0.433473416697
4.48667726515 -1.17597311146 0.856417096113
3.43820075399 -2.26424132916 0.629008729347
4.55115399339 -0.838853717968 1.32122385861
3.94505708653 -1.10481228298 1.26348676887
2.73908678453 -2.40471922438 0.406306117725
3.05542397658 -1.90312070602 0.895402091682
3.53415720612 -1.0752834472 0.979123203333
4.79126491238 -1.24589774612 0.959650061443
3.6282600199 -1.75452156773 0.828294779461
2.59468201592 -1.65931075622 0.873950289914
2.47734703437 -2.37385339729 0.654040837958
5.05179102938 -1.05331657804 1.2134904232
2.48490595838 -2.0276003237 0.506594295441
2.31540674343 -2.56173046891 0.741434789041
2.60891802177 -1.71417943152 0.710508633072
3.01436206546 -2.10585458849 0.764908991033
2.94969535061 -2.11723612713 0.39853303209
1.67445015411 -2.62636925737 0.632475571862
2.39047661325 -2.812756059 0.524355143995
2.34900719487 -2.18139905981 0.855412433956
3.19470062695 -1.49012080455 0.881111264879
4.44013460728 -1.24827045652 1.08894048824
1.79816436519 -2.82443024931 0.281164960564
2.74687343274 -2.26228009178 0.406851440571
1.89588745926 -2.37154647934 0.54696671702
3.92036318781 -1.76021474377 0.554573020714
3.25512294267 -1.72742788187 1.05286121156
3.27581697537 -1.34836101482 0.881731408763
3.28257958899 -1.36191090695 0.924461183857
4.12387868538 -0.908645952238 1.1630170809
3.15415901355 -1.7573331831 0.507887279733
3.8469170971 -1.39722293111 1.14362673844
2.32265763151 -2.39803366328 0.433656428106
3.42693731344 -1.92107514501 0.754781537601
4.781230029 -0.92505904372 1.41820451425
4.66732440658 -1.34581848129 0.902233087518
3.76369843674 -1.85895120666 0.579857601866
2.00100802756 -2.21586670541 0.737768896403
2.52927358966 -1.73837259408 0.926936818367
4.32145935531 -1.32469346672 0.78947372051
2.77249124692 -2.24080847864 0.719870629369
1.97762900226 -2.49976031663 0.656652938764
3.78977990892 -1.47087296635 1.09503611693
3.85266879366 -1.71470170345 0.725153007363
2.58244379828 -2.42663121067 0.590320147353
4.21291795114 -1.73593146984 0.936915372545
3.88788815432 -1.15611321041 0.883884281683
1.91311302442 -2.44926225901 0.793808982623
3.02423348716 -1.72458100721 0.682613145986
3.14467479307 -2.21618847329 0.481655374829
4.1427710803 -1.10569025602 1.34263301364
3.15885187303 -1.11898692739 0.93572320514
2.90083577205 -2.29762730328 0.603857562182
2.62536872681 -2.03287510415 0.796464846253
2.32786662364 -2.74661852232 0.37021072975
4.55256707349 -1.2607367754 1.08095196374
2.273193701 -2.07944661635 0.517871047438
2.90002570878 -1.6342995231 0.658267546429
4.56852532013 -0.862959961772 1.50517676399
3.97836441659 -1.94825192203 0.715913900147
4.63097325657 -1.36749078587 0.854624020397
2.32688203383 -2.38388463934 0.685329861889
1.69937254594 -2.30069483841 0.601541862849
4.53964477957 -0.57833580771 1.45365807524
4.4555570831 -0.526415992686 1.30761831573
1.55023775692 -2.5691060369 0.629046093689
3.96682049281 -1.44941818021 0.797222287566
3.77062298914 -0.81766997112 1.01864441302
4.45921447084 -0.803150026653 1.32253846059
4.62887488428 -1.05038546578 1.34476810113
2.48947290821 -2.41957038852 0.530489526496
3.11128692065 -1.98261045225 0.752813792328
1.67721904929 -2.56936230605 0.428658892285
2.85963624301 -1.67059054172 0.871860532191
4.0156050889 -0.892589315407 0.966919675556
3.80553962388 -1.21535395566 1.3325897768
2.73363029059 -1.75991611748 0.67937510957
2.7275397656 -1.70144636436 0.739963267792
5.12758190054 -1.16166364212 1.16902282008
3.26143853319 -1.21273468195 0.812816281031
5.00546150078 -0.981694877227 1.03697167276
1.91215860626 -2.1807559202 0.474998350955
3.19388376652 -2.19264108337 0.739226980445
3.4984334816 -1.13386339378 1.1161954795
3.21284111662 -1.42647350068 1.15386564873
3.47866088848 -1.97065801235 0.777177837966
2.28275991684 -1.92975219426 0.919750157297
1.76797134035 -2.95052878325 0.306945475681
3.51574070386 -2.20965158691 0.692010727978
4.01201865129 -1.83296857865 0.901380635932
4.89535370947 -1.18505970595 1.30232800036
1.48048851816 -2.57909544321 0.675717542132
3.87462782713 -1.5021322622 0.742598361841
4.32833705065 -1.35402224483 0.794258047863
3.19616510798 -2.04280907692 0.810674186544
3.94107391516 -1.32818355126 1.12370915796
2.61388269252 -1.7581385609 0.610336732253
2.34005181027 -2.39980672974 0.672771220152
3.52844367117 -1.7871573984 0.946414638027
2.30657574231 -2.75500834191 0.393656573243
2.5188214452 -2.11455507881 0.788607775215
3.62678610115 -2.08794815899 0.406827773335
3.36459722486 -1.77958801212 0.970224007534
3.66654460411 -1.26497779979 1.15729452351
4.18447559628 -1.42943953684 0.668387872146
3.62103980841 -1.60357206843 0.78207780161
4.28513172683 -0.916185127767 1.00134111102
2.66609419605 -1.78631115993 0.727785981788
3.14300919265 -1.91256090556 0.55760285356
4.07178452579 -1.6944815835 0.938889727026
2.30106960178 -2.22240636306 0.763007591649
4.65395528913 -0.756600316895 1.28114504891
2.66090059528 -2.52914329754 0.61522039319
2.04955068939 -3.16429942327 0.251226560083
4.02892253246 -1.32685334003 0.950310367585
2.38978790576 -2.60708762283 0.557340922306
4.46608266379 -1.35986479835 0.948181554755
3.07066658739 -2.56972797861 0.426883875144
3.65659799955 -1.45833432857 1.1297225931
2.11802965501 -2.13955448198 0.399618715658
2.68484446515 -2.16625117579 0.809570353843
3.88641943208 -1.70465051235 0.688261678459
3.71461843332 -1.52421191487 0.834404862611
3.62239036718 -2.03857692511 0.844481564396
3.98426468845 -1.05076098501 1.06953331694
2.12694074035 -2.2559070687 0.756085986811
3.46346123478 -1.03234615858 1.23269857078
3.87267139268 -1.37873066669 1.14926187919
3.37103559858 -1.33961735432 1.11824830048
4.76503869375 -1.09052730983 0.941584359126
2.97408583458 -1.60076972739 0.941370424824
4.93877549003 -1.0975581802 1.20008573952
2.59912174049 -1.72042756935 0.909863184391
5.03510368763 -1.17176908273 1.01284180943
4.15093979805 -1.56943160168 0.937717690705
4.71012703278 -1.32683616462 0.907379189449
2.77385614073 -2.70994027025 0.18119215233
2.67102068449 -1.52252362053 0.784944659248
2.07915146285 -3.29908136186 0.438944560211
4.62015182142 -0.681032522487 1.3776887055
1.85037769902 -2.43016236753 0.450997707924
3.23452008513 -2.40190990363 0.572085038983
4.65185405302 -0.995974221437 1.25139299331
3.86405006527 -0.959016858971 0.953673171166
2.04437323597 -1.95298528393 0.588215514768
3.91670669838 -1.47611314826 0.723658682987
2.86902399516 -1.34440112362 0.750655393009
3.09107996668 -1.99642552665 0.747893093239
2.19287599783 -3.17959931201 0.292863000771
3.96970627763 -1.68794604043 0.573151895633
3.14700354718 -1.95960091617 0.631254210202
3.95032010676 -0.683064525978 1.18707771822
3.45146859117 -1.95001175656 0.653319659199
3.67890934661 -1.05222679781 0.97730233115
3.64977721173 -1.28583917858 1.26506104951
2.68143053278 -2.07380116789 0.52898439381
4.50109775054 -0.736164399627 1.60687717991
2.2503196067 -2.54843928681 0.528188034964
2.89923776958 -2.71955093201 0.442010005435
2.57291892955 -2.29401981762 0.337794307599
2.83278466257 -2.71770581598 0.28655693841
3.5712229737 -1.01060276659 1.40384683713
2.14959778322 -2.70906301855 0.564598910862
2.56702713748 -2.64102410981 0.538747225894
4.19053887266 -1.69264122148 0.737771540964
3.338864866 -1.20602463023 0.865737732586
3.69419681575 -1.06179308146 1.30020104245
1.77888327289 -3.10234306667 0.50470530826
2.54579369986 -2.56303078155 0.478585363184
3.11603207802 -1.9892236549 0.838437142336
4.42659826944 -0.610817712061 1.67523041991
4.30303512236 -1.03243496604 1.13680835341
2.96067117125 -2.15282314463 0.47684690372
3.41559288974 -1.56877736438 0.769299720988
2.02873905542 -2.53362960449 0.393206934608
3.23747039155 -1.23507147817 1.06484262833
3.59253764553 -1.71322985767 0.775652357594
4.39036660242 -1.60950610657 0.854097444571
2.0353294404 -2.66905847755 0.576417547347
3.18481768934 -2.29260805899 0.703797149392
2.45711174631 -2.88773317394 0.531767303351
1.74738793816 -2.62420811999 0.492613770612
4.59262490581 -0.497280113904 1.40698507977
4.48255361932 -0.45966980893 1.53415860114
2.31628603437 -2.58155712002 0.524686021849
2.38695253251 -2.98015020676 0.201688917714
4.14923936273 -1.35764394037 0.914577460148
3.39920289897 -1.41153415741 0.819593443713
2.26168955554 -2.21422875807 0.406464468651
2.0960707222 -2.84455574304 0.555147393168
4.25546371857 -0.703216329893 1.50441557696
2.48834863643 -2.2502988194 0.641538765121
3.52500928666 -1.67088373027 0.914738686306
3.73663401355 -1.54977407587 0.635113495454
4.03900239781 -1.30231871555 1.1969139137
4.69892664125 -1.39903469489 1.0817587969
3.86909199096 -1.34594541557 0.789562109344
2.86949521262 -1.628390382 0.828890793111
1.90350203578 -3.16642499287 0.123821414329
3.86723373542 -0.899030099434 1.49198722326
2.56998451956 -2.21536227391 0.3776449673
1.95211723778 -3.07201764027 0.32310943705
1.79023879736 -2.68486643274 0.639728645634
3.4755969023 -1.73569417336 0.743596576742
3.26104530604 -2.16476001758 0.839770964087
3.55551091493 -1.18090521623 0.955147593499
2.4285908134 -2.71847669964 0.621037211043
2.71850707776 -1.70390932188 0.976424050241
2.41620869282 -2.96242374229 0.185599797574
4.29231139809 -1.35846865951 0.905975231589
3.36931479996 -1.89403406168 0.583184200447
3.09331356648 -2.25293328513 0.687527135944
2.61234092974 -2.30546732278 0.383534029952
2.93668466846 -2.50285209212 0.220986449544
3.86445211475 -1.89824261942 0.871635491379
3.91063319541 -1.02258408223 1.17964013864
3.45871055547 -1.34117625742 1.089528264
4.29317917989 -0.635809046421 1.59658349339
4.05033649029 -1.36413196351 0.691033270734
2.95131249682 -1.30209357004 0.888560254367
4.28342297878 -0.814515451175 1.55384654745
2.29753016002 -1.94324296859 0.956320255892
4.41057951166 -0.587019381448 1.44562061193
2.40023151633 -1.97478941627 0.880222977694
3.83874420116 -0.72884765082 1.36576227991
2.32072869185 -2.55939367558 0.593354197264
3.78676671881 -1.94096311526 0.611178043191
4.15094748923 -1.35387013462 0.899801457927
4.20384994533 -0.702238302207 1.33487039929
4.21129158616 -0.593854469541 1.2588933392
3.68390866812 -1.20777200553 0.908501475205
2.0156537866 -3.12380420291 0.118061514685
2.88997396396 -1.99315794982 0.652252343765
3.09194604528 -1.94591498473 0.613481941958
4.64019158257 -1.1215474551 0.950900984742
3.20073455038 -1.57863882856 0.787671608377
4.01765362447 -1.10965030137 0.841982496331
3.61243064015 -1.72367262743 0.541552811377
2.70098544742 -1.56383792072 0.869393373837
3.13257397079 -1.60896963507 1.02337687085
3.91900361315 -0.849359518737 1.08812771742
2.67615290043 -2.3500756891 0.284422881694
4.70691854942 -0.839386397908 1.12561475109
3.31890208661 -1.7112972125 0.853876367451
2.69858921276 -1.88351955599 0.662783736422
3.80519427545 -2.1158655828 0.797816252406
3.79567847058 -1.5107562586 0.786513947756
2.21850327597 -2.26109745045 0.747642012021
1.72249736108 -2.62399007193 0.472400189334
2.89618258536 -2.23382197701 0.577225942608
3.08088538389 -2.43988349917 0.287701073999
3.15382453012 -2.35653622324 0.557272941092
3.38945606118 -2.00306527284 0.846158738381
4.1568999098 -0.925256585674 1.50247338022
2.31795713588 -2.54820974441 0.699737114496
4.19035770416 -1.59501867036 0.932052769136
2.74725066291 -1.62138535838 0.649881440335
3.12344527228 -2.27568142671 0.782364393316
4.70761697817 -1.45633583705 0.958056338058
3.58114890958 -1.53343093619 0.800572206162
2.45947652544 -3.08284360634 0.476529112569
4.31059811827 -0.688306295305 1.68198557619
1.66281218302 -2.93522618271 0.299392092755
3.47273495014 -2.25779705387 0.681633642925
3.37893845611 -1.44047710303 1.08996382747
2.94920926348 -2.5368108824 0.483378596216
1.87784723193 -2.93366300084 0.680567181633
3.73014486659 -0.796496856211 1.08869822699
4.22874691601 -1.07318938761 0.891927303289
2.1263821846 -2.22791284259 0.443469350327
3.05464204077 -1.20333298498 0.928744653388
3.17525940556 -2.36866322584 0.518750652758
2.68831309619 -2.30167562433 0.639631271132
4.50539574431 -0.948722934242 1.44914716624
3.05511263166 -2.43182851266 0.459994103038
3.04477693048 -2.0136142729 0.390956565747
1.83823385284 -2.12263862536 0.712744214065
4.23924023139 -0.794106960376 1.0874805184
3.68473833619 -1.46565569165 1.02219376135
2.00302379378 -2.2100711969 0.404584014511
1.52737955925 -2.49592203572 0.475651428577
3.405712741 -1.23345461919 0.770590931139
4.7835003939 -1.00414661391 1.12585895681
3.74570031667 -1.7232640832 0.657905046681
2.71476885502 -2.12358073731 0.374864038685
1.82729928301 -2.80842209644 0.221624460774
2.11430643772 -2.50605062722 0.687882111588
1.84154118566 -2.4267984628 0.432782786142
1.69747671692 -3.0049342735 0.248132506642
2.33519928996 -2.37007059592 0.494497089006
5.0154423883 -1.12159950808 1.02624868707
4.9768960988 -1.06626458749 1.27813279909
1.69040248058 -2.6780752088 0.711208706403
2.97992051972 -1.90246871455 0.899758299333
3.57413719699 -1.55440581238 0.896754314953
4.53967779545 -1.11482358373 1.19023393143
3.2048730493 -2.02298514018 0.516549470071
4.04688627485 -1.48437603173 1.00727824111
3.82358301106 -1.59844611756 0.598977749088
2.25334449123 -1.88829622454 0.889893261242
1.76409771441 -2.28703241966 0.535466125748
2.72076810878 -1.61412405829 0.742797497387
2.5372990602 -2.96158722631 0.501139210063
1.50790540287 -2.7852971383 0.317803873534
2.36809499188 -2.09608628536 0.570250406686
2.44245787111 -2.15868616674 0.711250763156
3.99264276582 -1.78757429848 0.626770448516
2.00199773332 -3.12126621839 0.286413135686
4.02500938253 -1.9569324406 0.887051075327
2.69258225608 -2.74972656884 0.590212220566
2.03867691482 -3.17717274368 0.252573458835
3.01472840897 -1.39916327636 0.868063126206
2.84105722335 -1.63772269451 0.809443341047
3.67490779879 -2.10641252819 0.49173914872
4.13697047619 -1.34493803467 0.77326399674
3.71375334867 -1.69111648368 0.709493627784
3.69960034186 -1.91397598109 0.855320522458
2.50230824029 -2.72362973823 0.538965032701
2.15442668563 -1.98979547599 0.632654574798
3.85689137489 -1.55290566885 1.1134274887
1.89712831842 -2.71398693608 0.413625094632
3.14891411353 -1.8961647857 0.619969889257
4.32125112387 -0.645549528813 1.60550410122
2.95697069073 -2.01671617312 0.616514880237
1.88747910543 -3.04009649521 0.163172803868
3.01210178699 -2.35435584148 0.342214588653
4.45413139768 -0.789026764378 1.60241730403
2.69898339574 -1.64271466823 0.765661639795
4.38659629451 -1.47050107123 0.642966010892
4.24523664925 -1.17667441509 1.02760658201
4.89800272838 -1.07032311814 0.941663799107
2.77819684945 -1.57619866321 0.885559230343
4.01484667802 -1.61332312351 0.833131530393
1.83060483309 -2.08352627917 0.662499465453
3.98857521848 -1.20097729867 1.16020945185
2.18470643475 -2.99680918652 0.412433142054
2.32521294595 -2.9159922078 0.365707781435
4.52808680373 -0.646817164162 1.6636346832
1.96806382722 -2.46407616177 0.480808677992
4.09661398011 -0.970547799714 0.919205645949
4.16040562159 -0.859534267425 1.16949482033
3.93512914329 -0.743059931039 1.56094771563
3.37750818211 -1.83904819867 0.851425736019
3.58370087261 -2.08565757309 0.549595900169
3.70847916627 -1.58433897575 1.07533580138
4.42335268683 -0.762376203399 1.51902435704
3.10273215877 -2.15813089954 0.370202909037
3.97933792531 -0.688155679502 1.4752825542
3.43409424043 -1.19468460989 1.15265995621
2.67350678614 -1.74151703228 0.528236990772
2.6406117755 -2.64330015401 0.396992882403
3.67628903605 -1.22052374516 0.811173113501
2.66223896961 -2.72113444133 0.53626396289
1.83245713471 -2.60607963441 0.380448441668
3.35190765769 -1.24097958893 0.756810482567
3.68293022319 -1.93973661268 0.73551781646
4.12295035833 -1.45302891921 0.840824773912
4.3640513243 -0.904956037253 1.28163623203
1.59035933953 -2.67256299152 0.329220371983
2.83230771904 -2.67490139958 0.427976818697
1.82905386189 -2.81373027475 0.414533458864
2.52266198066 -1.74258273594 0.950522742296
2.41919542709 -3.00319252721 0.390413595888
2.51827787765 -2.65563881304 0.243063686474
2.84080956759 -2.34118712195 0.523401359445
3.64848399196 -1.56912337225 0.895679581418
3.25281048505 -1.98148637298 0.830952379393
4.34177559424 -1.23819560038 1.11100649649
3.84438925002 -1.18127527696 1.32303510859
3.72310642329 -1.00676368041 1.26210179413
3.05663102862 -1.3781697093 0.871310954501
4.64514743683 -1.08100298607 0.984455412807
4.49918268013 -1.38801803315 0.99902386836
3.08887462582 -1.94448343576 0.915703763909
3.01674890729 -1.7223550077 0.795937899683
1.98276349866 -3.09920754778 0.270822139882
2.55747653653 -2.83990503169 0.433335982971
4.19096680025 -1.5306205801 0.994868114715
2.44591818776 -1.68882960274 0.900216041888
1.91919251043 -2.5790385416 0.748244736273
4.66362221419 -0.945270780792 1.14815771396
3.3652608659 -2.42224316285 0.626671662321
3.67476077836 -1.52208475317 0.94355883944
4.81724722834 -0.983913657335 0.943894827441
3.14059042378 -2.44223356319 0.500896019369
4.07566750823 -1.77219494051 0.50682780375
1.78438141493 -2.33539304396 0.594097177937
4.74844943339 -1.33304853267 1.04780810585
4.3195985336 -0.722482710784 1.53130357324
2.46234400018 -2.59546374637 0.705135084915
2.7317452275 -2.62960992939 0.512597035071
2.79764584986 -1.75486177932 0.896190451656
3.3676689419 -1.85963301064 0.83303860728
3.81706291742 -0.895741697301 1.22430000519
2.85315667354 -2.00902232809 0.518999364524
3.88941285621 -1.52370837154 1.1589809327
3.13762109963 -2.19217961037 0.768827038787
2.59833075394 -2.31491640504 0.461904958778
2.15573142477 -3.03300919703 0.174767783253
4.53405763346 -0.788738369462 1.60921387596
3.41815230013 -1.25516709745 0.777420199983
4.25700728684 -0.703542517695 1.38905851093
2.02245866112 -1.99913767372 0.760312810603
4.90595714644 -1.02755118818 1.1752468111
2.25453992095 -3.04962566949 0.358213681347
2.55022813426 -2.15264416468 0.595059563127
2.87447123125 -1.4396825245 1.06131268071
2.37184482894 -2.10649782847 0.805039602173
3.91769555955 -0.974096814642 1.15470338156
4.56374537208 -0.814715299827 1.04421451434
4.61284500732 -0.906092597735 1.17619807962
4.42326910909 -0.781741385445 1.36470698512
2.81234016734 -1.93369782828 0.799235658802
3.74752819474 -1.5755550647 1.10972389499
4.2894770517 -1.17550793983 0.94883738834
2.82458020367 -1.84947261706 0.755264017891
2.5633066158 -2.41204175897 0.780105364666
2.07127256263 -1.8681694656 0.619867094617
3.68123260457 -1.90969233943 0.927801274708
1.86977522445 -2.49550672032 0.307147997668
3.47735975452 -1.09212514351 1.40233954988
2.73853967541 -2.44949092818 0.670907092881
2.42310792314 -2.27875527036 0.394171516647
4.3673892981 -1.21644493384 0.933722657724
2.88048610388 -2.68041603987 0.673900352266
4.99460633337 -1.19100825339 1.08439511272
3.89799314011 -1.27955856808 0.956780767943
3.0911870161 -1.7347043166 0.902287832217
3.76894909702 -1.51420486812 0.827564052562
3.52323093122 -2.01199824999 0.824962102553
3.58222058233 -1.14473585949 1.33025035444
2.159786816 -2.77241778277 0.196140759436
1.5026238104 -2.73545870424 0.664599037698
4.89560535804 -1.1864515317 1.26749112875
3.33292173106 -1.37922092007 0.708786403031
3.58112924706 -1.8632597916 0.926360779127
1.70373121376 -2.56817159935 0.580216970874
4.32067195216 -0.648579884135 1.28632000591
4.69038932636 -0.615414650815 1.38130228371
4.65021074604 -1.23778732039 1.13199122381
2.66981075095 -1.99498018198 0.782340346198
4.63531410764 -1.56495172065 1.10778247137
3.89258698389 -1.59470655553 1.0962429231
2.03087981845 -3.29399074577 0.433965593217
4.04017762858 -1.79170991159 0.959265381921
4.37992369937 -1.54675787309 0.580580031046
2.27231848475 -1.93164424109 0.622203511024
2.3658456458 -2.88271315561 0.611262202691
2.56455311278 -1.90481307276 0.73770027201
3.75818121182 -1.58849627425 0.954104635252
2.48923554095 -2.00820287159 0.66418425012
4.9313273367 -1.07303441968 0.900352500641
3.42931484205 -1.24766969699 1.26431014405
2.52046346942 -2.11939968673 0.708885483329
4.47178692873 -0.701260904029 1.36757551838
4.67205447468 -1.31163689577 1.11453166899
2.66376129423 -2.71549092594 0.498033276711
3.14120443985 -2.29975305024 0.546332859649
4.47601480254 -0.724558124199 1.24078285257
1.80846335999 -2.86368342387 0.373528943987
4.31276144039 -1.46137252759 1.12558160575
4.54308614929 -0.660223057917 1.52386994494
4.09168657283 -1.70715995741 0.507132323827
4.72886124039 -0.927656202347 1.20733737753
4.58097092157 -0.624695373832 1.7589695779
3.81496741518 -0.913106422334 1.33846604412
4.16155978747 -1.21140744801 1.31021190019
2.89943312306 -1.71725621795 0.561555817208
1.79513950709 -2.20676727603 0.746292655045
2.33438390866 -1.79100926468 0.766155997926
2.4020032102 -2.01138117408 0.763424159901
2.87112770433 -2.07372536977 0.687579702627
4.2170742678 -1.61801578505 0.882124069772
2.77986189252 -1.86579627206 0.927621588085
4.67408397409 -0.724969305172 1.47159464408
3.98000535325 -0.75031915914 1.24204965729
2.86300002747 -2.14646918781 0.645088229636
2.19664217229 -3.00571778263 0.372327917334
3.37113514309 -1.44423183842 0.758360424345
4.11261925564 -0.657908715639 1.29756654665
2.31974576707 -3.06740330987 0.419544755593
2.11456662659 -3.17249781331 0.185770614361
5.06235718997 -1.01182041754 0.949601571159
2.0623379363 -3.34790981876 0.289862686135
4.915955584 -1.16561871886 0.925769820039
3.20778074351 -1.49521731851 0.712948352236
4.6167229199 -1.26165397363 0.931520805106
3.79953648059 -1.66714073323 0.674720111807
3.49634181832 -1.00884516409 1.15494874288
3.20306169617 -2.10929460916 0.465211459487
3.7071813908 -1.02284112305 1.05955826672
1.82329654544 -2.40221280693 0.325504761638
3.91133726143 -1.67088299998 0.76888013295
2.26513611479 -2.08145850636 0.835560579748
4.13477154656 -0.890806441817 1.21417582061
1.80081484243 -2.95788073032 0.393310377992
2.04711919332 -3.32040623698 0.56423485948
3.41792379767 -1.58763662392 1.01101654182
2.24858757196 -2.81632732501 0.253056208961
2.30216834175 -3.09813469917 0.220472174907
3.79172004141 -1.62787380658 0.573099726912
2.52459094539 -1.76948658824 0.679776682734
4.5013570579 -1.30218383981 0.861657550938
4.8036198996 -0.939883351905 1.19842553114
2.67502576869 -1.78416685742 0.833514299403
3.64173872633 -1.37128986622 0.919301413456
1.89439826502 -2.93064101946 0.319924388803
3.83544366443 -1.8599145019 0.930818427973
1.87133593101 -3.09746487622 0.550037353454
1.77010485701 -2.9471425087 0.609171704137
3.0895190168 -1.50583274097 0.663802399493
3.15401151654 -1.8057456001 0.584532649868
1.69813260015 -2.63435405537 0.616520578089
3.48124292631 -1.63029652104 0.987828633666
2.78794059456 -1.62729717197 0.7928181899
4.27927145526 -1.13583788044 1.3359391736
2.16856349809 -2.90232612062 0.429956103674
1.45255276547 -2.38867726658 0.555678787303
4.30297031051 -0.987689148512 0.998610461016
2.546422488 -2.19245585019 0.464194595644
3.28781070185 -1.19306900308 0.908975741191
2.0084611498 -2.29663110314 0.78034608174
3.32455823652 -1.57381353706 1.05527328851
2.93193224861 -2.55731374217 0.478613828535
3.85037604224 -1.50185730407 0.719826985675
2.18284822844 -2.6050559549 0.288778889184
3.20965797588 -1.21880527208 0.828204029086
1.95135666559 -2.04987126403 0.533877509617
3.08252159775 -1.68708435198 0.539233989981
4.38859603282 -1.41459890234 0.702506940831
2.83906387714 -1.58316229356 0.767377769854
3.03486471402 -1.43161066285 1.06997074274
1.67901599636 -2.41198837963 0.497881302768
2.86661745485 -2.2496875981 0.308478794313
2.82196907979 -2.37003761101 0.417440924948
3.1405204346 -2.3041205268 0.630321491672
3.13781639932 -1.17498329518 0.860551764617
3.69948693825 -1.05525734511 0.953691673772
4.71060951353 -1.40106079531 1.06098529787
2.16222382814 -2.99805444836 0.329377660062
4.78096617528 -1.26497667368 1.16963920193
2.06635230376 -2.08070818966 0.513275754673
2.83726901154 -2.13934685444 0.647754859909
4.54757054164 -0.599761115321 1.56987263993
4.77669315459 -1.05422611342 1.26078889913
3.43032090723 -1.96160370431 0.627576146784
2.49152782867 -2.73658720405 0.522166812427
2.10392496258 -2.66031131892 0.405699437111
3.58602076174 -1.46502753073 1.05922343613
4.14482948443 -1.61293436758 0.811528469546
2.77521774477 -2.30920377256 0.449285289366
1.76337610535 -2.28252251354 0.453774382517
1.93825801619 -3.36411522025 0.308138244302
2.60536139716 -2.51616248835 0.251866995939
3.31895092013 -2.42055136417 0.684302400672
1.54398881023 -2.79398704487 0.562273218257
2.79280426209 -2.58995022359 0.587284292198
3.4332321313 -1.39978698711 0.776980046818
2.56610265539 -1.89827101457 0.670936154171
3.23693255658 -2.08057863466 0.683764568192
3.17424833793 -2.40845703215 0.538393272015
3.33544977212 -1.22070010288 1.25102467093
4.37699161438 -1.46448967197 0.834735243783
4.20873344053 -1.26469631991 0.784964643436
4.31150552978 -0.69616182088 1.54739450575
4.2830981379 -0.814299340313 1.31831628402
4.27751553364 -1.28820715781 0.842734777876
1.71311144766 -2.57253059743 0.336293362349
3.78416251003 -0.874312827009 1.24106908364
3.03430189428 -2.45655244775 0.222486816451
3.13486125561 -2.14536111779 0.583713995639
3.57111215497 -2.11466932399 0.715936496174
4.54579375942 -0.730899659968 1.13955492925
4.80445411016 -1.20008060813 0.871165088551
4.0717477937 -1.71482377568 0.502511839799
2.35567960113 -2.58194587083 0.387910741598
4.82771240055 -1.24979926505 1.2930977664
3.27540978731 -2.16921135161 0.394853860491
2.20746252421 -2.21840459359 0.506202008461
2.79244187281 -1.86776796891 0.830591892857
3.91344026533 -1.10561850109 1.22240743561
1.42166056009 -2.4947048851 0.345726248728
4.76072768627 -1.36851591335 1.06113194295
2.09091789827 -1.92334193253 0.673496500657
2.35462895992 -2.55852699435 0.352993567954
3.8122438136 -2.03002070898 0.686808583767
4.15537318151 -1.83689359928 0.806478074206
2.09869848583 -2.15246518098 0.629062840703
2.47356759575 -1.84865515837 0.752947783578
4.48812520746 -0.606369656597 1.73381462626
4.20140664902 -0.911799952527 1.51192287973
4.59765372731 -1.27091604143 1.05850931301
4.07292030161 -1.79289247257 0.983397952172
2.80556752413 -1.50152200418 0.777247203723
3.55520152781 -1.0691932122 0.943298369832
4.20370669591 -1.65232064453 0.77267218334
3.38263270541 -2.29174539002 0.384282842907
4.99143439938 -1.10562409858 0.897631682976
3.60207155261 -1.75818626431 0.962817551361
3.0186384772 -1.37792825237 1.15768253701
3.28080830374 -1.90069310366 0.832308553663
1.5446965872 -2.50809949107 0.532774522184
4.08342596339 -1.14493841606 1.00625116885
3.44371281241 -1.54914652238 0.832459589773
2.99565750897 -1.51718421749 1.01670538716
2.49325648491 -2.90609398488 0.547416112727
1.81651700118 -3.14835030618 0.237101103236
3.73393879008 -1.49754484937 0.82116508308
3.91895404286 -1.77721193319 0.915697948974
2.3782333525 -2.25134727451 0.522719153681
3.93657175825 -0.663492296693 1.34825185511
2.02835050941 -2.20692861783 0.615013009822
2.39898967743 -2.28694182057 0.358118737433
2.83501478479 -1.88580572519 0.824648169197
3.05544485664 -1.29694492288 1.14984314338
1.48219377013 -2.80693410599 0.726636887722
4.85134886797 -0.946312562751 1.18463567334
4.11141901494 -0.815824127138 1.59109288843
3.35737686918 -2.24410302539 0.484191296258
1.99880099844 -3.11967955697 0.130082057914
4.66190626684 -0.906964388762 1.29978501308
3.80468355501 -1.1568800092 0.979443109425
3.2988079543 -2.42607406498 0.62792235175
3.55206600068 -1.48552934332 0.956386692728
3.16294112209 -2.05661656146 0.847467054849
2.4403253942 -1.95891750422 0.836775967654
1.99263899225 -2.13411821999 0.89236062089
1.84436066111 -3.20511387016 0.362917976607
3.93292594668 -0.675919773575 1.29661610807
2.15841283983 -3.22873287022 0.236549977595
2.97467402456 -2.46088182998 0.6108495047
2.66369836049 -1.93517066412 0.888799817845
4.76076487697 -0.955837677778 1.29520092257
3.79055992133 -1.90522560974 0.502793158657
3.53365171828 -1.80855561249 0.746830877067
2.74773259799 -1.49274395635 0.876538704925
4.15105720924 -1.09117450392 0.885748277293
4.27402653432 -0.789552672879 1.43809907516
2.28369325473 -2.24154660358 0.817277069152
1.86710774548 -2.24963716486 0.85092814338
2.10772230069 -2.44991745317 0.705940774037
3.79850375448 -1.94426003427 0.740626378211
2.24664468861 -1.92530468894 0.709451006896
2.6563682686 -1.97047914506 0.756375453064
4.36947720306 -0.538821591573 1.47984958911
2.61326759532 -2.97388918914 0.429622717464
1.97390350434 -3.26714665215 0.210476603506
2.68298699539 -1.98536800034 0.681644125613
3.75247607704 -1.80745776491 0.860073205992
2.17387621165 -3.06144305604 0.492227339755
3.87574166939 -1.23395981306 0.816938447179
4.97140258497 -1.25336518535 1.28650272673
4.47399302345 -0.516056907373 1.45454683816
2.94188865108 -2.34825002816 0.714735198653
2.21089668481 -3.18726797283 0.186102404783
2.59848371573 -2.86659597813 0.446295583081
4.60589193536 -1.53667071999 1.14685617471
2.35691672183 -2.65958305116 0.311348097368
4.4512166647 -0.796122467377 1.32745213979
3.07162783891 -2.03694057626 0.570645230789
2.38217440917 -2.04645940892 0.754487364587
2.91050538452 -1.47294725333 0.700139568674
4.24230707335 -0.825742634379 1.58065410832
4.25497790492 -1.72289554859 0.966779544082
4.06137644327 -0.697471334116 1.4432165826
3.88435556078 -1.73261547646 1.01766158209
3.15505408055 -2.02002070693 0.693358315732
2.76457672547 -2.70979123641 0.417748201276
4.2419002581 -1.57278739322 0.591596343042
4.63337504436 -1.24570917239 1.0566946546
2.93442407092 -1.93356347221 0.781579397045
4.65786858362 -0.885292298713 1.48165767358
3.15662564967 -2.54669962703 0.681991631316
3.94610409451 -0.623797635308 1.25016797244
3.4253122008 -1.14109777168 0.933888100992
3.09927010761 -1.55077605825 0.660291706927
3.81002257545 -1.0717619306 1.39859063019
3.49858818665 -1.97517448972 0.559969560187
4.76660195017 -1.07108846046 0.992737029136
2.47605015662 -2.30824876151 0.780025215871
4.35444972408 -0.692247060405 1.58030707649
2.39353143052 -2.37392772543 0.742133262689
2.61191626349 -1.82214579623 0.607343598192
4.67107056919 -1.08669401894 1.23828006877
2.67883956719 -2.25573216197 0.693565511375
2.89017314335 -1.91388549909 0.704169849888
3.20960969828 -1.25032170801 1.21301805443
2.85219212881 -1.72277508292 0.557853980668
4.52276837918 -0.896669004389 1.40559357934
4.08047046583 -1.03023114778 1.08648477184
2.53349606517 -2.55879031776 0.385924989067
4.49849344942 -1.1881447753 1.25433895793
4.50832850338 -1.19458907419 1.20497541147
4.17776969794 -1.42409198758 0.858673955377
4.43821405187 -0.784128873132 1.59340521402
2.03637015219 -2.83545822763 0.403310643832
4.69401197825 -0.86797079485 1.47576056271
4.52260658887 -0.613294776124 1.3471308069
4.46221227015 -0.689495922984 1.59888096027
4.13354399792 -1.59671268024 0.753855714665
2.9343805435 -1.9022848288 0.869124446202
4.09969866387 -1.62658888128 0.57544248705
4.16036357814 -1.1495429558 1.33425899287
3.49578165443 -0.956583640378 1.06572734254
3.63220443108 -1.66551831275 0.769587659758
3.8243059897 -1.83834562922 0.524093659594
2.83373809311 -2.02123105324 0.504187192574
2.47333470054 -3.10028066689 0.482261186844
4.05076045738 -1.36246132917 0.70364674687
3.43568504549 -1.50417185578 0.626733842296
4.14342161353 -1.30904926801 1.23991397958
3.10575389095 -1.91719785608 0.750434620622
3.02371368067 -2.11953838081 0.580750360918
1.87755531976 -2.72893489252 0.713030757128
1.93474518296 -3.26134715459 0.556964322663
4.48416088448 -1.32805103343 0.803160443753
4.51931558795 -0.685735496096 1.60991987449
4.43445191864 -1.01951463056 1.35036058654
3.79186944931 -0.954619740559 1.38612559716
4.57802257692 -0.602919287548 1.5994459986
2.13142580547 -3.11698165432 0.588423570917
3.79863731586 -1.83672340421 0.490796175322
3.03005218858 -2.29008132812 0.439284797584
3.33706925925 -1.27335890629 0.93077029538
3.67241608005 -1.85984904536 0.891105864644
2.84221198632 -1.93699133031 0.923045917644
4.50578890427 -1.18034644212 1.23199393278
4.28815088505 -1.15117475799 1.22069745551
2.78097650233 -2.44050501453 0.66269187547
3.28265101998 -2.34102973821 0.642214072628
4.24761232169 -1.37959366361 0.868997995951
2.11355791121 -2.14239950615 0.572596630014
3.64045094874 -1.38450464809 0.839880411566
3.00887393848 -1.35959711753 1.05727663102
3.444144199 -1.65189436203 0.647675915936
3.88518874771 -1.52194447659 1.08046123454
2.38748063228 -2.8334915939 0.661835464719
2.93820585711 -1.92920987582 0.731846215219
2.89480250779 -1.36820374389 0.914728063918
4.40367413099 -0.563140680575 1.71647123431
3.42376209725 -1.47979396954 0.919860489408
4.11336118001 -0.945868417909 1.17103729509
3.3406972616 -1.51834846809 0.6367257134
2.82004659039 -1.64948452285 0.734728177718
1.8927483521 -2.3674277945 0.662872031267
4.01088012828 -1.96224483757 0.79170486163
2.57664972638 -2.63014083505 0.644403685381
3.86020159344 -1.04048225008 1.30633055516
4.11237357805 -0.709952582312 1.63098564446
2.13554292215 -1.95953290137 0.897760253348
1.91506313809 -3.19696869688 0.52322804518
1.96697087368 -2.14434549769 0.854539158431
4.34313828662 -0.436114599377 1.49288034817
4.29657036047 -1.30427217313 1.19486565806
4.50599405669 -1.07290305248 1.33088434025
1.98468342457 -2.68266861485 0.352746259223
3.44021216039 -2.13564215341 0.499511666314
1.7068808853 -2.94890165608 0.600356550466
2.86356063928 -1.7896383286 0.786605070897
4.95492951049 -1.3269508117 1.11730339578
1.36474725622 -2.53114537464 0.351552981517
4.96668335363 -1.20447302315 1.24318164271
2.33583760956 -2.42167369272 0.478167707779
3.99785449944 -0.941070647446 1.23903593986
3.41915365448 -1.28900132769 1.07914518156
3.95533279952 -1.81797485644 0.807300266657
2.34200759618 -2.85326472243 0.582296770995
2.55646214348 -2.88609447062 0.605879532348
4.68023863423 -0.777073160874 1.46872707754
4.24602714276 -1.00920145384 1.33585453
2.52452600668 -2.11053250516 0.770353464366
3.85945826113 -0.997374793227 1.42005039368
2.79032308298 -1.83505721817 0.64578673484
1.94686750788 -2.80847170699 0.587667032646
4.61820094034 -0.913134175763 1.05162945745
2.00576374085 -3.26769363723 0.287901519793
1.83764022354 -2.02433733427 0.497507499606
3.20432548055 -1.70584870414 0.795135121633
4.41531639168 -1.4111451866 0.771312979976
1.69200756984 -2.69819062068 0.599484704771
2.08451610593 -2.31204515357 0.626446546726
3.23762859367 -1.5227189964 0.833782807467
4.80477280166 -1.13253710319 1.29323021224
1.99674487414 -2.69120263585 0.563014671997
4.51029419455 -1.06187899258 1.34288827876
3.54271983975 -1.63051037921 0.619270378669
2.01300879611 -2.7249531482 0.618597730251
3.43884606257 -1.99941125019 0.646502871967
2.51372564682 -1.86438137653 0.478528225514
1.66148290451 -2.27241450743 0.498716905054
4.10834355795 -0.872534276514 0.983700264729
3.55134128906 -2.22105980578 0.541229236619
1.75624597789 -2.39562477362 0.475018695194
2.37491404669 -3.09605366855 0.333318884603
2.41062713807 -2.96311473308 0.435922476468
3.94908133424 -1.66756351414 0.714563242588
4.56014146301 -1.17690661376 1.25090421526
2.13667901814 -2.23081088298 0.454890258213
3.71846224657 -1.9445883736 0.46570146292
4.50970035308 -1.50658795544 0.904043937666
3.99384510256 -0.650383953207 1.22843595453
4.41368854474 -0.384850334297 1.4710926862
1.40928392904 -2.60447581163 0.783834194876
4.25878112999 -0.484435220067 1.26705157336
2.24137596679 -2.24759125627 0.663246946437
4.66765635379 -1.14079422184 0.998902284106
4.37959573258 -0.99032306778 1.24578317876
4.32418066601 -0.995849443515 1.28973277707
'@

Write-BundleFile 'tests/CoreTests.cpp' @'
#include "PointCloud.h"
#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>

static void expect(bool ok,const char* message) {if(!ok) throw std::runtime_error(message);}
static pc::Cloud shape() {
    pc::Cloud c;std::mt19937 gen(9123);std::uniform_real_distribution<double> u(-1,1);
    for(int i=0;i<800;++i) {
        double x=u(gen),y=u(gen),z=u(gen);
        c.points.push_back({2*x,0.6*y,0.25*z+0.18*x*x+0.12*y*x});
    }return c;
}
static std::string header(const std::string& mode,int n) {
    return "VERSION 0.7\nFIELDS intensity z x y normal\nSIZE 4 4 4 4 4\nTYPE F F F F F\nCOUNT 1 1 1 1 3\nWIDTH "+std::to_string(n)+
        "\nHEIGHT 1\nPOINTS "+std::to_string(n)+"\nDATA "+mode+"\n";
}
static void parserTests(const std::filesystem::path& dir) {
    auto ascii=dir/"ascii.pcd";
    {std::ofstream f(ascii);f<<header("ascii",4)<<"5 3 1 2 0 0 1\n6 6 4 5 0 0 1\n7 9 7 8 0 0 1\n8 nan 1 1 0 0 1\n";}
    auto a=pc::readPCD(ascii);expect(a.points.size()==3&&a.invalidPoints==1,"ASCII finite filtering");
    expect(a.points[1].x==4&&a.points[1].y==5&&a.points[1].z==6,"Reordered xyz / multi-count fields");
    std::vector<float> raw={5,3,1,2,0,0,1,6,6,4,5,0,0,1,7,9,7,8,0,0,1};
    auto binary=dir/"binary.pcd";
    {std::ofstream f(binary,std::ios::binary);f<<header("binary",3);f.write(reinterpret_cast<const char*>(raw.data()),static_cast<std::streamsize>(raw.size()*4));}
    auto b=pc::readPCD(binary);expect(b.points[2].z==9&&b.points[2].x==7,"Binary interleaved");
    std::vector<unsigned char> soa;
    for(int field=0;field<5;++field) for(int point=0;point<3;++point) {
        int count=field==4?3:1;
        for(int j=0;j<count;++j) {
            float value=raw[point*7+field+j];const auto* bytes=reinterpret_cast<const unsigned char*>(&value);
            soa.insert(soa.end(),bytes,bytes+4);
        }
    }
    std::vector<unsigned char> packed;
    for(size_t i=0;i<soa.size();) {
        size_t length=std::min(size_t(32),soa.size()-i);packed.push_back(static_cast<unsigned char>(length-1));
        packed.insert(packed.end(),soa.begin()+i,soa.begin()+i+length);i+=length;
    }
    auto compressed=dir/"compressed.pcd";
    {std::ofstream f(compressed,std::ios::binary);f<<header("binary_compressed",3);
     uint32_t cs=static_cast<uint32_t>(packed.size()),us=static_cast<uint32_t>(soa.size());
     f.write(reinterpret_cast<const char*>(&cs),4);f.write(reinterpret_cast<const char*>(&us),4);
     f.write(reinterpret_cast<const char*>(packed.data()),static_cast<std::streamsize>(packed.size()));}
    auto c=pc::readPCD(compressed);expect(c.points[2].x==7&&c.points[1].y==5,"LZF SoA field blocks");
    auto repeated=dir/"lzf_backref.pcd";
    {std::ofstream f(repeated,std::ios::binary);
     f<<"FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nWIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA binary_compressed\n";
     // 36 zero bytes: one literal, then overlapping back-reference of length 35.
     const unsigned char bytes[]={0,0,224,26,0};uint32_t cs=5,us=36;
     f.write(reinterpret_cast<const char*>(&cs),4);f.write(reinterpret_cast<const char*>(&us),4);
     f.write(reinterpret_cast<const char*>(bytes),5);}
    auto repeatedCloud=pc::readPCD(repeated);expect(repeatedCloud.points.size()==3&&pc::norm2(repeatedCloud.points[2])==0,"LZF overlap back-reference");
    auto bad=dir/"bad.pcd";{std::ofstream f(bad);f<<header("binary",3)<<"short";}
    bool failed=false;try{pc::readPCD(bad);}catch(const std::exception&){failed=true;}expect(failed,"Reject truncated PCD");
    {std::ofstream f(bad);f<<"FIELDS x y\nSIZE 4 4\nTYPE F F\nWIDTH 3\nPOINTS 3\nDATA ascii\n";}
    failed=false;try{pc::readPCD(bad);}catch(const std::exception&){failed=true;}expect(failed,"Reject missing z");
    {std::ofstream f(bad);f<<header("binary_compressed",3);uint32_t cs=2,us=84;f.write(reinterpret_cast<char*>(&cs),4);f.write(reinterpret_cast<char*>(&us),4);char data[2]={char(255),0};f.write(data,2);}
    failed=false;try{pc::readPCD(bad);}catch(const std::exception&){failed=true;}expect(failed,"Reject corrupt LZF");
    std::atomic_bool cancel{true};failed=false;
    try{pc::readPCD(ascii,&cancel);}catch(const std::exception&){failed=true;}expect(failed,"Reader cancellation");
}
int main(int argc,char** argv) {
    try {
        auto dir=std::filesystem::path(argc>1?argv[1]:"core-test-data");std::filesystem::create_directories(dir);
        parserTests(dir);std::cout<<"PCD formats, LZF, invalid input and cancellation: passed\n";
        auto a=shape();auto same=pc::compare(a,a,1e-5);
        expect(same.similarity>99.99&&same.rmse<1e-6,"Identical clouds");
        pc::Cloud b;pc::Transform motion;
        double angle=0.73,c=std::cos(angle),s=std::sin(angle);
        motion.r={{c,-s,0,s,c,0,0,0,1}};motion.t={3.2,-1.8,0.75};
        for(auto p:a.points) b.points.push_back(motion.apply(p));
        auto registered=pc::compare(a,b,0.001);
        double maxError=0;
        for(size_t i=0;i<a.points.size();++i) maxError=std::max(maxError,std::sqrt(pc::norm2(a.points[i]-registered.alignedB[i])));
        std::cout<<"Rigid transform score="<<registered.similarity<<", max error="<<maxError<<"\n";
        expect(registered.similarity>99.99&&maxError<1e-5,"Rotation / translation recovery");
        for(int i=0;i<35;++i) b.points.push_back(motion.apply({2.5+0.003*i,0.5,0.4}));
        auto changed=pc::compare(a,b,0.03);
        std::cout<<"Differences A="<<a.points.size()-changed.matchedA<<", B="<<b.points.size()-changed.matchedB<<", score="<<changed.similarity<<"\n";
        expect(b.points.size()-changed.matchedB>=30,"Detect extra points in B");
        expect(changed.similarity>94&&changed.similarity<99,"Bidirectional score penalizes extra points");
        auto exportDir=dir/("export_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        pc::exportResult(exportDir,a,changed);
        auto exported=pc::readPCD(exportDir/"aligned_B.pcd");expect(exported.points.size()==b.points.size(),"Export / reload");
        // General 3D rotations (not only a rotation around Z).
        for(int trial=0;trial<8;++trial) {
            double ax=0.19+trial*0.31,ay=-0.43+trial*0.17,az=0.67+trial*0.23;
            double cx=std::cos(ax),sx=std::sin(ax),cy=std::cos(ay),sy=std::sin(ay),cz=std::cos(az),sz=std::sin(az);
            pc::Transform t;t.r={{cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx,
                sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx,-sy,cy*sx,cy*cx}};t.t={7,-2,1};
            pc::Cloud rotated;for(auto p:a.points) rotated.points.push_back(t.apply(p));
            auto r=pc::compare(a,rotated,0.001);
            expect(r.similarity>99.99&&r.rmse<1e-6,"General 3D rotation recovery");
        }
        pc::Cloud subset;subset.points.assign(a.points.begin(),a.points.begin()+650);
        auto partial=pc::compare(a,subset,0.015);
        expect(partial.similarity>85&&partial.similarity<97,"Partial overlap score");
        pc::Cloud noisy=a;
        for(size_t i=0;i<noisy.points.size();++i) noisy.points[i].z+=0.0002*std::sin(static_cast<double>(i));
        auto noise=pc::compare(a,noisy,0.002);expect(noise.similarity>99.5,"Small measurement noise");
        pc::Cloud line;line.points={{0,0,0},{1,0,0},{2,0,0}};
        bool degenerate=false;try{pc::compare(line,line);}catch(const std::exception&){degenerate=true;}
        expect(degenerate,"Reject ambiguous collinear cloud");
        pc::Cloud planar;
        for(int y=0;y<15;++y) for(int x=0;x<22;++x) planar.points.push_back({x*0.02,y*0.02,0});
        auto plane=pc::compare(planar,planar,0.001);expect(plane.similarity>99.9,"Planar geometry");
        std::atomic_bool cancel{true};bool cancelled=false;
        try{pc::compare(a,b,0,&cancel);}catch(const std::exception&){cancelled=true;}expect(cancelled,"ICP cancellation");
        bool invalid=false;try{pc::compare(a,b,-1);}catch(const std::exception&){invalid=true;}expect(invalid,"Reject negative threshold");
        std::cout<<"Identity, rigid registration, differences, export, planar geometry and cancellation: passed\n";
        return 0;
    }catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
'@

Write-BundleFile 'tests/CylinderTests.cpp' @'
#include "Cylinder.h"
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>

static void expect(bool c,const char* message){if(!c)throw std::runtime_error(message);}
static double dot(pc::Vec3 a,pc::Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
static pc::Vec3 axis{0.36,0.48,0.8};
static pc::Vec3 u{0.8,-0.6,0},v{0.48,0.64,-0.6};
static pc::Cloud cylinder(double radius,double height,double arc=6.283185307179586,double noise=0.002,int outliers=0) {
    pc::Cloud c;std::mt19937 random(12345);std::normal_distribution<double> jitter(0,noise);
    std::uniform_real_distribution<double> uniform(0,1);pc::Vec3 center{12,-4,7};
    for(int i=0;i<1600;++i){double angle=uniform(random)*arc,h=(uniform(random)-0.5)*height;
        double r=radius+jitter(random);c.points.push_back(center+axis*h+(u*std::cos(angle)+v*std::sin(angle))*r);}
    for(int i=0;i<outliers;++i)c.points.push_back(center+pc::Vec3{(uniform(random)-0.5)*4,(uniform(random)-0.5)*4,(uniform(random)-0.5)*4});
    return c;
}
static bool rejects(const pc::Cloud& c,double threshold=0){try{pc::reconstructCylinder(c,threshold);return false;}catch(const std::exception&){return true;}}
int main(int argc,char** argv){
 try{
    auto a=cylinder(0.7,3,6.283185307179586,0.002,180);
    auto full=pc::reconstructCylinder(a,0.012);
    std::cout<<"Full + 10% outliers: r="<<full.radius<<", h="<<full.height<<", axis dot="<<std::abs(dot(axis,full.axis))<<", inliers="<<full.inlierRatio<<", rmse="<<full.rmse<<'\n';
    expect(std::abs(full.radius-0.7)<0.01,"Full cylinder radius");
    expect(std::abs(full.height-3)<0.12,"Full cylinder height");expect(std::abs(dot(axis,full.axis))>0.999,"Full cylinder axis");
    expect(full.inlierRatio>0.85&&full.inlierRatio<0.94,"Outlier detection");expect(full.rmse<0.005,"Cylinder RMSE");
    auto shortResult=pc::reconstructCylinder(cylinder(1,0.4),0.012);
    expect(std::abs(shortResult.radius-1)<0.01&&std::abs(dot(axis,shortResult.axis))>0.999,"Short broad cylinder axis");
    auto partial=pc::reconstructCylinder(cylinder(0.7,3,3.5,0.002,80),0.012);
    std::cout<<"Partial arc: r="<<partial.radius<<", axis dot="<<std::abs(dot(axis,partial.axis))<<'\n';
    expect(std::abs(partial.radius-0.7)<0.02&&std::abs(dot(axis,partial.axis))>0.998,"Partial cylinder fit");
    // Ordered ring scan: a fixed stride of 64 would select only one angular ray.
    pc::Cloud organized;organized.points.reserve(384000);
    for(int h=0;h<6000;++h)for(int k=0;k<64;++k){double angle=6.283185307179586*k/64;
        organized.points.push_back({0.7*std::cos(angle),0.7*std::sin(angle),3.0*h/5999});}
    auto ordered=pc::reconstructCylinder(organized,0.006);
    expect(std::abs(ordered.radius-0.7)<0.001&&std::abs(ordered.axis.z)>0.999,"Organized ring scan sampling");
    auto autoResult=pc::reconstructCylinder(cylinder(0.7,3));expect(autoResult.inlierRatio>0.95,"Automatic threshold");
    auto moved=cylinder(0.7,3);for(auto& p:moved.points)p=p+pc::Vec3{1e6,-1e6,1e6};
    auto distant=pc::reconstructCylinder(moved,0.012);expect(std::abs(distant.radius-0.7)<0.01,"Large world offset");
    pc::Cloud plane;for(int y=0;y<35;++y)for(int x=0;x<35;++x)plane.points.push_back({x*0.02,y*0.02,0});
    expect(rejects(plane,0.002),"Reject planar cloud");expect(rejects(plane),"Reject plane at automatic tolerance");
    pc::Cloud line;for(int i=0;i<100;++i)line.points.push_back({i*0.01,0,0});expect(rejects(line),"Reject line");
    expect(rejects(cylinder(0.7,3,0.5),0.004),"Reject ambiguous narrow arc");
    pc::Cloud sphere;for(int i=0;i<1800;++i){double z=1-2*(i+0.5)/1800.0,angle=i*2.399963229728653;
        double r=std::sqrt(1-z*z);sphere.points.push_back({r*std::cos(angle),r*std::sin(angle),z});}
    expect(rejects(sphere),"Reject spherical cloud");
    expect(rejects(a,-1),"Reject negative threshold");expect(rejects(a,1),"Reject overly broad threshold");
    std::atomic_bool cancel{true};bool stopped=false;try{pc::reconstructCylinder(a,0,&cancel);}catch(const std::exception&){stopped=true;}
    expect(stopped,"Cylinder cancellation");
    auto dir=std::filesystem::path(argc>1?argv[1]:"cylinder-test-data")/std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    pc::exportCylinder(dir,a,full);auto model=pc::readPCD(dir/"cylinder_surface.pcd");
    expect(model.points.size()==1600,"Model surface PCD export");expect(std::filesystem::file_size(dir/"cylinder.obj")>1000,"OBJ mesh export");
    auto in=pc::readPCD(dir/"cylinder_inliers.pcd"),out=pc::readPCD(dir/"cylinder_outliers.pcd");
    expect(in.points.size()+out.points.size()==a.points.size(),"Export labels preserve original points");
    expect(full.triangles.size()==3072,"Mesh triangle count");
    std::cout<<"Cylinder reconstruction, rejection, cancellation, PCD and OBJ export: passed\n";return 0;
 }catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
'@

Write-BundleFile 'tests/RunCoreTests.ps1' @'
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
'@


if ($PSCommandPath -and (Test-Path -LiteralPath $PSCommandPath -PathType Leaf)) {
    $scripts = Join-Path $Destination 'scripts'
    New-Item -ItemType Directory -Path $scripts -Force | Out-Null
    Copy-Item -LiteralPath $PSCommandPath -Destination (Join-Path $scripts 'CreateProject.ps1')
}
Add-Type -AssemblyName System.IO.Compression
Add-Type -AssemblyName System.IO.Compression.FileSystem
$excludedDirs = @('node_modules', '.git', '.vs', 'bin', 'obj', '.cache', 'Debug', 'Release', 'x64', 'x86', 'core-test-data', 'cylinder-test-data')
$excludedExtensions = @('.user', '.suo', '.obj', '.pdb', '.ilk', '.ipch', '.log', '.exe', '.res', '.o')
$prefix = $Destination.TrimEnd([char[]]'\/') + [System.IO.Path]::DirectorySeparatorChar
$rootName = Split-Path -Leaf $Destination
$archive = [System.IO.Compression.ZipFile]::Open($zipPath, [System.IO.Compression.ZipArchiveMode]::Create)
try {
    Get-ChildItem -LiteralPath $Destination -Recurse -File -Force | ForEach-Object {
        $file = $_
        $relative = $file.FullName.Substring($prefix.Length)
        $skip = $false
        foreach ($segment in ($relative -split '[\\/]')) {
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
Write-Host "ZIP：$zipPath" -ForegroundColor Green
Write-Host '打开 VC2019/PointCloudWorkbench/PointCloudWorkbench.sln，推荐 VS2019 / x64 / Release。'
