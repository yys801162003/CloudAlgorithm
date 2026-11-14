#requires -Version 5.1
param([string]$Destination = (Join-Path (Get-Location).Path 'PCDRegistration'))
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
'@

Write-BundleFile 'VC2019/ResizableDialog/PointCloud.cpp' @'
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

Write-BundleFile 'VC2019/ResizableDialog/PointCloud.h' @'
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

Write-BundleFile 'VC2019/ResizableDialog/README.md' @'
# VC2019 PCD 点云配准与差异分析

## 编译运行

1. Visual Studio Installer 安装“使用 C++ 的桌面开发”、MSVC v142 和 Windows 10 SDK，无需 MFC、PCL、Eigen 或 VTK。
2. 使用 Visual Studio 2019 打开 `ResizableDialog.sln`。
3. 推荐选择 **x64 / Release**，按 F5 编译运行。也提供 Win32、Debug 配置；大点云建议使用 x64。
4. 若 Windows SDK 版本不匹配，右键解决方案选择“重定解决方案目标”。

使用 Windows 自带的 OpenGL 1.1 接口显示点云，需有可工作的 OpenGL 驱动。程序支持中文文件路径。

## 操作

1. 点击“读取点云 A”选择参考 PCD，再点击“读取点云 B”选择待配准 PCD。
2. 两个点云立即在同一视图中显示。左键拖动旋转，右键拖动平移，滚轮缩放，“适应视图”重置视野。
3. 输入“差异距离阈值”，单位与 PCD 坐标相同。例如坐标单位是米，`0.01` 表示 1 厘米。`0` 自动取两个点云各自包围盒对角线较大值的 1%。
4. 点击“配准并比较”，程序估计 **B → A 的旋转和平移**，然后计算相似度、双向重合率、差异数量和阈值内 RMSE。
5. 青色表示 A，绿色表示 B；红色标记 A 中的差异点，紫色标记 B 中的差异点。“仅显示差异点”过滤非差异点。“显示 B 原始位置”切换配准前/后的坐标；差异标签仍来自配准后的比较。
6. “导出结果”选择父目录后生成独立的 `PCD_Result_*` 子目录，包含 `aligned_B.pcd`、`different_A.pcd`、`different_B.pcd` 和 `report.txt`。报告中包含 4×4 B → A 变换矩阵；无差异时对应 PCD 的 POINTS 为 0。
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

## 验证

- Linux 原生 C++17 测试覆盖 PCD 三种编码、额外字段、LZF 回溯、NaN 过滤、坏文件、取消操作、同云、旋转平移、8 组三维旋转、噪声、部分重叠、差异点和导出回读。
- 使用 AddressSanitizer / UndefinedBehaviorSanitizer 检查核心逻辑。
- 使用 LLVM-MinGW 分别交叉编译 x64 与 x86，检查 Windows C++ 界面、资源及链接。
- 已在 PowerShell 7 中验证生成脚本及 ZIP 打包；脚本语法面向 Windows PowerShell 5.1。当前环境没有 Windows 桌面，尚未在 Visual Studio 2019 中实际构建或交互运行；交叉编译不等同于 MSVC v142 验证。

Windows 的核心测试：在 VS2019 Developer PowerShell 中执行仓库 `tests/RunCoreTests.ps1`。Linux 可执行：

```sh
g++ -std=c++17 -O2 -I VC2019/ResizableDialog VC2019/ResizableDialog/PointCloud.cpp tests/CoreTests.cpp -o core-tests
./core-tests
```
'@

Write-BundleFile 'VC2019/ResizableDialog/ResizableDialog.rc' @'
#pragma code_page(65001)
#include <windows.h>
#include "resource.h"
LANGUAGE LANG_CHINESE, SUBLANG_CHINESE_SIMPLIFIED
IDD_MAIN_DIALOG DIALOGEX 0, 0, 700, 470
STYLE DS_SETFONT | DS_CENTER | WS_POPUP | WS_CAPTION | WS_SYSMENU | WS_THICKFRAME | WS_MINIMIZEBOX | WS_MAXIMIZEBOX | WS_CLIPCHILDREN
EXSTYLE WS_EX_APPWINDOW
CAPTION "PCD 点云配准与差异分析 — Visual Studio 2019"
FONT 9, "Microsoft YaHei UI"
BEGIN
    PUSHBUTTON "读取点云 A", IDC_OPEN_A, 8, 8, 68, 20
    PUSHBUTTON "读取点云 B", IDC_OPEN_B, 82, 8, 68, 20
    LTEXT "差异距离阈值（0 = 自动）", -1, 160, 12, 120, 12
    EDITTEXT IDC_THRESHOLD, 284, 8, 64, 20, ES_AUTOHSCROLL | WS_TABSTOP
    PUSHBUTTON "配准并比较", IDC_COMPARE, 354, 8, 78, 20
    PUSHBUTTON "适应视图", IDC_FIT, 438, 8, 64, 20
    PUSHBUTTON "导出结果", IDC_EXPORT, 508, 8, 64, 20
    PUSHBUTTON "取消计算", IDC_CANCEL_WORK, 578, 8, 64, 20
    LTEXT "A：未加载", IDC_PATH_A, 8, 34, 680, 14, SS_PATHELLIPSIS
    LTEXT "B：未加载", IDC_PATH_B, 8, 50, 680, 14, SS_PATHELLIPSIS
    AUTOCHECKBOX "显示 B 原始位置", IDC_ORIGINAL, 8, 68, 112, 16
    AUTOCHECKBOX "仅显示差异点", IDC_DIFF_ONLY, 130, 68, 100, 16
    LTEXT "青=A  绿=B  红=A差异  紫=B差异 | 左键旋转，右键平移，滚轮缩放", IDC_HINT, 240, 70, 450, 14
    CONTROL "", IDC_VIEW, "PointCloudView", WS_CHILD | WS_VISIBLE | WS_BORDER, 8, 90, 684, 274
    EDITTEXT IDC_RESULTS, 8, 370, 684, 68, ES_MULTILINE | ES_READONLY | ES_AUTOVSCROLL | WS_VSCROLL
    LTEXT "请先读取两个 PCD 文件。", IDC_STATUS, 8, 446, 684, 16, SS_ENDELLIPSIS
END
'@

Write-BundleFile 'VC2019/ResizableDialog/ResizableDialog.sln' @'
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

Write-BundleFile 'VC2019/ResizableDialog/ResizableDialog.vcxproj' @'
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
  <ItemGroup><ClCompile Include="main.cpp" /><ClCompile Include="PointCloud.cpp" /></ItemGroup>
  <ItemGroup><ClInclude Include="resource.h" /><ClInclude Include="PointCloud.h" /></ItemGroup>
  <ItemGroup><ResourceCompile Include="ResizableDialog.rc" /></ItemGroup>
  <Import Project="$(VCTargetsPath)\Microsoft.Cpp.targets" />
</Project>
'@

Write-BundleFile 'VC2019/ResizableDialog/main.cpp' @'
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
    std::filesystem::path pathA,pathB;
    bool busy=false,hasResult=false,closePending=false;
    std::atomic_bool cancel{false}; std::thread worker;
};
struct WorkResult {
    int action=0; std::filesystem::path path;
    pc::Cloud cloud; pc::Result result;
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
    for(int id:{IDC_OPEN_A,IDC_OPEN_B,IDC_THRESHOLD,IDC_FIT}) EnableWindow(GetDlgItem(app.dialog,id),!value);
    EnableWindow(GetDlgItem(app.dialog,IDC_COMPARE),!value&&!app.a.points.empty()&&!app.b.points.empty());
    EnableWindow(GetDlgItem(app.dialog,IDC_EXPORT),!value&&app.hasResult);
    EnableWindow(GetDlgItem(app.dialog,IDC_CANCEL_WORK),value);
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
    DrawCloud(app->a.points,app->hasResult?&app->result.differentA:nullptr,true,only,v.center);
    const auto& b=app->hasResult&&!original?app->result.alignedB:app->b.points;
    DrawCloud(b,app->hasResult?&app->result.differentB:nullptr,false,only,v.center);
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
    RECT top{0,0,8,90};MapDialogRect(dialog,&top);
    RECT report{0,0,0,68};MapDialogRect(dialog,&report);
    RECT status{0,0,0,18};MapDialogRect(dialog,&status);
    int margin=top.right,viewTop=top.bottom,content=std::max(0,width-2*margin);
    int statusTop=height-status.bottom-margin;
    int reportTop=statusTop-report.bottom-margin;
    MoveWindow(GetDlgItem(dialog,IDC_VIEW),margin,viewTop,content,std::max(0,reportTop-viewTop-margin),TRUE);
    MoveWindow(GetDlgItem(dialog,IDC_RESULTS),margin,reportTop,content,report.bottom,TRUE);
    MoveWindow(GetDlgItem(dialog,IDC_STATUS),margin,statusTop,content,status.bottom,TRUE);
    RECT line{0,0,0,14};MapDialogRect(dialog,&line);
    RECT pathA{0,0,8,34},pathB{0,0,8,50};MapDialogRect(dialog,&pathA);MapDialogRect(dialog,&pathB);
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
    const auto& r=app.result;std::wostringstream out;
    out<<std::fixed<<std::setprecision(3)<<L"几何相似度："<<r.similarity<<L"%    阈值："<<std::setprecision(8)<<r.threshold
       <<L"（PCD 坐标单位）    ICP 迭代："<<r.iterations<<L"\r\n"
       <<L"A 重合率："<<std::setprecision(2)<<100*r.coverageA<<L"%    B 重合率："<<100*r.coverageB
       <<L"%    差异点 A："<<(app.a.points.size()-r.matchedA)<<L"    差异点 B："<<(app.b.points.size()-r.matchedB)<<L"\r\n";
    if(r.hasMatchedPairs) out<<L"阈值内双向 RMSE："<<std::setprecision(8)<<r.rmse;
    else out<<L"阈值内双向 RMSE：N/A（无匹配点）";
    out<<L"\r\n相似度 = (A 重合率 + B 重合率) / 2；仅比较刚体几何重合，不表示语义相同。";
    if(r.similarity<30) out<<L"\r\n重合率低：请核对坐标单位、距离阈值和重叠区域；ICP 可能未找到正确姿态。";
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
        const auto& cloud=a?app.a:app.b;
        std::wstring label=(a?L"A：":L"B：")+work->path.wstring()+L"  | 有效点："+std::to_wstring(cloud.points.size())+
            L"，跳过非有限点："+std::to_wstring(cloud.invalidPoints);
        SetDlgItemTextW(app.dialog,a?IDC_PATH_A:IDC_PATH_B,label.c_str());
        SetDlgItemTextW(app.dialog,IDC_RESULTS,L"读取完成。加载两份点云后点击“配准并比较”。阈值 0 表示自动使用点云包围盒对角线的 1%。");
        FitView(app);Status(app,L"点云读取完成。");
    } else if(work->action==3) {
        app.result=std::move(work->result);app.hasResult=true;
        CheckDlgButton(app.dialog,IDC_ORIGINAL,BST_UNCHECKED);ResultsText(app);FitView(app);
        Status(app,L"比较完成：青=A，绿=配准后的 B，红=A 差异，紫=B 差异。");
    } else if(work->action==4) Status(app,L"已导出："+work->path.wstring());
    Busy(app,false);Refresh(app);
}
static INT_PTR CALLBACK DialogProc(HWND dialog,UINT message,WPARAM wParam,LPARAM lParam) {
    App* app=GetApp(dialog);
    switch(message) {
    case WM_INITDIALOG:
        app=reinterpret_cast<App*>(lParam);app->dialog=dialog;
        SetWindowLongPtrW(dialog,DWLP_USER,reinterpret_cast<LONG_PTR>(app));
        SetDlgItemTextW(dialog,IDC_THRESHOLD,L"0");Busy(*app,false);Layout(dialog);
        if(!InitGL(*app)) {MessageBoxW(dialog,L"无法初始化 OpenGL，请检查显卡驱动或远程桌面的 OpenGL 支持。",L"启动失败",MB_OK|MB_ICONERROR);EndDialog(dialog,1);}
        else Refresh(*app);
        return TRUE;
    case WM_SIZE:if(app&&wParam!=SIZE_MINIMIZED) Layout(dialog);return TRUE;
    case WM_GETMINMAXINFO: {
        // Resource toolbar must remain usable. No custom maximum size.
        RECT rect{0,0,670,300};MapDialogRect(dialog,&rect);
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
                wchar_t input[128]{};GetDlgItemTextW(dialog,IDC_THRESHOLD,input,128);
                wchar_t* end=nullptr;double threshold=std::wcstod(input,&end);
                while(end&&(*end==L' '||*end==L'\t')) ++end;
                if(end==input||!end||*end||!std::isfinite(threshold)||threshold<0||threshold>1e12)
                    MessageBoxW(dialog,L"请输入非负的有限距离值；0 表示自动。单位与 PCD 坐标相同。",L"距离阈值无效",MB_OK|MB_ICONWARNING);
                else Launch(*app,3,{},threshold);
            }return TRUE;
        case IDC_FIT:if(!app->busy) FitView(*app);return TRUE;
        case IDC_ORIGINAL:FitView(*app);return TRUE;
        case IDC_DIFF_ONLY:Refresh(*app);return TRUE;
        case IDC_CANCEL_WORK:app->cancel=true;Status(*app,L"正在取消，请稍候……");return TRUE;
        case IDC_EXPORT:
            if(!app->busy&&app->hasResult) {
                wchar_t parent[32768]{};BROWSEINFOW browse{};browse.hwndOwner=dialog;
                browse.lpszTitle=L"选择父目录（将新建唯一的 PCD_Result 子目录）";
                browse.ulFlags=BIF_RETURNONLYFSDIRS|BIF_NEWDIALOGSTYLE;
                PIDLIST_ABSOLUTE item=SHBrowseForFolderW(&browse);
                if(item) {
                    if(SHGetPathFromIDListW(item,parent)) {
                        std::filesystem::path dir=std::filesystem::path(parent)/(L"PCD_Result_"+std::to_wstring(GetTickCount64()));
                        Launch(*app,4,dir);
                    }
                    CoTaskMemFree(item);
                }
            }return TRUE;
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

Write-BundleFile 'VC2019/ResizableDialog/resource.h' @'
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
'@

Write-BundleFile 'samples/README.md' @'
# 示例点云

- `sample_A.pcd`：参考点云，1400 点。
- `sample_B_rigid.pcd`：A 经已知三维旋转和平移得到，应可配准至约 100% 重合。
- `sample_B_changed.pcd`：在 A 坐标系中删除 80 点并增加 45 点，再施加相同刚体变换。

先载入 A 和 B_rigid，将差异阈值设为 `0.02`，点击“配准并比较”；然后将 B 换为 B_changed，再次比较。不同点的最终数量受阈值及近邻密度影响。

坐标为演示用任意单位，B 到 A 的变换由程序估计，不需要提前输入。
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

Write-BundleFile 'tests/RunCoreTests.ps1' @'
# Run from an x64 Native Tools Command Prompt for VS 2019 / Developer PowerShell.
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$build = Join-Path ([System.IO.Path]::GetTempPath()) ('PCDCoreTests_' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $build | Out-Null
$source = Join-Path $root 'VC2019\ResizableDialog'
& cl.exe /nologo /std:c++17 /EHsc /utf-8 /O2 "/I$source" (Join-Path $source 'PointCloud.cpp') (Join-Path $PSScriptRoot 'CoreTests.cpp') "/Fo$build\" "/Fe$build\CoreTests.exe"
if ($LASTEXITCODE -ne 0) { throw 'Core test build failed.' }
& (Join-Path $build 'CoreTests.exe') (Join-Path $build 'fixtures')
if ($LASTEXITCODE -ne 0) { throw 'Core tests failed.' }
'@


# Keep a copy of this generator in the generated bundle when run from a .ps1 file.
if ($PSCommandPath -and (Test-Path -LiteralPath $PSCommandPath -PathType Leaf)) {
    $scripts = Join-Path $Destination 'scripts'
    New-Item -ItemType Directory -Path $scripts -Force | Out-Null
    Copy-Item -LiteralPath $PSCommandPath -Destination (Join-Path $scripts 'CreateProject.ps1')
}
Add-Type -AssemblyName System.IO.Compression
Add-Type -AssemblyName System.IO.Compression.FileSystem
$excludedDirs = @('node_modules', '.git', '.vs', 'bin', 'obj', '.cache', 'Debug', 'Release', 'x64', 'x86', 'core-test-data')
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
Write-Host '用 VS2019 打开 VC2019/ResizableDialog/ResizableDialog.sln，推荐 x64 / Release。'
