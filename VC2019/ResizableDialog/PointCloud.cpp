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
