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
