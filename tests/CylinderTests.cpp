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
    const auto& round=ordered.measurements;
    expect(round.validSections==20&&std::abs(round.length-ordered.height)<0.013,"Round measurement coverage and length");
    expect(std::abs(round.maxDiameter.maxDiameter-1.4)<0.002,"Maximum diameter");
    expect(std::abs(round.minWidth.minWidth-1.4*std::cos(3.141592653589793/64))<0.002,"Minimum caliper width");
    expect(std::abs(round.minDiameter.equivalentDiameter-1.4)<0.003,"Minimum area-equivalent diameter");
    expect(round.maxEllipticity.ellipticity<0.4&&round.maxDiameter.offset<0.002,"Round ellipticity and axis offset");
    expect(partial.measurements.validSections==0&&partial.measurements.length>2.8,"Incomplete arc only supports length");

    pc::Cloud oval;
    for(int h=0;h<41;++h)for(int k=0;k<96;++k) {
        const double angle=6.283185307179586*k/96;
        const double axial=(h-20)*0.1;
        const bool bulge=std::abs(axial)<0.21;
        oval.points.push_back({(bulge?0.79:0.73)*std::cos(angle)+(bulge?0.04:0),
                              0.67*std::sin(angle),axial});
    }
    const auto deformed=pc::reconstructCylinder(oval,0.15);
    const auto& dimensions=deformed.measurements;
    expect(dimensions.validSections==20&&dimensions.length>3.9,"Oval coverage and length");
    expect(std::abs(dimensions.maxDiameter.maxDiameter-1.58)<0.07,"Oval largest diameter");
    expect(std::abs(dimensions.minWidth.minWidth-1.34)<0.035,"Oval minimum caliper width");
    expect(std::abs(dimensions.minDiameter.equivalentDiameter-2*std::sqrt(0.73*0.67))<0.025,"Oval minimum equivalent diameter");
    expect(dimensions.maxEllipticity.ellipticity>12&&dimensions.maxEllipticity.ellipticity<24,"Oval ellipticity");
    expect(dimensions.maxDiameter.offset>0.02&&dimensions.maxDiameter.offset<0.08,"Local section center offset");

    pc::Cylinder first,second;
    first.axis={0,0,1};second.axis={-0.5,0,-std::sqrt(0.75)};
    expect(std::abs(pc::measureAxisAngle(first,second).degrees-30)<1e-9,"Unsigned native axis angle");
    pc::Transform transform;
    transform.r={std::sqrt(0.75),0,-0.5,0,1,0,0.5,0,std::sqrt(0.75)};
    transform.t={100,-200,300};
    expect(pc::measureAxisAngle(first,second,&transform).degrees<1e-9,"Aligned axis angle ignores translation");
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
