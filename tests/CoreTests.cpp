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
