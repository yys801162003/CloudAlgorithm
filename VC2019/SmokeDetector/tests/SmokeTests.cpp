// SPDX-License-Identifier: AGPL-3.0-only
#include "SmokeEngine.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <chrono>

static void expect(bool value,const char* reason){if(!value)throw std::runtime_error(reason);}
template<class Function>static bool rejects(Function f){try{f();return false;}catch(const std::exception&){return true;}}
static void save(const std::filesystem::path& path,const cv::Mat& image,const char* extension){
 std::vector<unsigned char> bytes;cv::imencode(extension,image,bytes);std::ofstream out(path,std::ios::binary);
 out.write(reinterpret_cast<const char*>(bytes.data()),static_cast<std::streamsize>(bytes.size()));
}
int main(int argc,char** argv){
 try{
    expect(argc>=2,"Usage: SmokeTests <SmokeDetector-project-directory> [real-smoke-photo]");
    auto project=std::filesystem::path(argv[1]);auto temp=std::filesystem::temp_directory_path()/("SmokeTests_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(temp);cv::Mat green(180,240,CV_8UC3,cv::Scalar(20,200,40));
    auto png=temp/std::filesystem::u8path(u8"中文图片.PNG"),jpeg=temp/std::filesystem::u8path(u8"中文图片.JPEG");save(png,green,".png");save(jpeg,green,".jpg");
    auto decoded=smoke::loadImage(png),decodedJpeg=smoke::loadImage(jpeg);
    expect(cv::norm(decoded.bgr,green,cv::NORM_INF)==0,"PNG Unicode path / exact decode");
    expect(decodedJpeg.bgr.size()==green.size()&&cv::norm(decodedJpeg.bgr,green,cv::NORM_INF)<8,"JPEG decode");
    cv::Mat large(1000,2000,CV_8UC3,cv::Scalar(0,0,0));save(temp/"large.png",large,".png");auto scaled=smoke::loadImage(temp/"large.png");
    expect(scaled.originalWidth==2000&&scaled.bgr.cols==1600&&scaled.bgr.rows==800,"Analysis image resize");
    {std::ofstream bad(temp/"broken.png");bad<<"not an image";}
    expect(rejects([&]{smoke::loadImage(temp/"broken.png");}),"Damaged image rejection");
    expect(rejects([&]{smoke::loadImage(temp/"wrong.bmp");}),"Unsupported extension rejection");
    auto clean=smoke::detectOpenCV(green,1);expect(!clean.detected&&clean.value==0,"Saturated background negative");
    cv::Mat gray(200,200,CV_8UC3,cv::Scalar(140,140,140));expect(!smoke::detectOpenCV(gray,1).detected,"Uniform gray is not enough for rule detection");
    cv::Mat synthetic(300,400,CV_8UC3,cv::Scalar(180,80,20));cv::Mat noise(170,180,CV_8U);cv::RNG rng(123);rng.fill(noise,cv::RNG::NORMAL,150,10);
    cv::GaussianBlur(noise,noise,cv::Size(3,3),0);cv::Mat noiseBgr;cv::cvtColor(noise,noiseBgr,cv::COLOR_GRAY2BGR);noiseBgr.copyTo(synthetic(cv::Rect(110,60,180,170)));
    auto rule=smoke::detectOpenCV(synthetic,1);std::cout<<"Synthetic rule candidate area="<<rule.value<<"%\n";
    expect(rule.detected&&rule.value>5,"Constructed low-saturation texture candidate");
    expect(rejects([&]{smoke::detectOpenCV(green,0);}),"Area threshold validation");
    std::atomic_bool cancelled{true};expect(rejects([&]{smoke::detectOpenCV(green,1,&cancelled);}),"Rule cancellation");
    auto geometry=smoke::letterbox(cv::Mat(200,400,CV_8UC3,cv::Scalar(0,0,0)));
    expect(geometry.image.rows==640&&geometry.image.cols==640&&geometry.top==160&&geometry.left==0,"Letterbox geometry");
    expect(geometry.image.at<cv::Vec3b>(0,0)==cv::Vec3b(114,114,114),"Letterbox pad color");
    int dims[3]={1,6,4};cv::Mat tensor(3,dims,CV_32F,cv::Scalar(0));auto p=tensor.ptr<float>();
    auto candidate=[&](int i,float x,float y,float w,float h,float s,float f){p[i]=x;p[4+i]=y;p[8+i]=w;p[12+i]=h;p[16+i]=s;p[20+i]=f;};
    candidate(0,320,320,160,160,0.9f,0.01f);candidate(1,321,320,160,160,0.8f,0.01f);candidate(2,80,240,64,64,0.01f,0.95f);
    candidate(3,std::numeric_limits<float>::quiet_NaN(),300,50,50,0.99f,0.01f);
    double score=0;auto boxes=smoke::decodeYolo(tensor,{400,200},geometry,0.5f,score);
    expect(boxes.size()==2,"Class-aware NMS and NaN filtering");
    expect(boxes[0].classId==0&&boxes[0].rect==cv::Rect(150,50,100,100),"Undo letterbox coordinates");
    expect(boxes[1].classId==1,"Fire class remains separate");
    candidate(0,320,320,160,160,0.5f,0.01f);candidate(1,321,320,160,160,0.4f,0.01f);
    auto boundary=smoke::decodeYolo(tensor,{400,200},geometry,0.5f,score);
    expect(boundary.size()==2&&boundary[0].score==0.5f,"Score equal to threshold is retained");
    int badDims[3]={1,7,4};cv::Mat wrong(3,badDims,CV_32F,cv::Scalar(0));expect(rejects([&]{smoke::decodeYolo(wrong,{400,200},geometry,0.5f,score);}),"Reject incompatible model shape");
    smoke::NeuralDetector network;auto model=project/"models/firewatch-yolo11s.onnx";
    auto neural=network.detect(cv::Mat(240,320,CV_8UC3,cv::Scalar(0,0,0)),model,0.5f);
    std::cout<<"Real ONNX blank image: score="<<neural.value<<", detected="<<neural.detected<<'\n';expect(!neural.detected,"Real model inference on blank image");
    expect(rejects([&]{network.detect(green,model,0);}),"DNN threshold validation");
    smoke::NeuralDetector missing;expect(rejects([&]{missing.detect(green,temp/"missing.onnx",0.5f);}),"Missing model is an error, not a negative result");
    expect(rejects([&]{network.detect(green,model,0.5f,&cancelled);}),"Neural cancellation");
    smoke::Image exportImage;exportImage.bgr=synthetic;exportImage.originalWidth=400;exportImage.originalHeight=300;
    smoke::exportAnalyses(temp/"export",png,exportImage,&rule,nullptr,1,0.5);
    expect(std::filesystem::file_size(temp/"export/opencv_result.png")>0&&std::filesystem::file_size(temp/"export/report.txt")>0,"Annotated image and report export");
    expect(rejects([&]{smoke::exportAnalyses(temp/"export",png,exportImage,&rule,nullptr,1,0.5);}),"Do not overwrite export directory");
    if(argc>2){auto image=smoke::loadImage(std::filesystem::path(argv[2]));auto r=network.detect(image.bgr,model,0.5f);
        std::cout<<"Real smoke photo: score="<<r.value<<", boxes="<<r.boxes.size()<<", detected="<<r.detected<<'\n';expect(r.detected,"Smoke detection on external real-photo integration fixture");}
    if(argc>3){auto image=smoke::loadImage(std::filesystem::path(argv[3]));auto r=network.detect(image.bgr,model,0.5f);bool fire=false;
        for(const auto& box:r.boxes)fire=fire||box.classId==1;
        std::cout<<"Mixed fire/smoke regression photo: smoke score="<<r.value<<", smoke detected="<<r.detected<<", fire="<<fire<<'\n';
        expect(fire,"Auxiliary fire class decoded on the mixed-scene fixture (smoke recall is not asserted)");}
    std::cout<<"Image decoding, rule logic, YOLO postprocessing, real ONNX and exports: passed\n";
    return 0;
 }catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
