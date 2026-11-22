// SPDX-License-Identifier: AGPL-3.0-only
#include "SmokeEngine.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cwctype>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace smoke { namespace {
using Clock=std::chrono::steady_clock;
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
void checkCancel(const std::atomic_bool* cancel){if(cancel&&cancel->load())throw std::runtime_error("Operation cancelled.");}
void validate(const cv::Mat& image){require(!image.empty()&&image.type()==CV_8UC3,"Expected a non-empty 8-bit BGR image.");}
std::vector<unsigned char> readBytes(const std::filesystem::path& file,size_t limit){
    std::ifstream stream(file,std::ios::binary|std::ios::ate);require(bool(stream),"Cannot open image or model file.");
    auto length=stream.tellg();require(length>0&&static_cast<uint64_t>(length)<=limit,"File is empty or exceeds the supported size limit.");
    std::vector<unsigned char> bytes(static_cast<size_t>(length));stream.seekg(0);
    stream.read(reinterpret_cast<char*>(bytes.data()),static_cast<std::streamsize>(bytes.size()));require(bool(stream),"File read failed.");return bytes;
}
void drawBoxes(cv::Mat& image,const std::vector<Box>& boxes,bool neural){
    for(const auto& b:boxes){
        cv::Scalar color=b.classId==0?cv::Scalar(40,60,255):cv::Scalar(0,170,255);
        cv::rectangle(image,b.rect,color,2);
        std::string label=neural?(b.classId==0?"smoke ":"fire ")+cv::format("%.2f",b.score):"CV suspect";
        cv::putText(image,label,cv::Point(b.rect.x,std::max(16,b.rect.y-5)),cv::FONT_HERSHEY_SIMPLEX,0.55,color,2,cv::LINE_AA);
    }
}
double elapsed(Clock::time_point start){return std::chrono::duration<double,std::milli>(Clock::now()-start).count();}
void writePNG(const std::filesystem::path& file,const cv::Mat& image){
    std::vector<unsigned char> bytes;require(cv::imencode(".png",image,bytes),"PNG encoding failed.");
    std::ofstream out(file,std::ios::binary);out.write(reinterpret_cast<const char*>(bytes.data()),static_cast<std::streamsize>(bytes.size()));
    require(bool(out),"Failed to write annotated PNG.");
}
}
Image loadImage(const std::filesystem::path& path){
    std::wstring extension=path.extension().wstring();std::transform(extension.begin(),extension.end(),extension.begin(),[](wchar_t c){return static_cast<wchar_t>(std::towlower(c));});
    require(extension==L".png"||extension==L".jpg"||extension==L".jpeg","Only PNG, JPG and JPEG files are supported.");
    auto bytes=readBytes(path,128ull*1024*1024);cv::Mat image=cv::imdecode(bytes,cv::IMREAD_COLOR);
    require(!image.empty(),"The image is damaged or cannot be decoded.");
    require(image.total()<=40000000,"Image exceeds the 40-megapixel limit.");
    Image result;result.originalWidth=image.cols;result.originalHeight=image.rows;
    // Both methods analyze the same image. Preserve aspect ratio, with long edge <= 1600.
    if(std::max(image.cols,image.rows)>1600){double scale=1600.0/std::max(image.cols,image.rows);cv::resize(image,result.bgr,cv::Size(),scale,scale,cv::INTER_AREA);}
    else result.bgr=image;
    return result;
}
Letterbox letterbox(const cv::Mat& image){
    validate(image);Letterbox out;out.scale=std::min(640.0/image.cols,640.0/image.rows);
    int width=std::max(1,static_cast<int>(std::round(image.cols*out.scale))),height=std::max(1,static_cast<int>(std::round(image.rows*out.scale)));
    width=std::min(width,640);height=std::min(height,640);out.left=(640-width)/2;out.top=(640-height)/2;
    cv::Mat resized;cv::resize(image,resized,cv::Size(width,height),0,0,cv::INTER_LINEAR);
    cv::copyMakeBorder(resized,out.image,out.top,640-height-out.top,out.left,640-width-out.left,cv::BORDER_CONSTANT,cv::Scalar(114,114,114));
    return out;
}
std::vector<Box> decodeYolo(const cv::Mat& tensor,cv::Size original,const Letterbox& geometry,float confidence,double& highestSmokeScore){
    require(std::isfinite(confidence)&&confidence>0&&confidence<=1,"Neural threshold must be in (0,1].");
    require(original.width>0&&original.height>0&&geometry.scale>0,"Invalid letterbox geometry.");
    require(tensor.type()==CV_32F&&tensor.dims==3&&tensor.size[0]==1&&tensor.size[1]==6&&tensor.size[2]>0,
        "Unsupported model output: expected [1,6,N] FP32 (xywh, smoke, fire), without objectness or embedded NMS.");
    cv::Mat contiguous=tensor.isContinuous()?tensor:tensor.clone();int count=contiguous.size[2];const float* values=contiguous.ptr<float>();
    std::vector<cv::Rect> rects[2];std::vector<float> scores[2];highestSmokeScore=0;
    for(int i=0;i<count;++i){
        float x=values[i],y=values[count+i],w=values[2*count+i],h=values[3*count+i];
        float smokeScore=values[4*count+i],fireScore=values[5*count+i];
        if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(w)||!std::isfinite(h)||w<=0||h<=0||
            !std::isfinite(smokeScore)||!std::isfinite(fireScore)||smokeScore<0||smokeScore>1||fireScore<0||fireScore>1)continue;
        highestSmokeScore=std::max(highestSmokeScore,static_cast<double>(smokeScore));
        double x1=std::clamp((x-w*0.5-geometry.left)/geometry.scale,0.0,static_cast<double>(original.width));
        double y1=std::clamp((y-h*0.5-geometry.top)/geometry.scale,0.0,static_cast<double>(original.height));
        double x2=std::clamp((x+w*0.5-geometry.left)/geometry.scale,0.0,static_cast<double>(original.width));
        double y2=std::clamp((y+h*0.5-geometry.top)/geometry.scale,0.0,static_cast<double>(original.height));
        if(x2<=x1||y2<=y1)continue;
        cv::Rect box(static_cast<int>(std::floor(x1)),static_cast<int>(std::floor(y1)),
            static_cast<int>(std::ceil(x2))-static_cast<int>(std::floor(x1)),static_cast<int>(std::ceil(y2))-static_cast<int>(std::floor(y1)));
        // The highest class wins for this candidate; fire never counts as smoke.
        int label=smokeScore>=fireScore?0:1;float score=label==0?smokeScore:fireScore;
        if(score>=confidence){rects[label].push_back(box);scores[label].push_back(score);}
    }
    std::vector<Box> results;
    for(int label=0;label<2;++label){
        std::vector<int> indices;cv::dnn::NMSBoxes(rects[label],scores[label],0.0f,0.45f,indices,1.f,300);
        for(int index:indices)results.push_back({rects[label][index],scores[label][index],label});
    }
    return results;
}
Analysis detectOpenCV(const cv::Mat& image,double minimumAreaPercent,const std::atomic_bool* cancel){
    validate(image);require(std::isfinite(minimumAreaPercent)&&minimumAreaPercent>0&&minimumAreaPercent<=100,"CV area threshold must be in (0,100].");
    checkCancel(cancel);auto start=Clock::now();cv::Mat hsv,gray;cv::cvtColor(image,hsv,cv::COLOR_BGR2HSV);cv::cvtColor(image,gray,cv::COLOR_BGR2GRAY);
    std::vector<cv::Mat> channels;cv::split(hsv,channels);cv::Mat grayFloat,mean,squareMean,variance,stddev;
    gray.convertTo(grayFloat,CV_32F);cv::GaussianBlur(grayFloat,mean,cv::Size(15,15),0);
    cv::GaussianBlur(grayFloat.mul(grayFloat),squareMean,cv::Size(15,15),0);variance=squareMean-mean.mul(mean);cv::max(variance,0,variance);cv::sqrt(variance,stddev);
    cv::Mat lap;cv::Laplacian(grayFloat,lap,CV_32F,3);cv::Mat absLap=cv::abs(lap),mask;
    mask=(channels[1]<75)&(channels[2]>55)&(channels[2]<245)&(stddev>2)&(stddev<35)&(absLap<60);
    cv::morphologyEx(mask,mask,cv::MORPH_OPEN,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3,3)));
    cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(9,9)));
    checkCancel(cancel);cv::Mat labels,stats,centers;int components=cv::connectedComponentsWithStats(mask,labels,stats,centers,8,CV_32S);
    Analysis result;result.annotated=image.clone();cv::Mat accepted=cv::Mat::zeros(image.size(),CV_8U);size_t total=0;
    int minimumComponent=std::max(64,static_cast<int>(image.total()*0.003));
    for(int i=1;i<components;++i){
        int area=stats.at<int>(i,cv::CC_STAT_AREA),width=stats.at<int>(i,cv::CC_STAT_WIDTH),height=stats.at<int>(i,cv::CC_STAT_HEIGHT);
        if(area<minimumComponent||width<12||height<12||static_cast<double>(width)/height>6||static_cast<double>(height)/width>6)continue;
        total+=static_cast<size_t>(area);accepted.setTo(255,labels==i);
        result.boxes.push_back({cv::Rect(stats.at<int>(i,cv::CC_STAT_LEFT),stats.at<int>(i,cv::CC_STAT_TOP),width,height),0,0});
    }
    result.value=100.0*static_cast<double>(total)/static_cast<double>(image.total());result.detected=result.value>=minimumAreaPercent;
    cv::Mat tinted;cv::addWeighted(image,0.6,cv::Mat(image.size(),image.type(),cv::Scalar(20,50,230)),0.4,0,tinted);tinted.copyTo(result.annotated,accepted);
    drawBoxes(result.annotated,result.boxes,false);result.milliseconds=elapsed(start);checkCancel(cancel);return result;
}
Analysis NeuralDetector::detect(const cv::Mat& image,const std::filesystem::path& model,float confidence,const std::atomic_bool* cancel){
    validate(image);require(std::isfinite(confidence)&&confidence>0&&confidence<=1,"Neural threshold must be in (0,1].");
    checkCancel(cancel);auto start=Clock::now();
    if(network_.empty()||model_!=model){
        auto bytes=readBytes(model,100ull*1024*1024);auto network=cv::dnn::readNetFromONNX(bytes);
        require(!network.empty(),"Could not load ONNX smoke model.");network.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);network.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        network_=std::move(network);model_=model;
    }
    checkCancel(cancel);auto geometry=letterbox(image);cv::Mat blob=cv::dnn::blobFromImage(geometry.image,1.0/255,cv::Size(640,640),cv::Scalar(),true,false,CV_32F);
    network_.setInput(blob);cv::Mat output=network_.forward();checkCancel(cancel);Analysis result;
    result.boxes=decodeYolo(output,image.size(),geometry,confidence,result.value);
    result.detected=std::any_of(result.boxes.begin(),result.boxes.end(),[](const Box& box){return box.classId==0;});
    result.annotated=image.clone();drawBoxes(result.annotated,result.boxes,true);result.milliseconds=elapsed(start);return result;
}
void exportAnalyses(const std::filesystem::path& directory,const std::filesystem::path& source,const Image& image,
    const Analysis* cvResult,const Analysis* neuralResult,double areaThreshold,double neuralThreshold){
    require(cvResult||neuralResult,"Run a detector before exporting.");require(!std::filesystem::exists(directory),"Export directory already exists.");
    std::filesystem::create_directories(directory);std::ofstream report(directory/"report.txt",std::ios::binary);require(bool(report),"Cannot create report.");
    report<<"Source="<<source.u8string()<<"\nOriginal_size="<<image.originalWidth<<'x'<<image.originalHeight<<"\nAnalysis_size="<<image.bgr.cols<<'x'<<image.bgr.rows
        <<"\nBoxes use analysis-image pixel coordinates.\nCV_area_threshold_percent="<<areaThreshold<<"\nNeural_score_threshold="<<neuralThreshold<<'\n'<<std::setprecision(8);
    if(cvResult){writePNG(directory/"opencv_result.png",cvResult->annotated);report<<"CV_suspected="<<cvResult->detected<<"\nCV_suspected_area_percent="<<cvResult->value<<"\nCV_elapsed_ms="<<cvResult->milliseconds<<'\n';}
    if(neuralResult){writePNG(directory/"neural_result.png",neuralResult->annotated);report<<"DNN_smoke_detected="<<neuralResult->detected<<"\nDNN_max_smoke_score="<<neuralResult->value<<"\nDNN_elapsed_ms="<<neuralResult->milliseconds<<'\n';}
    for(auto item:{std::make_pair("CV",cvResult),std::make_pair("DNN",neuralResult)})if(item.second)for(const auto& box:item.second->boxes)
        report<<item.first<<"_box="<<box.classId<<','<<box.score<<','<<box.rect.x<<','<<box.rect.y<<','<<box.rect.width<<','<<box.rect.height<<'\n';
    report<<"CV area is a heuristic, and DNN scores are not calibrated probabilities. A negative result is not proof that smoke is absent.\n";
    require(bool(report),"Failed to write detection report.");
}
}
