// SPDX-License-Identifier: AGPL-3.0-only
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <commdlg.h>
#include <shlobj.h>
#include <opencv2/imgproc.hpp>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <cwchar>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <thread>
#include "resource.h"
#include "SmokeEngine.h"

constexpr UINT WM_FINISHED=WM_APP+1;
struct App {
    HWND dialog=nullptr;
    smoke::Image image;
    std::filesystem::path path,model;
    smoke::NeuralDetector neural;
    std::optional<smoke::Analysis> cvResult,nnResult;
    cv::Mat originalPreview,cvPreview,nnPreview;
    double cvThresholdUsed=1,nnThresholdUsed=0.5;
    std::wstring cvError,nnError;
    std::atomic_bool cancel{false};std::thread worker;
    bool busy=false,closePending=false;
};
struct Work {
    int action=0;bool attemptedCV=false,attemptedNN=false;
    double cvThreshold=1,nnThreshold=0.5;
    std::filesystem::path path;
    smoke::Image image;
    std::optional<smoke::Analysis> cvResult,nnResult;
    std::wstring error,cvError,nnError;
};
static App* GetApp(HWND dialog){return reinterpret_cast<App*>(GetWindowLongPtrW(dialog,DWLP_USER));}
static std::wstring Wide(const std::string& text){
    if(text.empty())return {};
    int n=MultiByteToWideChar(CP_UTF8,0,text.data(),static_cast<int>(text.size()),nullptr,0);
    std::wstring out(static_cast<size_t>(n),L' ');MultiByteToWideChar(CP_UTF8,0,text.data(),static_cast<int>(text.size()),&out[0],n);return out;
}
static void Status(App& app,const std::wstring& text){SetDlgItemTextW(app.dialog,IDC_STATUS,text.c_str());}
static void Refresh(App& app){for(int id:{IDC_VIEW_ORIGINAL,IDC_VIEW_CV,IDC_VIEW_NN})InvalidateRect(GetDlgItem(app.dialog,id),nullptr,FALSE);}
static cv::Mat Preview(const cv::Mat& bgr){cv::Mat out;if(!bgr.empty())cv::cvtColor(bgr,out,cv::COLOR_BGR2BGRA);return out;}
static void Busy(App& app,bool value){
    app.busy=value;EnableWindow(GetDlgItem(app.dialog,IDC_OPEN),!value);
    for(int id:{IDC_RUN_CV,IDC_RUN_NN,IDC_RUN_BOTH})EnableWindow(GetDlgItem(app.dialog,id),!value&&!app.image.bgr.empty());
    for(int id:{IDC_AREA_THRESHOLD,IDC_NN_THRESHOLD})EnableWindow(GetDlgItem(app.dialog,id),!value);
    EnableWindow(GetDlgItem(app.dialog,IDC_CANCEL),value);
    EnableWindow(GetDlgItem(app.dialog,IDC_EXPORT),!value&&(app.cvResult.has_value()||app.nnResult.has_value()));
}
static void PaintImage(HWND window){
    PAINTSTRUCT ps{};BeginPaint(window,&ps);RECT r{};GetClientRect(window,&r);
    HBRUSH background=CreateSolidBrush(RGB(24,30,39));FillRect(ps.hdc,&r,background);DeleteObject(background);
    App* app=GetApp(GetParent(window));const cv::Mat* image=nullptr;
    if(app){int id=GetDlgCtrlID(window);image=id==IDC_VIEW_ORIGINAL?&app->originalPreview:id==IDC_VIEW_CV?&app->cvPreview:&app->nnPreview;}
    if(image&&!image->empty()){
        double scale=std::min(static_cast<double>(r.right)/image->cols,static_cast<double>(r.bottom)/image->rows);
        int width=std::max(1,static_cast<int>(image->cols*scale)),height=std::max(1,static_cast<int>(image->rows*scale));
        BITMAPINFO info{};info.bmiHeader.biSize=sizeof(BITMAPINFOHEADER);info.bmiHeader.biWidth=image->cols;info.bmiHeader.biHeight=-image->rows;
        info.bmiHeader.biPlanes=1;info.bmiHeader.biBitCount=32;info.bmiHeader.biCompression=BI_RGB;
        SetStretchBltMode(ps.hdc,HALFTONE);SetBrushOrgEx(ps.hdc,0,0,nullptr);
        StretchDIBits(ps.hdc,(r.right-width)/2,(r.bottom-height)/2,width,height,0,0,image->cols,image->rows,image->data,&info,DIB_RGB_COLORS,SRCCOPY);
    }else{
        SetBkMode(ps.hdc,TRANSPARENT);SetTextColor(ps.hdc,RGB(175,185,200));
        DrawTextW(ps.hdc,L"尚无图片或检测结果",-1,&r,DT_CENTER|DT_VCENTER|DT_SINGLELINE);
    }
    EndPaint(window,&ps);
}
static LRESULT CALLBACK PreviewProc(HWND window,UINT message,WPARAM wParam,LPARAM lParam){
    if(message==WM_PAINT){PaintImage(window);return 0;}
    if(message==WM_ERASEBKGND)return 1;
    if(message==WM_SIZE){InvalidateRect(window,nullptr,FALSE);return 0;}
    return DefWindowProcW(window,message,wParam,lParam);
}
static void Layout(HWND dialog){
    RECT client{};GetClientRect(dialog,&client);int clientWidth=static_cast<int>(client.right),clientHeight=static_cast<int>(client.bottom);RECT units{0,0,8,128};MapDialogRect(dialog,&units);
    RECT report{0,0,0,106},status{0,0,0,16},title{0,0,0,16};MapDialogRect(dialog,&report);MapDialogRect(dialog,&status);MapDialogRect(dialog,&title);
    int margin=units.right,top=units.bottom,width=std::max(0,clientWidth-2*margin),statusY=clientHeight-margin-status.bottom;
    int reportY=statusY-margin-report.bottom,viewHeight=std::max(0,reportY-margin-top),column=std::max(0,(width-2*margin)/3);
    int views[]={IDC_VIEW_ORIGINAL,IDC_VIEW_CV,IDC_VIEW_NN},titles[]={IDC_TITLE_ORIGINAL,IDC_TITLE_CV,IDC_TITLE_NN};
    for(int i=0;i<3;++i){int x=margin+i*(column+margin),w=i==2?std::max(0,clientWidth-margin-x):column;
        MoveWindow(GetDlgItem(dialog,views[i]),x,top,w,viewHeight,TRUE);MoveWindow(GetDlgItem(dialog,titles[i]),x,top-title.bottom-margin/2,w,title.bottom,TRUE);}
    MoveWindow(GetDlgItem(dialog,IDC_REPORT),margin,reportY,width,report.bottom,TRUE);MoveWindow(GetDlgItem(dialog,IDC_STATUS),margin,statusY,width,status.bottom,TRUE);
    RECT path{0,0,0,66},model{0,0,0,86};MapDialogRect(dialog,&path);MapDialogRect(dialog,&model);
    MoveWindow(GetDlgItem(dialog,IDC_PATH),margin,path.bottom,width,title.bottom,TRUE);MoveWindow(GetDlgItem(dialog,IDC_MODEL),margin,model.bottom,width,title.bottom,TRUE);
}
static void Report(App& app){
    std::wostringstream out;out<<std::fixed;
    if(app.cvResult){const auto& r=*app.cvResult;
        out<<L"OpenCV："<<(r.detected?L"疑似含有烟雾":L"未达到疑似烟雾面积阈值")<<L"；疑似区域占比 "<<std::setprecision(2)<<r.value
           <<L"%，使用面积阈值 "<<app.cvThresholdUsed<<L"%；耗时 "<<r.milliseconds<<L" ms。\r\n";
    }else if(!app.cvError.empty())out<<L"OpenCV 检测失败："<<app.cvError<<L"\r\n";
    else out<<L"OpenCV：尚未检测。\r\n";
    if(app.nnResult){const auto& r=*app.nnResult;size_t smokeCount=0,fireCount=0;for(const auto& b:r.boxes)(b.classId==0?smokeCount:fireCount)++;
        out<<L"神经网络："<<(r.detected?L"检出烟雾":L"未检出烟雾")<<L"；烟雾框 "<<smokeCount<<L"，火焰框 "<<fireCount
           <<L"；最高烟雾候选分数 "<<std::setprecision(4)<<r.value<<L"，使用阈值 "<<app.nnThresholdUsed<<L"；耗时 "<<std::setprecision(2)<<r.milliseconds<<L" ms。\r\n";
    }else if(!app.nnError.empty())out<<L"神经网络检测失败："<<app.nnError<<L"\r\n";
    else out<<L"神经网络：尚未检测。\r\n";
    out<<L"红框=烟雾候选，橙框=火焰（仅辅助显示，不参与烟雾判定）。两种结果独立呈现，不合并成结论。\r\n"
       <<L"规则可能把云雾、蒸汽、灰色背景误判为烟雾；分数不是准确率。未检出不等于无烟，单张图像无法利用运动信息。";
    SetDlgItemTextW(app.dialog,IDC_REPORT,out.str().c_str());
}
static void Launch(App& app,int action,const std::filesystem::path& path={},double cvThreshold=1,double nnThreshold=0.5){
    app.cancel=false;Busy(app,true);Status(app,L"正在处理……取消在解码/推理阶段边界生效。");
    app.worker=std::thread([&app,action,path,cvThreshold,nnThreshold]{
        auto work=std::make_unique<Work>();work->action=action;work->path=path;work->cvThreshold=cvThreshold;work->nnThreshold=nnThreshold;
        try{
            if(action==1)work->image=smoke::loadImage(path);
            else if(action==5)smoke::exportAnalyses(path,app.path,app.image,app.cvResult?&*app.cvResult:nullptr,app.nnResult?&*app.nnResult:nullptr,app.cvThresholdUsed,app.nnThresholdUsed);
            else{
                if(action==2||action==4){work->attemptedCV=true;try{work->cvResult=smoke::detectOpenCV(app.image.bgr,cvThreshold,&app.cancel);}catch(const std::exception& e){work->cvError=Wide(e.what());}}
                if((action==3||action==4)&&!app.cancel.load()){
                    work->attemptedNN=true;try{work->nnResult=app.neural.detect(app.image.bgr,app.model,static_cast<float>(nnThreshold),&app.cancel);}catch(const std::exception& e){work->nnError=Wide(e.what());}
                }
            }
            if(app.cancel.load())work->error=L"操作已取消。";
        }catch(const std::exception& e){work->error=Wide(e.what());}
        if(PostMessageW(app.dialog,WM_FINISHED,0,reinterpret_cast<LPARAM>(work.get())))work.release();
    });
}
static void Finish(App& app,std::unique_ptr<Work> work){
    if(app.worker.joinable())app.worker.join();
    if(app.closePending){EndDialog(app.dialog,0);return;}
    if(!work->error.empty()){
        Status(app,work->error);if(!app.cancel.load())MessageBoxW(app.dialog,work->error.c_str(),L"处理失败",MB_OK|MB_ICONERROR);
    }else if(work->action==1){
        app.image=std::move(work->image);app.path=work->path;app.cvResult.reset();app.nnResult.reset();app.cvError.clear();app.nnError.clear();
        app.originalPreview=Preview(app.image.bgr);app.cvPreview.release();app.nnPreview.release();
        std::wstring text=app.path.wstring()+L" | 原图 "+std::to_wstring(app.image.originalWidth)+L"×"+std::to_wstring(app.image.originalHeight)+L"，分析图 "+std::to_wstring(app.image.bgr.cols)+L"×"+std::to_wstring(app.image.bgr.rows);
        SetDlgItemTextW(app.dialog,IDC_PATH,text.c_str());Status(app,L"图片已载入，选择一种方法或对比两种方法。");
    }else if(work->action==5)Status(app,L"已导出："+work->path.wstring());
    else{
        if(work->attemptedCV){app.cvResult=std::move(work->cvResult);app.cvError=work->cvError;app.cvThresholdUsed=work->cvThreshold;app.cvPreview=app.cvResult?Preview(app.cvResult->annotated):cv::Mat();}
        if(work->attemptedNN){app.nnResult=std::move(work->nnResult);app.nnError=work->nnError;app.nnThresholdUsed=work->nnThreshold;app.nnPreview=app.nnResult?Preview(app.nnResult->annotated):cv::Mat();}
        Status(app,app.cvError.empty()&&app.nnError.empty()?L"检测完成。结果和实际使用的阈值见下方报告。":L"部分方法检测失败，详见下方报告；失败不会显示成“无烟雾”。");
    }
    Report(app);Busy(app,false);Refresh(app);
}
static bool Threshold(App& app,int id,double& value,double maximum){
    wchar_t text[128]{};GetDlgItemTextW(app.dialog,id,text,128);wchar_t* end=nullptr;value=std::wcstod(text,&end);
    while(end&&(*end==L' '||*end==L'\t'))++end;
    if(end==text||!end||*end||!std::isfinite(value)||value<=0||value>maximum){MessageBoxW(app.dialog,L"面积阈值应在 (0,100]，神经网络阈值应在 (0,1]。",L"参数无效",MB_OK|MB_ICONWARNING);return false;}return true;
}
static void OpenImage(App& app){
    wchar_t file[32768]{};OPENFILENAMEW ofn{};ofn.lStructSize=sizeof(ofn);ofn.hwndOwner=app.dialog;ofn.lpstrFile=file;ofn.nMaxFile=32768;
    ofn.lpstrFilter=L"PNG / JPEG 图片\0*.png;*.jpg;*.jpeg\0";ofn.Flags=OFN_FILEMUSTEXIST|OFN_PATHMUSTEXIST|OFN_NOCHANGEDIR;
    if(GetOpenFileNameW(&ofn))Launch(app,1,file);
}
static void ChooseExport(App& app){
    BROWSEINFOW browse{};browse.hwndOwner=app.dialog;browse.lpszTitle=L"选择父目录，将新建 SmokeResult 子目录";browse.ulFlags=BIF_RETURNONLYFSDIRS|BIF_NEWDIALOGSTYLE;
    PIDLIST_ABSOLUTE item=SHBrowseForFolderW(&browse);if(item){wchar_t parent[MAX_PATH]{};
        if(SHGetPathFromIDListW(item,parent))Launch(app,5,std::filesystem::path(parent)/(L"SmokeResult_"+std::to_wstring(GetTickCount64())));CoTaskMemFree(item);}
}
static INT_PTR CALLBACK DialogProc(HWND dialog,UINT message,WPARAM wParam,LPARAM lParam){
    App* app=GetApp(dialog);
    switch(message){
    case WM_INITDIALOG:
        app=reinterpret_cast<App*>(lParam);app->dialog=dialog;SetWindowLongPtrW(dialog,DWLP_USER,reinterpret_cast<LONG_PTR>(app));
        SetDlgItemTextW(dialog,IDC_AREA_THRESHOLD,L"1.0");SetDlgItemTextW(dialog,IDC_NN_THRESHOLD,L"0.5");Busy(*app,false);Layout(dialog);Report(*app);return TRUE;
    case WM_SIZE:if(app&&wParam!=SIZE_MINIMIZED)Layout(dialog);return TRUE;
    case WM_GETMINMAXINFO:{RECT rect{0,0,796,350};MapDialogRect(dialog,&rect);AdjustWindowRectEx(&rect,static_cast<DWORD>(GetWindowLongPtrW(dialog,GWL_STYLE)),FALSE,static_cast<DWORD>(GetWindowLongPtrW(dialog,GWL_EXSTYLE)));
        auto* limits=reinterpret_cast<MINMAXINFO*>(lParam);limits->ptMinTrackSize={rect.right-rect.left,rect.bottom-rect.top};return TRUE;}
    case WM_FINISHED:if(app)Finish(*app,std::unique_ptr<Work>(reinterpret_cast<Work*>(lParam)));return TRUE;
    case WM_COMMAND:
        if(!app)break;
        switch(LOWORD(wParam)){
        case IDC_OPEN:if(!app->busy)OpenImage(*app);return TRUE;
        case IDC_RUN_CV:case IDC_RUN_NN:case IDC_RUN_BOTH:
            if(!app->busy&&!app->image.bgr.empty()){
                int id=LOWORD(wParam);double area=1,threshold=0.5;
                if(id!=IDC_RUN_NN&&!Threshold(*app,IDC_AREA_THRESHOLD,area,100))return TRUE;
                if(id!=IDC_RUN_CV&&!Threshold(*app,IDC_NN_THRESHOLD,threshold,1))return TRUE;
                Launch(*app,id==IDC_RUN_CV?2:id==IDC_RUN_NN?3:4,{},area,threshold);
            }return TRUE;
        case IDC_CANCEL:app->cancel=true;Status(*app,L"已请求取消，等待当前阶段结束……");return TRUE;
        case IDC_EXPORT:if(!app->busy&&(app->cvResult||app->nnResult))ChooseExport(*app);return TRUE;
        case IDCANCEL:SendMessageW(dialog,WM_CLOSE,0,0);return TRUE;
        }break;
    case WM_CLOSE:if(app&&app->busy){app->closePending=true;app->cancel=true;Status(*app,L"正在结束后台操作并关闭……");}else EndDialog(dialog,0);return TRUE;
    }return FALSE;
}
int APIENTRY wWinMain(HINSTANCE instance,HINSTANCE,LPWSTR,int){
    CoInitializeEx(nullptr,COINIT_APARTMENTTHREADED);WNDCLASSW wc{};wc.hInstance=instance;wc.lpfnWndProc=PreviewProc;wc.lpszClassName=L"SmokeImagePreview";wc.hCursor=LoadCursorW(nullptr,IDC_ARROW);
    if(!RegisterClassW(&wc)){CoUninitialize();return 1;}
    App app;wchar_t path[32768]{};DWORD count=GetModuleFileNameW(nullptr,path,32768);
    if(!count||count>=32768){CoUninitialize();return 1;}app.model=std::filesystem::path(path).parent_path()/L"models"/L"firewatch-yolo11s.onnx";
    INT_PTR result=DialogBoxParamW(instance,MAKEINTRESOURCEW(IDD_MAIN_DIALOG),nullptr,DialogProc,reinterpret_cast<LPARAM>(&app));
    if(app.worker.joinable()){app.cancel=true;app.worker.join();}
    if(result==-1)MessageBoxW(nullptr,L"无法创建对话框，请检查资源编译和 OpenCV 运行库。",L"启动失败",MB_OK|MB_ICONERROR);
    CoUninitialize();return result==-1?1:0;
}
