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
