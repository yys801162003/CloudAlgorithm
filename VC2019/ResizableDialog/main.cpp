#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include "resource.h"

// Convert dialog units using the dialog's actual font.
static RECT DialogUnits(HWND dialog, int x, int y, int width, int height)
{
    RECT rect = { x, y, x + width, y + height };
    MapDialogRect(dialog, &rect);
    return rect;
}

static void LayoutControls(HWND dialog)
{
    RECT client = {};
    GetClientRect(dialog, &client);
    const RECT metrics = DialogUnits(dialog, 10, 10, 60, 16);
    const int marginX = metrics.left;
    const int marginY = metrics.top;
    const int buttonWidth = metrics.right - metrics.left;
    const int buttonHeight = metrics.bottom - metrics.top;
    const int labelHeight = DialogUnits(dialog, 0, 0, 0, 14).bottom;
    const int gap = DialogUnits(dialog, 0, 0, 0, 6).bottom;
    const int width = client.right;
    const int height = client.bottom;
    const int contentWidth = (width > 2 * marginX) ? width - 2 * marginX : 0;
    const int editTop = marginY + labelHeight + gap;
    const int buttonTop = height - marginY - buttonHeight;
    const int editHeight = (buttonTop > editTop + gap) ? buttonTop - editTop - gap : 0;

    MoveWindow(GetDlgItem(dialog, IDC_SIZE_LABEL), marginX, marginY,
        contentWidth, labelHeight, TRUE);
    MoveWindow(GetDlgItem(dialog, IDC_CONTENT_EDIT), marginX, editTop,
        contentWidth, editHeight, TRUE);
    MoveWindow(GetDlgItem(dialog, IDCANCEL), width - marginX - buttonWidth,
        buttonTop, buttonWidth, buttonHeight, TRUE);

    wchar_t text[160] = {};
    wsprintfW(text, L"拖动边框或四角调整大小；当前客户区：%d × %d", width, height);
    SetDlgItemTextW(dialog, IDC_SIZE_LABEL, text);
}

static INT_PTR CALLBACK DialogProc(HWND dialog, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
    case WM_INITDIALOG:
        SetDlgItemTextW(dialog, IDC_CONTENT_EDIT,
            L"这是一个可以自由调整大小的 Win32 对话框。\r\n\r\n"
            L"拖动四边或四角，文本框将自动伸缩，关闭按钮保持在右下角。\r\n"
            L"也可以最大化、还原，或双击标题栏。\r\n\r\n"
            L"工程使用 VS2019 / v142，不依赖 MFC。");
        LayoutControls(dialog);
        return TRUE;

    case WM_SIZE:
        if (wParam != SIZE_MINIMIZED)
            LayoutControls(dialog);
        return TRUE;

    case WM_GETMINMAXINFO:
    {
        // Small minimum preserves usable controls; there is no custom maximum.
        RECT minimum = DialogUnits(dialog, 0, 0, 220, 120);
        AdjustWindowRectEx(&minimum,
            static_cast<DWORD>(GetWindowLongPtrW(dialog, GWL_STYLE)), FALSE,
            static_cast<DWORD>(GetWindowLongPtrW(dialog, GWL_EXSTYLE)));
        auto* limits = reinterpret_cast<MINMAXINFO*>(lParam);
        limits->ptMinTrackSize.x = minimum.right - minimum.left;
        limits->ptMinTrackSize.y = minimum.bottom - minimum.top;
        return TRUE;
    }

    case WM_COMMAND:
        if (LOWORD(wParam) == IDCANCEL)
        {
            EndDialog(dialog, IDCANCEL);
            return TRUE;
        }
        break;

    case WM_CLOSE:
        EndDialog(dialog, IDCANCEL);
        return TRUE;
    }
    return FALSE;
}

int APIENTRY wWinMain(HINSTANCE instance, HINSTANCE, LPWSTR, int)
{
    const INT_PTR result = DialogBoxParamW(instance,
        MAKEINTRESOURCEW(IDD_MAIN_DIALOG), nullptr, DialogProc, 0);
    if (result == -1)
    {
        MessageBoxW(nullptr, L"无法创建对话框。请检查资源文件是否正确编译。",
            L"启动失败", MB_OK | MB_ICONERROR);
        return 1;
    }
    return 0;
}
