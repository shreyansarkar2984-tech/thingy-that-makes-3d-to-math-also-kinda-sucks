#include "analysis.h"
#include "model.h"
#include "view.h"

#include <algorithm>
#include <chrono>
#include <cwctype>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <memory>
#include <string>

#define NOMINMAX
#include <commctrl.h>
#include <commdlg.h>
#include <windows.h>

#ifndef MODEL_DOWNSCALE_ENABLED
#define MODEL_DOWNSCALE_ENABLED 0
#endif

#ifndef MODEL_GPU_COMPUTE_ENABLED
#define MODEL_GPU_COMPUTE_ENABLED 0
#endif

namespace {

constexpr wchar_t kWindowClassName[] = L"AppWindow";
constexpr wchar_t kPanelClassName[] = L"AppPanel";
constexpr int kTimerId = 1;
constexpr int kMargin = 16;
constexpr int kControlHeight = 30;
constexpr int kPagePadding = 14;
constexpr bool kDownscaleEnabled = MODEL_DOWNSCALE_ENABLED != 0;
constexpr bool kGpuComputeEnabled = MODEL_GPU_COMPUTE_ENABLED != 0;
constexpr std::size_t kDownscaleVertexBudget = 1500;
constexpr std::size_t kDisplayVertexBudget = 60000;

enum ControlId {
    kOpenButtonId = 1001,
    kAnalyzeButtonId = 1002,
    kMarchingButtonId = 1003,
    kTabControlId = 1004,
    kWorkspacePanelId = 1005,
    kSettingsPanelId = 1006,
    kViewportId = 1007,
    kOutputEditId = 1008,
    kStatusLabelId = 1009,
    kMethodLabelId = 1010,
    kMethodComboId = 1011,
    kExportButtonId = 1012,
    kExportMeshButtonId = 1013,
};

struct AsyncFitResult {
    unsigned long long revision = 0;
    FitResult fit;
};

struct AsyncMarchingResult {
    unsigned long long revision = 0;
    MarchingCubesResult preview;
};

struct AppState {
    HWND window = nullptr;
    HWND openButton = nullptr;
    HWND analyzeButton = nullptr;
    HWND marchingButton = nullptr;
    HWND tabControl = nullptr;
    HWND workspacePanel = nullptr;
    HWND settingsPanel = nullptr;
    HWND viewport = nullptr;
    HWND outputEdit = nullptr;
    HWND statusLabel = nullptr;
    HWND methodLabel = nullptr;
    HWND methodCombo = nullptr;
    HWND exportButton = nullptr;
    HWND exportMeshButton = nullptr;
    HFONT font = nullptr;

    bool hasModel = false;
    bool fitDirty = true;
    bool fitRunning = false;
    bool previewRunning = false;
    bool showingPreview = false;
    unsigned long long modelRevision = 0;
    ModelData displayModel;
    ModelData analysisModel;
    ModelData previewModel;
    FitResult fitResult;
    std::wstring outputText;
    std::wstring statusText;
    std::wstring importSummary;
    std::future<AsyncFitResult> fitFuture;
    std::future<AsyncMarchingResult> previewFuture;
} gApp;

std::wstring BuildWelcomeText() {
    return L"Load an OBJ to begin.";
}

std::wstring BuildLoadedText(const ModelData& displayModel, const ModelData& analysisModel, const std::wstring& importSummary) {
    std::wstring text = L"Source: " + displayModel.label
        + L"\r\nView vertices: " + std::to_wstring(displayModel.VertexCount())
        + L"\r\nView triangles: " + std::to_wstring(displayModel.TriangleCount());
    if (analysisModel.VertexCount() != displayModel.VertexCount()
        || analysisModel.TriangleCount() != displayModel.TriangleCount()) {
        text += L"\r\nAnalysis vertices: " + std::to_wstring(analysisModel.VertexCount());
        text += L"\r\nAnalysis triangles: " + std::to_wstring(analysisModel.TriangleCount());
    }
    if (!importSummary.empty()) {
        text += L"\r\n" + importSummary;
    }
    return text;
}

std::wstring Lowercase(std::wstring text) {
    std::transform(text.begin(), text.end(), text.begin(), [](wchar_t value) {
        return static_cast<wchar_t>(std::towlower(value));
    });
    return text;
}

LRESULT CALLBACK PanelProc(HWND window, UINT message, WPARAM wParam, LPARAM lParam) {
    switch (message) {
        case WM_COMMAND:
        case WM_NOTIFY: {
            HWND root = GetAncestor(window, GA_ROOT);
            if (root != nullptr) {
                return SendMessageW(root, message, wParam, lParam);
            }
            break;
        }
    }
    return DefWindowProcW(window, message, wParam, lParam);
}

bool RegisterPanelClass(HINSTANCE instance) {
    WNDCLASSW panelClass{};
    panelClass.lpfnWndProc = PanelProc;
    panelClass.hInstance = instance;
    panelClass.lpszClassName = kPanelClassName;
    panelClass.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    panelClass.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    return RegisterClassW(&panelClass) != 0 || GetLastError() == ERROR_CLASS_ALREADY_EXISTS;
}

void SetControlFont(HWND control) {
    SendMessageW(control, WM_SETFONT, reinterpret_cast<WPARAM>(gApp.font), TRUE);
}

void SetOutputText(const std::wstring& text) {
    gApp.outputText = text;
    SetWindowTextW(gApp.outputEdit, gApp.outputText.c_str());
}

void SetStatusText(const std::wstring& text) {
    gApp.statusText = text;
    SetWindowTextW(gApp.statusLabel, gApp.statusText.c_str());
}

ConversionMethod CurrentMethod() {
    switch (SendMessageW(gApp.methodCombo, CB_GETCURSEL, 0, 0)) {
        case 1: return ConversionMethod::Plane;
        case 2: return ConversionMethod::Sphere;
        case 3: return ConversionMethod::Cylinder;
        case 4: return ConversionMethod::ImplicitQuadratic;
        case 5: return ConversionMethod::ImplicitCubic;
        case 6: return ConversionMethod::RbfImplicit;
        default: return ConversionMethod::AutoDetect;
    }
}

void UpdateButtons() {
    const BOOL canRun = gApp.hasModel && !gApp.fitRunning && !gApp.previewRunning;
    EnableWindow(gApp.analyzeButton, canRun);
    EnableWindow(gApp.marchingButton, canRun);
    EnableWindow(gApp.exportButton, gApp.fitResult.success ? TRUE : FALSE);
    EnableWindow(gApp.exportMeshButton, gApp.showingPreview && gApp.previewModel.TriangleCount() > 0 ? TRUE : FALSE);
}

void RefreshStatus() {
    if (!gApp.hasModel) {
        SetStatusText(L"No OBJ loaded.");
        return;
    }

    if (gApp.fitRunning) {
        SetStatusText(L"Analyzing...");
        return;
    }

    if (gApp.previewRunning) {
        SetStatusText(L"Building preview...");
        return;
    }

    if (gApp.fitDirty) {
        SetStatusText(L"OBJ loaded.");
        return;
    }

    if (gApp.showingPreview) {
        SetStatusText(L"Surface preview ready.");
        return;
    }

    if (gApp.fitResult.success) {
        SetStatusText(L"Equation ready.");
        return;
    }

    SetStatusText(L"OBJ loaded.");
}

RECT GetTabContentRect() {
    RECT rect{};
    GetClientRect(gApp.tabControl, &rect);
    TabCtrl_AdjustRect(gApp.tabControl, FALSE, &rect);
    return rect;
}

void LayoutWorkspacePanel() {
    RECT client{};
    GetClientRect(gApp.workspacePanel, &client);

    const int topRow = kPagePadding;
    MoveWindow(gApp.openButton, kPagePadding, topRow, 118, kControlHeight, TRUE);
    MoveWindow(gApp.analyzeButton, kPagePadding + 130, topRow, 92, kControlHeight, TRUE);
    MoveWindow(gApp.marchingButton, kPagePadding + 234, topRow, 126, kControlHeight, TRUE);
    MoveWindow(gApp.statusLabel, kPagePadding + 376, topRow + 6, std::max(160L, client.right - (kPagePadding + 376) - kPagePadding), 20, TRUE);

    const int contentTop = topRow + kControlHeight + 12;
    const int contentWidth = client.right - kPagePadding * 2;
    const int viewportWidth = std::min(460, std::max(320, contentWidth / 2));
    const int outputLeft = kPagePadding + viewportWidth + kPagePadding;
    const int outputWidth = std::max(260L, client.right - outputLeft - kPagePadding);

    const int bodyHeight = std::max(220L, client.bottom - contentTop - kPagePadding);
    MoveWindow(gApp.viewport, kPagePadding, contentTop, viewportWidth, bodyHeight, TRUE);
    MoveWindow(gApp.outputEdit, outputLeft, contentTop, outputWidth, bodyHeight, TRUE);
}

void LayoutSettingsPanel() {
    RECT client{};
    GetClientRect(gApp.settingsPanel, &client);

    MoveWindow(gApp.methodLabel, kPagePadding, kPagePadding + 6, 120, 18, TRUE);
    MoveWindow(gApp.methodCombo, kPagePadding, kPagePadding + 30, 280, 260, TRUE);
    MoveWindow(gApp.exportButton, kPagePadding, kPagePadding + 76, 140, kControlHeight, TRUE);
    MoveWindow(gApp.exportMeshButton, kPagePadding, kPagePadding + 116, 220, kControlHeight, TRUE);
}

void UpdateTabVisibility() {
    const int selection = TabCtrl_GetCurSel(gApp.tabControl);
    ShowWindow(gApp.workspacePanel, selection == 0 ? SW_SHOW : SW_HIDE);
    ShowWindow(gApp.settingsPanel, selection == 1 ? SW_SHOW : SW_HIDE);
}

void LayoutControls() {
    RECT client{};
    GetClientRect(gApp.window, &client);
    MoveWindow(gApp.tabControl, kMargin, kMargin, client.right - kMargin * 2, client.bottom - kMargin * 2, TRUE);

    const RECT pageRect = GetTabContentRect();
    MoveWindow(gApp.workspacePanel, pageRect.left, pageRect.top, pageRect.right - pageRect.left, pageRect.bottom - pageRect.top, TRUE);
    MoveWindow(gApp.settingsPanel, pageRect.left, pageRect.top, pageRect.right - pageRect.left, pageRect.bottom - pageRect.top, TRUE);

    LayoutWorkspacePanel();
    LayoutSettingsPanel();
    UpdateTabVisibility();
}

void MarkDirty() {
    gApp.fitDirty = true;
    if (gApp.showingPreview) {
        gApp.showingPreview = false;
        ViewportSetModel(gApp.viewport, &gApp.displayModel);
    }
    if (!gApp.fitRunning && gApp.hasModel) {
        RefreshStatus();
    }
}

void AdoptModels(ModelData&& displayModel, ModelData&& analysisModel) {
    gApp.displayModel = std::move(displayModel);
    gApp.analysisModel = std::move(analysisModel);
    gApp.previewModel = {};
    gApp.hasModel = true;
    gApp.fitDirty = true;
    gApp.showingPreview = false;
    gApp.fitResult = {};
    ++gApp.modelRevision;
    ViewportSetModel(gApp.viewport, &gApp.displayModel);
    SetOutputText(BuildLoadedText(gApp.displayModel, gApp.analysisModel, gApp.importSummary));
    RefreshStatus();
    UpdateButtons();
}

bool OpenObjDialog(HWND window, std::wstring& filePath) {
    wchar_t buffer[MAX_PATH] = L"";
    OPENFILENAMEW dialog{};
    dialog.lStructSize = sizeof(dialog);
    dialog.hwndOwner = window;
    dialog.lpstrFile = buffer;
    dialog.nMaxFile = MAX_PATH;
    dialog.lpstrFilter = L"Wavefront OBJ (*.obj)\0*.obj\0All Files (*.*)\0*.*\0";
    dialog.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_EXPLORER;
    if (!GetOpenFileNameW(&dialog)) {
        return false;
    }
    filePath = buffer;
    return true;
}

void LoadObjFromDialog(HWND window) {
    std::wstring path;
    if (!OpenObjDialog(window, path)) {
        return;
    }

    ModelData displayModel;
    std::wstring error;
    if (!LoadObjModel(path, displayModel, error)) {
        MessageBoxW(window, error.c_str(), L"Load failed", MB_OK | MB_ICONERROR);
        return;
    }

    displayModel.label = path;
    gApp.importSummary.clear();
    ModelData analysisModel = displayModel;
    if (kDownscaleEnabled) {
        ModelData downscaled;
        std::wstring summary;
        if (!DownsampleModel(displayModel, kDownscaleVertexBudget, downscaled, summary)) {
            MessageBoxW(window, summary.c_str(), L"Downscale failed", MB_OK | MB_ICONERROR);
            return;
        }
        if (downscaled.VertexCount() != displayModel.VertexCount()) {
            analysisModel = std::move(downscaled);
            if (displayModel.VertexCount() <= kDisplayVertexBudget) {
                gApp.importSummary = summary + L" View keeps the original mesh.";
            } else {
                displayModel = analysisModel;
                gApp.importSummary = summary + L" View also uses the lighter mesh.";
            }
        } else {
            gApp.importSummary = L"Model stayed at full resolution because it already fit the light budget.";
        }
    }
    AdoptModels(std::move(displayModel), std::move(analysisModel));
}

void StartAnalysis() {
    if (!gApp.hasModel || gApp.fitRunning || gApp.previewRunning) {
        return;
    }

    const auto modelCopy = std::make_shared<ModelData>(gApp.analysisModel);
    const ConversionMethod method = CurrentMethod();
    const unsigned long long revision = gApp.modelRevision;
    gApp.showingPreview = false;
    ViewportSetModel(gApp.viewport, &gApp.displayModel);
    gApp.fitRunning = true;
    RefreshStatus();
    UpdateButtons();
    SetOutputText(L"Analyzing...");
    gApp.fitFuture = std::async(std::launch::async, [modelCopy, method, revision]() {
        AsyncFitResult result;
        result.revision = revision;
        result.fit = AnalyzeModel(*modelCopy, method);
        return result;
    });
}

void StartMarchingCubes() {
    if (!gApp.hasModel || gApp.fitRunning || gApp.previewRunning) {
        return;
    }

    const auto modelCopy = std::make_shared<ModelData>(gApp.analysisModel);
    const ConversionMethod method = CurrentMethod();
    const unsigned long long revision = gApp.modelRevision;
    gApp.previewRunning = true;
    RefreshStatus();
    UpdateButtons();
    SetOutputText(L"Building preview...");
    gApp.previewFuture = std::async(std::launch::async, [modelCopy, method, revision]() {
        AsyncMarchingResult result;
        result.revision = revision;
        result.preview = GenerateMarchingCubesPreview(*modelCopy, method);
        return result;
    });
}

void PollAnalysis() {
    if (!gApp.fitRunning || !gApp.fitFuture.valid()) {
        return;
    }

    if (gApp.fitFuture.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
        return;
    }

    AsyncFitResult asyncResult = gApp.fitFuture.get();
    gApp.fitRunning = false;

    if (asyncResult.revision == gApp.modelRevision) {
        gApp.fitResult = std::move(asyncResult.fit);
        gApp.fitDirty = false;
        gApp.showingPreview = false;
        ViewportSetModel(gApp.viewport, &gApp.displayModel);
        SetOutputText(gApp.fitResult.report);
    }

    RefreshStatus();
    UpdateButtons();
}

void PollMarchingCubes() {
    if (!gApp.previewRunning || !gApp.previewFuture.valid()) {
        return;
    }

    if (gApp.previewFuture.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
        return;
    }

    AsyncMarchingResult asyncResult = gApp.previewFuture.get();
    gApp.previewRunning = false;

    if (asyncResult.revision == gApp.modelRevision) {
        gApp.fitResult = std::move(asyncResult.preview.fit);
        gApp.fitDirty = !gApp.fitResult.success;
        SetOutputText(asyncResult.preview.report);
        if (asyncResult.preview.success) {
            gApp.previewModel = std::move(asyncResult.preview.mesh);
            gApp.showingPreview = true;
            ViewportSetModel(gApp.viewport, &gApp.previewModel);
        } else {
            gApp.showingPreview = false;
            ViewportSetModel(gApp.viewport, &gApp.displayModel);
        }
    }

    RefreshStatus();
    UpdateButtons();
}

std::string Utf8FromWide(const std::wstring& text) {
    if (text.empty()) {
        return {};
    }

    const int size = WideCharToMultiByte(CP_UTF8, 0, text.c_str(), static_cast<int>(text.size()), nullptr, 0, nullptr, nullptr);
    std::string output(size, '\0');
    WideCharToMultiByte(CP_UTF8, 0, text.c_str(), static_cast<int>(text.size()), output.data(), size, nullptr, nullptr);
    return output;
}

std::filesystem::path BuildReconstructedMeshPath() {
    const std::filesystem::path sourcePath(gApp.displayModel.label);
    const std::filesystem::path directory = sourcePath.has_parent_path()
        ? sourcePath.parent_path()
        : std::filesystem::current_path();

    const std::wstring lowerName = Lowercase(sourcePath.filename().wstring());
    if (lowerName.find(L"apple") != std::wstring::npos) {
        return directory / L"apple_rbf_reconstructed.obj";
    }

    const std::wstring suffix = gApp.fitResult.resolvedMethod == ConversionMethod::RbfImplicit
        ? L"_rbf_reconstructed.obj"
        : L"_reconstructed.obj";
    const std::wstring stem = sourcePath.has_stem() ? sourcePath.stem().wstring() : L"model";
    return directory / (stem + suffix);
}

bool SaveObjMesh(const std::filesystem::path& path, const ModelData& mesh, std::wstring& errorMessage) {
    if (mesh.VertexCount() == 0 || mesh.TriangleCount() == 0) {
        errorMessage = L"No reconstructed mesh is available to export.";
        return false;
    }

    std::ofstream output(path, std::ios::binary);
    if (!output) {
        errorMessage = L"Could not create the reconstructed OBJ file.";
        return false;
    }

    output << "# Reconstructed mesh export\n";
    output << std::fixed << std::setprecision(6);
    for (std::size_t index = 0; index < mesh.VertexCount(); ++index) {
        const std::size_t offset = index * 3;
        output << "v "
               << mesh.positions[offset] << ' '
               << mesh.positions[offset + 1] << ' '
               << mesh.positions[offset + 2] << '\n';
    }

    for (std::size_t index = 0; index + 2 < mesh.triangles.size(); index += 3) {
        output << "f "
               << (mesh.triangles[index] + 1) << ' '
               << (mesh.triangles[index + 1] + 1) << ' '
               << (mesh.triangles[index + 2] + 1) << '\n';
    }

    return true;
}

void ExportReport(HWND window) {
    if (gApp.outputText.empty()) {
        return;
    }

    wchar_t buffer[MAX_PATH] = L"equation_report.txt";
    OPENFILENAMEW dialog{};
    dialog.lStructSize = sizeof(dialog);
    dialog.hwndOwner = window;
    dialog.lpstrFile = buffer;
    dialog.nMaxFile = MAX_PATH;
    dialog.lpstrFilter = L"Text Files (*.txt)\0*.txt\0All Files (*.*)\0*.*\0";
    dialog.Flags = OFN_OVERWRITEPROMPT | OFN_EXPLORER;
    if (!GetSaveFileNameW(&dialog)) {
        return;
    }

    std::ofstream output(std::filesystem::path(buffer), std::ios::binary);
    output << Utf8FromWide(gApp.outputText);
}

void ExportReconstructedMesh(HWND window) {
    if (!gApp.showingPreview || gApp.previewModel.TriangleCount() == 0) {
        MessageBoxW(window, L"Run Marching Cubes first so there is a mesh to export.", L"No mesh", MB_OK | MB_ICONINFORMATION);
        return;
    }

    const std::filesystem::path outputPath = BuildReconstructedMeshPath();
    std::wstring errorMessage;
    if (!SaveObjMesh(outputPath, gApp.previewModel, errorMessage)) {
        MessageBoxW(window, errorMessage.c_str(), L"Export failed", MB_OK | MB_ICONERROR);
        return;
    }

    SetStatusText(L"Reconstructed mesh exported.");
    const std::wstring message = L"Saved reconstructed mesh to:\n" + outputPath.wstring();
    MessageBoxW(window, message.c_str(), L"Export complete", MB_OK | MB_ICONINFORMATION);
}

LRESULT CALLBACK WindowProc(HWND window, UINT message, WPARAM wParam, LPARAM lParam) {
    switch (message) {
        case WM_CREATE: {
            gApp.window = window;
            gApp.font = static_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));

            gApp.tabControl = CreateWindowExW(0, WC_TABCONTROLW, L"", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS, 0, 0, 0, 0, window, reinterpret_cast<HMENU>(kTabControlId), nullptr, nullptr);
            gApp.workspacePanel = CreateWindowW(kPanelClassName, L"", WS_CHILD | WS_VISIBLE, 0, 0, 0, 0, gApp.tabControl, reinterpret_cast<HMENU>(kWorkspacePanelId), nullptr, nullptr);
            gApp.settingsPanel = CreateWindowW(kPanelClassName, L"", WS_CHILD, 0, 0, 0, 0, gApp.tabControl, reinterpret_cast<HMENU>(kSettingsPanelId), nullptr, nullptr);

            gApp.openButton = CreateWindowW(L"BUTTON", L"Insert OBJ", WS_CHILD | WS_VISIBLE, 0, 0, 0, 0, gApp.workspacePanel, reinterpret_cast<HMENU>(kOpenButtonId), nullptr, nullptr);
            gApp.analyzeButton = CreateWindowW(L"BUTTON", L"Analyze", WS_CHILD | WS_VISIBLE, 0, 0, 0, 0, gApp.workspacePanel, reinterpret_cast<HMENU>(kAnalyzeButtonId), nullptr, nullptr);
            gApp.marchingButton = CreateWindowW(L"BUTTON", L"Marching Cubes", WS_CHILD | WS_VISIBLE, 0, 0, 0, 0, gApp.workspacePanel, reinterpret_cast<HMENU>(kMarchingButtonId), nullptr, nullptr);
            gApp.statusLabel = CreateWindowW(L"STATIC", L"", WS_CHILD | WS_VISIBLE, 0, 0, 0, 0, gApp.workspacePanel, reinterpret_cast<HMENU>(kStatusLabelId), nullptr, nullptr);
            gApp.viewport = CreateViewportWindow(reinterpret_cast<LPCREATESTRUCTW>(lParam)->hInstance, gApp.workspacePanel, kViewportId);
            gApp.outputEdit = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"", WS_CHILD | WS_VISIBLE | ES_MULTILINE | ES_AUTOVSCROLL | ES_READONLY | WS_VSCROLL, 0, 0, 0, 0, gApp.workspacePanel, reinterpret_cast<HMENU>(kOutputEditId), nullptr, nullptr);

            gApp.methodLabel = CreateWindowW(L"STATIC", L"Method", WS_CHILD | WS_VISIBLE, 0, 0, 0, 0, gApp.settingsPanel, reinterpret_cast<HMENU>(kMethodLabelId), nullptr, nullptr);
            gApp.methodCombo = CreateWindowW(L"COMBOBOX", nullptr, WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST | WS_VSCROLL, 0, 0, 0, 0, gApp.settingsPanel, reinterpret_cast<HMENU>(kMethodComboId), nullptr, nullptr);
            gApp.exportButton = CreateWindowW(L"BUTTON", L"Export Text", WS_CHILD | WS_VISIBLE, 0, 0, 0, 0, gApp.settingsPanel, reinterpret_cast<HMENU>(kExportButtonId), nullptr, nullptr);
            gApp.exportMeshButton = CreateWindowW(L"BUTTON", L"Export Mesh", WS_CHILD | WS_VISIBLE, 0, 0, 0, 0, gApp.settingsPanel, reinterpret_cast<HMENU>(kExportMeshButtonId), nullptr, nullptr);

            for (HWND control : {gApp.tabControl, gApp.openButton, gApp.analyzeButton, gApp.marchingButton, gApp.statusLabel, gApp.outputEdit, gApp.methodLabel, gApp.methodCombo, gApp.exportButton, gApp.exportMeshButton}) {
                SetControlFont(control);
            }

            TCITEMW workspaceItem{};
            workspaceItem.mask = TCIF_TEXT;
            workspaceItem.pszText = const_cast<wchar_t*>(L"Main");
            TabCtrl_InsertItem(gApp.tabControl, 0, &workspaceItem);

            TCITEMW settingsItem{};
            settingsItem.mask = TCIF_TEXT;
            settingsItem.pszText = const_cast<wchar_t*>(L"Settings");
            TabCtrl_InsertItem(gApp.tabControl, 1, &settingsItem);

            const ConversionMethod methodOptions[] = {
                ConversionMethod::AutoDetect,
                ConversionMethod::Plane,
                ConversionMethod::Sphere,
                ConversionMethod::Cylinder,
                ConversionMethod::ImplicitQuadratic,
                ConversionMethod::ImplicitCubic,
                ConversionMethod::RbfImplicit,
            };
            for (ConversionMethod option : methodOptions) {
                const std::wstring name = ConversionMethodName(option);
                SendMessageW(gApp.methodCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(name.c_str()));
            }
            SendMessageW(gApp.methodCombo, CB_SETCURSEL, 0, 0);

            SetOutputText(BuildWelcomeText());
            RefreshStatus();
            LayoutControls();
            UpdateButtons();
            SetTimer(window, kTimerId, 80, nullptr);
            return 0;
        }

        case WM_SIZE:
            LayoutControls();
            return 0;

        case WM_GETMINMAXINFO: {
            auto* info = reinterpret_cast<MINMAXINFO*>(lParam);
            info->ptMinTrackSize.x = 1040;
            info->ptMinTrackSize.y = 660;
            return 0;
        }

        case WM_TIMER:
            PollAnalysis();
            PollMarchingCubes();
            return 0;

        case WM_NOTIFY: {
            const auto* header = reinterpret_cast<const NMHDR*>(lParam);
            if (header->idFrom == kTabControlId && header->code == TCN_SELCHANGE) {
                UpdateTabVisibility();
                return 0;
            }
            break;
        }

        case WM_COMMAND: {
            const int controlId = LOWORD(wParam);
            if (controlId == kOpenButtonId && HIWORD(wParam) == BN_CLICKED) {
                LoadObjFromDialog(window);
                return 0;
            }
            if (controlId == kAnalyzeButtonId && HIWORD(wParam) == BN_CLICKED) {
                StartAnalysis();
                return 0;
            }
            if (controlId == kMarchingButtonId && HIWORD(wParam) == BN_CLICKED) {
                StartMarchingCubes();
                return 0;
            }
            if (controlId == kExportButtonId && HIWORD(wParam) == BN_CLICKED) {
                ExportReport(window);
                return 0;
            }
            if (controlId == kExportMeshButtonId && HIWORD(wParam) == BN_CLICKED) {
                ExportReconstructedMesh(window);
                return 0;
            }
            if (controlId == kMethodComboId && HIWORD(wParam) == CBN_SELCHANGE) {
                MarkDirty();
                return 0;
            }
            break;
        }

        case WM_DESTROY:
            KillTimer(window, kTimerId);
            PostQuitMessage(0);
            return 0;
    }

    return DefWindowProcW(window, message, wParam, lParam);
}

}  // namespace

int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, PWSTR, int showCommand) {
    INITCOMMONCONTROLSEX controls{};
    controls.dwSize = sizeof(controls);
    controls.dwICC = ICC_STANDARD_CLASSES | ICC_TAB_CLASSES;
    InitCommonControlsEx(&controls);

    RegisterPanelClass(instance);
    RegisterViewportClass(instance);

    WNDCLASSW windowClass{};
    windowClass.lpfnWndProc = WindowProc;
    windowClass.hInstance = instance;
    windowClass.lpszClassName = kWindowClassName;
    windowClass.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    windowClass.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    RegisterClassW(&windowClass);

    const wchar_t* title = kGpuComputeEnabled
        ? L"Heavy"
        : (kDownscaleEnabled ? L"Light" : L"Normal");
    HWND window = CreateWindowW(kWindowClassName, title, WS_OVERLAPPEDWINDOW | WS_VISIBLE, CW_USEDEFAULT, CW_USEDEFAULT, 1260, 820, nullptr, nullptr, instance, nullptr);
    if (window == nullptr) {
        return 0;
    }

    ShowWindow(window, showCommand);
    UpdateWindow(window);

    MSG message{};
    while (GetMessageW(&message, nullptr, 0, 0) > 0) {
        TranslateMessage(&message);
        DispatchMessageW(&message);
    }
    return static_cast<int>(message.wParam);
}
