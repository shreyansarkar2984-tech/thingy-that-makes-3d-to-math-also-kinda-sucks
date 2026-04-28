#include "view.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/wglext.h>

namespace {

constexpr wchar_t kViewportClassName[] = L"ModelEquationViewport";
constexpr UINT kViewportSetModelMessage = WM_APP + 41;
constexpr UINT kViewportClearMessage = WM_APP + 42;

PFNWGLCREATECONTEXTATTRIBSARBPROC gCreateContextAttribs = nullptr;
PFNGLGENVERTEXARRAYSPROC glGenVertexArraysFn = nullptr;
PFNGLBINDVERTEXARRAYPROC glBindVertexArrayFn = nullptr;
PFNGLDELETEVERTEXARRAYSPROC glDeleteVertexArraysFn = nullptr;
PFNGLGENBUFFERSPROC glGenBuffersFn = nullptr;
PFNGLBINDBUFFERPROC glBindBufferFn = nullptr;
PFNGLBUFFERDATAPROC glBufferDataFn = nullptr;
PFNGLDELETEBUFFERSPROC glDeleteBuffersFn = nullptr;
PFNGLCREATESHADERPROC glCreateShaderFn = nullptr;
PFNGLSHADERSOURCEPROC glShaderSourceFn = nullptr;
PFNGLCOMPILESHADERPROC glCompileShaderFn = nullptr;
PFNGLGETSHADERIVPROC glGetShaderivFn = nullptr;
PFNGLGETSHADERINFOLOGPROC glGetShaderInfoLogFn = nullptr;
PFNGLCREATEPROGRAMPROC glCreateProgramFn = nullptr;
PFNGLATTACHSHADERPROC glAttachShaderFn = nullptr;
PFNGLLINKPROGRAMPROC glLinkProgramFn = nullptr;
PFNGLGETPROGRAMIVPROC glGetProgramivFn = nullptr;
PFNGLGETPROGRAMINFOLOGPROC glGetProgramInfoLogFn = nullptr;
PFNGLUSEPROGRAMPROC glUseProgramFn = nullptr;
PFNGLDELETEPROGRAMPROC glDeleteProgramFn = nullptr;
PFNGLDELETESHADERPROC glDeleteShaderFn = nullptr;
PFNGLVERTEXATTRIBPOINTERPROC glVertexAttribPointerFn = nullptr;
PFNGLENABLEVERTEXATTRIBARRAYPROC glEnableVertexAttribArrayFn = nullptr;
PFNGLGETUNIFORMLOCATIONPROC glGetUniformLocationFn = nullptr;
PFNGLUNIFORMMATRIX4FVPROC glUniformMatrix4fvFn = nullptr;
PFNGLUNIFORM3FPROC glUniform3fFn = nullptr;

struct Mat4 {
    float values[16] = {};
};

struct ViewState {
    HDC deviceContext = nullptr;
    HGLRC renderContext = nullptr;
    GLuint program = 0;
    GLuint vao = 0;
    GLuint vertexBuffer = 0;
    GLuint indexBuffer = 0;
    GLint mvpLocation = -1;
    GLint colorLocation = -1;
    std::vector<float> renderPositions;
    std::vector<std::uint32_t> renderIndices;
    int width = 1;
    int height = 1;
    float orbitYaw = 0.78f;
    float orbitPitch = -0.48f;
    float zoomDistance = 3.0f;
    bool dragging = false;
    POINT lastMouse{};
};

Mat4 IdentityMatrix() {
    Mat4 matrix{};
    matrix.values[0] = 1.0f;
    matrix.values[5] = 1.0f;
    matrix.values[10] = 1.0f;
    matrix.values[15] = 1.0f;
    return matrix;
}

Mat4 Multiply(const Mat4& left, const Mat4& right) {
    Mat4 result{};
    for (int column = 0; column < 4; ++column) {
        for (int row = 0; row < 4; ++row) {
            for (int k = 0; k < 4; ++k) {
                result.values[column * 4 + row] += left.values[k * 4 + row] * right.values[column * 4 + k];
            }
        }
    }
    return result;
}

Mat4 RotationX(float angle) {
    Mat4 matrix = IdentityMatrix();
    const float c = std::cos(angle);
    const float s = std::sin(angle);
    matrix.values[5] = c;
    matrix.values[6] = s;
    matrix.values[9] = -s;
    matrix.values[10] = c;
    return matrix;
}

Mat4 RotationY(float angle) {
    Mat4 matrix = IdentityMatrix();
    const float c = std::cos(angle);
    const float s = std::sin(angle);
    matrix.values[0] = c;
    matrix.values[2] = -s;
    matrix.values[8] = s;
    matrix.values[10] = c;
    return matrix;
}

Mat4 Translation(float x, float y, float z) {
    Mat4 matrix = IdentityMatrix();
    matrix.values[12] = x;
    matrix.values[13] = y;
    matrix.values[14] = z;
    return matrix;
}

Mat4 Perspective(float fovY, float aspect, float nearPlane, float farPlane) {
    Mat4 matrix{};
    const float f = 1.0f / std::tan(fovY * 0.5f);
    matrix.values[0] = f / aspect;
    matrix.values[5] = f;
    matrix.values[10] = (farPlane + nearPlane) / (nearPlane - farPlane);
    matrix.values[11] = -1.0f;
    matrix.values[14] = (2.0f * farPlane * nearPlane) / (nearPlane - farPlane);
    return matrix;
}

PROC LoadGlSymbol(const char* name) {
    PROC address = wglGetProcAddress(name);
    if (address == nullptr || address == reinterpret_cast<PROC>(0x1) || address == reinterpret_cast<PROC>(0x2) || address == reinterpret_cast<PROC>(0x3) || address == reinterpret_cast<PROC>(-1)) {
        static HMODULE module = GetModuleHandleW(L"opengl32.dll");
        address = GetProcAddress(module, name);
    }
    return address;
}

bool LoadModernFunctions() {
    glGenVertexArraysFn = reinterpret_cast<PFNGLGENVERTEXARRAYSPROC>(LoadGlSymbol("glGenVertexArrays"));
    glBindVertexArrayFn = reinterpret_cast<PFNGLBINDVERTEXARRAYPROC>(LoadGlSymbol("glBindVertexArray"));
    glDeleteVertexArraysFn = reinterpret_cast<PFNGLDELETEVERTEXARRAYSPROC>(LoadGlSymbol("glDeleteVertexArrays"));
    glGenBuffersFn = reinterpret_cast<PFNGLGENBUFFERSPROC>(LoadGlSymbol("glGenBuffers"));
    glBindBufferFn = reinterpret_cast<PFNGLBINDBUFFERPROC>(LoadGlSymbol("glBindBuffer"));
    glBufferDataFn = reinterpret_cast<PFNGLBUFFERDATAPROC>(LoadGlSymbol("glBufferData"));
    glDeleteBuffersFn = reinterpret_cast<PFNGLDELETEBUFFERSPROC>(LoadGlSymbol("glDeleteBuffers"));
    glCreateShaderFn = reinterpret_cast<PFNGLCREATESHADERPROC>(LoadGlSymbol("glCreateShader"));
    glShaderSourceFn = reinterpret_cast<PFNGLSHADERSOURCEPROC>(LoadGlSymbol("glShaderSource"));
    glCompileShaderFn = reinterpret_cast<PFNGLCOMPILESHADERPROC>(LoadGlSymbol("glCompileShader"));
    glGetShaderivFn = reinterpret_cast<PFNGLGETSHADERIVPROC>(LoadGlSymbol("glGetShaderiv"));
    glGetShaderInfoLogFn = reinterpret_cast<PFNGLGETSHADERINFOLOGPROC>(LoadGlSymbol("glGetShaderInfoLog"));
    glCreateProgramFn = reinterpret_cast<PFNGLCREATEPROGRAMPROC>(LoadGlSymbol("glCreateProgram"));
    glAttachShaderFn = reinterpret_cast<PFNGLATTACHSHADERPROC>(LoadGlSymbol("glAttachShader"));
    glLinkProgramFn = reinterpret_cast<PFNGLLINKPROGRAMPROC>(LoadGlSymbol("glLinkProgram"));
    glGetProgramivFn = reinterpret_cast<PFNGLGETPROGRAMIVPROC>(LoadGlSymbol("glGetProgramiv"));
    glGetProgramInfoLogFn = reinterpret_cast<PFNGLGETPROGRAMINFOLOGPROC>(LoadGlSymbol("glGetProgramInfoLog"));
    glUseProgramFn = reinterpret_cast<PFNGLUSEPROGRAMPROC>(LoadGlSymbol("glUseProgram"));
    glDeleteProgramFn = reinterpret_cast<PFNGLDELETEPROGRAMPROC>(LoadGlSymbol("glDeleteProgram"));
    glDeleteShaderFn = reinterpret_cast<PFNGLDELETESHADERPROC>(LoadGlSymbol("glDeleteShader"));
    glVertexAttribPointerFn = reinterpret_cast<PFNGLVERTEXATTRIBPOINTERPROC>(LoadGlSymbol("glVertexAttribPointer"));
    glEnableVertexAttribArrayFn = reinterpret_cast<PFNGLENABLEVERTEXATTRIBARRAYPROC>(LoadGlSymbol("glEnableVertexAttribArray"));
    glGetUniformLocationFn = reinterpret_cast<PFNGLGETUNIFORMLOCATIONPROC>(LoadGlSymbol("glGetUniformLocation"));
    glUniformMatrix4fvFn = reinterpret_cast<PFNGLUNIFORMMATRIX4FVPROC>(LoadGlSymbol("glUniformMatrix4fv"));
    glUniform3fFn = reinterpret_cast<PFNGLUNIFORM3FPROC>(LoadGlSymbol("glUniform3f"));

    return glGenVertexArraysFn && glBindVertexArrayFn && glDeleteVertexArraysFn
        && glGenBuffersFn && glBindBufferFn && glBufferDataFn && glDeleteBuffersFn
        && glCreateShaderFn && glShaderSourceFn && glCompileShaderFn && glGetShaderivFn
        && glCreateProgramFn && glAttachShaderFn && glLinkProgramFn && glGetProgramivFn
        && glUseProgramFn && glDeleteProgramFn && glDeleteShaderFn
        && glVertexAttribPointerFn && glEnableVertexAttribArrayFn
        && glGetUniformLocationFn && glUniformMatrix4fvFn && glUniform3fFn;
}

GLuint CompileShader(GLenum type, const char* source) {
    const GLuint shader = glCreateShaderFn(type);
    glShaderSourceFn(shader, 1, &source, nullptr);
    glCompileShaderFn(shader);

    GLint success = GL_FALSE;
    glGetShaderivFn(shader, GL_COMPILE_STATUS, &success);
    if (success == GL_FALSE) {
        glDeleteShaderFn(shader);
        return 0;
    }
    return shader;
}

GLuint CreateProgram() {
    constexpr const char* vertexShaderSource =
        "#version 330 core\n"
        "layout(location = 0) in vec3 aPos;\n"
        "uniform mat4 uMVP;\n"
        "void main() {\n"
        "  gl_Position = uMVP * vec4(aPos, 1.0);\n"
        "  gl_PointSize = 5.0;\n"
        "}\n";

    constexpr const char* fragmentShaderSource =
        "#version 330 core\n"
        "uniform vec3 uColor;\n"
        "out vec4 FragColor;\n"
        "void main() {\n"
        "  FragColor = vec4(uColor, 1.0);\n"
        "}\n";

    const GLuint vertexShader = CompileShader(GL_VERTEX_SHADER, vertexShaderSource);
    const GLuint fragmentShader = CompileShader(GL_FRAGMENT_SHADER, fragmentShaderSource);
    if (vertexShader == 0 || fragmentShader == 0) {
        if (vertexShader != 0) {
            glDeleteShaderFn(vertexShader);
        }
        if (fragmentShader != 0) {
            glDeleteShaderFn(fragmentShader);
        }
        return 0;
    }

    const GLuint program = glCreateProgramFn();
    glAttachShaderFn(program, vertexShader);
    glAttachShaderFn(program, fragmentShader);
    glLinkProgramFn(program);

    GLint success = GL_FALSE;
    glGetProgramivFn(program, GL_LINK_STATUS, &success);
    glDeleteShaderFn(vertexShader);
    glDeleteShaderFn(fragmentShader);
    if (success == GL_FALSE) {
        glDeleteProgramFn(program);
        return 0;
    }

    return program;
}

bool EnsureWglCreateContextLoaded() {
    if (gCreateContextAttribs != nullptr) {
        return true;
    }

    WNDCLASSW dummyClass{};
    dummyClass.lpfnWndProc = DefWindowProcW;
    dummyClass.hInstance = GetModuleHandleW(nullptr);
    dummyClass.lpszClassName = L"DummyOpenGLLoaderWindow";
    dummyClass.style = CS_OWNDC;
    RegisterClassW(&dummyClass);

    HWND dummyWindow = CreateWindowW(dummyClass.lpszClassName, L"", WS_OVERLAPPED, 0, 0, 1, 1, nullptr, nullptr, dummyClass.hInstance, nullptr);
    if (dummyWindow == nullptr) {
        return false;
    }

    HDC dummyDc = GetDC(dummyWindow);
    PIXELFORMATDESCRIPTOR pfd{};
    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 32;
    pfd.cDepthBits = 24;
    pfd.iLayerType = PFD_MAIN_PLANE;

    const int pixelFormat = ChoosePixelFormat(dummyDc, &pfd);
    SetPixelFormat(dummyDc, pixelFormat, &pfd);

    HGLRC dummyContext = wglCreateContext(dummyDc);
    wglMakeCurrent(dummyDc, dummyContext);
    gCreateContextAttribs = reinterpret_cast<PFNWGLCREATECONTEXTATTRIBSARBPROC>(wglGetProcAddress("wglCreateContextAttribsARB"));
    wglMakeCurrent(nullptr, nullptr);
    wglDeleteContext(dummyContext);
    ReleaseDC(dummyWindow, dummyDc);
    DestroyWindow(dummyWindow);
    return gCreateContextAttribs != nullptr;
}

ViewState* GetState(HWND window) {
    return reinterpret_cast<ViewState*>(GetWindowLongPtrW(window, GWLP_USERDATA));
}

int MouseXFromLParam(LPARAM value) {
    return static_cast<int>(static_cast<short>(LOWORD(value)));
}

int MouseYFromLParam(LPARAM value) {
    return static_cast<int>(static_cast<short>(HIWORD(value)));
}

void UploadModel(ViewState& state, const ModelData& model) {
    state.renderPositions.clear();
    state.renderIndices = model.lineIndices;
    state.renderPositions.reserve(model.positions.size());

    for (std::size_t index = 0; index < model.VertexCount(); ++index) {
        const float x = (model.positions[index * 3] - model.center[0]) / model.scale;
        const float y = (model.positions[index * 3 + 1] - model.center[1]) / model.scale;
        const float z = (model.positions[index * 3 + 2] - model.center[2]) / model.scale;
        state.renderPositions.push_back(x);
        state.renderPositions.push_back(y);
        state.renderPositions.push_back(z);
    }

    wglMakeCurrent(state.deviceContext, state.renderContext);
    glBindVertexArrayFn(state.vao);
    glBindBufferFn(GL_ARRAY_BUFFER, state.vertexBuffer);
    glBufferDataFn(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(state.renderPositions.size() * sizeof(float)), state.renderPositions.data(), GL_STATIC_DRAW);

    glBindBufferFn(GL_ELEMENT_ARRAY_BUFFER, state.indexBuffer);
    const void* indexData = state.renderIndices.empty() ? nullptr : state.renderIndices.data();
    glBufferDataFn(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(state.renderIndices.size() * sizeof(std::uint32_t)), indexData, GL_STATIC_DRAW);
    wglMakeCurrent(nullptr, nullptr);
}

bool InitializeView(HWND window, ViewState& state) {
    auto fail = [&]() {
        if (state.renderContext != nullptr) {
            wglMakeCurrent(nullptr, nullptr);
            wglDeleteContext(state.renderContext);
            state.renderContext = nullptr;
        }
        if (state.deviceContext != nullptr) {
            ReleaseDC(window, state.deviceContext);
            state.deviceContext = nullptr;
        }
        return false;
    };

    state.deviceContext = GetDC(window);
    PIXELFORMATDESCRIPTOR pfd{};
    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 32;
    pfd.cDepthBits = 24;
    pfd.iLayerType = PFD_MAIN_PLANE;

    const int pixelFormat = ChoosePixelFormat(state.deviceContext, &pfd);
    if (pixelFormat == 0 || !SetPixelFormat(state.deviceContext, pixelFormat, &pfd)) {
        return fail();
    }

    EnsureWglCreateContextLoaded();
    if (gCreateContextAttribs != nullptr) {
        const int attributes[] = {
            WGL_CONTEXT_MAJOR_VERSION_ARB, 3,
            WGL_CONTEXT_MINOR_VERSION_ARB, 3,
            WGL_CONTEXT_PROFILE_MASK_ARB, WGL_CONTEXT_CORE_PROFILE_BIT_ARB,
            0,
        };
        state.renderContext = gCreateContextAttribs(state.deviceContext, nullptr, attributes);
    }

    if (state.renderContext == nullptr) {
        state.renderContext = wglCreateContext(state.deviceContext);
    }
    if (state.renderContext == nullptr || !wglMakeCurrent(state.deviceContext, state.renderContext)) {
        return fail();
    }

    if (!LoadModernFunctions()) {
        return fail();
    }

    state.program = CreateProgram();
    if (state.program == 0) {
        return fail();
    }

    glGenVertexArraysFn(1, &state.vao);
    glBindVertexArrayFn(state.vao);
    glGenBuffersFn(1, &state.vertexBuffer);
    glGenBuffersFn(1, &state.indexBuffer);

    glBindBufferFn(GL_ARRAY_BUFFER, state.vertexBuffer);
    glVertexAttribPointerFn(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    glEnableVertexAttribArrayFn(0);

    glBindBufferFn(GL_ELEMENT_ARRAY_BUFFER, state.indexBuffer);
    state.mvpLocation = glGetUniformLocationFn(state.program, "uMVP");
    state.colorLocation = glGetUniformLocationFn(state.program, "uColor");
    glEnable(GL_DEPTH_TEST);
    wglMakeCurrent(nullptr, nullptr);
    return true;
}

void DestroyView(ViewState& state) {
    if (state.deviceContext != nullptr && state.renderContext != nullptr) {
        wglMakeCurrent(state.deviceContext, state.renderContext);
        if (state.program != 0) {
            glDeleteProgramFn(state.program);
        }
        if (state.vertexBuffer != 0) {
            glDeleteBuffersFn(1, &state.vertexBuffer);
        }
        if (state.indexBuffer != 0) {
            glDeleteBuffersFn(1, &state.indexBuffer);
        }
        if (state.vao != 0) {
            glDeleteVertexArraysFn(1, &state.vao);
        }
        wglMakeCurrent(nullptr, nullptr);
        wglDeleteContext(state.renderContext);
        state.renderContext = nullptr;
    }

    if (state.deviceContext != nullptr) {
        ReleaseDC(WindowFromDC(state.deviceContext), state.deviceContext);
        state.deviceContext = nullptr;
    }
}

void Render(ViewState& state) {
    if (state.renderContext == nullptr) {
        return;
    }

    wglMakeCurrent(state.deviceContext, state.renderContext);
    glViewport(0, 0, std::max(1, state.width), std::max(1, state.height));
    glClearColor(0.94f, 0.96f, 0.99f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgramFn(state.program);
    const float aspect = static_cast<float>(std::max(1, state.width)) / static_cast<float>(std::max(1, state.height));
    const Mat4 projection = Perspective(1.0f, aspect, 0.1f, 20.0f);
    const Mat4 view = Translation(0.0f, 0.0f, -state.zoomDistance);
    const Mat4 model = Multiply(RotationY(state.orbitYaw), RotationX(state.orbitPitch));
    const Mat4 mvp = Multiply(projection, Multiply(view, model));

    glUniformMatrix4fvFn(state.mvpLocation, 1, GL_FALSE, mvp.values);
    glUniform3fFn(state.colorLocation, 0.16f, 0.38f, 0.69f);

    glBindVertexArrayFn(state.vao);
    if (!state.renderIndices.empty()) {
        glDrawElements(GL_LINES, static_cast<GLsizei>(state.renderIndices.size()), GL_UNSIGNED_INT, nullptr);
    } else if (!state.renderPositions.empty()) {
        glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(state.renderPositions.size() / 3));
    }

    SwapBuffers(state.deviceContext);
    wglMakeCurrent(nullptr, nullptr);
}

LRESULT CALLBACK ViewportProc(HWND window, UINT message, WPARAM wParam, LPARAM lParam) {
    switch (message) {
        case WM_CREATE: {
            auto* state = new ViewState();
            SetWindowLongPtrW(window, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(state));
            InitializeView(window, *state);
            return 0;
        }

        case WM_SIZE: {
            if (ViewState* state = GetState(window)) {
                state->width = LOWORD(lParam);
                state->height = HIWORD(lParam);
                InvalidateRect(window, nullptr, FALSE);
            }
            return 0;
        }

        case WM_ERASEBKGND:
            return 1;

        case WM_LBUTTONDOWN: {
            if (ViewState* state = GetState(window)) {
                state->dragging = true;
                state->lastMouse.x = MouseXFromLParam(lParam);
                state->lastMouse.y = MouseYFromLParam(lParam);
                SetFocus(window);
                SetCapture(window);
            }
            return 0;
        }

        case WM_MOUSEMOVE: {
            if (ViewState* state = GetState(window); state != nullptr && state->dragging) {
                const int currentX = MouseXFromLParam(lParam);
                const int currentY = MouseYFromLParam(lParam);
                const int deltaX = currentX - state->lastMouse.x;
                const int deltaY = currentY - state->lastMouse.y;
                state->lastMouse.x = currentX;
                state->lastMouse.y = currentY;

                state->orbitYaw += static_cast<float>(deltaX) * 0.012f;
                state->orbitPitch += static_cast<float>(deltaY) * 0.012f;
                state->orbitPitch = std::clamp(state->orbitPitch, -1.45f, 1.45f);
                InvalidateRect(window, nullptr, FALSE);
            }
            return 0;
        }

        case WM_LBUTTONUP: {
            if (ViewState* state = GetState(window)) {
                state->dragging = false;
            }
            ReleaseCapture();
            return 0;
        }

        case WM_CAPTURECHANGED: {
            if (ViewState* state = GetState(window)) {
                state->dragging = false;
            }
            return 0;
        }

        case WM_MOUSEWHEEL: {
            if (ViewState* state = GetState(window)) {
                const float wheelDelta = static_cast<float>(GET_WHEEL_DELTA_WPARAM(wParam)) / static_cast<float>(WHEEL_DELTA);
                state->zoomDistance -= wheelDelta * 0.22f;
                state->zoomDistance = std::clamp(state->zoomDistance, 1.1f, 8.0f);
                InvalidateRect(window, nullptr, FALSE);
            }
            return 0;
        }

        case WM_PAINT: {
            PAINTSTRUCT paint{};
            BeginPaint(window, &paint);
            if (ViewState* state = GetState(window)) {
                Render(*state);
            }
            EndPaint(window, &paint);
            return 0;
        }

        case kViewportSetModelMessage: {
            if (ViewState* state = GetState(window)) {
                const auto* model = reinterpret_cast<const ModelData*>(lParam);
                if (model != nullptr) {
                    UploadModel(*state, *model);
                    InvalidateRect(window, nullptr, FALSE);
                }
            }
            return 0;
        }

        case kViewportClearMessage: {
            if (ViewState* state = GetState(window)) {
                state->renderPositions.clear();
                state->renderIndices.clear();
                UploadModel(*state, ModelData{});
                InvalidateRect(window, nullptr, FALSE);
            }
            return 0;
        }

        case WM_DESTROY: {
            if (ViewState* state = GetState(window)) {
                DestroyView(*state);
                delete state;
                SetWindowLongPtrW(window, GWLP_USERDATA, 0);
            }
            return 0;
        }
    }

    return DefWindowProcW(window, message, wParam, lParam);
}

}  // namespace

bool RegisterViewportClass(HINSTANCE instance) {
    WNDCLASSW windowClass{};
    windowClass.lpfnWndProc = ViewportProc;
    windowClass.hInstance = instance;
    windowClass.lpszClassName = kViewportClassName;
    windowClass.style = CS_OWNDC;
    windowClass.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    return RegisterClassW(&windowClass) != 0 || GetLastError() == ERROR_CLASS_ALREADY_EXISTS;
}

HWND CreateViewportWindow(HINSTANCE instance, HWND parent, int controlId) {
    return CreateWindowW(kViewportClassName, L"", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | WS_CLIPCHILDREN, 0, 0, 0, 0, parent, reinterpret_cast<HMENU>(controlId), instance, nullptr);
}

void ViewportSetModel(HWND viewport, const ModelData* model) {
    SendMessageW(viewport, kViewportSetModelMessage, 0, reinterpret_cast<LPARAM>(model));
}

void ViewportClear(HWND viewport) {
    SendMessageW(viewport, kViewportClearMessage, 0, 0);
}
