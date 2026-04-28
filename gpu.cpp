#include "gpu.h"

#ifndef MODEL_GPU_COMPUTE_ENABLED
#define MODEL_GPU_COMPUTE_ENABLED 0
#endif

#include <algorithm>
#include <cstring>

#define NOMINMAX
#include <windows.h>

#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/wglext.h>

namespace {

constexpr wchar_t kGpuWindowClassName[] = L"ModelEquationGpuComputeWindow";

PFNWGLCREATECONTEXTATTRIBSARBPROC gCreateContextAttribs = nullptr;
PFNGLGENBUFFERSPROC glGenBuffersFn = nullptr;
PFNGLBINDBUFFERPROC glBindBufferFn = nullptr;
PFNGLBUFFERDATAPROC glBufferDataFn = nullptr;
PFNGLDELETEBUFFERSPROC glDeleteBuffersFn = nullptr;
PFNGLBINDBUFFERBASEPROC glBindBufferBaseFn = nullptr;
PFNGLMAPBUFFERRANGEPROC glMapBufferRangeFn = nullptr;
PFNGLUNMAPBUFFERPROC glUnmapBufferFn = nullptr;
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
PFNGLGETUNIFORMLOCATIONPROC glGetUniformLocationFn = nullptr;
PFNGLUNIFORM1IPROC glUniform1iFn = nullptr;
PFNGLUNIFORM1FPROC glUniform1fFn = nullptr;
PFNGLUNIFORM3FPROC glUniform3fFn = nullptr;
PFNGLDISPATCHCOMPUTEPROC glDispatchComputeFn = nullptr;
PFNGLMEMORYBARRIERPROC glMemoryBarrierFn = nullptr;

PROC LoadGlSymbol(const char* name) {
    PROC address = wglGetProcAddress(name);
    if (address == nullptr || address == reinterpret_cast<PROC>(0x1) || address == reinterpret_cast<PROC>(0x2) || address == reinterpret_cast<PROC>(0x3) || address == reinterpret_cast<PROC>(-1)) {
        static HMODULE module = GetModuleHandleW(L"opengl32.dll");
        address = GetProcAddress(module, name);
    }
    return address;
}

bool LoadComputeFunctions() {
    glGenBuffersFn = reinterpret_cast<PFNGLGENBUFFERSPROC>(LoadGlSymbol("glGenBuffers"));
    glBindBufferFn = reinterpret_cast<PFNGLBINDBUFFERPROC>(LoadGlSymbol("glBindBuffer"));
    glBufferDataFn = reinterpret_cast<PFNGLBUFFERDATAPROC>(LoadGlSymbol("glBufferData"));
    glDeleteBuffersFn = reinterpret_cast<PFNGLDELETEBUFFERSPROC>(LoadGlSymbol("glDeleteBuffers"));
    glBindBufferBaseFn = reinterpret_cast<PFNGLBINDBUFFERBASEPROC>(LoadGlSymbol("glBindBufferBase"));
    glMapBufferRangeFn = reinterpret_cast<PFNGLMAPBUFFERRANGEPROC>(LoadGlSymbol("glMapBufferRange"));
    glUnmapBufferFn = reinterpret_cast<PFNGLUNMAPBUFFERPROC>(LoadGlSymbol("glUnmapBuffer"));
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
    glGetUniformLocationFn = reinterpret_cast<PFNGLGETUNIFORMLOCATIONPROC>(LoadGlSymbol("glGetUniformLocation"));
    glUniform1iFn = reinterpret_cast<PFNGLUNIFORM1IPROC>(LoadGlSymbol("glUniform1i"));
    glUniform1fFn = reinterpret_cast<PFNGLUNIFORM1FPROC>(LoadGlSymbol("glUniform1f"));
    glUniform3fFn = reinterpret_cast<PFNGLUNIFORM3FPROC>(LoadGlSymbol("glUniform3f"));
    glDispatchComputeFn = reinterpret_cast<PFNGLDISPATCHCOMPUTEPROC>(LoadGlSymbol("glDispatchCompute"));
    glMemoryBarrierFn = reinterpret_cast<PFNGLMEMORYBARRIERPROC>(LoadGlSymbol("glMemoryBarrier"));

    return glGenBuffersFn && glBindBufferFn && glBufferDataFn && glDeleteBuffersFn
        && glBindBufferBaseFn && glMapBufferRangeFn && glUnmapBufferFn
        && glCreateShaderFn && glShaderSourceFn && glCompileShaderFn && glGetShaderivFn && glGetShaderInfoLogFn
        && glCreateProgramFn && glAttachShaderFn && glLinkProgramFn && glGetProgramivFn && glGetProgramInfoLogFn
        && glUseProgramFn && glDeleteProgramFn && glDeleteShaderFn
        && glGetUniformLocationFn && glUniform1iFn && glUniform1fFn && glUniform3fFn
        && glDispatchComputeFn && glMemoryBarrierFn;
}

LRESULT CALLBACK HiddenWindowProc(HWND window, UINT message, WPARAM wParam, LPARAM lParam) {
    return DefWindowProcW(window, message, wParam, lParam);
}

bool RegisterHiddenWindowClass(HINSTANCE instance) {
    WNDCLASSW windowClass{};
    windowClass.lpfnWndProc = HiddenWindowProc;
    windowClass.hInstance = instance;
    windowClass.lpszClassName = kGpuWindowClassName;
    windowClass.style = CS_OWNDC;
    return RegisterClassW(&windowClass) != 0 || GetLastError() == ERROR_CLASS_ALREADY_EXISTS;
}

bool EnsureCreateContextLoaded(HINSTANCE instance) {
    if (gCreateContextAttribs != nullptr) {
        return true;
    }

    if (!RegisterHiddenWindowClass(instance)) {
        return false;
    }

    HWND dummyWindow = CreateWindowW(kGpuWindowClassName, L"", WS_OVERLAPPED, 0, 0, 1, 1, nullptr, nullptr, instance, nullptr);
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

GLuint CreateComputeProgram() {
    constexpr const char* computeShaderSource =
        "#version 430 core\n"
        "layout(local_size_x = 64) in;\n"
        "layout(std430, binding = 0) readonly buffer CenterBuffer { vec4 centers[]; };\n"
        "layout(std430, binding = 1) readonly buffer WeightBuffer { float weights[]; };\n"
        "layout(std430, binding = 2) writeonly buffer OutputBuffer { float values[]; };\n"
        "uniform int uGridSize;\n"
        "uniform int uCenterCount;\n"
        "uniform vec3 uMinBounds;\n"
        "uniform vec3 uStep;\n"
        "uniform float uRadiusSquared;\n"
        "uniform float uConstantTerm;\n"
        "void main() {\n"
        "  uint total = uint(uGridSize * uGridSize * uGridSize);\n"
        "  uint index = gl_GlobalInvocationID.x;\n"
        "  if (index >= total) {\n"
        "    return;\n"
        "  }\n"
        "  uint gx = index % uint(uGridSize);\n"
        "  uint gy = (index / uint(uGridSize)) % uint(uGridSize);\n"
        "  uint gz = index / uint(uGridSize * uGridSize);\n"
        "  vec3 point = uMinBounds + vec3(gx, gy, gz) * uStep;\n"
        "  float value = uConstantTerm;\n"
        "  for (int centerIndex = 0; centerIndex < uCenterCount; ++centerIndex) {\n"
        "    vec3 delta = point - centers[centerIndex].xyz;\n"
        "    value += weights[centerIndex] * exp(-dot(delta, delta) / uRadiusSquared);\n"
        "  }\n"
        "  values[index] = value;\n"
        "}\n";

    const GLuint shader = CompileShader(GL_COMPUTE_SHADER, computeShaderSource);
    if (shader == 0) {
        return 0;
    }

    const GLuint program = glCreateProgramFn();
    glAttachShaderFn(program, shader);
    glLinkProgramFn(program);

    GLint success = GL_FALSE;
    glGetProgramivFn(program, GL_LINK_STATUS, &success);
    glDeleteShaderFn(shader);
    if (success == GL_FALSE) {
        glDeleteProgramFn(program);
        return 0;
    }

    return program;
}

bool CreateComputeContext(HINSTANCE instance, HWND& outWindow, HDC& outDc, HGLRC& outContext) {
    outWindow = nullptr;
    outDc = nullptr;
    outContext = nullptr;

    if (!RegisterHiddenWindowClass(instance) || !EnsureCreateContextLoaded(instance)) {
        return false;
    }

    outWindow = CreateWindowW(kGpuWindowClassName, L"", WS_OVERLAPPED, 0, 0, 1, 1, nullptr, nullptr, instance, nullptr);
    if (outWindow == nullptr) {
        return false;
    }

    outDc = GetDC(outWindow);
    PIXELFORMATDESCRIPTOR pfd{};
    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 32;
    pfd.cDepthBits = 24;
    pfd.iLayerType = PFD_MAIN_PLANE;

    const int pixelFormat = ChoosePixelFormat(outDc, &pfd);
    if (pixelFormat == 0 || !SetPixelFormat(outDc, pixelFormat, &pfd)) {
        return false;
    }

    const int attributes[] = {
        WGL_CONTEXT_MAJOR_VERSION_ARB, 4,
        WGL_CONTEXT_MINOR_VERSION_ARB, 3,
        WGL_CONTEXT_PROFILE_MASK_ARB, WGL_CONTEXT_CORE_PROFILE_BIT_ARB,
        0,
    };
    outContext = gCreateContextAttribs(outDc, nullptr, attributes);
    if (outContext == nullptr || !wglMakeCurrent(outDc, outContext)) {
        return false;
    }

    if (!LoadComputeFunctions()) {
        return false;
    }

    return true;
}

void DestroyComputeContext(HWND window, HDC dc, HGLRC context) {
    if (context != nullptr) {
        wglMakeCurrent(nullptr, nullptr);
        wglDeleteContext(context);
    }
    if (window != nullptr && dc != nullptr) {
        ReleaseDC(window, dc);
    }
    if (window != nullptr) {
        DestroyWindow(window);
    }
}

}  // namespace

bool GpuComputeEnabled() {
    return MODEL_GPU_COMPUTE_ENABLED != 0;
}

bool EvaluateRbfFieldOnGpu(const GpuRbfFieldRequest& request, GpuFieldResult& result) {
    result = {};

    if (!GpuComputeEnabled()) {
        result.status = L"GPU compute build disabled.";
        return false;
    }

    if (request.gridSize <= 0 || request.centers.empty() || request.weights.empty() || request.centers.size() != request.weights.size() * 3 || request.radiusSquared <= 0.0f) {
        result.status = L"GPU compute request was incomplete.";
        return false;
    }

    HINSTANCE instance = GetModuleHandleW(nullptr);
    HWND window = nullptr;
    HDC dc = nullptr;
    HGLRC context = nullptr;
    if (!CreateComputeContext(instance, window, dc, context)) {
        DestroyComputeContext(window, dc, context);
        result.status = L"OpenGL 4.3 compute support was not available.";
        return false;
    }

    GLuint program = CreateComputeProgram();
    if (program == 0) {
        DestroyComputeContext(window, dc, context);
        result.status = L"GPU compute shader compilation failed.";
        return false;
    }

    const std::size_t centerCount = request.weights.size();
    const std::size_t totalCount = static_cast<std::size_t>(request.gridSize) * static_cast<std::size_t>(request.gridSize) * static_cast<std::size_t>(request.gridSize);
    std::vector<float> centers4;
    centers4.reserve(centerCount * 4);
    for (std::size_t index = 0; index < centerCount; ++index) {
        centers4.push_back(request.centers[index * 3]);
        centers4.push_back(request.centers[index * 3 + 1]);
        centers4.push_back(request.centers[index * 3 + 2]);
        centers4.push_back(0.0f);
    }

    GLuint buffers[3] = {};
    glGenBuffersFn(3, buffers);

    glBindBufferFn(GL_SHADER_STORAGE_BUFFER, buffers[0]);
    glBufferDataFn(GL_SHADER_STORAGE_BUFFER, static_cast<GLsizeiptr>(centers4.size() * sizeof(float)), centers4.data(), GL_STATIC_DRAW);
    glBindBufferBaseFn(GL_SHADER_STORAGE_BUFFER, 0, buffers[0]);

    glBindBufferFn(GL_SHADER_STORAGE_BUFFER, buffers[1]);
    glBufferDataFn(GL_SHADER_STORAGE_BUFFER, static_cast<GLsizeiptr>(request.weights.size() * sizeof(float)), request.weights.data(), GL_STATIC_DRAW);
    glBindBufferBaseFn(GL_SHADER_STORAGE_BUFFER, 1, buffers[1]);

    glBindBufferFn(GL_SHADER_STORAGE_BUFFER, buffers[2]);
    glBufferDataFn(GL_SHADER_STORAGE_BUFFER, static_cast<GLsizeiptr>(totalCount * sizeof(float)), nullptr, GL_DYNAMIC_READ);
    glBindBufferBaseFn(GL_SHADER_STORAGE_BUFFER, 2, buffers[2]);

    glUseProgramFn(program);
    glUniform1iFn(glGetUniformLocationFn(program, "uGridSize"), request.gridSize);
    glUniform1iFn(glGetUniformLocationFn(program, "uCenterCount"), static_cast<GLint>(centerCount));
    glUniform3fFn(glGetUniformLocationFn(program, "uMinBounds"), request.minBounds[0], request.minBounds[1], request.minBounds[2]);
    glUniform3fFn(glGetUniformLocationFn(program, "uStep"), request.step[0], request.step[1], request.step[2]);
    glUniform1fFn(glGetUniformLocationFn(program, "uRadiusSquared"), request.radiusSquared);
    glUniform1fFn(glGetUniformLocationFn(program, "uConstantTerm"), request.constantTerm);

    const GLuint groupCount = static_cast<GLuint>((totalCount + 63u) / 64u);
    glDispatchComputeFn(groupCount, 1, 1);
    glMemoryBarrierFn(GL_SHADER_STORAGE_BARRIER_BIT | GL_BUFFER_UPDATE_BARRIER_BIT);

    glBindBufferFn(GL_SHADER_STORAGE_BUFFER, buffers[2]);
    void* mapped = glMapBufferRangeFn(GL_SHADER_STORAGE_BUFFER, 0, static_cast<GLsizeiptr>(totalCount * sizeof(float)), GL_MAP_READ_BIT);
    if (mapped == nullptr) {
        glDeleteBuffersFn(3, buffers);
        glDeleteProgramFn(program);
        DestroyComputeContext(window, dc, context);
        result.status = L"GPU field buffer could not be read back.";
        return false;
    }

    result.values.resize(totalCount);
    std::memcpy(result.values.data(), mapped, totalCount * sizeof(float));
    glUnmapBufferFn(GL_SHADER_STORAGE_BUFFER);

    glDeleteBuffersFn(3, buffers);
    glDeleteProgramFn(program);
    DestroyComputeContext(window, dc, context);

    result.success = true;
    result.status = L"GPU compute field sampling succeeded.";
    return true;
}
