@echo off
setlocal

if not exist build (
    mkdir build
)

set BUILD_TAG=%RANDOM%
set BUILD_DIR=build\run_%BUILD_TAG%
mkdir %BUILD_DIR%

clang++ ^
  -std=c++20 ^
  -O2 ^
  -Wall ^
  -Wextra ^
  -municode ^
  -DUNICODE ^
  -D_UNICODE ^
  code\simple_main.cpp ^
  code\model_loader.cpp ^
  code\analysis_engine.cpp ^
  code\gpu_compute.cpp ^
  code\renderer.cpp ^
  -lgdi32 ^
  -lcomdlg32 ^
  -lcomctl32 ^
  -lopengl32 ^
  -mwindows ^
  -o %BUILD_DIR%\normal.exe

if errorlevel 1 (
    exit /b 1
)

clang++ ^
  -std=c++20 ^
  -O2 ^
  -Wall ^
  -Wextra ^
  -municode ^
  -DUNICODE ^
  -D_UNICODE ^
  -DMODEL_DOWNSCALE_ENABLED=1 ^
  code\simple_main.cpp ^
  code\model_loader.cpp ^
  code\analysis_engine.cpp ^
  code\gpu_compute.cpp ^
  code\renderer.cpp ^
  -lgdi32 ^
  -lcomdlg32 ^
  -lcomctl32 ^
  -lopengl32 ^
  -mwindows ^
  -o %BUILD_DIR%\light.exe

if errorlevel 1 (
    exit /b 1
)

clang++ ^
  -std=c++20 ^
  -O2 ^
  -Wall ^
  -Wextra ^
  -municode ^
  -DUNICODE ^
  -D_UNICODE ^
  -DMODEL_GPU_COMPUTE_ENABLED=1 ^
  code\simple_main.cpp ^
  code\model_loader.cpp ^
  code\analysis_engine.cpp ^
  code\gpu_compute.cpp ^
  code\renderer.cpp ^
  -lgdi32 ^
  -lcomdlg32 ^
  -lcomctl32 ^
  -lopengl32 ^
  -mwindows ^
  -o %BUILD_DIR%\heavy.exe

if errorlevel 1 (
    exit /b 1
)

echo Built %BUILD_DIR%\normal.exe
echo Built %BUILD_DIR%\light.exe
echo Built %BUILD_DIR%\heavy.exe
