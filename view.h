#pragma once

#include "model.h"

#define NOMINMAX
#include <windows.h>

bool RegisterViewportClass(HINSTANCE instance);
HWND CreateViewportWindow(HINSTANCE instance, HWND parent, int controlId);
void ViewportSetModel(HWND viewport, const ModelData* model);
void ViewportClear(HWND viewport);
