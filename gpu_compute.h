#pragma once

#include <string>
#include <vector>

struct GpuRbfFieldRequest {
    std::vector<float> centers;
    std::vector<float> weights;
    float constantTerm = 0.0f;
    float radiusSquared = 0.0f;
    float minBounds[3] = {0.0f, 0.0f, 0.0f};
    float step[3] = {0.0f, 0.0f, 0.0f};
    int gridSize = 0;
};

struct GpuFieldResult {
    bool success = false;
    std::wstring status;
    std::vector<float> values;
};

bool GpuComputeEnabled();
bool EvaluateRbfFieldOnGpu(const GpuRbfFieldRequest& request, GpuFieldResult& result);
