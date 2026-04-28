#pragma once

#include "model.h"

#include <chrono>
#include <cstddef>
#include <string>

struct FitResult {
    bool success = false;
    ConversionMethod requestedMethod = ConversionMethod::AutoDetect;
    ConversionMethod resolvedMethod = ConversionMethod::AutoDetect;
    std::size_t inputPointCount = 0;
    std::size_t triangleCount = 0;
    double meanAbsoluteResidual = 0.0;
    double rmsResidual = 0.0;
    double maxResidual = 0.0;
    double nearestNeighborMean = 0.0;
    double nearestNeighborMax = 0.0;
    double elapsedMilliseconds = 0.0;
    std::wstring equation;
    std::wstring report;
};

struct MarchingCubesResult {
    bool success = false;
    FitResult fit;
    ModelData mesh;
    std::wstring report;
};

FitResult AnalyzeModel(const ModelData& model, ConversionMethod method);
MarchingCubesResult GenerateMarchingCubesPreview(const ModelData& model, ConversionMethod method);
