#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

enum class ConversionMethod {
    AutoDetect,
    Plane,
    Sphere,
    Cylinder,
    ImplicitQuadratic,
    ImplicitCubic,
    RbfImplicit,
};

struct ModelData {
    std::vector<float> positions;
    std::vector<std::uint32_t> triangles;
    std::vector<std::uint32_t> lineIndices;
    std::wstring label;
    float boundsMin[3] = {0.0f, 0.0f, 0.0f};
    float boundsMax[3] = {0.0f, 0.0f, 0.0f};
    float center[3] = {0.0f, 0.0f, 0.0f};
    float scale = 1.0f;

    std::size_t VertexCount() const { return positions.size() / 3; }
    std::size_t TriangleCount() const { return triangles.size() / 3; }
    std::size_t MemoryBytes() const {
        return positions.size() * sizeof(float)
            + triangles.size() * sizeof(std::uint32_t)
            + lineIndices.size() * sizeof(std::uint32_t);
    }
};

bool LoadObjModel(const std::filesystem::path& path, ModelData& outModel, std::wstring& errorMessage);
bool DownsampleModel(const ModelData& input, std::size_t maxVertices, ModelData& output, std::wstring& summary);
std::wstring DescribeModel(const ModelData& model);
std::wstring ConversionMethodName(ConversionMethod method);
void GetVertexPosition(const ModelData& model, std::size_t index, double& x, double& y, double& z);
void FinalizeGeneratedModel(ModelData& model);
