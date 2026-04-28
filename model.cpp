#include "model.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace {

std::string FormatDouble(double value, int precision = 4) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    std::string text = stream.str();
    while (!text.empty() && text.back() == '0') {
        text.pop_back();
    }
    if (!text.empty() && text.back() == '.') {
        text.pop_back();
    }
    if (text.empty() || text == "-0") {
        return "0";
    }
    return text;
}

void BuildLineIndices(ModelData& model);

void PushVertex(ModelData& model, float x, float y, float z) {
    model.positions.push_back(x);
    model.positions.push_back(y);
    model.positions.push_back(z);
}

void ComputeDerivedData(ModelData& model) {
    if (model.positions.empty()) {
        model.scale = 1.0f;
        return;
    }

    model.boundsMin[0] = model.boundsMax[0] = model.positions[0];
    model.boundsMin[1] = model.boundsMax[1] = model.positions[1];
    model.boundsMin[2] = model.boundsMax[2] = model.positions[2];

    for (std::size_t index = 0; index < model.positions.size(); index += 3) {
        for (int axis = 0; axis < 3; ++axis) {
            model.boundsMin[axis] = std::min(model.boundsMin[axis], model.positions[index + axis]);
            model.boundsMax[axis] = std::max(model.boundsMax[axis], model.positions[index + axis]);
        }
    }

    for (int axis = 0; axis < 3; ++axis) {
        model.center[axis] = (model.boundsMin[axis] + model.boundsMax[axis]) * 0.5f;
    }

    const float extentX = model.boundsMax[0] - model.boundsMin[0];
    const float extentY = model.boundsMax[1] - model.boundsMin[1];
    const float extentZ = model.boundsMax[2] - model.boundsMin[2];
    model.scale = std::max({extentX, extentY, extentZ}) * 0.5f;
    if (model.scale < 1e-6f) {
        model.scale = 1.0f;
    }
}

void FinalizeModel(ModelData& model) {
    BuildLineIndices(model);
    ComputeDerivedData(model);
}

void BuildLineIndices(ModelData& model) {
    model.lineIndices.clear();
    if (!model.triangles.empty()) {
        std::unordered_set<std::uint64_t> edges;
        edges.reserve(model.triangles.size());

        auto addEdge = [&](std::uint32_t a, std::uint32_t b) {
            const std::uint32_t low = std::min(a, b);
            const std::uint32_t high = std::max(a, b);
            const std::uint64_t key = (static_cast<std::uint64_t>(low) << 32) | static_cast<std::uint64_t>(high);
            if (edges.insert(key).second) {
                model.lineIndices.push_back(a);
                model.lineIndices.push_back(b);
            }
        };

        for (std::size_t index = 0; index + 2 < model.triangles.size(); index += 3) {
            const std::uint32_t a = model.triangles[index];
            const std::uint32_t b = model.triangles[index + 1];
            const std::uint32_t c = model.triangles[index + 2];
            addEdge(a, b);
            addEdge(b, c);
            addEdge(c, a);
        }
        return;
    }
}

struct CellRepresentative {
    std::size_t index = 0;
    float distanceSquared = 0.0f;
};

float DistanceSquaredToCellCenter(
    const ModelData& input,
    std::size_t vertexIndex,
    float cellCenterX,
    float cellCenterY,
    float cellCenterZ) {
    const std::size_t offset = vertexIndex * 3;
    const float dx = input.positions[offset] - cellCenterX;
    const float dy = input.positions[offset + 1] - cellCenterY;
    const float dz = input.positions[offset + 2] - cellCenterZ;
    return dx * dx + dy * dy + dz * dz;
}

std::vector<std::size_t> BuildSpatialVertexSelection(const ModelData& input, std::size_t targetCount) {
    if (input.VertexCount() <= targetCount) {
        std::vector<std::size_t> indices(input.VertexCount());
        for (std::size_t index = 0; index < input.VertexCount(); ++index) {
            indices[index] = index;
        }
        return indices;
    }

    const float extentX = std::max(input.boundsMax[0] - input.boundsMin[0], 1e-6f);
    const float extentY = std::max(input.boundsMax[1] - input.boundsMin[1], 1e-6f);
    const float extentZ = std::max(input.boundsMax[2] - input.boundsMin[2], 1e-6f);
    const int gridResolution = std::max(2, static_cast<int>(std::ceil(std::cbrt(static_cast<double>(targetCount) * 1.5))));
    const float stepX = extentX / static_cast<float>(gridResolution);
    const float stepY = extentY / static_cast<float>(gridResolution);
    const float stepZ = extentZ / static_cast<float>(gridResolution);

    std::unordered_map<std::uint64_t, CellRepresentative> cells;
    cells.reserve(targetCount * 2);

    for (std::size_t index = 0; index < input.VertexCount(); ++index) {
        const std::size_t offset = index * 3;
        const float x = input.positions[offset];
        const float y = input.positions[offset + 1];
        const float z = input.positions[offset + 2];

        const int ix = std::clamp(static_cast<int>(((x - input.boundsMin[0]) / extentX) * gridResolution), 0, gridResolution - 1);
        const int iy = std::clamp(static_cast<int>(((y - input.boundsMin[1]) / extentY) * gridResolution), 0, gridResolution - 1);
        const int iz = std::clamp(static_cast<int>(((z - input.boundsMin[2]) / extentZ) * gridResolution), 0, gridResolution - 1);

        const std::uint64_t key = static_cast<std::uint64_t>(ix)
            + static_cast<std::uint64_t>(gridResolution) * (static_cast<std::uint64_t>(iy)
            + static_cast<std::uint64_t>(gridResolution) * static_cast<std::uint64_t>(iz));

        const float centerX = input.boundsMin[0] + (static_cast<float>(ix) + 0.5f) * stepX;
        const float centerY = input.boundsMin[1] + (static_cast<float>(iy) + 0.5f) * stepY;
        const float centerZ = input.boundsMin[2] + (static_cast<float>(iz) + 0.5f) * stepZ;
        const float distanceSquared = DistanceSquaredToCellCenter(input, index, centerX, centerY, centerZ);

        auto [it, inserted] = cells.try_emplace(key, CellRepresentative{index, distanceSquared});
        if (!inserted && distanceSquared < it->second.distanceSquared) {
            it->second = {index, distanceSquared};
        }
    }

    std::vector<std::size_t> selected;
    selected.reserve(std::min(targetCount, cells.size()));
    for (const auto& entry : cells) {
        selected.push_back(entry.second.index);
    }
    std::sort(selected.begin(), selected.end());

    if (selected.size() > targetCount) {
        std::vector<std::size_t> trimmed;
        trimmed.reserve(targetCount);
        for (std::size_t sample = 0; sample < targetCount; ++sample) {
            const std::size_t sourceIndex = (sample * (selected.size() - 1)) / (targetCount - 1);
            trimmed.push_back(selected[sourceIndex]);
        }
        selected = std::move(trimmed);
    } else if (selected.size() < targetCount) {
        std::vector<bool> chosen(input.VertexCount(), false);
        for (std::size_t index : selected) {
            chosen[index] = true;
        }
        for (std::size_t sample = 0; sample < targetCount && selected.size() < targetCount; ++sample) {
            const std::size_t index = (sample * (input.VertexCount() - 1)) / (targetCount - 1);
            if (!chosen[index]) {
                chosen[index] = true;
                selected.push_back(index);
            }
        }
        std::sort(selected.begin(), selected.end());
    }

    return selected;
}

bool ParseFaceIndex(const std::string& token, std::size_t vertexCount, std::uint32_t& outIndex) {
    if (token.empty()) {
        return false;
    }

    const std::size_t slash = token.find('/');
    const std::string indexText = token.substr(0, slash);
    if (indexText.empty()) {
        return false;
    }

    int rawIndex = 0;
    try {
        rawIndex = std::stoi(indexText);
    } catch (...) {
        return false;
    }

    if (rawIndex > 0) {
        outIndex = static_cast<std::uint32_t>(rawIndex - 1);
        return outIndex < vertexCount;
    }

    if (rawIndex < 0) {
        const int converted = static_cast<int>(vertexCount) + rawIndex;
        if (converted >= 0) {
            outIndex = static_cast<std::uint32_t>(converted);
            return true;
        }
    }

    return false;
}

}  // namespace

bool LoadObjModel(const std::filesystem::path& path, ModelData& outModel, std::wstring& errorMessage) {
    std::ifstream input(path);
    if (!input) {
        errorMessage = L"Could not open the OBJ file.";
        return false;
    }

    ModelData model;
    model.label = path.filename().wstring();

    std::string line;
    while (std::getline(input, line)) {
        std::istringstream row(line);
        std::string tag;
        row >> tag;
        if (tag == "v") {
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            if (row >> x >> y >> z) {
                PushVertex(model, x, y, z);
            }
        } else if (tag == "f") {
            std::vector<std::uint32_t> polygon;
            std::string token;
            while (row >> token) {
                std::uint32_t vertexIndex = 0;
                if (ParseFaceIndex(token, model.VertexCount(), vertexIndex)) {
                    polygon.push_back(vertexIndex);
                }
            }

            if (polygon.size() >= 3) {
                for (std::size_t index = 1; index + 1 < polygon.size(); ++index) {
                    model.triangles.push_back(polygon[0]);
                    model.triangles.push_back(polygon[index]);
                    model.triangles.push_back(polygon[index + 1]);
                }
            }
        }
    }

    if (model.positions.empty()) {
        errorMessage = L"No vertex positions were found in the OBJ file.";
        return false;
    }

    FinalizeModel(model);
    outModel = std::move(model);
    return true;
}

bool DownsampleModel(const ModelData& input, std::size_t maxVertices, ModelData& output, std::wstring& summary) {
    if (input.VertexCount() == 0) {
        summary = L"No vertices were available for downsampling.";
        return false;
    }

    if (maxVertices < 3) {
        summary = L"Downsampling requires a vertex budget of at least 3.";
        return false;
    }

    if (input.VertexCount() <= maxVertices) {
        output = input;
        summary = L"Model stayed at full resolution because it was already inside the vertex budget.";
        return true;
    }

    const std::size_t targetCount = maxVertices;
    std::vector<std::uint32_t> remap(input.VertexCount(), UINT32_MAX);
    ModelData reduced;
    reduced.label = input.label;
    reduced.positions.reserve(targetCount * 3);

    const std::vector<std::size_t> selectedIndices = BuildSpatialVertexSelection(input, targetCount);
    for (std::size_t oldIndex : selectedIndices) {
        remap[oldIndex] = static_cast<std::uint32_t>(reduced.VertexCount());
        reduced.positions.push_back(input.positions[oldIndex * 3]);
        reduced.positions.push_back(input.positions[oldIndex * 3 + 1]);
        reduced.positions.push_back(input.positions[oldIndex * 3 + 2]);
    }

    if (reduced.VertexCount() < 3) {
        summary = L"Downsampling collapsed the mesh too far.";
        return false;
    }

    reduced.triangles.reserve(input.triangles.size());
    for (std::size_t index = 0; index + 2 < input.triangles.size(); index += 3) {
        const std::uint32_t a = input.triangles[index];
        const std::uint32_t b = input.triangles[index + 1];
        const std::uint32_t c = input.triangles[index + 2];
        if (a >= remap.size() || b >= remap.size() || c >= remap.size()) {
            continue;
        }

        const std::uint32_t mappedA = remap[a];
        const std::uint32_t mappedB = remap[b];
        const std::uint32_t mappedC = remap[c];
        if (mappedA == UINT32_MAX || mappedB == UINT32_MAX || mappedC == UINT32_MAX) {
            continue;
        }
        if (mappedA == mappedB || mappedB == mappedC || mappedA == mappedC) {
            continue;
        }

        reduced.triangles.push_back(mappedA);
        reduced.triangles.push_back(mappedB);
        reduced.triangles.push_back(mappedC);
    }

    const std::size_t originalTriangles = input.TriangleCount();
    if (reduced.TriangleCount() < 64
        || reduced.TriangleCount() * 20 < reduced.VertexCount()
        || (originalTriangles > 0 && reduced.TriangleCount() * 200 < originalTriangles)) {
        reduced.triangles.clear();
    }

    FinalizeModel(reduced);
    output = std::move(reduced);
    summary = L"Model downscaled from "
        + std::to_wstring(input.VertexCount())
        + L" vertices to "
        + std::to_wstring(output.VertexCount())
        + L" vertices for lighter analysis.";
    return true;
}

std::wstring DescribeModel(const ModelData& model) {
    std::ostringstream stream;
    stream << "Label: " << std::string(model.label.begin(), model.label.end()) << "\r\n"
           << "Vertices: " << model.VertexCount() << "\r\n"
           << "Triangles: " << model.TriangleCount() << "\r\n"
           << "Model memory: " << FormatDouble(static_cast<double>(model.MemoryBytes()) / 1024.0) << " KB\r\n"
           << "Bounds min: (" << FormatDouble(model.boundsMin[0]) << ", " << FormatDouble(model.boundsMin[1]) << ", " << FormatDouble(model.boundsMin[2]) << ")\r\n"
           << "Bounds max: (" << FormatDouble(model.boundsMax[0]) << ", " << FormatDouble(model.boundsMax[1]) << ", " << FormatDouble(model.boundsMax[2]) << ")\r\n"
           << "Center: (" << FormatDouble(model.center[0]) << ", " << FormatDouble(model.center[1]) << ", " << FormatDouble(model.center[2]) << ")\r\n"
           << "Half extent scale: " << FormatDouble(model.scale);
    const std::string text = stream.str();
    return std::wstring(text.begin(), text.end());
}

std::wstring ConversionMethodName(ConversionMethod method) {
    switch (method) {
        case ConversionMethod::AutoDetect:
            return L"Auto";
        case ConversionMethod::Plane:
            return L"Plane";
        case ConversionMethod::Sphere:
            return L"Sphere";
        case ConversionMethod::Cylinder:
            return L"Cylinder";
        case ConversionMethod::ImplicitQuadratic:
            return L"Quadric";
        case ConversionMethod::ImplicitCubic:
            return L"Cubic";
        case ConversionMethod::RbfImplicit:
            return L"RBF Surface";
    }
    return L"Unknown";
}

void GetVertexPosition(const ModelData& model, std::size_t index, double& x, double& y, double& z) {
    const std::size_t offset = index * 3;
    x = model.positions[offset];
    y = model.positions[offset + 1];
    z = model.positions[offset + 2];
}

void FinalizeGeneratedModel(ModelData& model) {
    FinalizeModel(model);
}
