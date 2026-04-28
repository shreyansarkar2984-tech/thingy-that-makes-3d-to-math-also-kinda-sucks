#include "analysis.h"
#include "gpu.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#ifndef MODEL_DOWNSCALE_ENABLED
#define MODEL_DOWNSCALE_ENABLED 0
#endif

#ifndef MODEL_GPU_COMPUTE_ENABLED
#define MODEL_GPU_COMPUTE_ENABLED 0
#endif

namespace {

[[maybe_unused]] constexpr bool kDownscaleBuild = MODEL_DOWNSCALE_ENABLED != 0;
[[maybe_unused]] constexpr bool kGpuComputeBuild = MODEL_GPU_COMPUTE_ENABLED != 0;

struct Vec3d {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct MonomialTerm {
    int xExponent = 0;
    int yExponent = 0;
    int zExponent = 0;
    std::string label;
};

struct NearestNeighborStats {
    double mean = 0.0;
    double max = 0.0;
};

struct CandidateFit {
    bool valid = false;
    ConversionMethod method = ConversionMethod::AutoDetect;
    double score = std::numeric_limits<double>::max();
    double meanResidual = 0.0;
    double rmsResidual = 0.0;
    double maxResidual = 0.0;
    std::wstring equation;
    std::wstring description;
    std::function<double(const Vec3d&)> evaluator;
    std::vector<float> gpuCenters;
    std::vector<float> gpuWeights;
    float gpuConstant = 0.0f;
    float gpuRadiusSquared = 0.0f;
    bool gpuEvaluable = false;
};

struct FittingSample {
    Vec3d point;
    double target = 0.0;
    double weight = 1.0;
};

struct RbfFitSettings {
    std::size_t minCenterCount = 64;
    std::size_t maxCenterCount = 192;
    std::size_t centerDivisor = 6000;
    std::size_t centerCandidateCount = 18000;
    std::size_t surfaceCandidateCount = 24000;
    std::size_t surfaceSampleBase = 1800;
    std::size_t surfaceSamplesPerCenter = 24;
    double radiusScale = 1.3;
    double minRadius = 0.05;
    double maxRadius = 0.26;
};

struct Bounds3d {
    double minX = 0.0;
    double minY = 0.0;
    double minZ = 0.0;
    double maxX = 0.0;
    double maxY = 0.0;
    double maxZ = 0.0;
};

Vec3d operator+(const Vec3d& left, const Vec3d& right) {
    return {left.x + right.x, left.y + right.y, left.z + right.z};
}

Vec3d operator-(const Vec3d& left, const Vec3d& right) {
    return {left.x - right.x, left.y - right.y, left.z - right.z};
}

Vec3d operator*(const Vec3d& vector, double scalar) {
    return {vector.x * scalar, vector.y * scalar, vector.z * scalar};
}

double Dot(const Vec3d& left, const Vec3d& right) {
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

double Length(const Vec3d& value) {
    return std::sqrt(Dot(value, value));
}

double DistanceSquared(const Vec3d& left, const Vec3d& right) {
    const double dx = left.x - right.x;
    const double dy = left.y - right.y;
    const double dz = left.z - right.z;
    return dx * dx + dy * dy + dz * dz;
}

Vec3d Normalize(const Vec3d& value) {
    const double length = Length(value);
    if (length < 1e-12) {
        return {0.0, 1.0, 0.0};
    }
    return value * (1.0 / length);
}

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

std::wstring ToWide(const std::string& text) {
    return std::wstring(text.begin(), text.end());
}

std::wstring ToWide(double value, int precision = 4) {
    return ToWide(FormatDouble(value, precision));
}

double& At(std::vector<double>& matrix, int size, int row, int column) {
    return matrix[static_cast<std::size_t>(row) * static_cast<std::size_t>(size) + static_cast<std::size_t>(column)];
}

std::vector<Vec3d> ReadAllVertices(const ModelData& model, bool normalized) {
    std::vector<Vec3d> vertices;
    vertices.reserve(model.VertexCount());
    for (std::size_t index = 0; index < model.VertexCount(); ++index) {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        GetVertexPosition(model, index, x, y, z);
        if (normalized) {
            x = (x - model.center[0]) / model.scale;
            y = (y - model.center[1]) / model.scale;
            z = (z - model.center[2]) / model.scale;
        }
        vertices.push_back({x, y, z});
    }
    return vertices;
}

Vec3d ComputeCentroid(const std::vector<Vec3d>& vertices);
std::vector<std::size_t> BuildFarthestPointIndices(const std::vector<Vec3d>& points, std::size_t maxCount);

std::vector<std::size_t> BuildSampleIndices(std::size_t totalCount, std::size_t maxCount) {
    if (totalCount == 0) {
        return {};
    }

    const std::size_t count = std::min(totalCount, maxCount);
    std::vector<std::size_t> indices;
    indices.reserve(count);
    if (count == totalCount) {
        for (std::size_t index = 0; index < totalCount; ++index) {
            indices.push_back(index);
        }
        return indices;
    }

    if (count == 1) {
        indices.push_back(0);
        return indices;
    }

    for (std::size_t sampleIndex = 0; sampleIndex < count; ++sampleIndex) {
        const std::size_t index = (sampleIndex * (totalCount - 1)) / (count - 1);
        indices.push_back(index);
    }
    return indices;
}

std::vector<Vec3d> GatherPointsByIndex(const std::vector<Vec3d>& points, const std::vector<std::size_t>& indices) {
    std::vector<Vec3d> gathered;
    gathered.reserve(indices.size());
    for (std::size_t index : indices) {
        if (index < points.size()) {
            gathered.push_back(points[index]);
        }
    }
    return gathered;
}

std::vector<std::size_t> BuildSpatialSampleIndices(const std::vector<Vec3d>& points, std::size_t maxCount) {
    if (points.empty() || maxCount == 0) {
        return {};
    }

    if (points.size() <= maxCount) {
        std::vector<std::size_t> indices(points.size());
        std::iota(indices.begin(), indices.end(), 0);
        return indices;
    }

    Vec3d minPoint = points.front();
    Vec3d maxPoint = points.front();
    for (const Vec3d& point : points) {
        minPoint.x = std::min(minPoint.x, point.x);
        minPoint.y = std::min(minPoint.y, point.y);
        minPoint.z = std::min(minPoint.z, point.z);
        maxPoint.x = std::max(maxPoint.x, point.x);
        maxPoint.y = std::max(maxPoint.y, point.y);
        maxPoint.z = std::max(maxPoint.z, point.z);
    }

    const double extentX = std::max(maxPoint.x - minPoint.x, 1e-9);
    const double extentY = std::max(maxPoint.y - minPoint.y, 1e-9);
    const double extentZ = std::max(maxPoint.z - minPoint.z, 1e-9);
    const int gridResolution = std::max(2, static_cast<int>(std::ceil(std::cbrt(static_cast<double>(maxCount) * 1.6))));
    const double stepX = extentX / static_cast<double>(gridResolution);
    const double stepY = extentY / static_cast<double>(gridResolution);
    const double stepZ = extentZ / static_cast<double>(gridResolution);

    struct CellRepresentative {
        std::size_t index = 0;
        double distanceSquared = std::numeric_limits<double>::max();
    };

    std::unordered_map<std::uint64_t, CellRepresentative> cells;
    cells.reserve(maxCount * 2);
    for (std::size_t index = 0; index < points.size(); ++index) {
        const Vec3d& point = points[index];
        const int ix = std::clamp(static_cast<int>(((point.x - minPoint.x) / extentX) * gridResolution), 0, gridResolution - 1);
        const int iy = std::clamp(static_cast<int>(((point.y - minPoint.y) / extentY) * gridResolution), 0, gridResolution - 1);
        const int iz = std::clamp(static_cast<int>(((point.z - minPoint.z) / extentZ) * gridResolution), 0, gridResolution - 1);
        const std::uint64_t key = static_cast<std::uint64_t>(ix)
            + static_cast<std::uint64_t>(gridResolution) * (static_cast<std::uint64_t>(iy)
            + static_cast<std::uint64_t>(gridResolution) * static_cast<std::uint64_t>(iz));

        const Vec3d cellCenter{
            minPoint.x + (static_cast<double>(ix) + 0.5) * stepX,
            minPoint.y + (static_cast<double>(iy) + 0.5) * stepY,
            minPoint.z + (static_cast<double>(iz) + 0.5) * stepZ,
        };
        const double cellDistanceSquared = DistanceSquared(point, cellCenter);

        auto [iterator, inserted] = cells.try_emplace(key, CellRepresentative{index, cellDistanceSquared});
        if (!inserted && cellDistanceSquared < iterator->second.distanceSquared) {
            iterator->second = {index, cellDistanceSquared};
        }
    }

    std::vector<std::size_t> indices;
    indices.reserve(cells.size());
    for (const auto& entry : cells) {
        indices.push_back(entry.second.index);
    }

    if (indices.size() > maxCount) {
        const std::vector<Vec3d> candidates = GatherPointsByIndex(points, indices);
        const std::vector<std::size_t> sampledOffsets = BuildFarthestPointIndices(candidates, maxCount);
        std::vector<std::size_t> reduced;
        reduced.reserve(sampledOffsets.size());
        for (std::size_t offset : sampledOffsets) {
            reduced.push_back(indices[offset]);
        }
        indices = std::move(reduced);
    } else if (indices.size() < maxCount) {
        std::unordered_map<std::size_t, bool> chosen;
        chosen.reserve(indices.size() * 2 + 1);
        for (std::size_t index : indices) {
            chosen[index] = true;
        }
        const std::vector<std::size_t> fallback = BuildSampleIndices(points.size(), maxCount);
        for (std::size_t index : fallback) {
            if (chosen.find(index) == chosen.end()) {
                chosen[index] = true;
                indices.push_back(index);
                if (indices.size() >= maxCount) {
                    break;
                }
            }
        }
    }

    return indices;
}

Bounds3d BoundsFromModel(const ModelData& model) {
    return {
        static_cast<double>(model.boundsMin[0]),
        static_cast<double>(model.boundsMin[1]),
        static_cast<double>(model.boundsMin[2]),
        static_cast<double>(model.boundsMax[0]),
        static_cast<double>(model.boundsMax[1]),
        static_cast<double>(model.boundsMax[2]),
    };
}

bool HasMeaningfulExtent(const Bounds3d& bounds) {
    return (bounds.maxX - bounds.minX) > 1e-6
        && (bounds.maxY - bounds.minY) > 1e-6
        && (bounds.maxZ - bounds.minZ) > 1e-6;
}

Bounds3d ComputeTrimmedVertexBounds(const ModelData& model, std::size_t maxSamples) {
    if (model.VertexCount() == 0) {
        return BoundsFromModel(model);
    }

    const std::vector<std::size_t> spatialSourceIndices = BuildSpatialSampleIndices(ReadAllVertices(model, false), maxSamples);
    const std::vector<std::size_t>& sampleIndices = spatialSourceIndices;
    if (sampleIndices.empty()) {
        return BoundsFromModel(model);
    }

    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> zs;
    xs.reserve(sampleIndices.size());
    ys.reserve(sampleIndices.size());
    zs.reserve(sampleIndices.size());
    for (std::size_t index : sampleIndices) {
        xs.push_back(static_cast<double>(model.positions[index * 3]));
        ys.push_back(static_cast<double>(model.positions[index * 3 + 1]));
        zs.push_back(static_cast<double>(model.positions[index * 3 + 2]));
    }

    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());
    std::sort(zs.begin(), zs.end());

    const std::size_t trimCount = sampleIndices.size() >= 400 ? sampleIndices.size() / 100 : 0;
    const std::size_t low = trimCount;
    const std::size_t high = sampleIndices.size() - 1 - trimCount;

    Bounds3d bounds{
        xs[low], ys[low], zs[low],
        xs[high], ys[high], zs[high],
    };
    if (!HasMeaningfulExtent(bounds)) {
        return BoundsFromModel(model);
    }
    return bounds;
}

Bounds3d MergeBounds(const Bounds3d& left, const Bounds3d& right) {
    return {
        std::min(left.minX, right.minX),
        std::min(left.minY, right.minY),
        std::min(left.minZ, right.minZ),
        std::max(left.maxX, right.maxX),
        std::max(left.maxY, right.maxY),
        std::max(left.maxZ, right.maxZ),
    };
}

Bounds3d ExpandBounds(const Bounds3d& bounds, double paddingFactor, double minimumPadding) {
    const double extentX = bounds.maxX - bounds.minX;
    const double extentY = bounds.maxY - bounds.minY;
    const double extentZ = bounds.maxZ - bounds.minZ;
    const double padX = std::max(minimumPadding, extentX * paddingFactor);
    const double padY = std::max(minimumPadding, extentY * paddingFactor);
    const double padZ = std::max(minimumPadding, extentZ * paddingFactor);
    return {
        bounds.minX - padX,
        bounds.minY - padY,
        bounds.minZ - padZ,
        bounds.maxX + padX,
        bounds.maxY + padY,
        bounds.maxZ + padZ,
    };
}

Bounds3d ComputeRbfCenterBounds(const CandidateFit& fit) {
    Bounds3d bounds{};
    if (fit.gpuCenters.size() < 12) {
        return bounds;
    }

    bounds.minX = bounds.maxX = static_cast<double>(fit.gpuCenters[0]);
    bounds.minY = bounds.maxY = static_cast<double>(fit.gpuCenters[1]);
    bounds.minZ = bounds.maxZ = static_cast<double>(fit.gpuCenters[2]);
    for (std::size_t index = 3; index + 2 < fit.gpuCenters.size(); index += 3) {
        bounds.minX = std::min(bounds.minX, static_cast<double>(fit.gpuCenters[index]));
        bounds.minY = std::min(bounds.minY, static_cast<double>(fit.gpuCenters[index + 1]));
        bounds.minZ = std::min(bounds.minZ, static_cast<double>(fit.gpuCenters[index + 2]));
        bounds.maxX = std::max(bounds.maxX, static_cast<double>(fit.gpuCenters[index]));
        bounds.maxY = std::max(bounds.maxY, static_cast<double>(fit.gpuCenters[index + 1]));
        bounds.maxZ = std::max(bounds.maxZ, static_cast<double>(fit.gpuCenters[index + 2]));
    }
    return bounds;
}

[[maybe_unused]] Bounds3d ComputeReconstructionBounds(const ModelData& source, const CandidateFit& fit) {
    Bounds3d bounds = ComputeTrimmedVertexBounds(source, 16000);
    if (fit.method == ConversionMethod::RbfImplicit && fit.gpuCenters.size() >= 12 && fit.gpuRadiusSquared > 0.0f) {
        const double radius = std::sqrt(static_cast<double>(fit.gpuRadiusSquared));
        const Bounds3d centerBounds = ExpandBounds(ComputeRbfCenterBounds(fit), 0.0, radius * 2.4);
        if (HasMeaningfulExtent(centerBounds)) {
            bounds = MergeBounds(bounds, centerBounds);
        }
    }

    if (!HasMeaningfulExtent(bounds)) {
        bounds = BoundsFromModel(source);
    }

    return ExpandBounds(bounds, 0.08, std::max(0.015, static_cast<double>(source.scale) * 0.01));
}

int DeterminePreviewResolution(const ModelData& source, const CandidateFit& fit) {
    (void)fit;
    return source.VertexCount() > 10000 ? 28 : (source.VertexCount() > 2500 ? 30 : 32);
}

std::vector<Vec3d> ComputeVertexNormals(const ModelData& model, const std::vector<Vec3d>& normalizedVertices) {
    std::vector<Vec3d> normals(normalizedVertices.size(), Vec3d{});
    if (!model.triangles.empty()) {
        for (std::size_t index = 0; index + 2 < model.triangles.size(); index += 3) {
            const std::uint32_t ia = model.triangles[index];
            const std::uint32_t ib = model.triangles[index + 1];
            const std::uint32_t ic = model.triangles[index + 2];
            if (ia >= normals.size() || ib >= normals.size() || ic >= normals.size()) {
                continue;
            }

            const Vec3d& a = normalizedVertices[ia];
            const Vec3d& b = normalizedVertices[ib];
            const Vec3d& c = normalizedVertices[ic];
            const Vec3d ab = b - a;
            const Vec3d ac = c - a;
            const Vec3d faceNormal{
                ab.y * ac.z - ab.z * ac.y,
                ab.z * ac.x - ab.x * ac.z,
                ab.x * ac.y - ab.y * ac.x,
            };

            normals[ia] = normals[ia] + faceNormal;
            normals[ib] = normals[ib] + faceNormal;
            normals[ic] = normals[ic] + faceNormal;
        }
    }

    const Vec3d centroid = ComputeCentroid(normalizedVertices);
    for (std::size_t index = 0; index < normals.size(); ++index) {
        if (Length(normals[index]) < 1e-10) {
            normals[index] = normalizedVertices[index] - centroid;
        }
        normals[index] = Normalize(normals[index]);
    }
    return normals;
}

class KDTree {
public:
    explicit KDTree(const ModelData& model)
        : model_(model) {
        indices_.resize(model.VertexCount());
        std::iota(indices_.begin(), indices_.end(), 0u);
        if (!indices_.empty()) {
            root_ = Build(0, static_cast<int>(indices_.size()), 0);
        }
    }

    double NearestDistance(std::uint32_t index) const {
        if (model_.VertexCount() <= 1) {
            return 0.0;
        }

        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        GetVertexPosition(model_, index, x, y, z);

        double bestSquared = std::numeric_limits<double>::max();
        Search(root_, index, {x, y, z}, bestSquared);
        return std::sqrt(bestSquared);
    }

private:
    struct Node {
        std::uint32_t index = 0;
        int axis = 0;
        int left = -1;
        int right = -1;
    };

    const ModelData& model_;
    std::vector<std::uint32_t> indices_;
    std::vector<Node> nodes_;
    int root_ = -1;

    int Build(int begin, int end, int depth) {
        if (begin >= end) {
            return -1;
        }

        const int axis = depth % 3;
        const int middle = (begin + end) / 2;

        auto comparator = [&](std::uint32_t left, std::uint32_t right) {
            return model_.positions[left * 3 + axis] < model_.positions[right * 3 + axis];
        };

        std::nth_element(indices_.begin() + begin, indices_.begin() + middle, indices_.begin() + end, comparator);

        const int nodeIndex = static_cast<int>(nodes_.size());
        nodes_.push_back({indices_[middle], axis, -1, -1});
        nodes_[nodeIndex].left = Build(begin, middle, depth + 1);
        nodes_[nodeIndex].right = Build(middle + 1, end, depth + 1);
        return nodeIndex;
    }

    static double DistanceSquared(const Vec3d& left, const Vec3d& right) {
        const double dx = left.x - right.x;
        const double dy = left.y - right.y;
        const double dz = left.z - right.z;
        return dx * dx + dy * dy + dz * dz;
    }

    void Search(int nodeIndex, std::uint32_t excludedIndex, const Vec3d& query, double& bestSquared) const {
        if (nodeIndex < 0) {
            return;
        }

        const Node& node = nodes_[nodeIndex];
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        GetVertexPosition(model_, node.index, x, y, z);
        const Vec3d current{x, y, z};

        if (node.index != excludedIndex) {
            bestSquared = std::min(bestSquared, DistanceSquared(query, current));
        }

        const double axisDelta = node.axis == 0 ? query.x - current.x : (node.axis == 1 ? query.y - current.y : query.z - current.z);
        const int nearChild = axisDelta <= 0.0 ? node.left : node.right;
        const int farChild = axisDelta <= 0.0 ? node.right : node.left;

        Search(nearChild, excludedIndex, query, bestSquared);
        if (axisDelta * axisDelta < bestSquared) {
            Search(farChild, excludedIndex, query, bestSquared);
        }
    }
};

bool JacobiEigenDecomposition(const std::vector<double>& inputMatrix, int size, std::vector<double>& eigenvalues, std::vector<double>& eigenvectors) {
    std::vector<double> matrix = inputMatrix;
    eigenvectors.assign(static_cast<std::size_t>(size) * static_cast<std::size_t>(size), 0.0);

    for (int index = 0; index < size; ++index) {
        At(eigenvectors, size, index, index) = 1.0;
    }

    const int maxIterations = size * size * 40;
    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        int pivotRow = 0;
        int pivotColumn = 1;
        double largestOffDiagonal = 0.0;

        for (int row = 0; row < size; ++row) {
            for (int column = row + 1; column < size; ++column) {
                const double value = std::abs(At(matrix, size, row, column));
                if (value > largestOffDiagonal) {
                    largestOffDiagonal = value;
                    pivotRow = row;
                    pivotColumn = column;
                }
            }
        }

        if (largestOffDiagonal < 1e-11) {
            break;
        }

        const double app = At(matrix, size, pivotRow, pivotRow);
        const double aqq = At(matrix, size, pivotColumn, pivotColumn);
        const double apq = At(matrix, size, pivotRow, pivotColumn);

        const double angle = 0.5 * std::atan2(2.0 * apq, aqq - app);
        const double cosine = std::cos(angle);
        const double sine = std::sin(angle);

        for (int column = 0; column < size; ++column) {
            if (column == pivotRow || column == pivotColumn) {
                continue;
            }

            const double apr = At(matrix, size, pivotRow, column);
            const double aqr = At(matrix, size, pivotColumn, column);

            const double newApr = cosine * apr - sine * aqr;
            const double newAqr = sine * apr + cosine * aqr;

            At(matrix, size, pivotRow, column) = newApr;
            At(matrix, size, column, pivotRow) = newApr;
            At(matrix, size, pivotColumn, column) = newAqr;
            At(matrix, size, column, pivotColumn) = newAqr;
        }

        const double newApp = cosine * cosine * app - 2.0 * sine * cosine * apq + sine * sine * aqq;
        const double newAqq = sine * sine * app + 2.0 * sine * cosine * apq + cosine * cosine * aqq;

        At(matrix, size, pivotRow, pivotRow) = newApp;
        At(matrix, size, pivotColumn, pivotColumn) = newAqq;
        At(matrix, size, pivotRow, pivotColumn) = 0.0;
        At(matrix, size, pivotColumn, pivotRow) = 0.0;

        for (int row = 0; row < size; ++row) {
            const double vip = At(eigenvectors, size, row, pivotRow);
            const double viq = At(eigenvectors, size, row, pivotColumn);
            At(eigenvectors, size, row, pivotRow) = cosine * vip - sine * viq;
            At(eigenvectors, size, row, pivotColumn) = sine * vip + cosine * viq;
        }
    }

    eigenvalues.resize(size);
    for (int index = 0; index < size; ++index) {
        eigenvalues[index] = At(matrix, size, index, index);
    }

    return true;
}

Vec3d ComputeCentroid(const std::vector<Vec3d>& vertices) {
    Vec3d centroid{};
    if (vertices.empty()) {
        return centroid;
    }
    for (const Vec3d& vertex : vertices) {
        centroid = centroid + vertex;
    }
    return centroid * (1.0 / static_cast<double>(vertices.size()));
}

std::array<double, 9> ComputeCovariance(const std::vector<Vec3d>& vertices, const Vec3d& centroid) {
    std::array<double, 9> covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (vertices.empty()) {
        return covariance;
    }

    for (const Vec3d& vertex : vertices) {
        const Vec3d delta = vertex - centroid;
        covariance[0] += delta.x * delta.x;
        covariance[1] += delta.x * delta.y;
        covariance[2] += delta.x * delta.z;
        covariance[4] += delta.y * delta.y;
        covariance[5] += delta.y * delta.z;
        covariance[8] += delta.z * delta.z;
    }

    covariance[3] = covariance[1];
    covariance[6] = covariance[2];
    covariance[7] = covariance[5];
    return covariance;
}

bool SolveLinearSystem(std::vector<double> matrix, std::vector<double> rhs, int size, std::vector<double>& solution) {
    for (int pivot = 0; pivot < size; ++pivot) {
        int bestRow = pivot;
        double bestValue = std::abs(At(matrix, size, pivot, pivot));
        for (int row = pivot + 1; row < size; ++row) {
            const double value = std::abs(At(matrix, size, row, pivot));
            if (value > bestValue) {
                bestValue = value;
                bestRow = row;
            }
        }

        if (bestValue < 1e-12) {
            return false;
        }

        if (bestRow != pivot) {
            for (int column = pivot; column < size; ++column) {
                std::swap(At(matrix, size, pivot, column), At(matrix, size, bestRow, column));
            }
            std::swap(rhs[pivot], rhs[bestRow]);
        }

        const double divisor = At(matrix, size, pivot, pivot);
        for (int column = pivot; column < size; ++column) {
            At(matrix, size, pivot, column) /= divisor;
        }
        rhs[pivot] /= divisor;

        for (int row = 0; row < size; ++row) {
            if (row == pivot) {
                continue;
            }
            const double factor = At(matrix, size, row, pivot);
            for (int column = pivot; column < size; ++column) {
                At(matrix, size, row, column) -= factor * At(matrix, size, pivot, column);
            }
            rhs[row] -= factor * rhs[pivot];
        }
    }

    solution = std::move(rhs);
    return true;
}

NearestNeighborStats ComputeNearestNeighborStats(const ModelData& model) {
    NearestNeighborStats stats{};
    if (model.VertexCount() <= 1) {
        return stats;
    }

    KDTree tree(model);
    for (std::uint32_t index = 0; index < model.VertexCount(); ++index) {
        const double distance = tree.NearestDistance(index);
        stats.mean += distance;
        stats.max = std::max(stats.max, distance);
    }
    stats.mean /= static_cast<double>(model.VertexCount());
    return stats;
}

CandidateFit FitPlane(const ModelData& model, const std::vector<Vec3d>& vertices) {
    CandidateFit fit;
    fit.method = ConversionMethod::Plane;
    if (vertices.size() < 3) {
        return fit;
    }

    const Vec3d centroid = ComputeCentroid(vertices);
    const auto covariance = ComputeCovariance(vertices, centroid);
    std::vector<double> matrix = {
        covariance[0], covariance[1], covariance[2],
        covariance[3], covariance[4], covariance[5],
        covariance[6], covariance[7], covariance[8],
    };

    std::vector<double> eigenvalues;
    std::vector<double> eigenvectors;
    JacobiEigenDecomposition(matrix, 3, eigenvalues, eigenvectors);

    int smallest = 0;
    for (int index = 1; index < 3; ++index) {
        if (eigenvalues[index] < eigenvalues[smallest]) {
            smallest = index;
        }
    }

    Vec3d normal{
        At(eigenvectors, 3, 0, smallest),
        At(eigenvectors, 3, 1, smallest),
        At(eigenvectors, 3, 2, smallest),
    };
    normal = Normalize(normal);
    const double d = Dot(normal, centroid);

    for (const Vec3d& vertex : vertices) {
        const double residual = std::abs(Dot(normal, vertex) - d);
        fit.meanResidual += residual;
        fit.rmsResidual += residual * residual;
        fit.maxResidual = std::max(fit.maxResidual, residual);
    }

    fit.valid = true;
    fit.meanResidual /= static_cast<double>(vertices.size());
    fit.rmsResidual = std::sqrt(fit.rmsResidual / static_cast<double>(vertices.size()));
    fit.score = fit.meanResidual / std::max(1e-9, static_cast<double>(model.scale));
    fit.equation = ToWide(
        "Plane: "
        + FormatDouble(normal.x) + "x + "
        + FormatDouble(normal.y) + "y + "
        + FormatDouble(normal.z) + "z = "
        + FormatDouble(d));
    fit.description = L"Plane fit computed from PCA using the smallest covariance eigenvector.";
    fit.evaluator = [normal, d](const Vec3d& point) {
        return Dot(normal, point) - d;
    };
    return fit;
}

CandidateFit FitSphere(const ModelData& model, const std::vector<Vec3d>& vertices) {
    CandidateFit fit;
    fit.method = ConversionMethod::Sphere;
    if (vertices.size() < 4) {
        return fit;
    }

    std::vector<double> normalMatrix(16, 0.0);
    std::vector<double> rhs(4, 0.0);
    for (const Vec3d& vertex : vertices) {
        const double row[4] = {vertex.x, vertex.y, vertex.z, 1.0};
        const double target = -(vertex.x * vertex.x + vertex.y * vertex.y + vertex.z * vertex.z);
        for (int left = 0; left < 4; ++left) {
            rhs[left] += row[left] * target;
            for (int right = 0; right < 4; ++right) {
                At(normalMatrix, 4, left, right) += row[left] * row[right];
            }
        }
    }

    std::vector<double> solution;
    if (!SolveLinearSystem(normalMatrix, rhs, 4, solution)) {
        return fit;
    }

    const Vec3d center{-solution[0] * 0.5, -solution[1] * 0.5, -solution[2] * 0.5};
    const double radiusSquared = Dot(center, center) - solution[3];
    if (radiusSquared <= 1e-12) {
        return fit;
    }

    const double radius = std::sqrt(radiusSquared);
    for (const Vec3d& vertex : vertices) {
        const double residual = std::abs(Length(vertex - center) - radius);
        fit.meanResidual += residual;
        fit.rmsResidual += residual * residual;
        fit.maxResidual = std::max(fit.maxResidual, residual);
    }

    fit.valid = true;
    fit.meanResidual /= static_cast<double>(vertices.size());
    fit.rmsResidual = std::sqrt(fit.rmsResidual / static_cast<double>(vertices.size()));
    fit.score = fit.meanResidual / std::max(1e-9, static_cast<double>(model.scale));
    fit.equation = ToWide(
        "Sphere: center=("
        + FormatDouble(center.x) + ", "
        + FormatDouble(center.y) + ", "
        + FormatDouble(center.z) + "), r="
        + FormatDouble(radius) + " -> (x"
        + (center.x >= 0.0 ? "-" : "+") + FormatDouble(std::abs(center.x)) + ")^2 + (y"
        + (center.y >= 0.0 ? "-" : "+") + FormatDouble(std::abs(center.y)) + ")^2 + (z"
        + (center.z >= 0.0 ? "-" : "+") + FormatDouble(std::abs(center.z)) + ")^2 = "
        + FormatDouble(radius * radius));
    fit.description = L"Sphere fit computed with algebraic least squares.";
    fit.evaluator = [center, radius](const Vec3d& point) {
        return Length(point - center) - radius;
    };
    return fit;
}

CandidateFit FitCylinder(const ModelData& model, const std::vector<Vec3d>& vertices) {
    CandidateFit fit;
    fit.method = ConversionMethod::Cylinder;
    if (vertices.size() < 6) {
        return fit;
    }

    const Vec3d centroid = ComputeCentroid(vertices);
    const auto covariance = ComputeCovariance(vertices, centroid);
    std::vector<double> matrix = {
        covariance[0], covariance[1], covariance[2],
        covariance[3], covariance[4], covariance[5],
        covariance[6], covariance[7], covariance[8],
    };

    std::vector<double> eigenvalues;
    std::vector<double> eigenvectors;
    JacobiEigenDecomposition(matrix, 3, eigenvalues, eigenvectors);

    int largest = 0;
    for (int index = 1; index < 3; ++index) {
        if (eigenvalues[index] > eigenvalues[largest]) {
            largest = index;
        }
    }

    Vec3d axis{
        At(eigenvectors, 3, 0, largest),
        At(eigenvectors, 3, 1, largest),
        At(eigenvectors, 3, 2, largest),
    };
    axis = Normalize(axis);

    std::vector<double> radialDistances;
    radialDistances.reserve(vertices.size());
    double totalRadius = 0.0;
    for (const Vec3d& vertex : vertices) {
        const Vec3d delta = vertex - centroid;
        const double axial = Dot(delta, axis);
        const Vec3d radialVector = delta - axis * axial;
        const double radialDistance = Length(radialVector);
        radialDistances.push_back(radialDistance);
        totalRadius += radialDistance;
    }

    const double radius = totalRadius / static_cast<double>(radialDistances.size());
    if (radius <= 1e-9) {
        return fit;
    }

    for (double radialDistance : radialDistances) {
        const double residual = std::abs(radialDistance - radius);
        fit.meanResidual += residual;
        fit.rmsResidual += residual * residual;
        fit.maxResidual = std::max(fit.maxResidual, residual);
    }

    fit.valid = true;
    fit.meanResidual /= static_cast<double>(radialDistances.size());
    fit.rmsResidual = std::sqrt(fit.rmsResidual / static_cast<double>(radialDistances.size()));
    fit.score = fit.meanResidual / std::max(1e-9, static_cast<double>(model.scale));

    const double alignmentX = std::abs(axis.x);
    const double alignmentY = std::abs(axis.y);
    const double alignmentZ = std::abs(axis.z);
    if (alignmentZ > 0.95 && alignmentZ >= alignmentX && alignmentZ >= alignmentY) {
        fit.equation = ToWide(
            std::string("Cylinder: (x")
            + (centroid.x >= 0.0 ? "-" : "+") + FormatDouble(std::abs(centroid.x)) + ")^2 + (y"
            + (centroid.y >= 0.0 ? "-" : "+") + FormatDouble(std::abs(centroid.y)) + ")^2 = "
            + FormatDouble(radius * radius));
    } else if (alignmentY > 0.95 && alignmentY >= alignmentX) {
        fit.equation = ToWide(
            std::string("Cylinder: (x")
            + (centroid.x >= 0.0 ? "-" : "+") + FormatDouble(std::abs(centroid.x)) + ")^2 + (z"
            + (centroid.z >= 0.0 ? "-" : "+") + FormatDouble(std::abs(centroid.z)) + ")^2 = "
            + FormatDouble(radius * radius));
    } else if (alignmentX > 0.95) {
        fit.equation = ToWide(
            std::string("Cylinder: (y")
            + (centroid.y >= 0.0 ? "-" : "+") + FormatDouble(std::abs(centroid.y)) + ")^2 + (z"
            + (centroid.z >= 0.0 ? "-" : "+") + FormatDouble(std::abs(centroid.z)) + ")^2 = "
            + FormatDouble(radius * radius));
    } else {
        fit.equation = ToWide(
            std::string("Cylinder: center=(")
            + FormatDouble(centroid.x) + ", "
            + FormatDouble(centroid.y) + ", "
            + FormatDouble(centroid.z) + "), axis=("
            + FormatDouble(axis.x) + ", "
            + FormatDouble(axis.y) + ", "
            + FormatDouble(axis.z) + "), r="
            + FormatDouble(radius) + "\r\n"
            + "Implicit form: ||(p-c) - a((p-c)·a)||^2 = "
            + FormatDouble(radius * radius));
    }
    fit.description = L"Cylinder fit estimated from the dominant PCA axis and radial distances.";
    fit.evaluator = [centroid, axis, radius](const Vec3d& point) {
        const Vec3d delta = point - centroid;
        const double axial = Dot(delta, axis);
        const Vec3d radialVector = delta - axis * axial;
        return Length(radialVector) - radius;
    };
    return fit;
}

std::vector<MonomialTerm> BuildBasis(int degree) {
    std::vector<MonomialTerm> terms = {
        {0, 0, 0, "1"},
        {1, 0, 0, "u"},
        {0, 1, 0, "v"},
        {0, 0, 1, "w"},
        {2, 0, 0, "u^2"},
        {0, 2, 0, "v^2"},
        {0, 0, 2, "w^2"},
        {1, 1, 0, "u*v"},
        {1, 0, 1, "u*w"},
        {0, 1, 1, "v*w"},
    };

    if (degree >= 3) {
        const std::vector<MonomialTerm> cubicTerms = {
            {3, 0, 0, "u^3"},
            {0, 3, 0, "v^3"},
            {0, 0, 3, "w^3"},
            {2, 1, 0, "u^2*v"},
            {2, 0, 1, "u^2*w"},
            {1, 2, 0, "u*v^2"},
            {0, 2, 1, "v^2*w"},
            {1, 0, 2, "u*w^2"},
            {0, 1, 2, "v*w^2"},
            {1, 1, 1, "u*v*w"},
        };
        terms.insert(terms.end(), cubicTerms.begin(), cubicTerms.end());
    }

    return terms;
}

double EvaluateMonomial(const MonomialTerm& term, const Vec3d& point) {
    return std::pow(point.x, term.xExponent) * std::pow(point.y, term.yExponent) * std::pow(point.z, term.zExponent);
}

double EvaluateRbfKernel(const Vec3d& point, const Vec3d& center, double radius) {
    const double radiusSquared = std::max(radius * radius, 1e-8);
    return std::exp(-DistanceSquared(point, center) / radiusSquared);
}

double EstimateCenterSpacing(const std::vector<Vec3d>& centers) {
    if (centers.size() < 2) {
        return 0.12;
    }

    double total = 0.0;
    for (std::size_t left = 0; left < centers.size(); ++left) {
        double bestSquared = std::numeric_limits<double>::max();
        for (std::size_t right = 0; right < centers.size(); ++right) {
            if (left == right) {
                continue;
            }
            bestSquared = std::min(bestSquared, DistanceSquared(centers[left], centers[right]));
        }
        total += std::sqrt(bestSquared);
    }
    return total / static_cast<double>(centers.size());
}

std::vector<std::size_t> BuildFarthestPointIndices(const std::vector<Vec3d>& points, std::size_t maxCount) {
    if (points.empty() || maxCount == 0) {
        return {};
    }

    const std::size_t count = std::min(points.size(), maxCount);
    if (count == points.size()) {
        std::vector<std::size_t> indices(points.size());
        std::iota(indices.begin(), indices.end(), 0);
        return indices;
    }

    std::vector<std::size_t> indices;
    indices.reserve(count);

    const Vec3d centroid = ComputeCentroid(points);
    std::size_t firstIndex = 0;
    double firstDistanceSquared = -1.0;
    for (std::size_t index = 0; index < points.size(); ++index) {
        const double distanceSquared = DistanceSquared(points[index], centroid);
        if (distanceSquared > firstDistanceSquared) {
            firstDistanceSquared = distanceSquared;
            firstIndex = index;
        }
    }

    indices.push_back(firstIndex);
    std::vector<double> minDistanceSquared(points.size(), std::numeric_limits<double>::max());
    for (std::size_t index = 0; index < points.size(); ++index) {
        minDistanceSquared[index] = DistanceSquared(points[index], points[firstIndex]);
    }
    minDistanceSquared[firstIndex] = 0.0;

    while (indices.size() < count) {
        std::size_t nextIndex = 0;
        double nextDistanceSquared = -1.0;
        for (std::size_t index = 0; index < points.size(); ++index) {
            if (minDistanceSquared[index] > nextDistanceSquared) {
                nextDistanceSquared = minDistanceSquared[index];
                nextIndex = index;
            }
        }

        indices.push_back(nextIndex);
        minDistanceSquared[nextIndex] = 0.0;
        for (std::size_t index = 0; index < points.size(); ++index) {
            const double distanceSquared = DistanceSquared(points[index], points[nextIndex]);
            if (distanceSquared < minDistanceSquared[index]) {
                minDistanceSquared[index] = distanceSquared;
            }
        }
    }

    return indices;
}

CandidateFit FitRbfImplicit(const ModelData& model, const std::vector<Vec3d>& normalizedVertices, double offsetStepNormalized) {
    CandidateFit fit;
    fit.method = ConversionMethod::RbfImplicit;
    if (normalizedVertices.size() < 20) {
        return fit;
    }

    const std::size_t centerCount = std::min<std::size_t>(112, std::max<std::size_t>(48, model.VertexCount() / 24));
    const std::vector<std::size_t> centerIndices = BuildFarthestPointIndices(normalizedVertices, centerCount);
    std::vector<Vec3d> centers;
    centers.reserve(centerIndices.size());
    for (std::size_t index : centerIndices) {
        centers.push_back(normalizedVertices[index]);
    }

    if (centers.size() < 8) {
        return fit;
    }

    const double radius = std::clamp(EstimateCenterSpacing(centers) * 1.65, 0.08, 0.34);
    const std::size_t surfaceSampleCount = std::min<std::size_t>(normalizedVertices.size(), std::max<std::size_t>(900, centers.size() * 20));
    const std::vector<Vec3d> normals = ComputeVertexNormals(model, normalizedVertices);
    const std::vector<std::size_t> sampleIndices = BuildFarthestPointIndices(normalizedVertices, surfaceSampleCount);
    const double step = std::clamp(offsetStepNormalized * 0.8, 0.012, 0.05);
    std::vector<FittingSample> samples;
    samples.reserve(sampleIndices.size() * 3);
    for (std::size_t sampleIndex : sampleIndices) {
        const Vec3d& point = normalizedVertices[sampleIndex];
        const Vec3d& normal = normals[sampleIndex];
        samples.push_back({point, 0.0, 10.0});
        if (Length(normal) > 0.0) {
            samples.push_back({point + normal * step, step, 1.5});
            samples.push_back({point - normal * step, -step, 1.5});
        }
    }

    const int unknownCount = static_cast<int>(centers.size() + 1);
    std::vector<double> normalMatrix(static_cast<std::size_t>(unknownCount) * static_cast<std::size_t>(unknownCount), 0.0);
    std::vector<double> rhs(static_cast<std::size_t>(unknownCount), 0.0);

    for (const FittingSample& sample : samples) {
        std::vector<double> row(static_cast<std::size_t>(unknownCount), 0.0);
        for (std::size_t centerIndex = 0; centerIndex < centers.size(); ++centerIndex) {
            row[centerIndex] = EvaluateRbfKernel(sample.point, centers[centerIndex], radius);
        }
        row[centers.size()] = 1.0;

        for (int left = 0; left < unknownCount; ++left) {
            rhs[left] += sample.weight * row[left] * sample.target;
            for (int right = left; right < unknownCount; ++right) {
                At(normalMatrix, unknownCount, left, right) += sample.weight * row[left] * row[right];
            }
        }
    }

    for (int row = 0; row < unknownCount; ++row) {
        for (int column = row + 1; column < unknownCount; ++column) {
            At(normalMatrix, unknownCount, column, row) = At(normalMatrix, unknownCount, row, column);
        }
    }

    const double regularization = static_cast<double>(samples.size()) * 0.00002;
    for (std::size_t centerIndex = 0; centerIndex < centers.size(); ++centerIndex) {
        At(normalMatrix, unknownCount, static_cast<int>(centerIndex), static_cast<int>(centerIndex)) += regularization;
    }
    At(normalMatrix, unknownCount, unknownCount - 1, unknownCount - 1) += 1e-8;

    std::vector<double> coefficients;
    if (!SolveLinearSystem(normalMatrix, rhs, unknownCount, coefficients)) {
        return fit;
    }

    for (const Vec3d& vertex : normalizedVertices) {
        double value = 0.0;
        for (std::size_t centerIndex = 0; centerIndex < centers.size(); ++centerIndex) {
            value += coefficients[centerIndex] * EvaluateRbfKernel(vertex, centers[centerIndex], radius);
        }
        value += coefficients[centers.size()];

        const double residual = std::abs(value);
        fit.meanResidual += residual;
        fit.rmsResidual += residual * residual;
        fit.maxResidual = std::max(fit.maxResidual, residual);
    }

    fit.valid = true;
    fit.meanResidual /= static_cast<double>(normalizedVertices.size());
    fit.rmsResidual = std::sqrt(fit.rmsResidual / static_cast<double>(normalizedVertices.size()));
    fit.score = fit.rmsResidual + static_cast<double>(centers.size()) * 0.00002;

    std::ostringstream equation;
    equation << "RBF surface (gaussian)\r\n"
             << "u = (x - " << FormatDouble(model.center[0]) << ") / " << FormatDouble(model.scale) << "\r\n"
             << "v = (y - " << FormatDouble(model.center[1]) << ") / " << FormatDouble(model.scale) << "\r\n"
             << "w = (z - " << FormatDouble(model.center[2]) << ") / " << FormatDouble(model.scale) << "\r\n"
             << "radius = " << FormatDouble(radius) << "\r\n"
             << "center count = " << centers.size() << "\r\n"
             << "f(u, v, w) =\r\n"
             << "  " << FormatDouble(coefficients[centers.size()]) << "\r\n";

    for (std::size_t centerIndex = 0; centerIndex < centers.size(); ++centerIndex) {
        equation << "  " << (coefficients[centerIndex] >= 0.0 ? "+ " : "- ")
                 << FormatDouble(std::abs(coefficients[centerIndex]))
                 << " * exp(-(((u - " << FormatDouble(centers[centerIndex].x)
                 << ")^2 + (v - " << FormatDouble(centers[centerIndex].y)
                 << ")^2 + (w - " << FormatDouble(centers[centerIndex].z)
                 << ")^2) / " << FormatDouble(radius * radius) << "))\r\n";
    }
    equation << "= 0";

    fit.equation = ToWide(equation.str());
    fit.description = L"RBF surface fitted with Gaussian kernels, spatially distributed centers, and signed offset samples.";
    fit.evaluator = [coefficients, centers, radius, centerX = static_cast<double>(model.center[0]), centerY = static_cast<double>(model.center[1]), centerZ = static_cast<double>(model.center[2]), scale = static_cast<double>(model.scale)](const Vec3d& point) {
        Vec3d normalized{
            (point.x - centerX) / scale,
            (point.y - centerY) / scale,
            (point.z - centerZ) / scale,
        };
        double value = coefficients[centers.size()];
        for (std::size_t centerIndex = 0; centerIndex < centers.size(); ++centerIndex) {
            value += coefficients[centerIndex] * EvaluateRbfKernel(normalized, centers[centerIndex], radius);
        }
        return value;
    };
    fit.gpuCenters.reserve(centers.size() * 3);
    fit.gpuWeights.reserve(centers.size());
    for (std::size_t centerIndex = 0; centerIndex < centers.size(); ++centerIndex) {
        fit.gpuCenters.push_back(static_cast<float>(static_cast<double>(model.center[0]) + centers[centerIndex].x * static_cast<double>(model.scale)));
        fit.gpuCenters.push_back(static_cast<float>(static_cast<double>(model.center[1]) + centers[centerIndex].y * static_cast<double>(model.scale)));
        fit.gpuCenters.push_back(static_cast<float>(static_cast<double>(model.center[2]) + centers[centerIndex].z * static_cast<double>(model.scale)));
        fit.gpuWeights.push_back(static_cast<float>(coefficients[centerIndex]));
    }
    fit.gpuConstant = static_cast<float>(coefficients[centers.size()]);
    fit.gpuRadiusSquared = static_cast<float>((radius * static_cast<double>(model.scale)) * (radius * static_cast<double>(model.scale)));
    fit.gpuEvaluable = true;
    return fit;
}

CandidateFit FitImplicitPolynomial(const ModelData& model, const std::vector<Vec3d>& normalizedVertices, double offsetStepNormalized, int degree) {
    CandidateFit fit;
    fit.method = degree == 2 ? ConversionMethod::ImplicitQuadratic : ConversionMethod::ImplicitCubic;
    if (normalizedVertices.size() < 10) {
        return fit;
    }

    const std::vector<MonomialTerm> basis = BuildBasis(degree);
    const int basisCount = static_cast<int>(basis.size());
    std::vector<double> normalMatrix(static_cast<std::size_t>(basisCount) * static_cast<std::size_t>(basisCount), 0.0);

    std::vector<double> rhs(static_cast<std::size_t>(basisCount), 0.0);
    const std::vector<Vec3d> normals = ComputeVertexNormals(model, normalizedVertices);
    const std::vector<std::size_t> sampleIndices = BuildSampleIndices(normalizedVertices.size(), 2800);
    const double step = std::clamp(offsetStepNormalized, 0.01, 0.08);
    std::vector<FittingSample> samples;
    samples.reserve(sampleIndices.size() * 3);

    for (std::size_t sampleIndex : sampleIndices) {
        const Vec3d& point = normalizedVertices[sampleIndex];
        const Vec3d& normal = normals[sampleIndex];
        samples.push_back({point, 0.0, 4.0});
        if (Length(normal) > 0.0) {
            samples.push_back({point + normal * step, 1.0, 1.0});
            samples.push_back({point - normal * step, -1.0, 1.0});
        }
    }

    for (const FittingSample& sample : samples) {
        std::vector<double> row;
        row.reserve(basis.size());
        for (const MonomialTerm& term : basis) {
            row.push_back(EvaluateMonomial(term, sample.point));
        }

        for (int left = 0; left < basisCount; ++left) {
            rhs[left] += sample.weight * row[left] * sample.target;
            for (int right = left; right < basisCount; ++right) {
                At(normalMatrix, basisCount, left, right) += sample.weight * row[left] * row[right];
            }
        }
    }

    for (int row = 0; row < basisCount; ++row) {
        for (int column = row + 1; column < basisCount; ++column) {
            At(normalMatrix, basisCount, column, row) = At(normalMatrix, basisCount, row, column);
        }
    }

    const double regularizationScale = static_cast<double>(samples.size()) * 0.0006;
    for (int index = 0; index < basisCount; ++index) {
        const int totalDegree = basis[index].xExponent + basis[index].yExponent + basis[index].zExponent;
        const double degreePenalty = totalDegree == 0 ? 0.15 : (totalDegree == 1 ? 0.35 : (totalDegree == 2 ? 1.0 : 2.4));
        At(normalMatrix, basisCount, index, index) += regularizationScale * degreePenalty;
    }

    std::vector<double> coefficients;
    if (!SolveLinearSystem(normalMatrix, rhs, basisCount, coefficients)) {
        return fit;
    }

    for (const Vec3d& vertex : normalizedVertices) {
        double value = 0.0;
        for (int index = 0; index < basisCount; ++index) {
            value += coefficients[index] * EvaluateMonomial(basis[index], vertex);
        }
        const double residual = std::abs(value);
        fit.meanResidual += residual;
        fit.rmsResidual += residual * residual;
        fit.maxResidual = std::max(fit.maxResidual, residual);
    }

    fit.valid = true;
    fit.meanResidual /= static_cast<double>(normalizedVertices.size());
    fit.rmsResidual = std::sqrt(fit.rmsResidual / static_cast<double>(normalizedVertices.size()));
    fit.score = fit.meanResidual;

    double displayScale = 0.0;
    for (double coefficient : coefficients) {
        displayScale = std::max(displayScale, std::abs(coefficient));
    }
    if (displayScale < 1e-12) {
        return fit;
    }

    std::ostringstream equation;
    equation << "Implicit fit (degree " << degree << ")\r\n"
             << "u = (x - " << FormatDouble(model.center[0]) << ") / " << FormatDouble(model.scale) << "\r\n"
             << "v = (y - " << FormatDouble(model.center[1]) << ") / " << FormatDouble(model.scale) << "\r\n"
             << "w = (z - " << FormatDouble(model.center[2]) << ") / " << FormatDouble(model.scale) << "\r\n"
             << "Signed regularized fit with surface and offset samples\r\n"
             << "f(u, v, w) =\r\n";
    for (std::size_t index = 0; index < basis.size(); ++index) {
        const double coefficient = coefficients[index] / displayScale;
        if (std::abs(coefficient) < 0.0005) {
            continue;
        }
        equation << "  " << (coefficient >= 0.0 ? "+ " : "- ") << FormatDouble(std::abs(coefficient));
        if (basis[index].label != "1") {
            equation << " * " << basis[index].label;
        }
        equation << "\r\n";
    }
    equation << "= 0";

    fit.equation = ToWide(equation.str());
    fit.description = L"Implicit fit built in normalized coordinates with regularized signed samples from the mesh surface.";
    fit.evaluator = [coefficients, basis, centerX = static_cast<double>(model.center[0]), centerY = static_cast<double>(model.center[1]), centerZ = static_cast<double>(model.center[2]), scale = static_cast<double>(model.scale)](const Vec3d& point) {
        const Vec3d normalized{
            (point.x - centerX) / scale,
            (point.y - centerY) / scale,
            (point.z - centerZ) / scale,
        };
        double value = 0.0;
        for (std::size_t index = 0; index < basis.size(); ++index) {
            value += coefficients[index] * EvaluateMonomial(basis[index], normalized);
        }
        return value;
    };
    return fit;
}

CandidateFit ResolveAutoFit(const ModelData& model, const std::vector<Vec3d>& vertices, const std::vector<Vec3d>& normalizedVertices, double offsetStepNormalized) {
    CandidateFit plane = FitPlane(model, vertices);
    CandidateFit sphere = FitSphere(model, vertices);
    CandidateFit cylinder = FitCylinder(model, vertices);

    CandidateFit best = plane;
    if (!best.valid || (sphere.valid && sphere.score < best.score)) {
        best = sphere;
    }
    if (!best.valid || (cylinder.valid && cylinder.score < best.score)) {
        best = cylinder;
    }

    if (best.valid && best.score < 0.025) {
        return best;
    }

    CandidateFit polynomial = FitImplicitPolynomial(model, normalizedVertices, offsetStepNormalized, 3);
    CandidateFit rbf = FitRbfImplicit(model, normalizedVertices, offsetStepNormalized);

    if (rbf.valid) {
        const bool polynomialIsWeak = !polynomial.valid
            || polynomial.rmsResidual > 0.04
            || polynomial.maxResidual > 0.12;
        const bool rbfClearlyBetter = !polynomial.valid || (rbf.rmsResidual + 0.003 < polynomial.rmsResidual);
        const bool rbfCompetitiveOnComplexMesh = model.VertexCount() > 2500
            && polynomial.valid
            && polynomial.rmsResidual > 0.03
            && rbf.rmsResidual <= polynomial.rmsResidual * 1.08;
        if (polynomialIsWeak || rbfClearlyBetter || rbfCompetitiveOnComplexMesh) {
            return rbf;
        }
    }

    if (polynomial.valid) {
        return polynomial;
    }

    if (rbf.valid) {
        return rbf;
    }
    return best;
}

CandidateFit SolveRequestedFit(
    const ModelData& model,
    ConversionMethod method,
    const std::vector<Vec3d>& vertices,
    const std::vector<Vec3d>& normalizedVertices,
    double offsetStepNormalized) {
    switch (method) {
        case ConversionMethod::AutoDetect:
            return ResolveAutoFit(model, vertices, normalizedVertices, offsetStepNormalized);
        case ConversionMethod::Plane:
            return FitPlane(model, vertices);
        case ConversionMethod::Sphere:
            return FitSphere(model, vertices);
        case ConversionMethod::Cylinder:
            return FitCylinder(model, vertices);
        case ConversionMethod::ImplicitQuadratic:
            return FitImplicitPolynomial(model, normalizedVertices, offsetStepNormalized, 2);
        case ConversionMethod::ImplicitCubic:
            return FitImplicitPolynomial(model, normalizedVertices, offsetStepNormalized, 3);
        case ConversionMethod::RbfImplicit:
            return FitRbfImplicit(model, normalizedVertices, offsetStepNormalized);
    }
    return {};
}

CandidateFit ResolveReconstructionAutoFit(
    const ModelData& model,
    const std::vector<Vec3d>& vertices,
    const std::vector<Vec3d>& normalizedVertices,
    double offsetStepNormalized) {
    CandidateFit rbf = FitRbfImplicit(model, normalizedVertices, offsetStepNormalized);
    CandidateFit cubic = FitImplicitPolynomial(model, normalizedVertices, offsetStepNormalized, 3);
    CandidateFit quadratic = FitImplicitPolynomial(model, normalizedVertices, offsetStepNormalized, 2);

    if (rbf.valid) {
        const bool cubicMissingOrWeak = !cubic.valid
            || cubic.rmsResidual > 0.05
            || cubic.maxResidual > 0.16;
        const bool rbfBetterThanCubic = !cubic.valid || (rbf.rmsResidual <= cubic.rmsResidual * 1.15);
        const bool denseOrganicMesh = model.VertexCount() > 1200 || model.TriangleCount() > 2000;
        if (cubicMissingOrWeak || rbfBetterThanCubic || denseOrganicMesh) {
            return rbf;
        }
    }

    if (cubic.valid) {
        return cubic;
    }

    if (quadratic.valid) {
        return quadratic;
    }

    return ResolveAutoFit(model, vertices, normalizedVertices, offsetStepNormalized);
}

[[maybe_unused]] CandidateFit SolveRequestedReconstructionFit(
    const ModelData& model,
    ConversionMethod method,
    const std::vector<Vec3d>& vertices,
    const std::vector<Vec3d>& normalizedVertices,
    double offsetStepNormalized) {
    if (method == ConversionMethod::AutoDetect) {
        return ResolveReconstructionAutoFit(model, vertices, normalizedVertices, offsetStepNormalized);
    }
    return SolveRequestedFit(model, method, vertices, normalizedVertices, offsetStepNormalized);
}

Vec3d InterpolateIsoPoint(const Vec3d& a, const Vec3d& b, double valueA, double valueB) {
    const double denominator = valueA - valueB;
    const double t = std::abs(denominator) < 1e-12
        ? 0.5
        : std::clamp(valueA / denominator, 0.0, 1.0);
    return a + (b - a) * t;
}

void PushTriangle(ModelData& model, const Vec3d& a, const Vec3d& b, const Vec3d& c) {
    const std::uint32_t baseIndex = static_cast<std::uint32_t>(model.VertexCount());
    model.positions.push_back(static_cast<float>(a.x));
    model.positions.push_back(static_cast<float>(a.y));
    model.positions.push_back(static_cast<float>(a.z));
    model.positions.push_back(static_cast<float>(b.x));
    model.positions.push_back(static_cast<float>(b.y));
    model.positions.push_back(static_cast<float>(b.z));
    model.positions.push_back(static_cast<float>(c.x));
    model.positions.push_back(static_cast<float>(c.y));
    model.positions.push_back(static_cast<float>(c.z));
    model.triangles.push_back(baseIndex);
    model.triangles.push_back(baseIndex + 1);
    model.triangles.push_back(baseIndex + 2);
}

void PolygonizeTetrahedron(
    const std::array<Vec3d, 4>& points,
    const std::array<double, 4>& values,
    ModelData& mesh) {
    std::array<int, 4> inside{};
    std::array<int, 4> outside{};
    int insideCount = 0;
    int outsideCount = 0;
    for (int index = 0; index < 4; ++index) {
        if (values[index] <= 0.0) {
            inside[insideCount++] = index;
        } else {
            outside[outsideCount++] = index;
        }
    }

    if (insideCount == 0 || insideCount == 4) {
        return;
    }

    auto interpolate = [&](int first, int second) {
        return InterpolateIsoPoint(points[first], points[second], values[first], values[second]);
    };

    if (insideCount == 1 || insideCount == 3) {
        const bool invert = insideCount == 3;
        const int pivot = invert ? outside[0] : inside[0];
        const std::array<int, 3> others = invert
            ? std::array<int, 3>{inside[0], inside[1], inside[2]}
            : std::array<int, 3>{outside[0], outside[1], outside[2]};
        const Vec3d p0 = interpolate(pivot, others[0]);
        const Vec3d p1 = interpolate(pivot, others[1]);
        const Vec3d p2 = interpolate(pivot, others[2]);
        if (invert) {
            PushTriangle(mesh, p0, p2, p1);
        } else {
            PushTriangle(mesh, p0, p1, p2);
        }
        return;
    }

    const int a = inside[0];
    const int b = inside[1];
    const int c = outside[0];
    const int d = outside[1];
    const Vec3d p0 = interpolate(a, c);
    const Vec3d p1 = interpolate(a, d);
    const Vec3d p2 = interpolate(b, c);
    const Vec3d p3 = interpolate(b, d);
    PushTriangle(mesh, p0, p1, p2);
    PushTriangle(mesh, p1, p3, p2);
}

bool BuildMarchingCubesMesh(
    const ModelData& source,
    const CandidateFit& fit,
    ModelData& mesh,
    int& outResolution,
    double& outExtractionMs,
    std::wstring& outComputeMode,
    std::wstring& errorMessage) {
    if (!fit.evaluator) {
        errorMessage = L"The selected fit could not be sampled for reconstruction.";
        return false;
    }

    const auto start = std::chrono::steady_clock::now();
    const int resolution = DeterminePreviewResolution(source, fit);
    outResolution = resolution;

    const double padding = std::max(0.06, 0.12);
    const double minX = source.boundsMin[0] - (source.boundsMax[0] - source.boundsMin[0]) * padding;
    const double minY = source.boundsMin[1] - (source.boundsMax[1] - source.boundsMin[1]) * padding;
    const double minZ = source.boundsMin[2] - (source.boundsMax[2] - source.boundsMin[2]) * padding;
    const double maxX = source.boundsMax[0] + (source.boundsMax[0] - source.boundsMin[0]) * padding;
    const double maxY = source.boundsMax[1] + (source.boundsMax[1] - source.boundsMin[1]) * padding;
    const double maxZ = source.boundsMax[2] + (source.boundsMax[2] - source.boundsMin[2]) * padding;

    const int gridSize = resolution + 1;
    const double stepX = (maxX - minX) / static_cast<double>(resolution);
    const double stepY = (maxY - minY) / static_cast<double>(resolution);
    const double stepZ = (maxZ - minZ) / static_cast<double>(resolution);
    const auto scalarIndex = [gridSize](int x, int y, int z) {
        return (static_cast<std::size_t>(z) * static_cast<std::size_t>(gridSize) + static_cast<std::size_t>(y)) * static_cast<std::size_t>(gridSize) + static_cast<std::size_t>(x);
    };

    std::vector<double> values(static_cast<std::size_t>(gridSize) * static_cast<std::size_t>(gridSize) * static_cast<std::size_t>(gridSize), 0.0);
    bool usedGpu = false;
    if (GpuComputeEnabled() && fit.gpuEvaluable) {
        GpuRbfFieldRequest request;
        request.centers = fit.gpuCenters;
        request.weights = fit.gpuWeights;
        request.constantTerm = fit.gpuConstant;
        request.radiusSquared = fit.gpuRadiusSquared;
        request.minBounds[0] = static_cast<float>(minX);
        request.minBounds[1] = static_cast<float>(minY);
        request.minBounds[2] = static_cast<float>(minZ);
        request.step[0] = static_cast<float>(stepX);
        request.step[1] = static_cast<float>(stepY);
        request.step[2] = static_cast<float>(stepZ);
        request.gridSize = gridSize;

        GpuFieldResult gpuResult;
        if (EvaluateRbfFieldOnGpu(request, gpuResult) && gpuResult.values.size() == values.size()) {
            for (std::size_t index = 0; index < values.size(); ++index) {
                values[index] = gpuResult.values[index];
            }
            usedGpu = true;
            outComputeMode = L"GPU compute (OpenGL 4.3)";
        } else {
            outComputeMode = gpuResult.status.empty() ? L"CPU fallback" : L"CPU fallback after GPU attempt";
        }
    } else {
        outComputeMode = GpuComputeEnabled() ? L"CPU (current fit stays on CPU)" : L"CPU";
    }

    if (!usedGpu) {
        for (int z = 0; z < gridSize; ++z) {
            for (int y = 0; y < gridSize; ++y) {
                for (int x = 0; x < gridSize; ++x) {
                    const Vec3d point{
                        minX + stepX * static_cast<double>(x),
                        minY + stepY * static_cast<double>(y),
                        minZ + stepZ * static_cast<double>(z),
                    };
                    values[scalarIndex(x, y, z)] = fit.evaluator(point);
                }
            }
        }
    }

    constexpr int cornerOffsets[8][3] = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
        {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1},
    };
    constexpr int tetrahedra[6][4] = {
        {0, 5, 1, 6},
        {0, 1, 2, 6},
        {0, 2, 3, 6},
        {0, 3, 7, 6},
        {0, 7, 4, 6},
        {0, 4, 5, 6},
    };

    ModelData generated;
    generated.label = source.label + L" [Preview]";
    generated.positions.reserve(static_cast<std::size_t>(resolution) * static_cast<std::size_t>(resolution) * 18);
    generated.triangles.reserve(static_cast<std::size_t>(resolution) * static_cast<std::size_t>(resolution) * 12);

    for (int z = 0; z < resolution; ++z) {
        for (int y = 0; y < resolution; ++y) {
            for (int x = 0; x < resolution; ++x) {
                std::array<Vec3d, 8> cubePoints{};
                std::array<double, 8> cubeValues{};
                for (int corner = 0; corner < 8; ++corner) {
                    const int gx = x + cornerOffsets[corner][0];
                    const int gy = y + cornerOffsets[corner][1];
                    const int gz = z + cornerOffsets[corner][2];
                    cubePoints[corner] = {
                        minX + stepX * static_cast<double>(gx),
                        minY + stepY * static_cast<double>(gy),
                        minZ + stepZ * static_cast<double>(gz),
                    };
                    cubeValues[corner] = values[scalarIndex(gx, gy, gz)];
                }

                for (const auto& tetrahedron : tetrahedra) {
                    std::array<Vec3d, 4> tetraPoints{
                        cubePoints[tetrahedron[0]],
                        cubePoints[tetrahedron[1]],
                        cubePoints[tetrahedron[2]],
                        cubePoints[tetrahedron[3]],
                    };
                    std::array<double, 4> tetraValues{
                        cubeValues[tetrahedron[0]],
                        cubeValues[tetrahedron[1]],
                        cubeValues[tetrahedron[2]],
                        cubeValues[tetrahedron[3]],
                    };
                    PolygonizeTetrahedron(tetraPoints, tetraValues, generated);
                }
            }
        }
    }

    if (generated.triangles.empty()) {
        errorMessage = L"No isosurface could be extracted from the selected fit.";
        return false;
    }

    FinalizeGeneratedModel(generated);
    mesh = std::move(generated);
    outExtractionMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    return true;
}

std::wstring BuildReport(const ModelData& model, const FitResult& result, const std::wstring& description) {
    std::wstring report;
    report += L"Model\r\n";
    report += L"-----\r\n";
    report += L"Source: " + model.label + L"\r\n";
    report += L"Vertices: " + std::to_wstring(model.VertexCount()) + L"\r\n";
    report += L"Triangles: " + std::to_wstring(model.TriangleCount()) + L"\r\n";
    report += L"Model memory: " + ToWide(static_cast<double>(model.MemoryBytes()) / 1024.0) + L" KB\r\n";
    report += L"Mean nearest-neighbor spacing (KD-tree): " + ToWide(result.nearestNeighborMean) + L"\r\n";
    report += L"Max nearest-neighbor spacing: " + ToWide(result.nearestNeighborMax) + L"\r\n";
    report += L"Half extent scale: " + ToWide(model.scale) + L"\r\n\r\n";
    report += L"Method\r\n";
    report += L"------\r\n";
    report += L"Requested: " + ConversionMethodName(result.requestedMethod) + L"\r\n";
    report += L"Resolved: " + ConversionMethodName(result.resolvedMethod) + L"\r\n";
    report += L"Background fit time: " + ToWide(result.elapsedMilliseconds, 2) + L" ms\r\n\r\n";
    report += L"Equation\r\n";
    report += L"--------\r\n";
    report += result.equation + L"\r\n\r\n";
    report += L"Fit metrics\r\n";
    report += L"-----------\r\n";
    report += L"Mean residual: " + ToWide(result.meanAbsoluteResidual) + L"\r\n";
    report += L"RMS residual: " + ToWide(result.rmsResidual) + L"\r\n";
    report += L"Max residual: " + ToWide(result.maxResidual) + L"\r\n\r\n";
    report += L"Notes\r\n";
    report += L"-----\r\n";
    report += description;
    return report;
}

}  // namespace

FitResult AnalyzeModel(const ModelData& model, ConversionMethod method) {
    FitResult result;
    result.requestedMethod = method;
    result.resolvedMethod = method;
    result.inputPointCount = model.VertexCount();
    result.triangleCount = model.TriangleCount();

    const auto start = std::chrono::steady_clock::now();
    if (model.VertexCount() == 0) {
        result.report = L"No model vertices were available.";
        return result;
    }

    const std::vector<Vec3d> vertices = ReadAllVertices(model, false);
    const std::vector<Vec3d> normalizedVertices = ReadAllVertices(model, true);
    const NearestNeighborStats spacing = ComputeNearestNeighborStats(model);
    result.nearestNeighborMean = spacing.mean;
    result.nearestNeighborMax = spacing.max;
    const double offsetStepNormalized = model.scale > 1e-9f
        ? std::clamp((spacing.mean / static_cast<double>(model.scale)) * 0.6, 0.012, 0.08)
        : 0.03;

    CandidateFit candidate = SolveRequestedFit(model, method, vertices, normalizedVertices, offsetStepNormalized);

    result.elapsedMilliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();

    if (!candidate.valid) {
        result.report = L"The selected method could not fit the current model.";
        return result;
    }

    result.success = true;
    result.resolvedMethod = candidate.method;
    result.meanAbsoluteResidual = candidate.meanResidual;
    result.rmsResidual = candidate.rmsResidual;
    result.maxResidual = candidate.maxResidual;
    result.equation = candidate.equation;
    result.report = BuildReport(model, result, candidate.description);
    return result;
}

MarchingCubesResult GenerateMarchingCubesPreview(const ModelData& model, ConversionMethod method) {
    MarchingCubesResult result;
    result.fit.requestedMethod = method;
    result.fit.resolvedMethod = method;
    result.fit.inputPointCount = model.VertexCount();
    result.fit.triangleCount = model.TriangleCount();

    if (model.VertexCount() == 0) {
        result.report = L"No model vertices were available.";
        result.fit.report = result.report;
        return result;
    }

    const std::vector<Vec3d> vertices = ReadAllVertices(model, false);
    const std::vector<Vec3d> normalizedVertices = ReadAllVertices(model, true);
    const NearestNeighborStats spacing = ComputeNearestNeighborStats(model);
    result.fit.nearestNeighborMean = spacing.mean;
    result.fit.nearestNeighborMax = spacing.max;
    const double offsetStepNormalized = model.scale > 1e-9f
        ? std::clamp((spacing.mean / static_cast<double>(model.scale)) * 0.6, 0.012, 0.08)
        : 0.03;

    const auto fitStart = std::chrono::steady_clock::now();
    CandidateFit candidate = SolveRequestedFit(model, method, vertices, normalizedVertices, offsetStepNormalized);
    result.fit.elapsedMilliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - fitStart).count();

    if (!candidate.valid) {
        result.report = L"The selected method could not fit the current model.";
        result.fit.report = result.report;
        return result;
    }

    result.fit.success = true;
    result.fit.resolvedMethod = candidate.method;
    result.fit.meanAbsoluteResidual = candidate.meanResidual;
    result.fit.rmsResidual = candidate.rmsResidual;
    result.fit.maxResidual = candidate.maxResidual;
    result.fit.equation = candidate.equation;
    result.fit.report = BuildReport(model, result.fit, candidate.description);

    int resolution = 0;
    double extractionMilliseconds = 0.0;
    std::wstring computeMode;
    std::wstring extractionError;
    if (!BuildMarchingCubesMesh(model, candidate, result.mesh, resolution, extractionMilliseconds, computeMode, extractionError)) {
        result.report = result.fit.report + L"\r\n\r\nMarching Cubes\r\n--------------\r\n" + extractionError;
        return result;
    }

    result.success = true;
    result.report = result.fit.report
        + L"\r\n\r\nMarching Cubes\r\n--------------\r\n"
        + L"Grid: " + std::to_wstring(resolution) + L" x " + std::to_wstring(resolution) + L" x " + std::to_wstring(resolution) + L"\r\n"
        + L"Field sampling: " + computeMode + L"\r\n"
        + L"Preview vertices: " + std::to_wstring(result.mesh.VertexCount()) + L"\r\n"
        + L"Preview triangles: " + std::to_wstring(result.mesh.TriangleCount()) + L"\r\n"
        + L"Extraction time: " + ToWide(extractionMilliseconds, 2) + L" ms\r\n"
        + L"Viewport updated with the reconstructed surface.";
    return result;
}
