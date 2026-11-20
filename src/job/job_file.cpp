#include "job/job_file.h"
#include "job/binary_io.h"

#include <algorithm>
#include <string_view>

namespace navfast {
namespace {

constexpr std::string_view Magic = "NAVJOB";
constexpr std::uint32_t ClassicVersion = 1;
constexpr std::uint32_t Version = 2;

void blob(BinaryWriter& writer, std::span<const std::byte> bytes) {
    writer.put(binaryCount(bytes.size()));
    writer.append(bytes);
}

std::vector<std::byte> blob(BinaryReader& reader) {
    const auto bytes = reader.take(reader.get<std::uint32_t>());
    return {bytes.begin(), bytes.end()};
}

} // namespace

void writeJobFile(const std::filesystem::path& path, const JobInputs& inputs) {
    BinaryWriter writer;
    const auto& params = inputs.params;
    writer.append(std::as_bytes(std::span{Magic.data(), Magic.size()}));
    writer.put(params.exact ? Version : ClassicVersion);
    writer.put<std::uint8_t>(inputs.mode);
    writer.put<std::uint32_t>(inputs.threads);
    writer.put(params.slopeLimit);
    writer.put(params.slopeTolerance);
    writer.put<std::int32_t>(params.displacementTest);
    writer.put<std::uint8_t>(params.generateFenceTops);
    writer.put<std::uint8_t>(params.fixupJumpAreas);
    writer.put<std::uint8_t>(params.generateJumpConnections);
    writer.put<std::int32_t>(params.areaMaxSize);
    if (params.exact) writer.put<std::uint8_t>(params.exact);
    writer.string(inputs.outputPath);
    writer.string(inputs.dumpPath);
    blob(writer, inputs.world.bsp);

    std::vector<std::string_view> models;
    models.reserve(inputs.world.phyFiles.size());
    for (const auto& item : inputs.world.phyFiles) models.push_back(item.first);
    std::ranges::sort(models);
    writer.put(binaryCount(models.size()));
    for (const auto model : models) {
        writer.string(model);
        blob(writer, inputs.world.phyFiles.at(std::string(model)));
    }
    writer.put(binaryCount(inputs.world.entities.size()));
    for (const auto& entity : inputs.world.entities) {
        writer.string(entity.model);
        writer.vec(entity.origin);
        writer.vec(entity.angles);
    }
    writer.put(binaryCount(inputs.seedPositions.size()));
    for (const auto seed : inputs.seedPositions) writer.vec(seed);
    publishBinaryFile(path, writer.bytes());
}

OwnedJobInputs readJobFile(const std::filesystem::path& path) {
    const auto file = readBinaryFile(path);
    BinaryReader reader(file);
    if (!std::ranges::equal(reader.take(Magic.size()), std::as_bytes(std::span{Magic.data(), Magic.size()})))
        throw std::runtime_error("Invalid .navjob signature");
    const auto version = reader.get<std::uint32_t>();
    if (version != ClassicVersion && version != Version) throw std::runtime_error("Unsupported .navjob version");
    OwnedJobInputs owned;
    auto& inputs = owned.inputs;
    const auto mode = reader.get<std::uint8_t>();
    if (mode > JobInputs::WorldOnly) throw std::runtime_error("Invalid .navjob mode");
    inputs.mode = static_cast<JobInputs::Mode>(mode);
    inputs.threads = reader.get<std::uint32_t>();
    auto& params = inputs.params;
    params.slopeLimit = reader.get<float>();
    params.slopeTolerance = reader.get<float>();
    params.displacementTest = reader.get<std::int32_t>();
    params.generateFenceTops = reader.get<std::uint8_t>() != 0;
    params.fixupJumpAreas = reader.get<std::uint8_t>() != 0;
    params.generateJumpConnections = reader.get<std::uint8_t>() != 0;
    params.areaMaxSize = reader.get<std::int32_t>();
    if (version >= 2) params.exact = reader.get<std::uint8_t>() != 0;
    inputs.outputPath = reader.string();
    inputs.dumpPath = reader.string();
    owned.bsp = blob(reader);
    const auto models = reader.get<std::uint32_t>();
    for (std::uint32_t i = 0; i < models; ++i) {
        auto model = reader.string();
        if (!inputs.world.phyFiles.emplace(std::move(model), blob(reader)).second)
            throw std::runtime_error("Duplicate PHY model in .navjob");
    }
    const auto entities = reader.get<std::uint32_t>();
    for (std::uint32_t i = 0; i < entities; ++i)
        inputs.world.entities.push_back({reader.string(), reader.vec(), reader.vec()});
    const auto seeds = reader.get<std::uint32_t>();
    for (std::uint32_t i = 0; i < seeds; ++i) inputs.seedPositions.push_back(reader.vec());
    if (reader.remaining()) throw std::runtime_error("Unexpected trailing .navjob data");
    return owned;
}

} // namespace navfast
