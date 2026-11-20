#include "pack.h"
#include "job/binary_io.h"
#include "job/job_file.h"

#include <charconv>
#include <cctype>
#include <optional>
#include <sstream>
#include <string_view>

namespace navfast {
namespace {

class EntityTokens {
public:
    explicit EntityTokens(std::string_view text) : text_(text) {}

    std::optional<std::string> next() {
        while (!text_.empty()) {
            if (std::isspace(static_cast<unsigned char>(text_.front()))) text_.remove_prefix(1);
            else if (text_.starts_with("//")) {
                const auto end = text_.find('\n');
                text_.remove_prefix(end == text_.npos ? text_.size() : end + 1);
            } else break;
        }
        if (text_.empty() || text_.front() == '\0') return {};
        const auto first = text_.front();
        text_.remove_prefix(1);
        if (first == '{' || first == '}') return std::string(1, first);
        if (first != '"') throw std::runtime_error("Malformed BSP entity token");
        std::string value;
        while (!text_.empty()) {
            const auto ch = text_.front();
            text_.remove_prefix(1);
            if (ch == '"') return value;
            if (ch == '\\' && text_.starts_with('"')) {
                value.push_back('"');
                text_.remove_prefix(1);
            } else {
                value.push_back(ch);
            }
        }
        throw std::runtime_error("Unterminated BSP entity value");
    }

private:
    std::string_view text_;
};

std::vector<Vec3> bspSeeds(std::span<const std::byte> bsp) {
    BinaryReader reader(bsp);
    if (reader.get<std::uint32_t>() != 0x50534256) throw std::runtime_error("Invalid BSP signature");
    reader.skip(4); // version
    const auto offset = reader.get<std::uint32_t>();
    const auto length = reader.get<std::uint32_t>();
    reader.skip(4); // lump version
    if (reader.get<std::uint32_t>()) throw std::runtime_error("Compressed BSP entity lumps are unsupported by pack");
    BinaryReader entityLump(bsp);
    entityLump.skip(offset);
    const auto bytes = entityLump.take(length);
    EntityTokens tokens({reinterpret_cast<const char*>(bytes.data()), bytes.size()});

    std::vector<Vec3> seeds;
    while (const auto opening = tokens.next()) {
        if (*opening != "{") throw std::runtime_error("Expected an opening brace in the BSP entity lump");
        std::string classname;
        std::string origin;
        for (;;) {
            const auto key = tokens.next();
            if (!key) throw std::runtime_error("Unterminated entity in BSP");
            if (*key == "}") break;
            auto value = tokens.next();
            if (!value) throw std::runtime_error("Missing entity property value in BSP");
            if (*key == "classname") classname = std::move(*value);
            else if (*key == "origin") origin = std::move(*value);
        }
        if (!classname.starts_with("info_player_") && classname != "gmod_player_start") continue;
        Vec3 pos;
        std::istringstream coordinates(origin);
        if (coordinates >> pos.x >> pos.y >> pos.z) seeds.push_back(pos);
    }
    return seeds;
}

} // namespace

void pack(const std::filesystem::path& bspPath, const std::filesystem::path& phyDir,
          const std::filesystem::path& jobPath) {
    const auto bsp = readBinaryFile(bspPath);
    JobInputs inputs;
    inputs.world.bsp = bsp;
    inputs.seedPositions = bspSeeds(bsp);
    for (const auto& model : listStaticPropModels(bsp)) {
        auto filename = pathFromUtf8(model);
        if (filename.is_absolute() || filename.lexically_normal() != filename || model.find("..") != std::string::npos)
            throw std::runtime_error("Invalid static prop model path in BSP");
        filename.replace_extension(".phy");
        auto path = phyDir / filename;
        if (!std::filesystem::is_regular_file(path)) path = phyDir / filename.filename();
        if (std::filesystem::is_regular_file(path)) inputs.world.phyFiles.emplace(model, readBinaryFile(path));
    }
    writeJobFile(jobPath, inputs);
}

} // namespace navfast
