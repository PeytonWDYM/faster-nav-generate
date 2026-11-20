#include "formats/bsp_reader.h"

#include <cstring>

namespace navfast::formats {
namespace {

constexpr std::uint32_t vbsp = 0x50534256;
constexpr std::uint32_t lzma = 0x414d5a4c;
constexpr std::uint32_t sprp = 0x73707270;

void rejectCompression(Bytes bytes, bool flagged) {
    if (flagged || (bytes.size() >= 4 && bytes.read<std::uint32_t>(0) == lzma)) {
        throw std::runtime_error("compressed BSP lumps are not supported yet");
    }
}

struct GameEntry {
    std::uint32_t id;
    std::uint16_t flags, version;
    std::uint32_t offset, size;
};
static_assert(sizeof(GameEntry) == 16);

bool validPropStride(std::uint16_t version, std::size_t stride) {
    switch (version) {
    case 4: return stride == 56;
    case 5: return stride == 60;
    case 6: return stride == 64;
    case 7: return stride == 68 || stride == 72;
    case 10: return stride == 72 || stride == 76;
    case 11: return stride == 80;
    default: return false;
    }
}

} // namespace

BspReader::BspReader(Bytes::Span bytes) : bytes_(bytes, "malformed BSP"),
    entries_(bytes_.read<decltype(entries_)>(8)) {
    const auto version = bytes_.read<std::int32_t>(4);
    if (bytes_.read<std::uint32_t>(0) != vbsp || version < 19 || version > 21) {
        throw std::runtime_error("expected VBSP version 19, 20, or 21");
    }
    bytes_.read<std::int32_t>(1032); // Map revision at the end of the header.
    for (const auto& entry : entries_) {
        if (entry.uncompressedSize != 0) rejectCompression(bytes_, true);
        if (entry.size != 0) {
            rejectCompression(bytes_.slice(entry.offset, entry.size), false);
            if (entry.offset < 1036) bytes_.fail();
        }
    }
}

Bytes BspReader::lump(Lump id) const {
    const auto& entry = entries_[static_cast<std::size_t>(id)];
    return bytes_.slice(entry.size ? entry.offset : 0, entry.size);
}

Bytes BspReader::table(Lump id, std::size_t stride) const {
    const auto data = lump(id);
    if (data.size() % stride != 0) data.fail();
    return data;
}

BspModel BspReader::model(std::size_t index) const {
    const auto data = table(Lump::Models, sizeof(BspModel));
    if (index >= data.size() / sizeof(BspModel)) data.fail();
    return data.read<BspModel>(index * sizeof(BspModel));
}

std::uint32_t BspReader::surfaceFlags(std::int16_t texInfo) const {
    if (texInfo == -1) return 0;
    const auto data = table(Lump::TexInfo, 72);
    const auto index = static_cast<std::size_t>(texInfo);
    if (index >= data.size() / 72) data.fail();
    return data.read<std::uint32_t>(72 * index + 64);
}

std::vector<std::uint16_t> BspReader::modelBrushes(const BspModel& model) const {
    const auto nodes = table(Lump::Nodes, 32);
    const auto version = entries_[static_cast<std::size_t>(Lump::Leaves)].version;
    if (version > 1) throw std::runtime_error("unsupported BSP leaf version");
    const std::size_t leafStride = version == 0 ? 56 : 32;
    const auto leaves = table(Lump::Leaves, leafStride);
    const auto leafBrushes = table(Lump::LeafBrushes, 2);
    const auto brushCount = table(Lump::Brushes, sizeof(BspBrush)).size() / sizeof(BspBrush);

    std::vector<bool> seenNodes(nodes.size() / 32), seenLeaves(leaves.size() / leafStride);
    std::vector<bool> seenBrushes(brushCount);
    std::vector<std::int32_t> pending{model.headNode};
    std::vector<std::uint16_t> result;
    while (!pending.empty()) {
        const auto head = pending.back();
        pending.pop_back();
        if (head >= 0) {
            const auto node = static_cast<std::size_t>(head);
            if (node >= seenNodes.size() || seenNodes[node]) nodes.fail();
            seenNodes[node] = true;
            pending.push_back(nodes.read<std::int32_t>(32 * node + 8));
            pending.push_back(nodes.read<std::int32_t>(32 * node + 4));
            continue;
        }

        const auto leaf = static_cast<std::size_t>(~head);
        if (leaf >= seenLeaves.size()) leaves.fail();
        if (seenLeaves[leaf]) continue;
        seenLeaves[leaf] = true;
        const auto first = leaves.read<std::uint16_t>(leafStride * leaf + 24);
        const auto count = leaves.read<std::uint16_t>(leafStride * leaf + 26);
        if (count == 0) continue;
        const auto indices = leafBrushes.records(2 * first, count, 2);
        for (std::size_t i = 0; i < count; ++i) {
            const auto brush = indices.read<std::uint16_t>(2 * i);
            if (brush >= seenBrushes.size()) indices.fail();
            if (seenBrushes[brush]) continue;
            seenBrushes[brush] = true;
            result.push_back(brush);
        }
    }
    return result;
}

StaticProps BspReader::staticProps() const {
    const auto game = lump(Lump::Game);
    if (game.size() == 0) return {};
    const auto directory = game.records(4, game.read<std::uint32_t>(0), sizeof(GameEntry));

    Bytes props = game.slice(0, 0);
    std::uint16_t version = 0;
    for (std::size_t pos = 0; pos < directory.size(); pos += sizeof(GameEntry)) {
        const auto entry = directory.read<GameEntry>(pos);
        if (entry.flags & 1) rejectCompression(game, true);
        if (entry.size == 0) continue;
        const auto base = entries_[static_cast<std::size_t>(Lump::Game)].offset;
        if (entry.offset < base) game.fail();
        const auto data = game.slice(entry.offset - base, entry.size);
        rejectCompression(data, false);
        if (entry.id == sprp) {
            if (props.size() != 0) game.fail();
            props = data;
            version = entry.version;
        }
    }
    if (props.size() == 0) return {};

    StaticProps result;
    const auto dict = props.records(4, props.read<std::uint32_t>(0), 128);
    result.models.reserve(dict.size() / 128);
    for (std::size_t pos = 0; pos < dict.size(); pos += 128) {
        const auto name = dict.slice(pos, 128).span();
        const auto* start = reinterpret_cast<const char*>(name.data());
        const auto* end = static_cast<const char*>(std::memchr(start, 0, 128));
        if (!end || start == end) dict.fail();
        result.models.push_back(normalizeModelPath({start, end}));
    }

    const auto leafHeader = 4 + dict.size();
    const auto leaves = props.records(leafHeader + 4, props.read<std::uint32_t>(leafHeader), 2);
    const auto propHeader = leafHeader + 4 + leaves.size();
    const auto count = props.read<std::uint32_t>(propHeader);
    const auto instances = props.slice(propHeader + 4, props.size() - propHeader - 4);
    if (count == 0) {
        if (instances.size() != 0) props.fail();
        return result;
    }
    const auto stride = instances.size() / count;
    if (instances.size() % count || !validPropStride(version, stride)) {
        throw std::runtime_error("unsupported or malformed BSP static prop layout (version " +
                                 std::to_string(version) + ", stride " + std::to_string(stride) + ")");
    }
    result.instances.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        const auto prop = instances.read<StaticProp>(i * stride);
        if (prop.model >= result.models.size() ||
            static_cast<std::size_t>(prop.firstLeaf) + prop.leafCount > leaves.size() / 2) props.fail();
        result.instances.push_back(prop);
    }
    return result;
}

std::string normalizeModelPath(std::string_view name) {
    std::string path(name);
    for (auto& c : path) {
        if (c == '\\') c = '/';
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c + ('a' - 'A'));
    }
    return path;
}

} // namespace navfast::formats
