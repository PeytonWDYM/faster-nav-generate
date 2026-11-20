#pragma once

#include "core/vec3.h"

#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace navfast {

static_assert(std::endian::native == std::endian::little && sizeof(float) == 4);

inline std::uint32_t binaryCount(std::size_t size) {
    if (size > std::numeric_limits<std::uint32_t>::max()) throw std::runtime_error("Binary field exceeds 32-bit size");
    return static_cast<std::uint32_t>(size);
}

class BinaryReader {
public:
    explicit BinaryReader(std::span<const std::byte> bytes) : bytes_(bytes) {}

    std::span<const std::byte> take(std::size_t count) {
        if (count > bytes_.size()) throw std::runtime_error("Truncated binary file");
        const auto part = bytes_.first(count);
        bytes_ = bytes_.subspan(count);
        return part;
    }
    void skip(std::size_t count, std::size_t width = 1) {
        if (count > bytes_.size() / width) throw std::runtime_error("Truncated binary file");
        take(count * width);
    }
    template <typename T> requires std::is_arithmetic_v<T>
    T get() {
        const auto bytes = take(sizeof(T));
        T value;
        std::memcpy(&value, bytes.data(), sizeof value);
        return value;
    }
    Vec3 vec() { return {get<float>(), get<float>(), get<float>()}; }
    std::string string() {
        const auto bytes = take(get<std::uint32_t>());
        return {reinterpret_cast<const char*>(bytes.data()), bytes.size()};
    }
    std::size_t remaining() const { return bytes_.size(); }

private:
    std::span<const std::byte> bytes_;
};

class BinaryWriter {
public:
    void append(std::span<const std::byte> bytes) {
        if (!bytes.empty()) bytes_.insert(bytes_.end(), bytes.begin(), bytes.end());
    }
    template <typename T> requires std::is_arithmetic_v<T>
    void put(T value) {
        append(std::as_bytes(std::span{&value, 1}));
    }
    void vec(Vec3 value) { put(value.x); put(value.y); put(value.z); }
    void string(std::string_view string) {
        put(binaryCount(string.size()));
        append(std::as_bytes(std::span{string.data(), string.size()}));
    }
    std::span<const std::byte> bytes() const { return bytes_; }

private:
    std::vector<std::byte> bytes_;
};

std::vector<std::byte> readBinaryFile(const std::filesystem::path& path);
void publishBinaryFile(const std::filesystem::path& path, std::span<const std::byte> bytes, bool backup = false);

inline std::filesystem::path pathFromUtf8(std::string_view path) {
    return std::filesystem::path(std::u8string(path.begin(), path.end()));
}

} // namespace navfast
