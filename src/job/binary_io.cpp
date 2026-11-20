#include "job/binary_io.h"

#include <fstream>
#include <random>

namespace navfast {

std::vector<std::byte> readBinaryFile(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    const auto size = file.tellg();
    if (size < 0) throw std::runtime_error("Cannot open " + path.string());
    std::vector<std::byte> bytes(static_cast<std::size_t>(size));
    file.seekg(0);
    if (!file.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size())))
        throw std::runtime_error("Cannot read " + path.string());
    return bytes;
}

void publishBinaryFile(const std::filesystem::path& path, std::span<const std::byte> bytes, bool backup) {
    auto staged = path;
    staged += ".navfast-" + std::to_string(std::random_device{}()) + ".tmp";
    auto previous = path;
    previous += ".bak";
    bool backedUp = false;
    try {
        std::ofstream output(staged, std::ios::binary);
        output.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        output.close();
        if (!output) throw std::runtime_error("Cannot write " + staged.string());
        if (backup && std::filesystem::exists(path)) {
            std::filesystem::rename(path, previous);
            backedUp = true;
        }
        std::filesystem::rename(staged, path);
    } catch (...) {
        std::error_code ignored;
        std::filesystem::remove(staged, ignored);
        if (backedUp) std::filesystem::rename(previous, path, ignored);
        throw;
    }
}

} // namespace navfast
