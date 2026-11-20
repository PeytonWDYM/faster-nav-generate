#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include "probe.h"

namespace fs = std::filesystem;
namespace {

std::vector<std::byte> readFile(const fs::path& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) throw std::runtime_error("could not open " + path.string());
    const auto length = file.tellg();
    std::vector<std::byte> bytes(static_cast<std::size_t>(length));
    file.seekg(0);
    if (!file.read(reinterpret_cast<char*>(bytes.data()), length)) {
        throw std::runtime_error("could not read " + path.string());
    }
    return bytes;
}

void loadPhysics(const fs::path& root, navfast::WorldInputs& inputs) {
    for (const auto& model : navfast::listStaticPropModels(inputs.bsp)) {
        auto path = fs::path(model);
        if (path.is_absolute() || path.has_root_name()) continue;
        const auto relative = path.lexically_normal();
        if (relative.empty() || *relative.begin() == "..") continue;
        path = root / relative;
        path.replace_extension(".phy");
        if (fs::is_regular_file(path)) inputs.phyFiles.try_emplace(model, readFile(path));
    }
}

} // namespace

int main(int argc, char** argv) try {
    if (argc < 2 || argc > 4 || (argc == 3 && std::string_view(argv[2]) == "--report")) {
        std::cerr << "usage: world_probe <map.bsp> [phyRootDir] [report.json]\n"
                     "       world_probe <map.bsp> --report <report.json>\n";
        return 2;
    }

    const fs::path map(argv[1]);
    const auto bytes = readFile(map);
    navfast::WorldInputs inputs;
    inputs.bsp = bytes;
    if (argc >= 3 && std::string_view(argv[2]) != "--report") loadPhysics(argv[2], inputs);

    navfast::probe::Report report;
    const auto started = std::chrono::steady_clock::now();
    const auto world = navfast::buildCollisionWorldWithStats(inputs, report.world);
    report.buildMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
    navfast::probe::run(*world, map.stem().string(), report);
    navfast::probe::print(report, std::cout);
    if (argc == 4) {
        std::ofstream json(argv[3]);
        if (!json) throw std::runtime_error("could not open JSON report");
        navfast::probe::writeJson(report, json);
        json.close();
        if (!json) throw std::runtime_error("could not write JSON report");
    }
    return report.passed ? 0 : 1;
} catch (const std::exception& error) {
    std::cerr << "world_probe: " << error.what() << '\n';
    return 1;
}
