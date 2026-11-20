#include "clearance_check.h"
#include "pack.h"
#include "report.h"
#include "job/binary_io.h"
#include "job/job_file.h"

#include <charconv>
#include <fstream>
#include <iostream>
#include <string_view>

namespace {

using namespace navfast;

unsigned threads(std::string_view value) {
    unsigned result = 0;
    const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), result);
    if (error != std::errc{} || end != value.data() + value.size()) throw std::runtime_error("Invalid --threads value");
    return result;
}

std::ofstream outputReport(std::string_view path) {
    std::ofstream stream(pathFromUtf8(path), std::ios::out | std::ios::trunc);
    if (!stream) throw std::runtime_error("Cannot write report " + std::string(path));
    stream.exceptions(std::ios::badbit | std::ios::failbit);
    return stream;
}

void run(int argc, char** argv) {
    auto inputs = readJobFile(pathFromUtf8(argv[2]));
    inputs.inputs.outputPath = argv[3];
    inputs.inputs.dumpPath.clear();
    std::string_view json;
    for (int i = 4; i < argc;) {
        const std::string_view option = argv[i++];
        if (option == "--exact") inputs.inputs.params.exact = true;
        else if (option == "--classic") inputs.inputs.params.exact = false;
        else if (option == "--threads" || option == "--json") {
            if (i == argc) throw std::runtime_error("Missing value for " + std::string(option));
            const std::string_view value = argv[i++];
            if (option == "--threads") inputs.inputs.threads = threads(value);
            else json = value;
        } else throw std::runtime_error("Unknown option " + std::string(option));
    }
    if (!json.empty() && std::filesystem::absolute(pathFromUtf8(json)).lexically_normal() ==
                         std::filesystem::absolute(pathFromUtf8(argv[3])).lexically_normal())
        throw std::runtime_error(".nav and JSON report paths must differ");
    Job job;
    job.start(std::move(inputs));
    job.wait();
    const auto result = job.poll();
    if (!result.error.empty()) throw std::runtime_error(result.error);
    reportJob(std::cout, *result.stats, false);
    if (!json.empty()) {
        auto output = outputReport(json);
        reportJob(output, *result.stats, true);
        output.close();
    }
}

} // namespace

int main(int argc, char** argv) {
    try {
        const std::string_view command = argc > 1 ? argv[1] : "";
        if (command == "pack" && argc == 5) {
            navfast::pack(pathFromUtf8(argv[2]), pathFromUtf8(argv[3]), pathFromUtf8(argv[4]));
            return 0;
        }
        if (command == "run" && argc >= 4) {
            run(argc, argv);
            return 0;
        }
        if (command == "check-clearance" && argc == 3) {
            checkClearance(pathFromUtf8(argv[2]), std::cin, std::cout);
            return 0;
        }
        if (command == "inspect" && (argc == 3 || ((argc == 4 || argc == 5) && std::string_view(argv[3]) == "--json"))) {
            const auto nav = navfast::inspectNav(pathFromUtf8(argv[2]));
            if (argc == 5) {
                auto output = outputReport(argv[4]);
                navfast::reportNav(output, nav, true);
                output.close();
            } else navfast::reportNav(std::cout, nav, argc == 4);
            return 0;
        }
        std::cerr << "Usage\n"
                     "  navfast_cli pack <map.bsp> <phyDir> <out.navjob>\n"
                     "  navfast_cli run <job.navjob> <out.nav> [--exact | --classic] [--threads N] [--json report.json]\n"
                     "  navfast_cli check-clearance <job.navjob> < points.txt\n"
                     "  navfast_cli inspect <file.nav> [--json [report.json]]\n";
        return 2;
    } catch (const std::exception& error) {
        std::cerr << "navfast: " << error.what() << '\n';
        return 1;
    }
}
