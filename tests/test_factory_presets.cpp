// Factory presets shipped with the app (apps/PianoApp/PianoApp/Presets):
// every file must load cleanly on the current parameter table.
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "pm_api/pm_api.h"
#include "test_harness.h"

#ifndef PM_FACTORY_PRESET_DIR
#error "PM_FACTORY_PRESET_DIR must be defined by the build"
#endif

namespace {

std::vector<std::filesystem::path> presetFiles() {
    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::directory_iterator(PM_FACTORY_PRESET_DIR)) {
        if (entry.is_regular_file() && entry.path().extension() == ".json") files.push_back(entry.path());
    }
    std::sort(files.begin(), files.end());
    return files;
}

std::string readFile(const std::filesystem::path& p) {
    std::ifstream in(p, std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

}  // namespace

TEST(factory_presets_load_cleanly) {
    const auto files = presetFiles();
    CHECK(files.size() >= 4);
    std::set<std::string> names;
    for (const auto& f : files) {
        const std::string json = readFile(f);
        CHECK(!json.empty());
        pm_engine* e = pm_engine_create();
        CHECK(e != nullptr);
        pm_preset_result_t r{};
        pm_preset_read(e, json.data(), json.size(), PM_PRESET_READ_MISSING_TO_DEFAULT, &r);
        if (r.ok != 1 || r.unknown != 0 || r.wrong_type != 0 || r.clamped != 0) {
            std::fprintf(stderr, "%s: ok %d unknown %d wrong_type %d clamped %d error '%s' warnings '%s'\n",
                         f.filename().string().c_str(), r.ok, r.unknown, r.wrong_type, r.clamped, r.error,
                         r.warnings);
        }
        CHECK(r.ok == 1);
        CHECK(r.unknown == 0);
        CHECK(r.wrong_type == 0);
        CHECK(r.clamped == 0);
        CHECK(r.name[0] != '\0');
        CHECK(names.insert(r.name).second);
        pm_engine_destroy(e);
    }
}
