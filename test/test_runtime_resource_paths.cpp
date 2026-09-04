#include "app/resource_paths.h"

#include <filesystem>
#include <iostream>

int main()
{
    namespace fs = std::filesystem;
    const fs::path root = appProjectRoot();
    const fs::path config = appResourcePath("configs/config.json");
    const fs::path yolo = appResourcePath("AI/base/model/rknn_lt.rknn");
    const fs::path sign_fixture = appResourcePath(
        "test/img/Sign/sign_20260708_contact.png");

    const bool ok =
        fs::is_directory(root) &&
        fs::is_regular_file(config) &&
        fs::is_regular_file(yolo) &&
        fs::is_regular_file(sign_fixture) &&
        config == root / "configs/config.json";
    if (!ok) {
        std::cerr << "root=" << root << " config=" << config
                  << " yolo=" << yolo << " fixture=" << sign_fixture << "\n";
    }
    return ok ? 0 : 2;
}
