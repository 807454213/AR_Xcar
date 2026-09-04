#ifndef APP_RESOURCE_PATHS_H
#define APP_RESOURCE_PATHS_H

#include <filesystem>
#include <string>

#if defined(__linux__)
#include <unistd.h>
#endif

inline std::filesystem::path appProjectRoot()
{
    namespace fs = std::filesystem;

#if defined(__linux__)
    char exe_buf[4096] = {};
    const ssize_t n = ::readlink("/proc/self/exe", exe_buf, sizeof(exe_buf) - 1);
    if (n > 0) {
        exe_buf[n] = '\0';
        const fs::path candidate =
            fs::path(exe_buf).parent_path().parent_path().parent_path();
        if (fs::is_regular_file(candidate / "configs/config.json"))
            return candidate.lexically_normal();
    }
#endif

#ifdef XCAR_PROJECT_ROOT
    const fs::path compiled_root(XCAR_PROJECT_ROOT);
    if (fs::is_directory(compiled_root))
        return compiled_root.lexically_normal();
#endif

    return fs::current_path();
}

inline std::filesystem::path appResourcePath(const std::filesystem::path& path)
{
    if (path.is_absolute()) return path.lexically_normal();
    return (appProjectRoot() / path).lexically_normal();
}

inline std::string appResourcePathString(const std::filesystem::path& path)
{
    return appResourcePath(path).string();
}

#endif
