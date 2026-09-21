#include <cstdlib>
#include <exception>
#include <string>

#include <spdlog/spdlog.h>
#include <CLI/CLI.hpp>

#include <fpvsim/version.hpp>

namespace {

int run(int argc, char** argv) {
  CLI::App cli{"fpvsim simcore: physics, sensors and the Betaflight bridge"};
  cli.set_version_flag("--version", fpvsim::kVersion);

  std::string session_path;
  cli.add_option("--session", session_path, "Path to resolved session.json in the run directory")
      ->required();

  CLI11_PARSE(cli, argc, argv);

  spdlog::error("simcore {} has no simulation loop yet (session: {})", fpvsim::kVersion,
                session_path);
  return EXIT_FAILURE;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    return run(argc, argv);
  } catch (const std::exception& error) {
    spdlog::critical("startup failed: {}", error.what());
    return EXIT_FAILURE;
  }
}
