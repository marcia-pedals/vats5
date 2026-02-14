#include <CLI/CLI.hpp>
#include <iostream>
#include <string>

int main(int argc, char* argv[]) {
  CLI::App app{"Iterative expansion tool"};

  CLI11_PARSE(app, argc, argv);

  return 0;
}
