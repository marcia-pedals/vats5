{
  description = "C++ development environment with Crow framework";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = with pkgs; [
            clang
            clang-tools
            cmake
            crow
            gcc
            llvm
            ninja
            nlohmann_json
            nodejs
          ];

          shellHook = ''
            export CC=${pkgs.gcc}/bin/gcc
            export CXX=${pkgs.gcc}/bin/g++
            export RC_PARAMS="max_success=5000"
          '';
        };
      });
}
