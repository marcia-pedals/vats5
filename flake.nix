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

        # QSopt LP solver (prebuilt binary)
        qsopt = pkgs.callPackage ./third_party/qsopt.nix { };

        # Concorde TSP solver
        concorde = pkgs.callPackage ./third_party/concorde.nix { inherit qsopt; };
      in
      {
        # Export packages so they can be built with `nix build .#qsopt` etc.
        packages = {
          inherit qsopt concorde;
          default = concorde;
        };

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
          ] ++ [
            concorde
          ];

          shellHook = ''
            export CC=${pkgs.clang}/bin/clang
            export CXX=${pkgs.clang}/bin/clang++
            export RC_PARAMS="max_success=5000"
          '';
        };
      });
}
