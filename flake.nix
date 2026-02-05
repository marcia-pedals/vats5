{
  description = "C++ development environment with Crow framework";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    pre-commit-hooks.url = "github:cachix/pre-commit-hooks.nix";
  };

  outputs = { self, nixpkgs, flake-utils, pre-commit-hooks }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};

        pre-commit-check = pre-commit-hooks.lib.${system}.run {
          src = ./.;
          hooks = {
            clang-format = {
              enable = true;
              types_or = [ "c" "c++" ];
              files = "\\.(c|cpp|h|hpp|cc)$";
            };
          };
        };

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
            gh
            llvm
            ninja
            nlohmann_json
            nodejs
            pre-commit
          ] ++ [
            concorde
          ];

          shellHook = ''
            ${pre-commit-check.shellHook}
            export CC=${pkgs.clang}/bin/clang
            export CXX=${pkgs.clang}/bin/clang++
            export RC_PARAMS="max_success=5000"
          '';
        };
      });
}
