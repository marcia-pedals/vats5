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

        # Test data (GTFS feeds for tests and dev tools)
        testData = pkgs.fetchzip {
          url = "https://pub-42e805ab0ef7400f8f4eca373798da93.r2.dev/vats5-test-data-v1.tar.gz";
          sha256 = "sha256-9deT2h6s5qCBLukYbi7mvNhmEIfPwIxkFwPKNp0dGEk=";
          stripRoot = false;
        };
      in
      {
        # Export packages so they can be built with `nix build .#qsopt` etc.
        packages = {
          inherit qsopt concorde testData;
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
            git
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

            # Symlink test data if server/data doesn't exist
            if [ ! -e server/data ]; then
              echo "Linking test data from nix store..."
              ln -s ${testData} server/data
            fi
          '';
        };
      });
}
