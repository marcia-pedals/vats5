{ lib, stdenv, fetchurl, qsopt }:

stdenv.mkDerivation rec {
  pname = "concorde";
  version = "03.12.19";

  src = fetchurl {
    url = "https://www.math.uwaterloo.ca/tsp/concorde/downloads/codes/src/co031219.tgz";
    hash = "sha256-w2UKWcjVfgoA6BwSiLmUqZxaoD5dlqMUg0wtj5UFxyQ=";
  };

  buildInputs = [ qsopt ];

  # Skip the default configure phase - we'll do it manually
  dontConfigure = true;

  # Old C code compatibility - Concorde uses implicit int which modern compilers reject
  env.NIX_CFLAGS_COMPILE = "-Wno-error=implicit-int -Wno-error=implicit-function-declaration";

  # Concorde expects qsopt.a and qsopt.h in the same directory
  buildPhase = ''
    # Set up QSopt directory
    mkdir -p qsopt_dir
    ln -s ${qsopt}/lib/qsopt.a qsopt_dir/
    ln -s ${qsopt}/include/qsopt.h qsopt_dir/

    # Configure with absolute path to qsopt_dir
    # Add flags for old C compatibility
    CFLAGS="-Wno-error=implicit-int -Wno-error=implicit-function-declaration -O3" \
      ./configure --with-qsopt=$PWD/qsopt_dir

    # Build
    make
  '';

  # Concorde's build system is old and doesn't support parallel builds well
  enableParallelBuilding = false;

  installPhase = ''
    mkdir -p $out/bin $out/lib $out/include

    # Install the main concorde executable
    cp TSP/concorde $out/bin/

    # Install linkern (Lin-Kernighan heuristic solver)
    cp LINKERN/linkern $out/bin/

    # Install the library
    cp concorde.a $out/lib/libconcorde.a

    # Install headers
    cp concorde.h $out/include/
  '';

  meta = with lib; {
    description = "Concorde TSP Solver - solves the Traveling Salesman Problem";
    homepage = "https://www.math.uwaterloo.ca/tsp/concorde/";
    license = licenses.unfree; # Free for academic use only
    platforms = [ "aarch64-darwin" "x86_64-darwin" "x86_64-linux" ];
  };
}
