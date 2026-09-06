{ lib, clangStdenv, fetchurl, qsopt, highs ? null, lpBackend ? "qsopt" }:

# `lpBackend` selects the LP solver linked into the library: "qsopt" (the
# prebuilt QSopt, installed alongside as libqsopt.a) or "highs" (the HiGHS
# library, via the LP/lphighs.c interface added by concorde-sparse.patch;
# link against ${highs}/lib as well).

# Use clang to build Concorde - its old C code triggers hard errors in GCC 14+
# (implicit-int, incompatible-pointer-types, missing h_addr macro).
assert lpBackend == "qsopt" || lpBackend == "highs";
assert lpBackend != "highs" || highs != null;
clangStdenv.mkDerivation rec {
  pname = "concorde" + (if lpBackend == "highs" then "-highs" else "");
  version = "03.12.19";

  src = fetchurl {
    url = "https://www.math.uwaterloo.ca/tsp/concorde/downloads/codes/src/co031219.tgz";
    hash = "sha256-w2UKWcjVfgoA6BwSiLmUqZxaoD5dlqMUg0wtj5UFxyQ=";
  };

  buildInputs = if lpBackend == "highs" then [ highs ] else [ qsopt ];

  # Makes the CC_SPARSE code paths (used via CCtsp_solve_dat on a sparse
  # datagroup) avoid all O(n^2) work: sparse initial edge set, sparse
  # Lin-Kernighan starting tour, and binary-search edge lookup. Also adds an
  # initial-LP warm start for bipartite graphs (e.g. doubled ATSP instances),
  # where Concorde's fractional matching cannot produce a basis. See
  # sparse-guide.md at the repository root.
  patches = [ ./concorde-sparse.patch ];

  # Skip the default configure phase - we'll do it manually
  dontConfigure = true;

  # Old C code compatibility - Concorde uses implicit int which modern compilers reject
  env.NIX_CFLAGS_COMPILE = "-Wno-error=implicit-int -Wno-error=implicit-function-declaration";

  # Concorde expects qsopt.a and qsopt.h in the same directory
  buildPhase = if lpBackend == "highs" then ''
    # Configure without an LP solver, then build the LP layer from the
    # HiGHS interface instead of the one configure picked.
    # (LDFLAGS rather than LIBS: the Makefiles list $(LIBS) as a
    # dependency of the executables.)
    CFLAGS="-Wno-error=implicit-int -Wno-error=implicit-function-declaration -O3" \
    CPPFLAGS="-I${highs}/include/highs" \
    LDFLAGS="-L${highs}/lib -lhighs" \
      ./configure

    make LPSOLVER_INTERFACE=lphighs.c
  '' else ''
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

    # Install the library.
    cp concorde.a $out/lib/libconcorde.a
    cp concorde.h $out/include/
  '' + lib.optionalString (lpBackend == "qsopt") ''
    # libconcorde.a has undefined references into QSopt, so also install
    # qsopt.a (renamed with the lib prefix so -lqsopt works) to make this
    # package self-contained for linking.
    cp ${qsopt}/lib/qsopt.a $out/lib/libqsopt.a
    cp ${qsopt}/include/qsopt.h $out/include/
  '';

  meta = with lib; {
    description = "Concorde TSP Solver - solves the Traveling Salesman Problem";
    homepage = "https://www.math.uwaterloo.ca/tsp/concorde/";
    license = licenses.unfree; # Free for academic use only
    platforms = [ "aarch64-darwin" "x86_64-darwin" "x86_64-linux" ];
  };
}
