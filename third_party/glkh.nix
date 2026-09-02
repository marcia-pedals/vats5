{ lib, clangStdenv, fetchurl }:

# GLKH - solver for the Equality Generalized Traveling Salesman Problem
# (E-GTSP), built on LKH. It transforms an (A)GTSP instance into an ATSP
# instance and solves it by invoking the LKH executable as a subprocess.
clangStdenv.mkDerivation rec {
  pname = "glkh";
  version = "1.1";

  src = fetchurl {
    url = "http://webhotel4.ruc.dk/~keld/research/GLKH/GLKH-${version}.tgz";
    hash = "sha256-7UR6ij3sYu6xOlQkh2QIUr5xzSIdGM/XZ9AQUDGb8Ts=";
  };

  # GLKH runs LKH with popen("./LKH ..."), i.e. it expects the LKH binary in
  # the current working directory. Patch it to resolve LKH via PATH so the
  # installed GLKH works from any directory. (GLKH still writes its scratch
  # files to TMP/ relative to the cwd, so callers must run it from a writable
  # directory containing a TMP subdirectory.)
  postPatch = ''
    substituteInPlace SRC/SolveTSP.c \
      --replace-fail '"./LKH %s"' '"LKH %s"'
  '';

  dontConfigure = true;

  enableParallelBuilding = false;

  installPhase = ''
    mkdir -p $out/bin
    cp GLKH GLKH_EXP GLKH_CHECK LKH $out/bin/
  '';

  meta = with lib; {
    description = "GLKH - LKH-based solver for the Generalized TSP";
    homepage = "http://webhotel4.ruc.dk/~keld/research/GLKH/";
    license = licenses.unfree; # Distributed for research use only
    platforms = [ "aarch64-darwin" "x86_64-darwin" "x86_64-linux" ];
  };
}
