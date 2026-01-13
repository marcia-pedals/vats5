{ lib, stdenv, fetchurl, gzip }:

let
  # Platform-specific sources for QSopt prebuilt binaries
  sources = {
    aarch64-darwin = {
      url = "https://www.math.uwaterloo.ca/~bico/qsopt/downloads/codes/m1/qsopt.a";
      hash = "sha256-D/6b7Z3q6VHQLo9d+il3ilHOQr0uAfzAiTrIsynemKk=";
    };
    x86_64-darwin = {
      url = "https://www.math.uwaterloo.ca/~bico/qsopt/downloads/codes/mac64/qsopt.a.gz";
      hash = "sha256:0p8v47w1ld3aqx8zj27dsxfl2asdsrh8cha8wkplj0qqc99zxbg6";
      gzipped = true;
    };
    x86_64-linux = {
      url = "https://www.math.uwaterloo.ca/~bico/qsopt/downloads/codes/ubuntu/qsopt.a";
      hash = "sha256:0wygq24f3v1mrcvhqjkcl3h0gv4k9qhiqjs31ady6gd4ms9hzxv3";
    };
  };

  platformSource = sources.${stdenv.hostPlatform.system} or (throw "Unsupported platform: ${stdenv.hostPlatform.system}");

  qsoptLib = fetchurl {
    url = platformSource.url;
    hash = platformSource.hash;
  };

  qsoptHeader = fetchurl {
    url = "https://www.math.uwaterloo.ca/~bico/qsopt/downloads/codes/m1/qsopt.h";
    hash = "sha256-ZHcp8b134SY+zzXhiXxwXvHLReLWXb2cuP3131rmViQ=";
  };

in stdenv.mkDerivation {
  pname = "qsopt";
  version = "1.0";

  dontUnpack = true;

  nativeBuildInputs = lib.optionals (platformSource.gzipped or false) [ gzip ];

  installPhase = ''
    mkdir -p $out/lib $out/include

    ${if platformSource.gzipped or false then ''
      gunzip -c ${qsoptLib} > $out/lib/qsopt.a
    '' else ''
      cp ${qsoptLib} $out/lib/qsopt.a
    ''}

    cp ${qsoptHeader} $out/include/qsopt.h
  '';

  meta = with lib; {
    description = "QSopt Linear Programming Solver";
    homepage = "https://www.math.uwaterloo.ca/~bico/qsopt/";
    license = licenses.unfree; # Free for academic use only
    platforms = [ "aarch64-darwin" "x86_64-darwin" "x86_64-linux" ];
  };
}
