#!/bin/sh
# Builds every helper in native/ with the GnuPoc toolchain (GCCE 3.4.3, S60 3.2 SDK, PyS60 1.4.5 SDK).
# SYMBIAN_SDK = directory holding s60_32/, gnupoc-bin/ and csl-gcc/ (see
# https://github.com/manojanasuri16/symbian-s60v3-toolchain). Outputs go to ./out/.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
S=${SYMBIAN_SDK:-$HERE/../symbian-sdk}
export EPOCROOT="$S/s60_32/"
export PATH="$S/gnupoc-bin:$S/csl-gcc/bin:$PATH"
mkdir -p "$HERE/out"
for p in vidprobe:tgvprobe.exe vidbench:tgvbench.exe tgvideo:tgvideo.exe tgvdisp:tgvdisp.pyd \
         audio:tgvoiptest.exe audio:tgaudiotest.exe; do
  dir=${p%%:*}; out=${p##*:}
  ( cd "$HERE/native/$dir/group" && bldmake bldfiles >/dev/null 2>&1 && abld build gcce urel > "$HERE/out/$dir.build.log" 2>&1 ) || true
  if [ -f "$EPOCROOT/epoc32/release/gcce/urel/$out" ]; then
    cp "$EPOCROOT/epoc32/release/gcce/urel/$out" "$HERE/out/$out"; echo "built $out"
  else
    echo "FAILED $out (see out/$dir.build.log)"
  fi
done
