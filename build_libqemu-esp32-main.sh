#!/bin/sh

set -x

target="xtensa-softmmu,riscv32-softmmu"

case "$(uname)" in
  Linux)
    ncpu="$(nproc)"
    ;;
  Darwin)
    ncpu="$(sysctl -n hw.physicalcpu)"
    ;;
esac

flags="$(./configure --help | perl -ne 'print if s/^  ([a-z][\w-]*) .*/\1/' | tail -n +2 | awk '{print "--disable-"$1}' ORS=' ')"

${2:-.}/configure --target-list=$target --extra-cflags=-fPIC --disable-slirp $flags --enable-tcg \
	--enable-system --disable-werror --disable-alsa  \
        --disable-debug-info \
        --enable-gcrypt \
#       --enable-slirp 
	#--enable-gtk

make clean >/dev/null

# Build everything as usual
make "-j$ncpu" 

