#!/usr/bin/bash

export CXXFLAGS='-Wall -Werror -O3 -m64'
if test $# -gt 0; then
    # If any arguments are given, distclean
    ./ns3 clean
    ./ns3 configure -d release --enable-static --disable-gtk --disable-tests --disable-examples
fi
./ns3 build

BUILDDIR=$PWD/build

for DIR in scratch/*; do
    test -d "$DIR" || continue
    if [ "$DIR" == "scratch/subdir" ]
    then
        continue
    fi
    NAME="$(basename $DIR)"
    mkdir -p "$DIR/static"
    SRC="$DIR/$NAME.cc"
    OBJ="$DIR/static/$NAME.o"
    BIN="$DIR/static/$NAME"

    # Compile
    g++ -std=c++17 -g -c -o "$OBJ" -I"$BUILDDIR/include" "$SRC"

    # Link static
    g++ -Wl,--dynamic-linker=/lib64/ld-linux-x86-64.so.2 -static -I${BUILDDIR}/include -o "$BIN" "$OBJ" -Wl,--whole-archive -L/usr/local/lib -L "$BUILDDIR/lib" \
    -lns3-dev-static -lrt -pthread \
    -Wl,--whole-archive -lpthread -Wl,--no-whole-archive -lgsl
done

