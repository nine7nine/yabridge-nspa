#!/usr/bin/env bash
# Install yabridge build outputs to /usr/{bin,lib}.
#
# Invoked by `meson.add_install_script('tools/install-yabridge.sh')` from
# top-level meson.build, so the canonical entry point is:
#
#     sudo ninja -C build install
#
# Can also be run directly:
#
#     MESON_BUILD_ROOT=$PWD/build sudo tools/install-yabridge.sh
#
# Why this exists: yabridge is a meson cross build (winegcc for the Wine host
# .exe.so + native gcc for the Linux audio host libraries).  Meson refuses
# `install : true` on `native : true` targets in a cross build, so we work
# around that here.  Honours $DESTDIR for staged installs (PKGBUILD etc.).

set -eu

BUILD_ROOT="${MESON_BUILD_ROOT:-${1:-build}}"
PREFIX="/usr"
DEST="${DESTDIR:-}${PREFIX}"

if [[ ! -d "$BUILD_ROOT" ]]; then
    echo "install-yabridge.sh: build root '$BUILD_ROOT' not found" >&2
    exit 1
fi

echo "Installing yabridge from $BUILD_ROOT to ${DEST}/{bin,lib}"

# install -D creates parent dirs as needed.

# Plugin-side libraries (Linux native .so — loaded into Carla/Element/etc.).
for lib in libyabridge-vst2.so \
           libyabridge-vst3.so \
           libyabridge-clap.so \
           libyabridge-chainloader-vst2.so \
           libyabridge-chainloader-vst3.so \
           libyabridge-chainloader-clap.so; do
    src="$BUILD_ROOT/$lib"
    if [[ -f "$src" ]]; then
        install -Dm755 "$src" "$DEST/lib/$lib"
        echo "  -> $DEST/lib/$lib"
    fi
done

# Wine host (PE .exe.so loaded under wine + winegcc shell wrapper).
# The .exe shell wrapper needs +x; the .so is dlopen'd so 0644 matches the
# layout shipped by distro packages.
for host in yabridge-host yabridge-host-32; do
    if [[ -f "$BUILD_ROOT/$host.exe" ]]; then
        install -Dm755 "$BUILD_ROOT/$host.exe"    "$DEST/bin/$host.exe"
        install -Dm644 "$BUILD_ROOT/$host.exe.so" "$DEST/bin/$host.exe.so"
        echo "  -> $DEST/bin/$host.exe + .so"
    fi
done

echo "yabridge install complete."
