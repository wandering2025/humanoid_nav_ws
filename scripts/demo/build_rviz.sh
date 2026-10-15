#!/usr/bin/env bash
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
source_dir="$root/scripts/demo/rviz_lifecycle"
build_dir="$root/build/demo-rviz"
expected="$(cat "$source_dir/CMakeLists.txt" "$source_dir/main.cpp" | sha256sum | cut -d' ' -f1)"
if [[ -x "$build_dir/t1_demo_rviz" && -f "$build_dir/source.sha256" && "$(cat "$build_dir/source.sha256")" == "$expected" ]]; then exit 0; fi
command -v cmake >/dev/null && command -v g++ >/dev/null || { echo 'Missing existing cmake/g++; restore documented build environment, no automatic installation.' >&2; exit 2; }
cmake -S "$source_dir" -B "$build_dir" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$build_dir" --parallel 2
"$build_dir/t1_demo_rviz" --check-build
printf '%s\n' "$expected" > "$build_dir/source.sha256"
