#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)"
test_dir="$(mktemp -d /tmp/px4-turtle-test.XXXXXX)"
trap 'rm -rf -- "$test_dir"' EXIT
"${CXX:-g++}" -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
    "$repo_dir/src/drivers/dshot/tests/turtle_recovery_test.cpp" -o "$test_dir/turtle-test"
# LeakSanitizer cannot use ptrace inside some managed environments. This test
# does not allocate heap memory; keep address/UB instrumentation enabled.
ASAN_OPTIONS=detect_leaks=0 "$test_dir/turtle-test"
cmake --build "$repo_dir/build/px4_sitl_test" --target functional-ManualControl functional-ModeManagement -j4
ctest --test-dir "$repo_dir/build/px4_sitl_test" \
    -R '^functional-(ManualControl|ModeManagement)$' --output-on-failure
