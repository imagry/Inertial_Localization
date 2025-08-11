#!/bin/bash

set -e

mkdir -p clang-tidy

conan install test_package \
  -pr:b default \
  -pr:h conan-profiles/debug.cfg \
  -of clang-tidy \
  -r imagry \
  -r conancenter \
  -c "tools.cmake.cmaketoolchain:user_toolchain=[\"$(pwd)/conan_dev_toolchain.cmake\"]"


conan install . \
  -pr:b default \
  -r imagry \
  -r conancenter \
  -pr:h conan-profiles/debug.cfg \
  -of clang-tidy \
  -c "tools.cmake.cmaketoolchain:user_toolchain=[\"$(pwd)/conan_dev_toolchain.cmake\"]"

cmake -S $PWD -B clang-tidy/build/Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Debug \
            -G Ninja -DPROJECT_BUILD_MODE=lint  \
            -DCMAKE_TOOLCHAIN_FILE=$(pwd)/clang-tidy/build/Debug/generators/conan_toolchain.cmake

# Get Changed files
CHANGED_FILES=$(git diff --name-only main -- "*.h" "*.hpp" "*.cpp" "*.cc" ":!*.pb.*" ":!*_test*" ":!*_mock*"  || true)

if [[ -z ${CHANGED_FILES} ]]; then
    echo "No files changed"
    exit 0
fi

# Run clang-tidy
clang-tidy --extra-arg='-std=gnu++20' \
       --extra-arg='-Wall' \
       --config="" \
       --extra-arg='-I/usr/include/c++/11' \
       --extra-arg='-I/usr/include/x86_64-linux-gnu/c++/11' \
       --extra-arg='-I/usr/lib/gcc/x86_64-linux-gnu/11/include' \
       --use-color \
       -p clang-tidy/build/Debug \
       ${CHANGED_FILES}