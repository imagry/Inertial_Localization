#!/bin/bash

set -e

# Install the required packages
conan install test_package \
  -pr:b default \
  -pr:h conan-profiles/debug.cfg \
  -of . \
  -r imagry \
  -r conancenter \
  -c "tools.cmake.cmaketoolchain:user_toolchain=[\"$(pwd)/conan_dev_toolchain.cmake\"]"

conan install . \
  -pr:b default \
  -r imagry \
  -r conancenter \
  -pr:h conan-profiles/debug.cfg \
  -c "tools.cmake.cmaketoolchain:user_toolchain=[\"$(pwd)/conan_dev_toolchain.cmake\"]"

# Edit the .vscode/settings.json with the preset environment variables
PRESET_FILE=build/Debug/generators/CMakePresets.json
SETTINGS_FILE=.vscode/settings.json
mkdir -p .vscode
if [[ ! -f $SETTINGS_FILE ]]; then
  echo "{}" > $SETTINGS_FILE
fi
TEST_ENV=$(jq -r '.testPresets[0].environment' $PRESET_FILE)
if [[ $TEST_ENV == "null" ]]; then
  TEST_ENV="{}"
fi
jq -r ".\"cmake.environment\" += $TEST_ENV" $SETTINGS_FILE > $SETTINGS_FILE.tmp.json
mv $SETTINGS_FILE.tmp.json $SETTINGS_FILE
