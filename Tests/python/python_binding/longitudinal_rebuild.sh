#!/bin/bash
# Initialize Conda
# source /home/dor/anaconda3/etc/profile.d/conda.sh
source /home/dor/anaconda3/etc/profile.d/conda.sh
# activate relevant environment
conda activate longitudinal_conda_env

# Define the paths
BUILD_DIR="build"

# Remove existing build directory
rm -rf "$BUILD_DIR"

# Create the build directory
mkdir "$BUILD_DIR"

# Run cmake and make from the parent directory of build
cmake -S . -B "$BUILD_DIR"
make -C "$BUILD_DIR"

# Rename .so files in the build directory
for file in "$BUILD_DIR"/*.so; do
  new_name="$BUILD_DIR/${file#$BUILD_DIR/lib}"
  mv "$file" "$new_name"
done


