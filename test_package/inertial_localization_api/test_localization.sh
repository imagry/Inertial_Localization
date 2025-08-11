#!/bin/bash

# test_localization.sh
# Regression test script for the localization functionality

set -e  # Exit immediately if a command exits with a non-zero status

# Define colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Define paths - adjusted for the new location in the Tests directory
REPO_ROOT=$(realpath $(dirname $0)/..)
BUILD_DIR="${REPO_ROOT}/build"
TEST_SCRIPT="${REPO_ROOT}/Tests/python/python_binding/carpose_offline_calculation.py"
DEFAULT_TRIP_PATH="${REPO_ROOT}/data/backed_data_files/2025-05-21T11_52_50"
RESULTS_DIR="${REPO_ROOT}/results"

# Expected values for regression testing (will be populated from first run)
EXPECTED_POSE_X=653.724931
EXPECTED_POSE_Y=513.178808
EXPECTED_POSE_YAW=0.790709

# Flag to indicate if we should update the expected values
UPDATE_EXPECTED=false

# Helper functions
print_header() {
    echo -e "\n${YELLOW}===== $1 =====${NC}\n"
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

# Parse command line arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --update-expected)
            UPDATE_EXPECTED=true
            ;;
        --trip-path)
            shift
            TRIP_PATH=$1
            ;;
        --help)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  --update-expected     Update the expected pose values based on this run"
            echo "  --trip-path PATH      Use specific trip path (default: ${DEFAULT_TRIP_PATH})"
            echo "  --help                Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown parameter: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
    shift
done

# Use default trip path if not specified
TRIP_PATH=${TRIP_PATH:-$DEFAULT_TRIP_PATH}

# Check if trip path exists
if [ ! -d "$TRIP_PATH" ]; then
    print_error "Trip path not found: $TRIP_PATH"
    exit 1
fi

# Create results directory if it doesn't exist
mkdir -p "$RESULTS_DIR"

# Step 1: Build the project
print_header "Building Project"
cd "$REPO_ROOT/Tests/python/python_binding"
./rebuild.sh

if [ $? -ne 0 ]; then
    print_error "Build failed"
    exit 1
fi
print_success "Build completed successfully"

# Step 2: Run the localization test
print_header "Running Localization Test"
cd "$REPO_ROOT"

# Activate the Python virtual environment
VENV_PATH="$REPO_ROOT/Tests/python/vehicle_control_env"
if [ ! -d "$VENV_PATH" ]; then
    print_error "Virtual environment not found at: $VENV_PATH"
    exit 1
fi
print_header "Activating Python Virtual Environment"
source "$VENV_PATH/bin/activate"

# Run the test script and capture output (without visualization)
TEST_OUTPUT=$(python3 "$TEST_SCRIPT" --trip_path "$TRIP_PATH" --output_dir "$RESULTS_DIR" 2>&1)

# Deactivate the virtual environment
deactivate

# Check if test completed successfully
if [ $? -ne 0 ]; then
    print_error "Localization test failed"
    echo "$TEST_OUTPUT"
    exit 1
fi

# Extract the final pose values
FINAL_X=$(echo "$TEST_OUTPUT" | grep "final_pose_x=" | cut -d'=' -f2)
FINAL_Y=$(echo "$TEST_OUTPUT" | grep "final_pose_y=" | cut -d'=' -f2)
FINAL_YAW=$(echo "$TEST_OUTPUT" | grep "final_pose_yaw=" | cut -d'=' -f2)

if [[ -z "$FINAL_X" || -z "$FINAL_Y" || -z "$FINAL_YAW" ]]; then
    print_error "Could not extract pose values from test output"
    echo "Test output:"
    echo "$TEST_OUTPUT"
    exit 1
fi

# Update expected values if requested
if [ "$UPDATE_EXPECTED" = true ]; then
    print_header "Updating Expected Pose Values"
    # Update this script with the new expected values
    sed -i "s/EXPECTED_POSE_X=.*/EXPECTED_POSE_X=$FINAL_X/" "$0"
    sed -i "s/EXPECTED_POSE_Y=.*/EXPECTED_POSE_Y=$FINAL_Y/" "$0"
    sed -i "s/EXPECTED_POSE_YAW=.*/EXPECTED_POSE_YAW=$FINAL_YAW/" "$0"
    print_success "Updated expected values:"
    echo "X: $FINAL_X"
    echo "Y: $FINAL_Y"
    echo "YAW: $FINAL_YAW"
    exit 0
fi

# Step 3: Compare results with expected values
print_header "Validating Results"

# Define acceptable tolerance for regression testing (adjust as needed)
TOLERANCE=0.001

# Function to check if a value is within tolerance
check_value() {
    local actual=$1
    local expected=$2
    local name=$3
    
    # Calculate absolute difference
    local diff=$(echo "$actual - $expected" | bc | tr -d '-')
    
    if (( $(echo "$diff <= $TOLERANCE" | bc -l) )); then
        print_success "$name: $actual (expected: $expected, diff: $diff)"
        return 0
    else
        print_error "$name: $actual (expected: $expected, diff: $diff)"
        return 1
    fi
}

# Check each component of the final pose
ERRORS=0
check_value "$FINAL_X" "$EXPECTED_POSE_X" "X position" || ERRORS=$((ERRORS+1))
check_value "$FINAL_Y" "$EXPECTED_POSE_Y" "Y position" || ERRORS=$((ERRORS+1))
check_value "$FINAL_YAW" "$EXPECTED_POSE_YAW" "YAW angle" || ERRORS=$((ERRORS+1))

# Step 4: Report overall results
print_header "Test Results"

if [ $ERRORS -eq 0 ]; then
    print_success "All tests passed! Localization functionality is working correctly."
    exit 0
else
    print_error "$ERRORS test(s) failed. Localization functionality may be broken."
    echo ""
    echo "Use --update-expected flag to update the expected values if the changes are intentional."
    exit 1
fi
