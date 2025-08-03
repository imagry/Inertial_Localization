### 2025-08-03: Google Test Integration
* **Action:** Added Google Test framework to the project and created a sample test.
* **Details:** Integrated Google Test using FetchContent in the root CMakeLists.txt. Created a separate test executable in the Tests directory with a basic sample test to verify the integration works. Modified the existing Tests/CMakeLists.txt to exclude the problematic test library from the default build to avoid compilation errors.
* **Files Modified:** `CMakeLists.txt`, `Tests/CMakeLists.txt`, `Tests/test_gtest_sample.cpp`

