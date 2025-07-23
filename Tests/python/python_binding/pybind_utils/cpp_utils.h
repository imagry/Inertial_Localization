#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <chrono>

// TODO: Check if everything works as expected if we include the declaration
//   of variableName in the Macro

#define TIMED_BLOCK(variableName, codeBlock)                               \
    auto variableName##_start = std::chrono::high_resolution_clock::now(); \
    codeBlock auto variableName##_end =                                    \
        std::chrono::high_resolution_clock::now();                         \
    variableName = std::chrono::duration_cast<std::chrono::microseconds>(  \
                       variableName##_end - variableName##_start)          \
                       .count() /                                          \
                   1000.0;

#define TIMED_BLOCK_PRINT(variableName, codeBlock)                         \
    auto variableName##_start = std::chrono::high_resolution_clock::now(); \
    codeBlock auto variableName##_end =                                    \
        std::chrono::high_resolution_clock::now();                         \
    variableName = std::chrono::duration_cast<std::chrono::microseconds>(  \
                       variableName##_end - variableName##_start)          \
                       .count() /                                          \
                   1000.0;                                                 \
    std::cout << #variableName << "= " << variableName << " ms\n";

namespace utils {

void writeCSV(const std::string& filename, const std::vector<double>& data,
              const std::vector<std::string>& columnTitles);

}  // namespace utils