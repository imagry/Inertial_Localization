#include "cpp_utils.h"

namespace utils {

void writeCSV(const std::string& filename, const std::vector<double>& data,
              const std::vector<std::string>& columnTitles) {
    std::ofstream file(filename, std::ios::app);  // Open file in append mode

    if (!file.is_open()) {  // namespace utils
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    // Check if the file is empty, and if so, write column titles
    if (file.tellp() == 0) {
        for (std::size_t i = 0; i < columnTitles.size(); ++i) {
            file << columnTitles[i];
            if (i < columnTitles.size() - 1) {
                file << ",";  // Add comma if it's not the last column
            }
        }
        file << "\n";  // Move to the next line after writing column titles
    }

    // Write data to CSV file
    for (std::size_t i = 0; i < data.size(); ++i) {
        file << data[i];
        if (i < data.size() - 1) {
            file << ",";  // Add comma if it's not the last column
        }
    }
    file << "\n";  // Move to the next line after writing data

    file.close();
}

}  // namespace utils