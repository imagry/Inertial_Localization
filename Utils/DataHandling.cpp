/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#include <iostream>
#include <fstream>
#include <vector>
#include <numeric>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <sstream>  // Added for stringstream
#include "units.hpp"// NOLINT
#include "DataHandling.hpp"// NOLINT

//==============================================================================
// Console and Debug Output
//==============================================================================

void PrintToTerminal(const std::string str) { 
    std::cout << str << std::endl; 
}

void ListFilesInDirectory(const std::filesystem::path &path) {
    if (!std::filesystem::exists(path) ||
        !std::filesystem::is_directory(path)) {
        std::cout << "Path does not exist or is not a directory.\n";
        return;
    }
    
    for (const auto &entry : std::filesystem::directory_iterator(path)) {
        std::cout << entry.path() << '\n';
    }
}

void PrintCurrentPath() {
    std::filesystem::path current_path = std::filesystem::current_path();
    std::cout << "Current path is " << current_path << std::endl;
}

//==============================================================================
// File I/O Operations
//==============================================================================

std::vector<std::pair<std::string, std::vector<double>>> ReadCSV(
    const std::filesystem::path &path) {
    std::vector<std::pair<std::string, std::vector<double>>> data;
    std::vector<std::string> names;
    std::string line, word;

    std::fstream file(path, std::ios::in);
    if (!file.is_open()) {
        std::cerr << "DataHandling::ReadCSV: Error: Could not open file " << path << std::endl;
        return {}; // Return empty container on error
    } 
    
    std::cout << "DataHandling::ReadCSV: File is read: " << path << std::endl;
    
    // Read header line
    getline(file, line);
    std::stringstream str(line);
    
    // Parse column names
    while (getline(str, word, ',')) {
        names.push_back(word);
        data.push_back({word, {}});
    }
    
    // Read data rows
    while (getline(file, line)) {
        std::stringstream row_ss(line);
        std::string cell;
        int col_idx = 0;
        
        // Parse each cell in the row
        while (getline(row_ss, cell, ',') && col_idx < data.size()) {
            try {
                double value = std::stod(cell);
                data[col_idx].second.push_back(value);
            } catch (const std::exception& e) {
                std::cerr << "Error parsing value '" << cell << "': " << e.what() << std::endl;
                // Push a default value (0.0) or handle as needed
                data[col_idx].second.push_back(0.0);
            }
            col_idx++;
        }
    }
    
    file.close();
    return data;
}
void PrintToCSV(
    std::filesystem::path path,
    const std::vector<std::pair<std::string, std::vector<double>>> &data,
    bool print_message) {
    
    // Validate input data
    if (data.empty()) {
        std::cerr << "DataHandling::PrintToCSV: Error: No data to write" << std::endl;
        return;
    }
    
    // Create parent directories if they don't exist
    std::filesystem::create_directories(path.parent_path());
    
    std::ofstream output_file(path);
    if (!output_file.is_open()) {
        std::cerr << "DataHandling::PrintToCSV: Error: Could not open file for writing: " 
                  << path << std::endl;
        return;
    }
    
    // Get dimensions
    size_t num_cols = data.size();
    size_t num_rows = data[0].second.size();
    
    // Validate that all columns have the same number of rows
    for (size_t i = 0; i < num_cols; i++) {
        if (data[i].second.size() != num_rows) {
            std::cerr << "DataHandling::PrintToCSV: Error: Column " << i 
                      << " has " << data[i].second.size() 
                      << " rows, expected " << num_rows << std::endl;
            return;
        }
    }
    
    // Write header line
    for (size_t i = 0; i < num_cols; i++) {
        output_file << data[i].first;
        if (i != num_cols - 1) {
            output_file << ",";
        }
    }
    output_file << "\n";
    
    // Write data rows
    for (size_t i = 0; i < num_rows; i++) {
        for (size_t j = 0; j < num_cols; j++) {
            output_file << std::to_string(data[j].second[i]);
            if (j != num_cols - 1) {
                output_file << ",";
            }
        }
        output_file << "\n";
    }
    
    output_file.close();
    
    if (print_message) {
        std::cout << "Data is written to " << path.string() << std::endl;
    }
}
std::vector<std::pair<std::string, std::vector<double>>>
PackVectorsToNameValuePairs(const std::vector<std::string> &names,
                            const std::vector<std::vector<double>> &vectors) {
    // Validate input
    if (vectors.size() != names.size()) {
        std::cerr << "DataHandling::PackVectorsToNameValuePairs: Error: "
                  << "Number of names (" << names.size() 
                  << ") does not match number of vectors (" 
                  << vectors.size() << ")" << std::endl;
        return {};
    }
    
    // Create result container
    std::vector<std::pair<std::string, std::vector<double>>> result;
    result.reserve(names.size());
    
    // Pack names and vectors together
    for (size_t i = 0; i < names.size(); i++) {
        result.push_back(std::make_pair(names[i], vectors[i]));
    }
    
    return result;
}
//==============================================================================
// Time Functions
//==============================================================================

/**
 * @brief Helper function to convert a time point to a formatted string.
 * 
 * @param tp Time point to convert
 * @return std::string Formatted time string (YYYY-MM-DD_HH:MM:SS)
 */
std::string WhatsTheTimeString(const std::chrono::system_clock::time_point &tp) {
    // Convert system clock to time_t
    std::time_t t = std::chrono::system_clock::to_time_t(tp);

    // Convert time_t to tm
    std::tm *tm = std::localtime(&t);

    // Create a string stream
    std::ostringstream oss;

    // Write formatted time into the string stream
    oss << std::put_time(tm, "%Y-%m-%d_%H:%M:%S");

    // Return the formatted string
    return oss.str();
}

std::string WhatsTheTimeString() {
    // Get current time
    auto now = std::chrono::system_clock::now();
    return WhatsTheTimeString(now);
}

PreciseSeconds WhatsTheTimeSeconds() {
    // Get current time
    auto now = std::chrono::system_clock::now();
    
    // Convert to seconds since epoch with high precision
    auto duration_in_seconds = std::chrono::duration<double>(
        now.time_since_epoch());
    
    return duration_in_seconds.count();
}
