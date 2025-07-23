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
#include <cassert>  // for assertion
#include <filesystem>
#include <iomanip>
#include <chrono>
#include <ctime>
#include "units.hpp"// NOLINT
#include "DataHandling.hpp"// NOLINT
#define assertm(exp, msg) assert(((void)msg, exp))

void PrintToTerminal(const std::string str) { std::cout << str << std::endl; }
std::vector<std::pair<std::string, std::vector<double>>> ReadCSV(
    const std::filesystem::path &path) {
    std::vector<std::pair<std::string, std::vector<double>>> data;
    std::vector<std::string> names;
    std::vector<std::string> row;
    std::string line, word;

    std::fstream file(path, std::ios::in);
    if (!file.is_open()) {
        std::cerr << "DataHandling::ReadCSV: Error: Could not open file " << path << std::endl;
        return {}; // or handle the error as appropriate
    } else {
        std::cout << "DataHandling::ReadCSV: File is read: " << path << std::endl;
    }
    getline(file, line);  // read first line - header names
    std::stringstream str(line);
    while (getline(str, word, ',')) {
        names.push_back(word);
        data.push_back({word, {}});
    }
    // Read the data
    int row_idx = 0;
    while (getline(file, line)) {
        // iterate on rows
        std::stringstream row_ss(line);
        std::string cell;
        int col_idx = 0;
        while (getline(row_ss, cell, ',')) {
            // iterate on columns
            double value = stod(cell);
            data[col_idx].second.push_back(value);
            col_idx++;
        }
        row_idx++;
    }
    file.close();
    return data;
}
void PrintToCSV(
    std::filesystem::path path,
    const std::vector<std::pair<std::string, std::vector<double>>> &data,
    bool print_message) {
    std::ofstream output_file(path);
    // write header line
    // vector<string> headers;
    size_t num_cols = data.size();
    size_t num_rows = data[0].second.size();
    for (int i = 0; i < num_cols; i++) {
        assert(data[i].second.size() == num_rows);
        // headers.push_back(data[i].first);
        output_file << data[i].first;
        if (i != num_cols - 1) {
            output_file << ",";
        }
    }
    output_file << "\n";
    for (int i = 0; i < num_rows; i++) {
        for (int j = 0; j < num_cols; j++) {
            output_file << std::to_string(data[j].second[i]);
            if (j != num_cols - 1) {
                output_file << ",";
            }
        }
        if (i != num_rows - 1) {
            output_file << "\n";
        }
    }
    output_file.close();
    if (print_message)    {
        std::cout << "data is written to " + path.string() << std::endl;
        }
}
std::vector<std::pair<std::string, std::vector<double>>>
PackVectorsToNameValuePairs(const std::vector<std::string> &names,
                            const std::vector<std::vector<double>> &vectors) {
    assert(vectors.size() == names.size());
    int num_fields = names.size();
    std::vector<std::pair<std::string, std::vector<double>>> result;
    for (int i = 0; i < num_fields; i++) {
        result.push_back(make_pair(names[i], vectors[i]));
    }
    return result;
}
void ListFilesInDirectory(const std::filesystem::path &path) {
    if (!std::filesystem::exists(path) ||
        !std::filesystem::is_directory(path)) {
        std::cout << "Path does not exist or is not a directory.\n";
    }
    for (const auto &entry : std::filesystem::directory_iterator(path)) {
        std::cout << entry.path() << '\n';
    }
}
void PrintCurrentPath() {
    std::filesystem::path current_path = std::filesystem::current_path();
    std::cout << "Current path is " << current_path << std::endl;
}
std::string WhatsTheTimeString(const std::chrono::system_clock::time_point &tp) {
    // Convert system clock to time_t
    std::time_t t = std::chrono::system_clock::to_time_t(tp);

    // Convert time_t to tm
    std::tm *tm = std::localtime(&t);

    // Create a string stream
    std::ostringstream oss;

    // Write formatted time into the string stream
    oss << std::put_time(tm, "%Y-%m-%d_%H:%M:%S");

    // Get the string
    std::string str = oss.str();

    // Print the string
    // std::cout << str << std::endl;
    return str;
}

std::string WhatsTheTimeString() {
    static auto now = std::chrono::system_clock::now();
    return WhatsTheTimeString(now);
}

PreciseSeconds WhatsTheTimeSeconds() {
    auto now = std::chrono::system_clock::now();
    // auto duration_in_microseconds = std::chrono::duration_cast
    // <std::chrono::microseconds>(now.time_since_epoch());
    // return static_cast<PreciseSeconds>(
    //  duration_in_microseconds.count()) / 1000000.0;

    auto duration_in_seconds = std::chrono::duration<double>(
        now.time_since_epoch());
    return duration_in_seconds.count();
}
