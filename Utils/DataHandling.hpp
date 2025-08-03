/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#pragma once
#include <vector>
#include <string>
#include <filesystem>
#include <utility>
#include "units.hpp"

//==============================================================================
// Console and Debug Output
//==============================================================================

/**
 * @brief Prints a message to the terminal.
 * 
 * @param str Message to print
 */
void PrintToTerminal(const std::string str);

/**
 * @brief Lists all files in a specified directory.
 * 
 * @param path Path to the directory
 */
void ListFilesInDirectory(const std::filesystem::path &path);

/**
 * @brief Prints the current working directory to the terminal.
 */
void PrintCurrentPath();

//==============================================================================
// File I/O Operations
//==============================================================================

/**
 * @brief Reads data from a CSV file.
 * 
 * @param path Path to the CSV file
 * @return std::vector<std::pair<std::string, std::vector<double>>> Vector of column name and value pairs
 */
std::vector<std::pair<std::string, std::vector<double>>> ReadCSV(
    const std::filesystem::path &path);

/**
 * @brief Writes data to a CSV file.
 * 
 * @param path Path where the CSV file should be saved
 * @param data Vector of column name and value pairs to write
 * @param print_message Whether to print a message indicating the file was written
 */
void PrintToCSV(
    std::filesystem::path path,
    const std::vector<std::pair<std::string, std::vector<double>>> &data,
    bool print_message = false);

/**
 * @brief Packs vectors of data into a format suitable for CSV output.
 * 
 * @param names Vector of column names
 * @param vectors Vector of data vectors (one per column)
 * @return std::vector<std::pair<std::string, std::vector<double>>> Vector of column name and value pairs
 */
std::vector<std::pair<std::string, std::vector<double>>>
PackVectorsToNameValuePairs(const std::vector<std::string> &names,
                            const std::vector<std::vector<double>> &vectors);

//==============================================================================
// Time Functions
//==============================================================================

/**
 * @brief Gets the current time as a formatted string.
 * 
 * @return std::string Current time in string format
 */
std::string WhatsTheTimeString();

/**
 * @brief Gets the current time in seconds since epoch.
 * 
 * @return PreciseSeconds Current time in seconds
 */
PreciseSeconds WhatsTheTimeSeconds();
